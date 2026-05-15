#!/usr/bin/env python3
"""
DECT NR+ Mesh Network – UART → MQTT Bridge
Parses assembled image frames from the sink FT UART output and publishes to MQTT.

UART frame format (produced by uart.c):
  [MAGIC: 8 bytes = AA 55 BB 44 AA 55 BB 44]
  [total_length: 4 bytes LE]
  [payload: total_length bytes  (raw image data)]
  [seq_num: 4 bytes LE]
  [timestamp_pt: 4 bytes LE]
  [offset_pt_to_ft: 4 bytes LE, signed]
  [num_links: 1 byte]
  [devices_visited: 4 * ROUTING_MAX_HOPS bytes LE]
  [per_link_delay: 4 * ROUTING_MAX_HOPS bytes LE, signed]
  [per_link_rssi: 1 * ROUTING_MAX_HOPS bytes, signed]
  [CRC16/Modbus: 2 bytes LE]  — covers total_length + payload + all metadata above

CRC covers: total_length(4) + payload(variable) + metadata(85) bytes.
"""

import argparse
import base64
import json
import logging
import struct
import time

import paho.mqtt.client as mqtt
import serial

# ── Constants ────────────────────────────────────────────────────────────────

MAGIC = bytes([0xAA, 0x55, 0xBB, 0x44, 0xAA, 0x55, 0xBB, 0x44])

ROUTING_MAX_HOPS = 8

# seq_num(4) + timestamp_pt(4) + offset_pt_to_ft(4) + num_links(1)
# + devices_visited(4*8) + per_link_delay(4*8) + per_link_rssi(8)
META_SIZE = 4 + 4 + 4 + 1 + (4 * ROUTING_MAX_HOPS) + (4 * ROUTING_MAX_HOPS) + ROUTING_MAX_HOPS
assert META_SIZE == 85

MAX_IMAGE_BYTES = 1024 * 1024  # 1 MB sanity limit

# ── Logging ──────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("dect_bridge")

# ── CRC ──────────────────────────────────────────────────────────────────────

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc & 0xFFFF

# ── Metadata parsing ─────────────────────────────────────────────────────────

def parse_metadata(meta_bytes: bytes) -> dict:
    """
    Parse the 85-byte metadata block appended after the image payload.
    Returns only the fields relevant to the number of actual links.
    """
    if len(meta_bytes) != META_SIZE:
        raise ValueError(f"Expected {META_SIZE} meta bytes, got {len(meta_bytes)}")

    off = 0
    seq_num          = struct.unpack_from("<I", meta_bytes, off)[0]; off += 4
    timestamp_pt     = struct.unpack_from("<I", meta_bytes, off)[0]; off += 4
    offset_pt_to_ft  = struct.unpack_from("<i", meta_bytes, off)[0]; off += 4  # signed
    num_links        = meta_bytes[off];                               off += 1

    devices_visited = []
    for _ in range(ROUTING_MAX_HOPS):
        devices_visited.append(struct.unpack_from("<I", meta_bytes, off)[0])
        off += 4

    per_link_delay = []
    for _ in range(ROUTING_MAX_HOPS):
        per_link_delay.append(struct.unpack_from("<i", meta_bytes, off)[0])  # signed
        off += 4

    per_link_rssi = []
    for _ in range(ROUTING_MAX_HOPS):
        per_link_rssi.append(struct.unpack_from("b", meta_bytes, off)[0])    # signed byte
        off += 1

    # Trim arrays to the number of hops actually used.
    # devices_visited has num_links + 1 entries (origin + one per hop).
    n_devices = min(num_links + 1, ROUTING_MAX_HOPS)
    return {
        "seq_num":            seq_num,
        "timestamp_pt_ms":    timestamp_pt,
        "offset_pt_to_ft_ms": offset_pt_to_ft,
        "num_links":          num_links,
        "devices_visited":    [f"0x{v:08x}" for v in devices_visited[:n_devices]],
        "per_link_delay_ms":  per_link_delay[:num_links],
        "per_link_rssi_dbm":  per_link_rssi[:num_links],
    }

# ── Bridge ────────────────────────────────────────────────────────────────────

class UartMqttBridge:
    def __init__(self, port: str, baudrate: int, client: mqtt.Client, topic: str):
        self.port     = port
        self.baudrate = baudrate
        self.client   = client
        self.topic    = topic

    def run(self) -> None:
        """Open the serial port and loop forever, reconnecting on errors."""
        while True:
            try:
                with serial.Serial(self.port, self.baudrate, timeout=2) as ser:
                    log.info("Opened %s at %d baud", self.port, self.baudrate)
                    self._read_loop(ser)
            except serial.SerialException as exc:
                log.error("Serial error: %s — retrying in 5 s", exc)
                time.sleep(5)

    # ── Internal ────────────────────────────────────────────────────────────

    def _read_loop(self, ser: serial.Serial) -> None:
        magic_idx = 0
        while True:
            raw = ser.read(1)
            if not raw:
                continue
            b = raw[0]
            if b == MAGIC[magic_idx]:
                magic_idx += 1
                if magic_idx == len(MAGIC):
                    magic_idx = 0
                    self._handle_frame(ser)
            else:
                magic_idx = 1 if b == MAGIC[0] else 0

    def _read_exact(self, ser: serial.Serial, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = ser.read(n - len(buf))
            if not chunk:
                raise IOError(f"Timeout: needed {n} bytes, got {len(buf)}")
            buf += chunk
        return buf

    def _handle_frame(self, ser: serial.Serial) -> None:
        try:
            # ── total_length ──────────────────────────────────────────────
            len_bytes    = self._read_exact(ser, 4)
            total_length = struct.unpack("<I", len_bytes)[0]
            if total_length == 0 or total_length > MAX_IMAGE_BYTES:
                log.warning("Implausible total_length=%d — discarding frame", total_length)
                return

            # ── payload ───────────────────────────────────────────────────
            payload = self._read_exact(ser, total_length)

            # ── metadata ──────────────────────────────────────────────────
            meta_bytes = self._read_exact(ser, META_SIZE)

            # ── CRC ───────────────────────────────────────────────────────
            crc_bytes    = self._read_exact(ser, 2)
            received_crc = struct.unpack("<H", crc_bytes)[0]

            # CRC covers: total_length(4) + payload + metadata(85)
            computed_crc = crc16_modbus(len_bytes + payload + meta_bytes)
            if computed_crc != received_crc:
                log.error(
                    "CRC mismatch: computed=0x%04X received=0x%04X — dropping",
                    computed_crc, received_crc,
                )
                return

            meta = parse_metadata(meta_bytes)
            self._publish(payload, meta, total_length)

        except (IOError, ValueError) as exc:
            log.error("Frame error: %s", exc)

    def _publish(self, image_bytes: bytes, meta: dict, total_length: int) -> None:
        payload = {
            **meta,
            "total_size":      total_length,
            "received_at_ms":  int(time.time() * 1000),
            "image_base64":    base64.b64encode(image_bytes).decode(),
        }
        result = self.client.publish(self.topic, json.dumps(payload), qos=1)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            log.error("MQTT publish failed: rc=%d", result.rc)
        else:
            log.info(
                "Published  seq=%d  size=%d B  links=%d  path=%s",
                meta["seq_num"],
                total_length,
                meta["num_links"],
                " → ".join(meta["devices_visited"]),
            )

# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="DECT NR+ UART → MQTT bridge")
    parser.add_argument("--port",      default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud",      type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--broker",    default="localhost",      help="MQTT broker host (default: localhost)")
    parser.add_argument("--mqtt-port", type=int, default=1883,   help="MQTT broker port (default: 1883)")
    parser.add_argument("--topic",     default="dect/images",    help="MQTT publish topic (default: dect/images)")
    parser.add_argument("--client-id", default="dect-uart-bridge", help="MQTT client ID")
    parser.add_argument("--verbose",   action="store_true",      help="Enable DEBUG logging")
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    client = mqtt.Client(client_id=args.client_id)
    client.on_connect    = lambda c, u, f, rc: log.info("MQTT connected (rc=%d)", rc)
    client.on_disconnect = lambda c, u, rc:    log.warning("MQTT disconnected (rc=%d)", rc)

    log.info("Connecting to MQTT broker %s:%d", args.broker, args.mqtt_port)
    client.connect(args.broker, args.mqtt_port, keepalive=60)
    client.loop_start()

    bridge = UartMqttBridge(args.port, args.baud, client, args.topic)
    bridge.run()


if __name__ == "__main__":
    main()