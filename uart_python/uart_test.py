"""
DECT NR+ Mesh — UART → MQTT Bridge

Wire format (uart.c):
  [MAGIC:8]
  [total_length:4][seq_num:4][timestamp_pt:4][offset_pt_to_ft:4]
  [num_links:1][devices_visited:32][per_link_delay:32][per_link_rssi:8]
  [payload:total_length]
  [crc16:2]

CRC covers everything after the magic (total_length + metadata + payload).
"""

import argparse
import json
import struct
import sys
from datetime import datetime

import paho.mqtt.client as mqtt
import serial

ROUTING_MAX_HOPS = 8
MAGIC            = b'\xAA\x55\xBB\x44\xAA\x55\xBB\x44'
METADATA_SIZE    = 4 + 4 + 4 + 1 + (4 * ROUTING_MAX_HOPS) + (4 * ROUTING_MAX_HOPS) + ROUTING_MAX_HOPS  # 85
HEADER_SIZE      = len(MAGIC) + 4 + METADATA_SIZE  # 97


# ── CRC ──────────────────────────────────────────────────────────────────────

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc


# ── Parsing ───────────────────────────────────────────────────────────────────

def parse_header(buf: bytes, offset: int) -> dict:
    """Parse the 89-byte block starting at offset (right after magic)."""
    total_length    = struct.unpack_from('<I', buf, offset)[0];     offset += 4
    seq_num         = struct.unpack_from('<I', buf, offset)[0];     offset += 4
    timestamp_pt    = struct.unpack_from('<I', buf, offset)[0];     offset += 4
    offset_pt_to_ft = struct.unpack_from('<i', buf, offset)[0];     offset += 4
    num_links       = buf[offset];                                  offset += 1
    devices_visited = list(struct.unpack_from(f'<{ROUTING_MAX_HOPS}I', buf, offset)); offset += 4 * ROUTING_MAX_HOPS
    per_link_delay  = list(struct.unpack_from(f'<{ROUTING_MAX_HOPS}i', buf, offset)); offset += 4 * ROUTING_MAX_HOPS
    per_link_rssi   = list(struct.unpack_from(f'<{ROUTING_MAX_HOPS}b', buf, offset))

    n = num_links + 1
    return {
        'total_length':    total_length,
        'seq_num':         seq_num,
        'timestamp_pt':    timestamp_pt,
        'offset_pt_to_ft': offset_pt_to_ft,
        'num_links':       num_links,
        'devices_visited': devices_visited[:n],
        'per_link_delay':  per_link_delay[:n],
        'per_link_rssi':   per_link_rssi[:n],
    }


# ── Log passthrough ───────────────────────────────────────────────────────────

def print_log(data: bytes):
    try:
        for line in data.decode('utf-8', errors='replace').splitlines():
            s = line.strip()
            if s and any(c.isalnum() for c in s):
                print(f"[LOG] {s}")
    except Exception:
        pass


# ── Main receive loop ─────────────────────────────────────────────────────────

def receive_images(port: str, baudrate: int, mqtt_broker: str, mqtt_port: int):
    client = None
    if mqtt_broker:
        try:
            client = mqtt.Client(client_id="dect_bridge")
            client.connect(mqtt_broker, mqtt_port, keepalive=60)
            client.loop_start()
            print(f"MQTT: connected to {mqtt_broker}:{mqtt_port}")
        except Exception as e:
            print(f"MQTT: connection failed — {e}")
            client = None

    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)
    except serial.SerialException as e:
        print(f"Serial: {e}")
        sys.exit(1)

    print(f"Port: {port}  Baud: {baudrate}")
    print(f"HEADER_SIZE={HEADER_SIZE}  METADATA_SIZE={METADATA_SIZE}  ROUTING_MAX_HOPS={ROUTING_MAX_HOPS}")
    print("Press Ctrl+C to stop\n")

    image_count = 0
    buf = b''

    try:
        while True:
            chunk = ser.read(4096)
            if not chunk:
                continue

            print(f"[DBG] got {len(chunk)} bytes: {chunk[:20].hex()}")
            buf += chunk

            while True:
                magic_pos = buf.find(MAGIC)

                if magic_pos == -1:
                    if len(buf) > len(MAGIC) - 1:
                        print_log(buf[:-(len(MAGIC) - 1)])
                        buf = buf[-(len(MAGIC) - 1):]
                    break

                if magic_pos > 0:
                    print_log(buf[:magic_pos])
                    buf = buf[magic_pos:]

                if len(buf) < HEADER_SIZE:
                    break

                hdr = parse_header(buf, len(MAGIC))
                total_length = hdr['total_length']

                if total_length == 0 or total_length > 1024 * 1024:
                    print(f"[WARN] implausible total_length={total_length}, resyncing")
                    buf = buf[len(MAGIC):]
                    continue

                frame_size = HEADER_SIZE + total_length + 2
                if len(buf) < frame_size:
                    break

                payload  = buf[HEADER_SIZE : HEADER_SIZE + total_length]
                crc_recv = struct.unpack_from('<H', buf, HEADER_SIZE + total_length)[0]
                crc_calc = crc16(buf[len(MAGIC) : HEADER_SIZE + total_length])

                if crc_recv != crc_calc:
                    print(f"[WARN] CRC mismatch recv=0x{crc_recv:04X} calc=0x{crc_calc:04X}, resyncing")
                    buf = buf[len(MAGIC):]
                    continue

                image_count += 1
                kb = total_length / 1024

                print(f"\n[IMAGE #{image_count}]  ({kb:.1f} KB, CRC=0x{crc_calc:04X} OK)")
                print(f"  seq={hdr['seq_num']}  num_links={hdr['num_links']}")
                print(f"  timestamp_pt={hdr['timestamp_pt']}  offset_pt_to_ft={hdr['offset_pt_to_ft']}")
                print(f"  devices_visited={[f'0x{d:08x}' for d in hdr['devices_visited']]}")
                print(f"  per_link_delay={hdr['per_link_delay']}")
                print(f"  per_link_rssi={hdr['per_link_rssi']} dBm")

                if client:
                    meta = {
                        "timestamp":        datetime.now().timestamp(),
                        "seq_num":          hdr['seq_num'],
                        "size_bytes":       total_length,
                        "crc":              f"0x{crc_calc:04X}",
                        "num_links":        hdr['num_links'],
                        "timestamp_pt":     hdr['timestamp_pt'],
                        "offset_pt_to_ft":  hdr['offset_pt_to_ft'],
                        "devices_visited":  hdr['devices_visited'],
                        "per_link_delay":   hdr['per_link_delay'],
                        "per_link_rssi":    hdr['per_link_rssi'],
                    }
                    client.publish("images/metadata", json.dumps(meta), qos=1)
                    client.publish("images/data",     payload,          qos=1)
                    print("  Published to MQTT")

                buf = buf[frame_size:]

    except KeyboardInterrupt:
        print(f"\nStopped. Received {image_count} images total.")
        ser.close()
        if client:
            client.loop_stop()
            client.disconnect()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="DECT NR+ UART → MQTT bridge")
    parser.add_argument("--port",        default="/dev/ttyUSB0",    help="Serial port")
    parser.add_argument("--baud",        type=int, default=1000000, help="Baud rate")
    parser.add_argument("--mqtt-broker", default="10.225.150.248",  help="MQTT broker address")
    parser.add_argument("--mqtt-port",   type=int, default=1883,    help="MQTT broker port")
    args = parser.parse_args()
    receive_images(args.port, args.baud, args.mqtt_broker, args.mqtt_port)


if __name__ == "__main__":
    main()