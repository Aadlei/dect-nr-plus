"""
nRF9151 DK Image Receiver with MQTT Publishing
Wire format:
  [MAGIC:8][total_length:4]
  [seq_num:4][timestamp_pt:4][offset_pt_to_ft:4]
  [num_links:1][devices_visited:32][per_link_delay:32][per_link_rssi:8]
  [payload:total_length]
  [crc16:2]
CRC covers total_length + metadata + payload (everything after the magic).
"""
import serial
import struct
import sys
import argparse
import json
from datetime import datetime
import paho.mqtt.client as mqtt

ROUTING_MAX_HOPS = 8
MAGIC = b'\xAA\x55\xBB\x44\xAA\x55\xBB\x44'

METADATA_SIZE = (4 + 4 + 4 + 1
                 + (4 * ROUTING_MAX_HOPS)
                 + (4 * ROUTING_MAX_HOPS)
                 + (1 * ROUTING_MAX_HOPS))  # 85 bytes

# Full header = magic(8) + total_length(4) + all metadata(85)
HEADER_SIZE = 8 + 4 + METADATA_SIZE  # 97 bytes

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def parse_metadata(buf, offset):
    seq_num         = struct.unpack_from('<I', buf, offset)[0];      offset += 4
    timestamp_pt    = struct.unpack_from('<I', buf, offset)[0];      offset += 4
    offset_pt_to_ft = struct.unpack_from('<i', buf, offset)[0];      offset += 4
    num_links       = buf[offset];                                    offset += 1
    devices_visited = list(struct.unpack_from(f'<{ROUTING_MAX_HOPS}I', buf, offset)); offset += 4 * ROUTING_MAX_HOPS
    per_link_delay  = list(struct.unpack_from(f'<{ROUTING_MAX_HOPS}i', buf, offset)); offset += 4 * ROUTING_MAX_HOPS
    per_link_rssi   = list(struct.unpack_from(f'<{ROUTING_MAX_HOPS}b', buf, offset))
    return {
        'seq_num':         seq_num,
        'timestamp_pt':    timestamp_pt,
        'offset_pt_to_ft': offset_pt_to_ft,
        'num_links':       num_links,
        'devices_visited': devices_visited[:num_links + 1],
        'per_link_delay':  per_link_delay[:num_links + 1],
        'per_link_rssi':   per_link_rssi[:num_links + 1],
    }

def print_log(data: bytes):
    try:
        text = data.decode('utf-8', errors='replace')
        for line in text.splitlines():
            stripped = line.strip()
            if stripped and any(c.isalnum() for c in stripped):
                print(f"[LOG] {stripped}")
    except Exception:
        pass

def receive_images(port, baudrate, mqtt_broker, mqtt_port):
    mqtt_client = None
    if mqtt_broker:
        try:
            mqtt_client = mqtt.Client(client_id="nRF9151_ImagePublisher")
            mqtt_client.connect(mqtt_broker, mqtt_port, 60)
            mqtt_client.loop_start()
            print(f"Connected to MQTT broker: {mqtt_broker}:{mqtt_port}")
        except Exception as e:
            print(f"MQTT connection failed: {e}")
            mqtt_client = None

    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)
    except serial.SerialException as e:
        print(f"Error: {e}")
        print(f"Make sure {port} is not already open.")
        sys.exit(1)

    print(f"=== nRF9151 Image Receiver ===")
    print(f"Port: {port}  Baud: {baudrate}")
    print(f"Header: {HEADER_SIZE} bytes  Metadata: {METADATA_SIZE} bytes  ROUTING_MAX_HOPS: {ROUTING_MAX_HOPS}")
    print(f"Press Ctrl+C to stop\n")

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
                    if len(buf) > 7:
                        print_log(buf[:-7])
                        buf = buf[-7:]
                    break

                if magic_pos > 0:
                    print_log(buf[:magic_pos])
                    buf = buf[magic_pos:]

                if len(buf) < HEADER_SIZE:
                    break

                total_length = struct.unpack_from('<I', buf, 8)[0]

                if total_length == 0 or total_length > 1024 * 1024:
                    print(f"[WARN] Implausible length {total_length}, resyncing")
                    buf = buf[4:]
                    continue

                frame_total = HEADER_SIZE + total_length + 2
                if len(buf) < frame_total:
                    break

                # Metadata sits between total_length and payload
                meta_start = 12  # after magic(8) + total_length(4)
                payload    = buf[HEADER_SIZE : HEADER_SIZE + total_length]
                crc_recv   = struct.unpack_from('<H', buf, HEADER_SIZE + total_length)[0]

                # CRC covers everything after the magic: total_length + metadata + payload
                crc_calc = crc16(buf[8 : HEADER_SIZE + total_length])

                if crc_recv != crc_calc:
                    print(f"[WARN] CRC mismatch (recv=0x{crc_recv:04X}, calc=0x{crc_calc:04X})")
                    buf = buf[4:]
                    continue

                hdr = parse_metadata(buf, meta_start)

                image_count += 1
                ts = datetime.now()
                kb = total_length / 1024

                print(f"\n[IMAGE #{image_count}]  ({kb:.1f} KB, CRC=0x{crc_calc:04X} OK)")
                print(f"  seq={hdr['seq_num']}  num_links={hdr['num_links']}")
                print(f"  timestamp_pt={hdr['timestamp_pt']}  offset_pt_to_ft={hdr['offset_pt_to_ft']}")
                print(f"  devices_visited={[hex(d) for d in hdr['devices_visited']]}")
                print(f"  per_link_delay={hdr['per_link_delay']}")
                print(f"  per_link_rssi={hdr['per_link_rssi']} dBm")

                magic_in_payload = payload.count(MAGIC)
                if magic_in_payload > 0:
                    print(f"  WARNING: Magic found {magic_in_payload} times in payload!")

                if mqtt_client:
                    mqtt_data = {
                        "timestamp":        ts.timestamp(),
                        "size_bytes":       total_length,
                        "size_kb":          round(kb, 2),
                        "crc":              f"0x{crc_calc:04X}",
                        "image_count":      image_count,
                        "seq_num":          hdr['seq_num'],
                        "num_links":        hdr['num_links'],
                        "timestamp_pt":     hdr['timestamp_pt'],
                        "offset_pt_to_ft":  hdr['offset_pt_to_ft'],
                        "devices_visited":  hdr['devices_visited'],
                        "per_link_delay":   hdr['per_link_delay'],
                        "per_link_rssi":    hdr['per_link_rssi'],
                    }
                    mqtt_client.publish("images/metadata", json.dumps(mqtt_data), qos=1)
                    mqtt_client.publish("images/data", payload, qos=1)
                    print(f"  Published to MQTT")

                buf = buf[frame_total:]

    except KeyboardInterrupt:
        print(f"\nStopped. Received {image_count} images total.")
        ser.close()
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()

def main():
    parser = argparse.ArgumentParser(
        description="Receive images from nRF9151 DK and publish via MQTT")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=1000000,
                        help="Baud rate (default: 1000000)")
    parser.add_argument("--mqtt-broker", default="10.225.150.248",
                        help="MQTT broker address")
    parser.add_argument("--mqtt-port", type=int, default=1883,
                        help="MQTT broker port")
    args = parser.parse_args()
    receive_images(args.port, args.baud, args.mqtt_broker, args.mqtt_port)

if __name__ == "__main__":
    main()