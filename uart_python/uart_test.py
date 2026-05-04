"""
nRF9151 DK Image Receiver with MQTT Publishing
Wire format (85-byte header):
  [MAGIC:4][total_length:4][seq_num:4][timestamp_pt:4][offset_pt_to_ft:4]
  [num_links:1][devices_visited:32][per_link_delay:32]
  [payload:total_length][crc16:2]
CRC covers header[4:] + payload.
"""
import serial
import struct
import sys
import argparse
import json
from datetime import datetime
import paho.mqtt.client as mqtt

ROUTING_MAX_HOPS = 8
MAGIC = b'\xAA\x55\xAA\x55'

# Header size: 4 magic + 4 length + 4 seq + 4 timestamp + 4 offset + 1 num_links
#              + 4*8 devices_visited + 4*8 per_link_delay = 85 bytes
HEADER_SIZE = 4 + 4 + 4 + 4 + 4 + 1 + (4 * ROUTING_MAX_HOPS) + (4 * ROUTING_MAX_HOPS)

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

def parse_header(buf):
    """Parse header fields from buffer starting at magic bytes."""
    total_length    = struct.unpack_from('<I', buf, 4)[0]
    seq_num         = struct.unpack_from('<I', buf, 8)[0]
    timestamp_pt    = struct.unpack_from('<I', buf, 12)[0]
    offset_pt_to_ft = struct.unpack_from('<i', buf, 16)[0]  # signed
    num_links       = buf[20]
    devices_visited = list(struct.unpack_from(f'<{ROUTING_MAX_HOPS}I', buf, 21))
    per_link_delay  = list(struct.unpack_from(f'<{ROUTING_MAX_HOPS}I', buf, 21 + 4 * ROUTING_MAX_HOPS))
    return {
        'total_length':    total_length,
        'seq_num':         seq_num,
        'timestamp_pt':    timestamp_pt,
        'offset_pt_to_ft': offset_pt_to_ft,
        'num_links':       num_links,
        'devices_visited': devices_visited[:num_links + 1],
        'per_link_delay':  per_link_delay[:num_links + 1],
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
    print(f"Header size: {HEADER_SIZE} bytes  ROUTING_MAX_HOPS: {ROUTING_MAX_HOPS}")
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
                    if len(buf) > 3:
                        print_log(buf[:-3])
                        buf = buf[-3:]
                    break

                if magic_pos > 0:
                    print_log(buf[:magic_pos])
                    buf = buf[magic_pos:]

                if len(buf) < HEADER_SIZE:
                    break

                hdr = parse_header(buf)
                total_length = hdr['total_length']

                if total_length == 0 or total_length > 1024 * 1024:
                    print(f"[WARN] Implausible length {total_length}, resyncing")
                    buf = buf[4:]
                    continue

                frame_total = HEADER_SIZE + total_length + 2
                if len(buf) < frame_total:
                    break

                # CRC covers header[4:] + payload
                meta_bytes = buf[4:HEADER_SIZE]
                payload    = buf[HEADER_SIZE : HEADER_SIZE + total_length]
                crc_recv   = struct.unpack_from('<H', buf, HEADER_SIZE + total_length)[0]
                crc_calc   = crc16(meta_bytes + payload)

                if crc_recv != crc_calc:
                    print(f"[WARN] CRC mismatch (recv=0x{crc_recv:04X}, calc=0x{crc_calc:04X})")
                    buf = buf[4:]
                    continue

                image_count += 1
                ts = datetime.now()
                kb = total_length / 1024

                print(f"\n[IMAGE #{image_count}]  ({kb:.1f} KB, CRC=0x{crc_calc:04X} OK)")
                print(f"  seq={hdr['seq_num']}  num_links={hdr['num_links']}")
                print(f"  timestamp_pt={hdr['timestamp_pt']}  offset_pt_to_ft={hdr['offset_pt_to_ft']}")
                print(f"  devices_visited={[hex(d) for d in hdr['devices_visited']]}")
                print(f"  per_link_delay={hdr['per_link_delay']}")

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