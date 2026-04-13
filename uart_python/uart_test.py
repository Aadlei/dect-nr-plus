"""
nRF9151 DK Image Receiver with MQTT Publishing
"""
import serial
import struct
import time
import sys
import argparse
import json
from pathlib import Path
from datetime import datetime
import paho.mqtt.client as mqtt

MAGIC = b'\xAA\x55\xAA\x55'
HEADER_SIZE = 15  # [MAGIC:4] [TX_ID:2] [HOP_COUNT:1] [SEQ:4] [LENGTH:4]
META_SIZE = 7     # tx_id(2) + hop_count(1) + seq_num(4)

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

def print_log(data: bytes):
    try:
        text = data.decode('utf-8', errors='replace')
        for line in text.splitlines():
            stripped = line.strip()
            if stripped and any(c.isalnum() for c in stripped):
                print(f"[LOG] {stripped}")
    except Exception:
        pass

def receive_images(port, baudrate, output_dir, mqtt_broker, mqtt_port):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

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
    print(f"Output: {out.absolute()}")
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

                tx_id = struct.unpack_from('<H', buf, 4)[0]
                hop_count = buf[6]
                seq_num = struct.unpack_from('<I', buf, 7)[0]
                length = struct.unpack_from('<I', buf, 11)[0]

                frame_total = HEADER_SIZE + length + 2
                if len(buf) < frame_total:
                    break

                meta_bytes = buf[4:4 + META_SIZE]
                payload = buf[HEADER_SIZE : HEADER_SIZE + length]
                crc_recv = struct.unpack_from('<H', buf, HEADER_SIZE + length)[0]
                crc_calc = crc16(meta_bytes + payload)

                if crc_recv != crc_calc:
                    print(f"[WARN] CRC mismatch (recv=0x{crc_recv:04X}, calc=0x{crc_calc:04X})")
                    buf = buf[4:]
                    continue

                image_count += 1
                ts = datetime.now()
                timestamp_str = ts.strftime("%Y%m%d_%H%M%S")
                filename = f"image_{timestamp_str}_{image_count:04d}.jpg"
                filepath = out / filename

                with open(filepath, 'wb') as f:
                    f.write(payload)

                kb = length / 1024
                print(f"\n[IMAGE #{image_count}] {filepath.name}  "
                      f"({kb:.1f} KB, CRC=0x{crc_calc:04X} OK)")
                print(f"  tx_id={tx_id}  hops={hop_count}  seq={seq_num}")

                if mqtt_client:
                    mqtt_data = {
                        "filename": filename,
                        "timestamp": ts.timestamp(),
                        "size_bytes": length,
                        "size_kb": round(kb, 2),
                        "crc": f"0x{crc_calc:04X}",
                        "image_count": image_count,
                        "tx_id": tx_id,
                        "hop_count": hop_count,
                        "seq_num": seq_num
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
    parser.add_argument("--port", default="COM7",
                        help="Serial port (default: COM7)")
    parser.add_argument("--baud", type=int, default=1000000,
                        help="Baud rate (default: 1000000)")
    parser.add_argument("--outdir", default="received_images",
                        help="Output directory (default: received_images)")
    parser.add_argument("--mqtt-broker", default="10.225.150.248",
                        help="MQTT broker address")
    parser.add_argument("--mqtt-port", type=int, default=1883,
                        help="MQTT broker port")
    args = parser.parse_args()
    receive_images(args.port, args.baud, args.outdir, args.mqtt_broker, args.mqtt_port)

if __name__ == "__main__":
    main()