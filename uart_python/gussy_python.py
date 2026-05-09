"""
nRF9151 DK Image Receiver - Live Viewer
Receives framed images over serial and displays them live using matplotlib.

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
import io
import threading
import queue
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")  # Change to "Qt5Agg" if TkAgg is not available
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from PIL import Image

ROUTING_MAX_HOPS = 8
MAGIC = b'\xAA\x55\xAA\x55'
HEADER_SIZE = 4 + 4 + 4 + 4 + 4 + 1 + (4 * ROUTING_MAX_HOPS) + (4 * ROUTING_MAX_HOPS)  # 85 bytes

# ── CRC ──────────────────────────────────────────────────────────────────────

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc

# ── Header parsing ────────────────────────────────────────────────────────────

def parse_header(buf: bytes) -> dict:
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
        'devices_visited': [hex(d) for d in devices_visited[:num_links + 1]],
        'per_link_delay':  per_link_delay[:num_links + 1],
    }

# ── JPEG validation ───────────────────────────────────────────────────────────

def validate_jpeg(data: bytes) -> tuple[bool, str]:
    """Returns (ok, reason). Checks SOI, EOI, and PIL decode."""
    if len(data) < 4:
        return False, "Too short"
    if data[:2] != b'\xff\xd8':
        return False, f"Missing SOI marker (got {data[:2].hex()})"
    if data[-2:] != b'\xff\xd9':
        return False, f"Missing EOI marker (got {data[-2:].hex()})"
    try:
        img = Image.open(io.BytesIO(data))
        img.verify()  # Checks internal JPEG structure
        return True, f"OK ({img.width}x{img.height} {img.mode})"
    except Exception as e:
        return False, f"PIL decode failed: {e}"

def decode_image(data: bytes):
    """Returns a PIL Image or None."""
    try:
        return Image.open(io.BytesIO(data))
    except Exception:
        return None

# ── Serial receiver thread ────────────────────────────────────────────────────

def receiver_thread(port: str, baudrate: int, image_queue: queue.Queue, stop_event: threading.Event):
    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port}: {e}")
        stop_event.set()
        return

    print(f"[RX] Opened {port} @ {baudrate} baud")
    buf = b''
    image_count = 0

    while not stop_event.is_set():
        chunk = ser.read(4096)
        if not chunk:
            continue
        buf += chunk

        while True:
            magic_pos = buf.find(MAGIC)

            if magic_pos == -1:
                buf = buf[-3:] if len(buf) > 3 else buf
                break

            if magic_pos > 0:
                buf = buf[magic_pos:]

            if len(buf) < HEADER_SIZE:
                break

            hdr = parse_header(buf)
            total_length = hdr['total_length']

            if total_length == 0 or total_length > 1024 * 1024:
                print(f"[WARN] Implausible payload length {total_length}, resyncing")
                buf = buf[4:]
                continue

            frame_total = HEADER_SIZE + total_length + 2
            if len(buf) < frame_total:
                break  # Wait for more bytes

            meta_bytes = buf[4:HEADER_SIZE]
            payload    = buf[HEADER_SIZE : HEADER_SIZE + total_length]
            crc_recv   = struct.unpack_from('<H', buf, HEADER_SIZE + total_length)[0]
            crc_calc   = crc16(meta_bytes + payload)

            if crc_recv != crc_calc:
                print(f"[WARN] CRC mismatch (recv=0x{crc_recv:04X}, calc=0x{crc_calc:04X}), resyncing")
                buf = buf[4:]
                continue

            image_count += 1
            jpeg_ok, jpeg_reason = validate_jpeg(payload)
            img = decode_image(payload) if jpeg_ok else None

            print(f"\n[IMAGE #{image_count}]  {len(payload)/1024:.1f} KB  CRC OK")
            print(f"  seq={hdr['seq_num']}  num_links={hdr['num_links']}")
            print(f"  timestamp_pt={hdr['timestamp_pt']}  offset_pt_to_ft={hdr['offset_pt_to_ft']} ms")
            print(f"  devices_visited={hdr['devices_visited']}")
            print(f"  per_link_delay={hdr['per_link_delay']} ms")
            print(f"  JPEG: {jpeg_reason}")

            image_queue.put({
                'count':         image_count,
                'hdr':           hdr,
                'img':           img,
                'jpeg_ok':       jpeg_ok,
                'jpeg_reason':   jpeg_reason,
                'size_bytes':    len(payload),
                'timestamp':     datetime.now(),
                'raw':           payload,
            })

            buf = buf[frame_total:]

    ser.close()
    print("[RX] Serial closed")

# ── Live display ──────────────────────────────────────────────────────────────

def run_display(image_queue: queue.Queue, save_images: bool, stop_event: threading.Event):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("nRF9151 Live Image Viewer", fontsize=13, fontweight='bold')

    ax_img   = axes[0]
    ax_stats = axes[1]

    ax_img.set_title("Waiting for first image...")
    ax_img.axis('off')
    ax_stats.axis('off')

    plt.tight_layout()
    plt.ion()
    plt.show()

    history = []  # List of (seq, delay_sum) for latency plot

    while not stop_event.is_set():
        plt.pause(0.1)

        try:
            frame = image_queue.get_nowait()
        except queue.Empty:
            continue

        hdr     = frame['hdr']
        img     = frame['img']
        count   = frame['count']
        ok      = frame['jpeg_ok']
        reason  = frame['jpeg_reason']
        delays  = hdr['per_link_delay']
        total_delay = sum(delays) if delays else 0

        history.append((hdr['seq_num'], total_delay))

        # ── Left panel: image ────────────────────────────────────────────
        ax_img.cla()
        if img is not None:
            ax_img.imshow(img)
            border_color = 'limegreen' if ok else 'red'
            for spine in ax_img.spines.values():
                spine.set_edgecolor(border_color)
                spine.set_linewidth(3)
        else:
            ax_img.set_facecolor('#2b2b2b')
            ax_img.text(0.5, 0.5, f"JPEG INVALID\n{reason}",
                        ha='center', va='center', color='red',
                        fontsize=11, transform=ax_img.transAxes)

        status_str = "✓ VALID" if ok else "✗ INVALID"
        ax_img.set_title(
            f"Image #{count}  |  seq={hdr['seq_num']}  |  {frame['size_bytes']/1024:.1f} KB  |  {status_str}",
            color='green' if ok else 'red',
            fontsize=10
        )
        ax_img.axis('off')

        # ── Right panel: stats ───────────────────────────────────────────
        ax_stats.cla()
        ax_stats.axis('off')

        lines = [
            ("Received",        f"#{count}"),
            ("Seq num",         str(hdr['seq_num'])),
            ("Size",            f"{frame['size_bytes']} B  ({frame['size_bytes']/1024:.1f} KB)"),
            ("JPEG",            reason),
            ("Timestamp PT",    f"{hdr['timestamp_pt']} ms"),
            ("PT→FT offset",    f"{hdr['offset_pt_to_ft']} ms"),
            ("Num links",       str(hdr['num_links'])),
            ("Total delay",     f"{total_delay} ms"),
            ("Received at",     frame['timestamp'].strftime('%H:%M:%S.%f')[:-3]),
        ]

        for i, (label, val) in enumerate(lines):
            y = 0.95 - i * 0.09
            ax_stats.text(0.02, y, f"{label}:", fontsize=9,
                          color='gray', transform=ax_stats.transAxes, va='top')
            ax_stats.text(0.42, y, val, fontsize=9,
                          color='white', transform=ax_stats.transAxes, va='top',
                          fontweight='bold')

        # Per-link delays
        if delays:
            y = 0.95 - len(lines) * 0.09 - 0.03
            ax_stats.text(0.02, y, "Per-link delays:", fontsize=9,
                          color='gray', transform=ax_stats.transAxes, va='top')
            devices = hdr['devices_visited']
            for j, (dev, d) in enumerate(zip(devices, delays)):
                y -= 0.07
                ax_stats.text(0.04, y, f"  {dev}  →  {d} ms", fontsize=8,
                              color='#aaddff', transform=ax_stats.transAxes, va='top')

        ax_stats.set_facecolor('#1e1e1e')
        ax_stats.set_title("Frame Metadata", fontsize=10, color='white')
        fig.patch.set_facecolor('#1e1e1e')

        # Save raw JPEG if requested
        if save_images and img is not None:
            fname = f"img_{count:04d}_seq{hdr['seq_num']}.jpg"
            with open(fname, 'wb') as f:
                f.write(frame['raw'])
            print(f"[SAVE] {fname}")

        plt.tight_layout()
        fig.canvas.draw()
        fig.canvas.flush_events()

    plt.close(fig)

# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Live JPEG viewer for nRF9151 image stream")
    parser.add_argument("--port",  default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud",  type=int, default=1000000, help="Baud rate")
    parser.add_argument("--save",  action="store_true", help="Save each valid JPEG to disk")
    args = parser.parse_args()

    img_queue  = queue.Queue()
    stop_event = threading.Event()

    rx = threading.Thread(
        target=receiver_thread,
        args=(args.port, args.baud, img_queue, stop_event),
        daemon=True
    )
    rx.start()

    try:
        run_display(img_queue, args.save, stop_event)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        rx.join(timeout=2)
        print("Done.")

if __name__ == "__main__":
    main()