# DECT NR+ Multi-Hop Mesh for Sustained Image Transmission

Firmware and supporting components for a DECT NR+ multi-hop mesh network that streams JPEG images from a camera node, across one or more relay hops, to a sink connected to a cloud backend and live dashboard.

This repository contains the Zephyr / nRF Connect SDK application that runs on Nordic Semiconductor **nRF9151-DK** boards. It is the firmware half of a master's thesis project carried out at the University of Agder in cooperation with VIMMS AS (Mandal, Norway).

---

## What the system does

A Raspberry Pi captures JPEG images and pushes them, byte-by-byte, into the mesh through an SPI-attached nRF9151-DK. The mesh forwards each image hop-by-hop over the DECT NR+ radio link until it reaches a sink node, which streams the image plus per-link metadata (delays, RSSI, route trace, sequence number) out over UART to a Linux server. A Python MQTT bridge parses the wire format and publishes to MongoDB (images + metadata) and InfluxDB (metrics). A Blazor WebAssembly dashboard renders live topology, per-link delay charts, end-to-end latency, throughput, and image previews.

```
┌──────────────┐  SPI   ┌──────────────┐  DECT NR+  ┌──────────────┐ DECT NR+ ┌──────────────┐  UART  ┌──────────────────┐
│ Raspberry Pi │ ─────▶ │   Edge PT    │ ─────────▶ │ Relay FTPT(s)│ ───────▶ │   Sink FT    │ ─────▶ │  MQTT bridge +   │
│  (camera)    │        │  nRF9151-DK  │            │ nRF9151-DK x2│          │  nRF9151-DK  │        │  MongoDB / InfluxDB │
└──────────────┘        └──────────────┘            └──────────────┘          └──────────────┘        └──────────┬───────┘
                                                                                                                 │
                                                                                                                 ▼
                                                                                                       ┌──────────────────┐
                                                                                                       │  Blazor WASM     │
                                                                                                       │  dashboard       │
                                                                                                       └──────────────────┘
```

A relay is built from **two boards** wired UART-to-UART: one running in PT mode (RX from upstream) and one in FT mode (TX to downstream). The pair exchanges sibling IDs over their UART link at startup so neither tries to associate with its own partner (which would form a routing loop).

---

## Repository contents

This repo is the **firmware application** — everything that runs on the nRF9151-DK boards. The backend bridge, database stack, and dashboard live in separate repositories (see *Related repositories* below).

| File / module | Role |
|---|---|
| `main.c` | Application entry point. Selects device role at compile time, sets up DECT L2 callbacks, runs the edge-PT, relay, or sink FT main loop. |
| `prj.conf` | Zephyr Kconfig fragment: modem library, DECT driver, IPv6 over DECT, SPI, UART (async), settings storage, logging. |
| `spi.c` / `spi.h` | SPI slave driver for the edge PT. Receives JPEG bytes from the Raspberry Pi using a ready-pin handshake for chunk-level flow control. Detects JPEG SOI/EOI markers to frame whole images. |
| `uart.c` / `uart.h` | UART image transport. On TX side (sink FT, relay FT) emits framed `[MAGIC][len][payload][meta][CRC16]` streams; on RX side (relay PT) reassembles them. Also implements the relay sibling-ID handshake. |
| `sync.c` / `sync.h` | NTP-style symmetric clock sync between PT and parent FT over a dedicated UDP socket on the DECT mesh, used to compute per-hop delays in a common time base. |
| `dect_net*.{c,h}`, `dect_mdm*.{c,h}`, `dect_utils.h` | Vendored Nordic L2 / modem-control layer for DECT NR+. Provides settings management, scan/association handling, cluster/network beacon control, and the HAL the application uses. |
| `ts_*.pdf` | ETSI DECT-2020 NR specifications (Parts 1–5 and the IPv6 Profile), kept for offline reference. |

---

## Hardware

* Nordic Semiconductor **nRF9151-DK** boards (one per role; a relay needs two).
* **Raspberry Pi** (tested with Camera Module 3) as the camera node, SPI-connected to the edge PT.
* A USB-UART bridge or direct USB cable from the sink FT to the Linux server.
* Between the two boards of a relay: a UART cross-connection (TX↔RX, RX↔TX, common GND).

The edge PT uses **UART0/VCOM0** for both logs and image data (deferred logging mode keeps log lines from interleaving with binary payload). UART1 is used for the relay sibling handshake.

---

## Build & flash

### Toolchain

* nRF Connect SDK **v3.3.0**
* Zephyr RTOS bundled with NCS v3.3.0
* Sysbuild + Ninja (default for NCS)
* VS Code with the nRF Connect extension, or `west` from the command line

> **Note:** the NCS install path must contain no spaces. The vendored DECT L2 / modem-control sources expect the v3.3.0 modem library API.

### Device roles

Each board's role is selected by a Kconfig symbol at build time:

| Role | Kconfig | Device type | Notes |
|---|---|---|---|
| Edge PT | `CONFIG_DECT_EDGE_PT=y` | PT | Pulls images from the Raspberry Pi over SPI, transmits them to its parent. |
| Relay PT | `CONFIG_DECT_RELAY_PT=y` | PT | RX board of a relay pair. Receives from upstream over DECT, forwards over UART to the relay FT. |
| Relay FT | `CONFIG_DECT_RELAY_FT=y` | FT | TX board of a relay pair. Receives over UART from the relay PT, transmits onward over DECT. |
| Sink FT | (no role symbol — default) | FT | Network root. Receives the image stream and streams it out over UART to the host PC. |

The application also relies on per-device **transmitter IDs** and an **ID-based scan filter** to enforce topology in the lab (see *Topology enforcement* below).

### Building

From the project root (or the NCS workspace containing it):

```bash
# Edge PT
west build -b nrf9151dk/nrf9151/ns -p always -- -DCONFIG_DECT_EDGE_PT=y

# Relay PT (board A of a relay)
west build -b nrf9151dk/nrf9151/ns -p always -- -DCONFIG_DECT_RELAY_PT=y

# Relay FT (board B of a relay)
west build -b nrf9151dk/nrf9151/ns -p always -- -DCONFIG_DECT_RELAY_FT=y

# Sink FT
west build -b nrf9151dk/nrf9151/ns -p always
```

Then flash:

```bash
west flash
```

---

## How the firmware works

### Edge PT (`main.c` + `spi.c`)

1. SPI slave driver arms, raises a GPIO ready pin (P0.06), and waits for a chunk from the Pi.
2. On each chunk it pulses the ready pin LOW immediately so the Pi backs off while the nRF processes.
3. SOI (`FF D8`) marks the start of a new image; EOI (`FF D9`) — including across chunk boundaries — marks the end.
4. When an image is complete, an edge TX thread polls the buffer and calls `tx_img_data()` to send it to the parent FT. Sending blocks on L2 flow control, so the send cadence self-paces to channel capacity.

### Relay (`main.c` relay branches + `uart.c`)

A relay is a PT+FT pair on the same physical site. At startup:

1. The relay FT broadcasts its long RD ID and a local timestamp over UART.
2. The relay PT receives them and stores them as `sibling_ft_long_rd_id` and `sibling_ft_offset`.
3. The relay PT then skips any scan result whose transmitter ID equals its sibling, breaking the loop.
4. In normal operation: the relay PT receives an image over DECT, hands the payload + metadata to the relay FT over UART, and the relay FT retransmits over DECT to its own children's parent (i.e. the next hop toward the sink).

Per-hop delay is computed in the PT-to-FT clock domain established by the SYNC handshake, then accumulated into `per_link_delay[]` as the image traverses the mesh.

### Sink FT (`main.c` FT branch + `uart.c`)

The sink runs the network beacon, accepts associations, receives the IPv6/DECT image stream, and emits the complete reassembled image plus full metadata over its UART (VCOM0) to the host.

### UART image wire format (97-byte header + payload + CRC)

The sink and the relay FT emit frames using:

```
[MAGIC      :  8]   0xAA 0x55 0xBB 0x44 (repeated twice)
[total_len  :  4]   little-endian
[seq_num    :  4]
[timestamp_pt   : 4]
[offset_pt_to_ft: 4]
[num_links  :  1]
[devices_visited : 4 * ROUTING_MAX_HOPS]
[per_link_delay  : 4 * ROUTING_MAX_HOPS]
[per_link_rssi   : 1 * ROUTING_MAX_HOPS]
[payload    :  N]
[CRC16      :  2]
```

`STREAM_HEADER_SIZE = 97` with the default `ROUTING_MAX_HOPS = 8`. CRC-16 (IBM/Modbus polynomial, 0xA001) is computed over `total_len`, payload, and metadata. The 8-byte magic plus CRC is necessary because JPEG payload is arbitrary binary — short magic sequences alone are not enough to avoid false syncs.

### Time sync (`sync.c`)

A small NTP-style four-timestamp exchange over UDP on the DECT interface:

* PT sends `T0`.
* FT records `T1` (RX) and `T2` (TX) and echoes them back.
* PT records `T3` and computes `offset = ((T1 - T0) + (T2 - T3)) / 2`.

This offset is stored as `offset_pt_to_ft` and travels with each image, so the sink-side timeline can be reconstructed in a common clock.

### Topology enforcement

Because the lab is small and beacons are reachable everywhere, topology is enforced primarily by **ID-based scan filtering** hardcoded per device, not by route cost. Route cost is used as a secondary criterion among already-filtered candidates. Multi-hop filter sets are configured manually per board; relay pairs additionally exchange sibling IDs over UART at startup to prevent routing loops. See the `NET_EVENT_DECT_SCAN_RESULT` handler in `main.c`.

---

## Configuration

Compile-time:

* `CONFIG_DECT_DEFAULT_NW_ID` — DECT network ID (default `0x12345678`).
* `CONFIG_DECT_TRANSMITTER_ID` — short transmitter ID. Each device must be unique on the same network.
* `CONFIG_DECT_EDGE_PT` / `CONFIG_DECT_RELAY_PT` / `CONFIG_DECT_RELAY_FT` — role selector.

Per-device hardcoded constants in `main.c`:

* `DECT_SINK_LONG_RD_ID` — long RD ID of the sink (each PT uses this to construct routes).
* Per-device long-RD-ID scan filter sets (e.g. `0xAAAAAAAA`, `0xCCCCCCCC`, `0xEEEEEEEE`).

These are intentionally explicit to make experimental scenarios reproducible.

---

## Experimental setup

The firmware is designed to support a parameter sweep across four axes:

| Axis | Values |
|---|---|
| Hop count | 1 (PT → sink), 2 (PT → relay → sink), 3, … |
| Inter-node distance | varied per scenario |
| Transmit power | configurable via `CONFIG_DECT_TX_MAX_POWER_DBM` (and the `dect_settings_common_tx` struct) |
| Environment | indoor / outdoor |

The dashboard CSV export produces per-image rows with sequence number, transmitter ID, device timestamp, received timestamp, size, hop count, end-to-end delay, per-link delays, and per-link RSSI — plus a summary footer (duration, total images, total bytes, throughput, average RSSI).

---

## Related repositories

| Component | Role |
|---|---|
| `dect-mqtt-bridge` (Python) | Reads the framed UART stream from the sink, parses metadata, publishes images to MongoDB and metrics to InfluxDB via MQTT. |
| `DECT_Api` (ASP.NET Core) | REST API over MongoDB / InfluxDB for the dashboard. |
| `DECT_Web` (Blazor WASM) | Live dashboard: topology, per-link delays, throughput, RSSI, image previews, experiment timer, CSV/ZIP export. |
| `DECT_Shared` | Shared DTOs between API and dashboard. |

The dashboard files `Home.razor` and `MetricTile.razor` are included here as references for the experiment-control UI but are not built as part of the firmware.

---

## Status

Working subsystems:

* SPI chunk-ack flow control between Raspberry Pi and edge PT.
* UART framing with magic + length + metadata header + CRC-16.
* End-to-end sequence numbering and per-link delay instrumentation.
* NTP-style symmetric SYNC for clock offset between PT and parent FT.
* RSSI capture at each receiving hop.
* Sibling handshake between relay PT and relay FT.
* MQTT bridge, MongoDB/InfluxDB persistence, Blazor dashboard with live topology rendering and CSV export.

Known limitations:

* `pending_sync_child_id` is a single static variable; if two children associate with the same FT simultaneously, one SYNC handshake can be missed. Fix in progress.
* PT auto-association occasionally fails with `MAC_STATUS_NO_RESOURCES` if a relay FT has stale slots; a 10-second reconnect backoff mitigates this in practice.
* Maximum DECT NR+ MAC-layer datagram size is ~1232 bytes; chunk sizes are set conservatively below this.

---

## Acknowledgements

* **VIMMS AS** (Mandal, Norway) — industry partner.
* **University of Agder** — host institution.
* **Nordic Semiconductor** — hardware, nRF Connect SDK, and the DECT NR+ L2 / modem-control sources vendored here.

DECT-2020 NR is specified by ETSI in the TS 103 636 series and TS 103 874-3 (IPv6 Profile); the relevant PDFs are included in this repository for offline reference.

---

## License

The DECT NR+ L2 and modem-control sources (`dect_net*.{c,h}`, `dect_mdm*.{c,h}`, `dect_utils.h`) are © 2025 Nordic Semiconductor ASA and distributed under `LicenseRef-Nordic-5-Clause`. Application code (`main.c`, `spi.c`, `uart.c`, `sync.c`, and their headers) is part of the master's thesis project — see `LICENSE` in the repository root for terms.
