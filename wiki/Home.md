# Fomu UART-over-USB — Learning Wiki

This project (`~/projects/fomu-usb`) implements a **USB Full-Speed
CDC-ACM device** on a [Fomu](https://tomu.im/) board — an
iCE40 UP5K FPGA (5K LUTs, 48 MHz clock) — whose application is a
**UART echo server**: bytes arriving from the host over USB are rendered
onto a raw 115200 8N1 UART line, and bytes read back from that line are
echoed to the host.

The USB device core and the application layer are both written in
**Virdant HDL**: the whole core — the pad interface, the power-on
reset, the CRC LFSRs, the RX/TX PHYs, the packet decoder, EP0, the EP1
bulk protocol, the token router, the response arbiter and the
descriptor paging — lives in `src/usb.vir` (~3450 lines of Virdant
mods), and the application layer lives in `src/top.vir`, `src/echo.vir`
and `src/uart.vir`. Only what Virdant cannot express stays as small
vendored Verilog **ext bodies**: `ext/usb_uart.sv` (~460 lines — the
generic `usb_fifo16x8` sync FIFO plus the fixed-shape wrappers
`\usb::TxFifo` / `\usb::RxFifo` / `\usb::DescRom`), `ext/usb_pads.sv`
(the two `SB_IO` tristate pad cells — the design's only `inout`), and
`ext/echo_fifo.sv` (the echo queue's 16×8 storage). Nothing is hidden
behind an IP block or a vendor stack — every wire of the protocol is
in this repo.

## What you can learn here

- **How USB Full-Speed actually works at the wire level** — NRZI,
  bit stuffing, sync patterns, PIDs, CRC5/CRC16, tokens, data toggles,
  handshakes — see [usb-primer.md](usb-primer.md).
- **How a real device core is layered** — pad → PHY → packet → router →
  endpoint state machines → FIFOs — see [system-overview.md](system-overview.md)
  and the per-module pages.
- **How a host enumerates a device** — SET_ADDRESS, GET_DESCRIPTOR,
  SET_CONFIGURATION, CDC-ACM class requests — see [usb-ep0.md](usb-ep0.md).
- **How flow control is built out of NAKs** — backpressure from a
  16-byte FIFO all the way back to the host's retry loop — see
  [usb-serial.md](usb-serial.md) and [echo-fifo.md](echo-fifo.md).
- **A complete FPGA build flow for beginners** — RTL → iverilog
  simulation → yosys synthesis → nextpnr place & route → bitstream —
  see [build-flow.md](build-flow.md).
- **How to read a timing report** — what a "cone" is, why 48 MHz is
  tight at 4 cycles per USB bit, and why pipelining only goes so far —
  see [timing-cones.md](timing-cones.md).

## Guided reading order (for beginners)

Start here, then descend through the receive path, the control path,
the transmit/application path, and finally the flow:

1. **[system-overview.md](system-overview.md)** — the board, the pins,
   the module hierarchy, the full data path. The map of the territory.
2. **[usb-primer.md](usb-primer.md)** — the USB protocol background you
   need before any module page makes sense (bit order, NRZI, packets,
   endpoints, enumeration, CDC-ACM).
3. **[build-flow.md](build-flow.md)** — how to build and simulate the
   project yourself; run the testbenches early so the later pages have
   something to poke at.
4. Receive path, bottom-up:
   [usb-crc.md](usb-crc.md) → [usb-rx-phy.md](usb-rx-phy.md) →
   [usb-rx-packet.md](usb-rx-packet.md).
5. Control path: [usb-ep0.md](usb-ep0.md) — enumeration and the CDC
   requests.
6. Transmit path: [usb-tx-phy.md](usb-tx-phy.md) — how a response gets
   back onto the wire.
7. Bulk data path: [usb-fifo.md](usb-fifo.md) →
   [usb-serial.md](usb-serial.md).
8. Core top and board glue: [usb-uart.md](usb-uart.md) →
   [fomu-top.md](fomu-top.md) → [top-vir.md](top-vir.md).
9. Application: [echo-fifo.md](echo-fifo.md) →
   [echo-server.md](echo-server.md) → [uart-vir.md](uart-vir.md).
10. Verification: [testbenches.md](testbenches.md).

## Page index

### System

| Page | Contents |
|---|---|
| [Home.md](Home.md) | This page — project intro, reading order, index |
| [system-overview.md](system-overview.md) | Board, FPGA, pins, module hierarchy, clocking, attach, USB spec mapping |
| [usb-primer.md](usb-primer.md) | USB protocol primer taught through this project |
| [build-flow.md](build-flow.md) | Build / test / synth flow, toolchain, artifacts |
| [timing-cones.md](timing-cones.md) | Timing cones explained — why 48 MHz is tight, reading nextpnr's critical path, the Virdant enable-cone toy example |

### USB core

| Page | Contents |
|---|---|
| [usb-crc.md](usb-crc.md) | `usb::Crc5` / `usb::Crc16` serial LFSR CRCs |
| [usb-rx-phy.md](usb-rx-phy.md) | `usb::RxPhy` — pads → bit stream (NRZI, bit stuffing, sync/EOP/reset detect) |
| [usb-rx-packet.md](usb-rx-packet.md) | `usb::RxPacket` — bit stream → packet records |
| [usb-tx-phy.md](usb-tx-phy.md) | `usb::TxPhy` — packet records → pads |
| [usb-ep0.md](usb-ep0.md) | `endpoint::Ep0` — control transfers, descriptors, enumeration |
| [usb-serial.md](usb-serial.md) | `endpoint::Serial` — EP1 bulk protocol, toggles, ACK/NAK |
| [usb-fifo.md](usb-fifo.md) | `usb_fifo16x8` — the core's generic sync FIFO (16×8 flops / 64×8 BRAM) |
| [usb-uart.md](usb-uart.md) | `usb::UsbUart` (`src/usb.vir`) — core top: token router, response arbiter, descriptor paging, FIFOs |

### Application (the echo server)

| Page | Contents |
|---|---|
| [fomu-top.md](fomu-top.md) | Board glue: the `SB_IO` pad cells, the POR/attach delay, the LED mapping |
| [top-vir.md](top-vir.md) | `src/top.vir` — platform export, ext mod contract, loopback jumper |
| [echo-fifo.md](echo-fifo.md) | `EchoFifo` — 16×8 queue between USB core and UART sender |
| [echo-server.md](echo-server.md) | `EchoServer` — the Virdant echo data path and its flow control |
| [uart-vir.md](uart-vir.md) | `src/uart.vir` — 115200 8N1 sender/receiver at 48 MHz |

### Virdant HDL

The application-layer pages ([top-vir.md](top-vir.md),
[echo-server.md](echo-server.md), [uart-vir.md](uart-vir.md)) double as a
Virdant HDL tutorial: `mod`/`ext mod`, `reg ... on clock`, `wire`,
`enum`, pattern-match state machines, and how a platform export
(`export mod Top for ice40::Fomu`) binds an ext mod to board pins.

### Verification

| Page | Contents |
|---|---|
| [testbenches.md](testbenches.md) | The iverilog testbenches: `tb_crc`, `tb_usb_phy`, `tb_usb_serial`, the host BFM, and the `SB_IO`/`SB_RAM` cell sim models |

### Build

| Page | Contents |
|---|---|
| [build-flow.md](build-flow.md) | `vir build`, Makefile targets, yosys + nextpnr, artifacts, the no-upload policy |

## Where to go next

- New to USB? → [usb-primer.md](usb-primer.md), then
  [system-overview.md](system-overview.md) for the big picture.
- Want to *run* something? → [build-flow.md](build-flow.md), then
  `make test` (see [testbenches.md](testbenches.md)).
- Want to read code? → follow the guided order above; start at
  [usb-crc.md](usb-crc.md), the smallest module in the repo.
- Curious how the host ever notices the device? → the pull-up/attach
  story in [fomu-top.md](fomu-top.md).