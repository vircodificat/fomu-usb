# Fomu UART-over-USB PoC

A **hand-written USB Full-Speed CDC-ACM device** on a
[Fomu](https://tomu.im/) iCE40 UP5K FPGA (5280 LUTs, 48 MHz clock),
implemented as a **UART echo server** — bytes arriving from the host
over USB are rendered onto a raw 115200 8N1 UART line, and bytes read
back from that line are echoed to the host.

The entire USB device core is written in **hand-written Verilog-2005**
(~2300 lines, eight modules); the application layer (UART sender,
UART receiver, echo server) is written in **Virdant HDL**. Every wire
of the protocol is in this repo — nothing is hidden behind an IP block
or a vendor stack.

## Quick start

```sh
# Prerequisites: iverilog, yosys, nextpnr-ice40, Virdant (vir)
make tools

# Build, test, and synthesize
make all

# Or step by step:
make build   # Virdant compilation + platform wrapper check
make test    # 3 iverilog testbenches (CRC, PHY, full serial echo)
make synth   # yosys synth_ice40 + nextpnr place & route @ 48 MHz
```

## Repository structure

```
src/              Virdant HDL sources (top, echo, uart)
├── top.vir       Platform export for ice40::Fomu + ext mod contract
├── echo.vir      Echo server: bridges USB byte stream ↔ raw UART
├── usb.vir       USB core in Virdant (PHY, packet, EP0, serial, etc.)
└── uart.vir      115200 8N1 UART sender/receiver @ 48 MHz

ext/              Vendored plain-Verilog modules (hand-written)
├── usb_uart.sv   USB device core (PHY, packet, token router, EP0, etc.)
├── usb_pads.sv   SB_IO tristate D+/D− pad cells (the only `inout`)
└── echo_fifo.sv  16×8 circular FIFO between USB core and UART sender

tb/               Iverilog testbenches and host BFM
├── tb_crc.v      CRC5/CRC16 known-answer vector tests
├── tb_usb_phy.v  PHY-level waveform tests
├── tb_usb_serial.v  Full enumeration + echo loopback simulation
├── usb_host_bfm.v  Host behavioural model (included by TBs)
├── sb_io_sim.v   SB_IO simulation model for iverilog
└── sb_ram_sim.v  SB_RAM40_4K simulation model for iverilog

wiki/             Project wiki (rendered to build/html/ via `make html`)
├── Home.md       Guided reading order and page index
├── system-overview.md  Board, pins, module hierarchy, data path
├── usb-primer.md       USB protocol primer taught through this project
├── build-flow.md       Build/test/synth flow walkthrough
├── testbenches.md      What each simulation exercises
├── timing-cones.md     How to read a timing report from nextpnr
├── usb-crc.md → usb-tx-phy.md  Per-module deep dives
├── fomu-top.md → top-vir.md    Pad glue, POR, loopback, ext mod contract
├── echo-fifo.md → uart-vir.md  Application layer deep dives
└── style.css    Dark stylesheet for wiki HTML

build/            Build artifacts (vir build, vvp images, bitstream)
fomu.pcf          Pin constraints (clock F4, usb_dp A1, usb_dn A2, …)
Makefile          Build / test / synth targets
Virdant.toml      Virdant project configuration
```

## Architecture in brief

```
module Top
├── \top::FomuTop (ext/usb_uart.sv)
│   └── usb_uart ── the USB device core
│       ├── usb_rx_phy       pads → bit stream (NRZI, bit stuffing, sync)
│       ├── usb_rx_packet    bit stream → packet records (PID, CRC)
│       ├── usb_ep0          control transfers, descriptors, enumeration
│       ├── usb_serial       EP1 bulk protocol, toggles, ACK/NAK
│       ├── usb_tx_phy       packet records → pads (CRC, stuffing, NRZI)
│       └── usb_fifo16x8 ×2  rx FIFO, tx FIFO
├── EchoServer (src/echo.vir)
│   ├── EchoFifo             16×8 queue host → UART
│   ├── UartSender           bytes → 115200 8N1 tx wire
│   ├── UartReceiver         115200 8N1 rx wire → bytes
│   └── hold slot            one-byte register, receiver → USB tx FIFO
└── loopback wire             uart_tx_line → uart_rx_line (echo)
```

- **One clock domain**: 48 MHz, 4 cycles per USB Full-Speed bit.
- **Attach via pull-up**: a ~175 ms power-on reset releases the D+ 1.5 kΩ
  pull-up (pin A4), which is how the host discovers the device.
- **No bridging**: D+ (A1) and D− (A2) are wired straight to FPGA pads
  through iCE40 `SB_IO` tristate cells — the entire USB PHY is soft logic.
- **Flow control end to end**: nothing drops a byte; the echo FIFO's
  `room` signal propagates back as NAKs.
- **Scope**: Full-Speed only, single configuration, single interface,
  CDC-ACM class. No suspend/resume, no high-speed, no isochronous
  endpoints. See the design plan for the full scope rationale.

## Learning from this project

The [wiki](wiki/Home.md) is written as a guided tour — it's meant for
beginners who want to understand how USB works at the wire level and
how an FPGA build flow looks end-to-end:

1. [**system-overview.md**](wiki/system-overview.md) — the map of the
   whole system: board, pins, module hierarchy, data path, clocking.
2. [**usb-primer.md**](wiki/usb-primer.md) — the USB protocol background
   you need (bit order, NRZI, packets, endpoints, enumeration, CDC-ACM).
3. [**build-flow.md**](wiki/build-flow.md) — how to build and simulate
   the project; what every Makefile target and build artifact is.

Then descend module by module: CRC → RX PHY → RX packet → EP0 →
TX PHY → FIFOs → serial → core top → application.

## Pin map (from `fomu.pcf`)

| Pin | Signal | Role |
|-----|--------|------|
| F4  | `clock` | 48 MHz board clock |
| A1  | `usb_dp` | USB D+ (inout via SB_IO) |
| A2  | `usb_dn` | USB D− (inout via SB_IO) |
| A4  | `usb_dp_pu` | D+ 1.5 kΩ pull-up enable (the attach signal) |
| B5  | `led_red` | Activity/heartbeat LED (active low) |
| A5  | `led_green` | USB TX activity LED (active low) |
| C5  | `led_blue` | Unused, tied off (active low) |

## Testbenches

| Target | Exercises |
|--------|-----------|
| `make test-crc` | CRC5/CRC16 serial LFSR known-answer vectors |
| `make test-usb-phy` | Raw PHY waveforms: sync, EOP, bit stuffing, NRZI, bus reset |
| `make test-usb-serial` | Full-device simulation: enumeration + CDC handshake + loopback echo |

## Hard rules (from the design)

1. **No new dependencies.** Every line is hand-written RTL in this repo.
2. **All RTL is vendored plain Verilog in `ext/`** (the established
   Virdant ext-mod pattern).
3. **Simulation is iverilog + Verilog testbenches** — no cocotb, no
   Python in the flow.
4. **Acceptance = simulation + synthesis only.** All iverilog TBs pass;
   yosys `synth_ice40` and nextpnr place the design cleanly at 48 MHz.
5. **Never run `vir upload`, `dfu-util`, `iceprog` or any flashing
   command.** Flashing requires the operator's physical intervention.
   The Makefile has no upload target.
6. **Virdant is read-only reference** — the PoC top is an ext mod
   implemented in Verilog. No Virdant compiler changes.

## References

- [USB 2.0 specification](https://www.usb.org/document-library/usb-20-specification)
  — chapters 7 (physical), 8 (protocol), 9 (enumeration) are the scope.
- [CDC-ACM specification](https://www.usb.org/document-library/usb-device-class-definitions)
  — USB Communications Device Class.
- [valentyusb](https://github.com/gregoryd/valentyusb) — design reference
  (BSD-3-Clause); reading material only, no code reused.
- [Fomu workshop](https://tomu.im/) — board documentation and resources.