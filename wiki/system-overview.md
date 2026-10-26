# System Overview — Fomu UART-over-USB

This page is the map of the whole project: the board, the pins, the
module hierarchy, the end-to-end data path, the clocking, and how each
piece maps onto the USB specification. Protocol background lives in
[usb-primer.md](usb-primer.md); how to build it lives in
[build-flow.md](build-flow.md).

## The board and the FPGA

The [Fomu](https://tomu.im/) is a tiny FPGA board in a USB plug shell:

- **FPGA**: iCE40 UP5K — 5280 LUTs, no hard USB peripheral. Every wire
  of the USB protocol is implemented in fabric, in this repo.
- **Clock**: a single 48 MHz oscillator on pin **F4** (`fomu.pcf`,
  `set_frequency clock 48.0`). No PLL, no clock dividers.
- **USB connector**: wired straight to two FPGA pins — this is not a
  USB PHY, just pads.

48 MHz is a convenient clock for USB **Full-Speed** (12 Mbit/s): one
wire bit takes exactly **4 clock cycles** (see the `usb::UsbUart`
header in `src/usb.vir`). The whole design is one synchronous domain —
no PLL, no CDC, no multi-clock headaches.

## Pins

From `fomu.pcf` (cross-checked against the Fomu PVT board file):

| Pin | Signal | Role |
|---|---|---|
| `F4` | `clock` | 48 MHz board clock |
| `A1` | `usb_dp` | USB D+ (inout, via `SB_IO` cells) |
| `A2` | `usb_dn` | USB D− (inout, via `SB_IO` cells) |
| `A4` | `usb_dp_pu` | D+ 1.5 kΩ pull-up enable — **this is how the device "attaches"** |
| `B5` / `A5` / `C5` | `led_red/green/blue` | LEDs, active low; red = receiving (`rxing`), green = sending (`txing`), blue tied off |

Because the iCE40 has no tristate fabric, D+/D− go through explicit
`SB_IO` tristate pad cells (`PIN_TYPE 6'b101001`) inside the
`usb::UsbPads` ext body (`ext/usb_pads.sv`) — one shared
`OUTPUT_ENABLE` for the pair, so the core never drives one line alone.

## Module hierarchy

The whole design hangs off an `export mod Top for ice40::Fomu`
platform export in `src/top.vir`. It instantiates four pieces:

```
module Top (build/top.sv, emitted from src/top.vir's platform export)
├── por: usb::UsbPor (src/usb.vir)          ← ~175 ms power-on count
│       por_done → usb_dp_pu pad (attach) and → UsbController reset
├── actual_top: usb::UsbController (src/usb.vir)
│   └── core: usb::UsbUart (src/usb.vir)    ← the USB core
│       ├── rxphy: usb::RxPhy               pads → bit stream
│       ├── rxpkt: usb::RxPacket            bit stream → packet records
│       │     ├── crc5: usb::Crc5           token CRC (x⁵+x²+1)
│       │     └── crc16: usb::Crc16         data CRC (x¹⁶+x¹⁵+x²+1)
│       ├── ep0: endpoint::Ep0                   EP0 control SM + CDC requests
│       ├── serial: endpoint::Serial             EP1 bulk protocol, toggles, ACK/NAK
│       ├── txphy: usb::TxPhy               packet records → pads
│       │     └── crc16: usb::Crc16         TX data CRC generator
│       ├── rxfifo: usb::RxFifo             ext body: usb_fifo16x8 64×8, BRAM
│       ├── txfifo: usb::TxFifo             ext body: usb_fifo16x8 16×8
│       ├── descrom: usb::DescRom           ext body: 256×8 descriptor ROM
│       ├── SnapRam / RxRam                 bulk-IN snapshot / bulk-OUT payload
│       │                                   RAMs (Virdant mods over
│       │                                   ice40::SbRam40_4k cells)
│       └── (token router + response        routing & mux logic in
│           arbiter + rx pusher)            usb::UsbUart
├── pads: usb::UsbPads (ext/usb_pads.sv)    ← two SB_IO tristate cells,
│       dp_tx/dn_tx/output_enable in, dp_rx/dn_rx out (the design's
│       only bidirectional nets live inside this ext body)
└── echo: echo::EchoServer (src/echo.vir)
    ├── fifo: echo::EchoFifo (ext/echo_fifo.sv)   16×8 queue host→UART
    ├── sender: uart::UartSender (src/uart.vir)   bytes → 115200 8N1 txd
    ├── receiver: uart::UartReceiver (src/uart.vir)  8N1 rxd → bytes
    └── hold slot (reg hold_data)                 receiver → USB tx FIFO
```

The loopback that makes the device an echo server is at the top level:
`echo.txd` is registered into `uart_rx_reg`, which feeds `echo.rxd`
one clock later (`src/top.vir`).

Per-module deep dives: [usb-uart.md](usb-uart.md) (core top),
[fomu-top.md](fomu-top.md) (board glue), [top-vir.md](top-vir.md),
[echo-server.md](echo-server.md).

## The end-to-end data path

Host → device (the "render" direction):

```
host USB pad
  → usb::RxPhy      (NRZI decode, unstuff, sync/EOP detect)
  → usb::RxPacket   (PID, addr/ep filter, CRC check, payload buffer)
  → usb::UsbUart    (token router: EP1 OUT → rx FIFO)
  → rx FIFO         (usb::RxFifo: usb_fifo16x8 64×8, BRAM)
  → EchoFifo        (echo::EchoFifo, ext/echo_fifo.sv — `room` backpressure)
  → UartSender      (uart::UartSender, 115200 8N1 frame)
  → uart_tx_line    (the raw UART tx wire)
  → [loopback]      (reg uart_rx_reg: rxd := txd one clk later, src/top.vir)
```

Device → host (the "echo" direction):

```
uart_rx_line
  → UartReceiver    (uart::UartReceiver: start-bit detect, mid-bit sampling)
  → hold slot       (reg hold_data, src/echo.vir)
  → tx FIFO         (usb::TxFifo, pushed while usb_tx_ready)
  → endpoint::Serial     (DATA0/1 toggle, host ACK handshake)
  → usb::TxPhy      (CRC16, stuffing, NRZI, sync, EOP)
  → usb::UsbUart    (response arbiter)
  → SB_IO pads      (usb::UsbPads, ext/usb_pads.sv)
  → host
```

Nothing in this chain can drop a byte: every hop has explicit flow
control (see [echo-server.md](echo-server.md) and
[usb-serial.md](usb-serial.md)).

## Clocking

One domain: **48 MHz** (see the `usb::UsbUart` header in
`src/usb.vir`).

- USB FS bit period = 4 clock cycles; the rx PHY recovers the bit
  phase by oversampling and re-aligning on every wire transition
  ([usb-rx-phy.md](usb-rx-phy.md)).
- UART side: 48_000_000 / 115200 ≈ 416.67, so one UART bit is 417
  cycles (−0.08 % error, well inside tolerance) — see the `src/uart.vir`
  header.
- All resets are synchronous, active high: `usb::UsbPor`'s `por_done`
  becomes the core reset (`por_rst`/`core_rst` in `usb::UsbController`)
  and is fanned out by `usb::UsbUart` to every engine and FIFO; the
  bus-reset strobe from `usb::RxPhy` additionally flushes the USB
  state.

## Power-on reset and attach

The POR lives in `usb::UsbPor` (`src/usb.vir`) and is wired up at the
top of `src/top.vir`:

1. On power-up a 24-bit counter runs for **8_400_000 cycles (~175 ms
   @ 48 MHz)**; during that time `por_done` is low, so `usb_dp_pu`
   (pin A4) is **low** — the 1.5 kΩ D+ pull-up is disconnected and the
   host sees nothing on the bus — and the core is held in reset (via
   `por_rst`/`core_rst` in `usb::UsbController`).
2. When the count completes, `por_done` sticks high: the core reset is
   released **and** `usb_dp_pu` goes high (it is wired straight to the
   POR output). The pull-up drags D+ high while D− floats low — the
   host sees a **full-speed attach** (see
   [usb-primer.md](usb-primer.md) § electrical) and starts enumeration
   ([usb-ep0.md](usb-ep0.md)).
3. The LEDs show bus activity, not a heartbeat: red lights while the
   core is receiving (`rxing`, active low), green while it is sending
   (`txing`), and blue is tied off (`src/top.vir`). Both signals are
   stretched ~175 ms past the last byte by 23-bit counters in
   `usb::UsbUart`, so sustained traffic reads as a solid light. If you
   see no `/dev/ttyUSB*`-style device, check the pull-up wiring first —
   the platform default for `usb_dp_pu` is low (= disconnected), so a
   wrapper that leaves it unwired produces an image no host can ever
   see (see the comment on `usb_dp_pu` in `src/top.vir`).

## Mapping onto the USB 2.0 specification

| This project | USB 2.0 spec | Notes |
|---|---|---|
| NRZI, bit stuffing, sync/EOP | §7.1 | [usb-rx-phy.md](usb-rx-phy.md), [usb-tx-phy.md](usb-tx-phy.md) |
| Full-speed electricals, pull-up | §7.3.2, §7.1.5 | 12 Mbit/s, 1.5 kΩ on D+ |
| CRC5/CRC16 | §8.3.5 | [usb-crc.md](usb-crc.md) |
| PIDs, packet formats | §8.3 | token/data/handshake, LSB first |
| Endpoint zero, control transfers | §8.5.5 | [usb-ep0.md](usb-ep0.md) |
| Enumeration: SET_ADDRESS etc. | §9.1-§9.4 | address latched at status stage |
| Data toggles on bulk | §8.5-§8.6 | [usb-serial.md](usb-serial.md) |
| CDC-ACM class (descriptors, line coding) | USB CDC 1.2 / PSTN §6 | EP0 requests, notification EP2 in descriptors only |

Scope limits (intentional — see the header of `ext/usb_uart.sv` and
the `usb::UsbUart` banner in `src/usb.vir`): FS only, one
configuration, EP0 control + EP1 IN/OUT bulk, no suspend/resume, no SOF
bookkeeping (SOF tokens decode but are ignored); EP2 IN exists in
the descriptors (Linux's `cdc_acm` requires it) but is never armed —
every token to EP2 is NAKed.

## Key takeaways

- One 48 MHz clock, one synchronous domain, 4 cycles per USB bit —
  no PLL, no CDC anywhere in the project.
- D+/D− are plain FPGA pads behind `SB_IO` tristate cells; the board
  has no USB PHY — the entire device core is fabric logic.
- Attach = asserting the 1.5 kΩ D+ pull-up (pin A4, driven by
  `usb::UsbPor`'s `por_done`) after a ~175 ms power-on reset; getting
  that wiring wrong makes the device invisible.
- The system is a strict layering: pads (`usb::UsbPads`) → rx PHY →
  packet → router → endpoint state machines → FIFOs → Virdant echo
  application → UART → and back. Every layer has its own wiki page.
- The red/green LEDs map the core's `rxing`/`txing` activity (stretched
  ~175 ms so it is visible), not a heartbeat.
- USB 2.0 chapters 7 (physical), 8 (protocol), 9 (enumeration) are the
  three spec chapters this project covers; suspend/resume and
  high-speed are deliberately out of scope.

*Next: [usb-primer.md](usb-primer.md) for the protocol background, or
[build-flow.md](build-flow.md) to build it yourself.*