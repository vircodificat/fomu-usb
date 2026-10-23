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
wire bit takes exactly **4 clock cycles** (`ext/usb_uart.sv:7`). The
whole design is one synchronous domain — no CDC, no multi-clock
headaches.

## Pins

From `fomu.pcf` (cross-checked against the Fomu PVT board file):

| Pin | Signal | Role |
|---|---|---|
| `F4` | `clock` | 48 MHz board clock |
| `A1` | `usb_dp` | USB D+ (inout, via `SB_IO` cells) |
| `A2` | `usb_dn` | USB D− (inout, via `SB_IO` cells) |
| `A4` | `usb_dp_pu` | D+ 1.5 kΩ pull-up enable — **this is how the device "attaches"** |
| `B5` / `A5` / `C5` | `led_red/green/blue` | LEDs, active low; red is the heartbeat |

Because the iCE40 has no tristate fabric, D+/D− go through explicit
`SB_IO` tristate pad cells (`PIN_TYPE 6'b101001`) inside
`ext/fomu_top.sv:90` — one shared `OUTPUT_ENABLE` for the pair, so the
core never drives one line alone.

## Module hierarchy

```
module Top (build/top.sv, emitted from src/top.vir's platform export)
├── \top::FomuTop (ext/fomu_top.sv)        ← pad glue, POR, pull-up, LED
│   └── usb_uart (ext/usb_uart.sv)         ← the USB core
│       ├── usb_rx_phy                      pads → bit stream
│       ├── usb_rx_packet                   bit stream → packet records
│       │     ├── usb_crc5                  token CRC (x⁵+x²+1)
│       │     └── usb_crc16                 data CRC (x¹⁶+x¹⁵+x²+1)
│       ├── usb_ep0                         EP0 control SM + descriptor ROM
│       ├── usb_serial                      EP1 bulk protocol, toggles, ACK/NAK
│       ├── usb_tx_phy                      packet records → pads
│       │     ├── usb_crc5 ─ usb_crc16       (shared instances)
│       │     └── usb_fifo16x8 ×2            rx FIFO, tx FIFO
│       └── (token router + arbiter)        routing & mux logic in usb_uart
├── EchoServer (src/echo.vir)
│   ├── EchoFifo (ext/echo_fifo.sv)         16×8 queue host→UART
│   ├── UartSender (src/uart.vir)           bytes → 115200 8N1 txd
│   ├── UartReceiver (src/uart.vir)         115200 8N1 rxd → bytes
│   └── hold slot                           receiver → USB tx FIFO
└── loopback wire: uart_tx_line := txd; uart_rx_line := txd  (src/top.vir:47-49)
```

Per-module deep dives: [usb-uart.md](usb-uart.md) (core top),
[fomu-top.md](fomu-top.md), [top-vir.md](top-vir.md),
[echo-server.md](echo-server.md).

## The end-to-end data path

Host → device (the "render" direction):

```
host USB pad
  → usb_rx_phy      (NRZI decode, unstuff, sync/EOP detect)
  → usb_rx_packet   (PID, addr/ep filter, CRC check, byte buffer)
  → usb_uart router (EP1 OUT → rx FIFO)
  → rx FIFO         (usb_fifo16x8)
  → EchoFifo        (src/echo.vir — with `room` backpressure)
  → UartSender      (115200 8N1 frame)
  → uart_tx_line    (the raw UART tx wire)
  → [loopback]      (uart_rx_line := uart_tx_line, src/top.vir:48)
```

Device → host (the "echo" direction):

```
uart_rx_line
  → UartReceiver    (start-bit detect, mid-bit sampling)
  → hold slot      (one-byte register, src/echo.vir:104)
  → tx FIFO        (usb_uart, pushed when usb_tx_ready)
  → usb_serial     (DATA0/1 toggle, host ACK handshake)
  → usb_tx_phy     (CRC16, stuffing, NRZI, sync, EOP)
  → usb_uart arbiter
  → SB_IO pads
  → host
```

Nothing in this chain can drop a byte: every hop has explicit flow
control (see [echo-server.md](echo-server.md) and
[usb-serial.md](usb-serial.md)).

## Clocking

One domain: **48 MHz** (`ext/usb_uart.sv:7`).

- USB FS bit period = 4 clock cycles; the rx PHY recovers the bit
  phase by oversampling and re-aligning on every wire transition
  ([usb-rx-phy.md](usb-rx-phy.md)).
- UART side: 48_000_000 / 115200 ≈ 416.67, so one UART bit is 417
  cycles (−0.08 % error, well inside tolerance) — `src/uart.vir:6-8`.
- All resets are synchronous, active high, distributed by `usb_uart`
  (and `por` in `ext/fomu_top.sv`).

## Power-on reset and attach

`ext/fomu_top.sv:44-59`:

1. On power-up a counter runs for **POR_CYCLES ≈ 8.4 M cycles (~175 ms
   @ 48 MHz)**; during that time `core_rst` is held asserted and
   `usb_dp_pu` (pin A4) is **low** — the 1.5 kΩ D+ pull-up is
   disconnected, so the host sees nothing on the bus.
2. When the counter expires, the core reset is released **and**
   `usb_dp_pu` goes high. The pull-up drags D+ high while D− floats low
   — the host sees a **full-speed attach** (see
   [usb-primer.md](usb-primer.md) § electrical) and starts enumeration
   ([usb-ep0.md](usb-ep0.md)).
3. The red LED starts blinking (~5.7 Hz) as a heartbeat; green/blue are
   off. If you see the blink but no `/dev/ttyUSB*`-style device, check
   the pull-up first — a wrapper that leaves `usb_dp_pu` unwired
   produces an image no host can ever see (the note in
   `src/top.vir:8-11` exists because that exact bug was found).

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

Scope limits (intentional, `ext/usb_uart.sv:8-11`): FS only, one
configuration, no suspend/resume, no SOF bookkeeping, EP2 IN exists in
the descriptors (Linux's `cdc_acm` requires it) but is never armed —
every IN to EP2 is NAKed.

## Key takeaways

- One 48 MHz clock, one synchronous domain, 4 cycles per USB bit —
  no PLL, no CDC anywhere in the project.
- D+/D− are plain FPGA pads behind `SB_IO` tristate cells; the board
  has no USB PHY — the entire device core is fabric logic.
- Attach = releasing the 1.5 kΩ D+ pull-up (pin A4) after a ~175 ms
  power-on reset; getting that wiring wrong makes the device invisible.
- The system is a strict layering: pads → rx PHY → packet → router →
  endpoint state machines → FIFOs → Virdant echo application → UART →
  and back. Every layer has its own wiki page.
- USB 2.0 chapters 7 (physical), 8 (protocol), 9 (enumeration) are the
  three spec chapters this project covers; suspend/resume and
  high-speed are deliberately out of scope.

*Next: [usb-primer.md](usb-primer.md) for the protocol background, or
[build-flow.md](build-flow.md) to build it yourself.*