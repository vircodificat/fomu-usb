# The platform export `Top` — wiring the POR, the USB core, and the echo server

**Source:** `src/top.vir` (`export mod Top`, line 5) · **Back:** [Home.md](Home.md) · **Parts:** [fomu-top.md](fomu-top.md) · [echo-server.md](echo-server.md) · [build-flow.md](build-flow.md)

`src/top.vir` is the top level of the whole device. It is a **platform export**: pure wiring, no logic of its own — every piece of behavior lives in the four modules it instantiates.

## What a platform export is

```text
export mod Top for ice40::Fomu { ... }
```

`export mod ... for <platform>` is Virdant's way of saying *this module targets a concrete board* (here `ice40::Fomu`): it declares the board-level ports (`clock`, `usb_dp`, pins…) and the synthesis flow pins/places them for that platform (see [build-flow.md](build-flow.md)). The pin constraints live in `fomu.pcf` — see [fomu-top.md](fomu-top.md) for the pin map.

## What it instantiates

- **`por`** of `usb::UsbPor` (`src/usb.vir`) — the ~175 ms power-on / attach-delay counter; `clock` in, sticky `por_done` out.
- **`actual_top`** of `usb::UsbController` (`src/usb.vir`) — the entire USB device core: PHYs, packet decode, EP0, bulk protocol, FIFOs.
- **`pads`** of `usb::UsbPads` (`ext/usb_pads.sv`) — the two iCE40 `SB_IO` tristate pad cells, the only ext module that touches a bidirectional net (see [fomu-top.md](fomu-top.md)).
- **`echo`** of `echo::EchoServer` (`src/echo.vir`) — the byte-level consumer (see [echo-server.md](echo-server.md)).

Only the **board pins** cross the export boundary: `clock` (in), `usb_dp`/`usb_dn`/`usb_dp_pu` and `led_red`/`led_green`/`led_blue` (out).

## The USB controller's two faces

`usb::UsbController` presents a byte-stream interface to the application and a unidirectional pad-facing interface to `UsbPads`:

| Port | Dir | Meaning |
|---|---|---|
| `rx_data` | out | byte from the host |
| `rx_valid` | out | 1-clk strobe per received byte |
| `tx_ready` | out | tx FIFO has room |
| `tx_data` | in | byte toward the host |
| `tx_strobe` | in | 1-clk push strobe |
| `rx_ready` | in | low = consumer full: the core stops popping and NAKs bulk-OUT |
| `rxing` / `txing` | out | activity indicators (drive the LEDs) |
| `dp_tx`/`dn_tx`/`output_enable` | out | drive levels + tristate control, to `UsbPads` |
| `dp_rx`/`dn_rx` | in | pad samples, from `UsbPads` |
| `por_done` | in | from `UsbPor`; registered internally into the core reset |

## Wiring the echo server into the USB stream

```text
mod echo of echo::EchoServer {
    it.clock        := clock
    it.usb_rx_data  := actual_top.rx_data
    it.usb_rx_valid := actual_top.rx_valid
    it.usb_tx_ready := actual_top.tx_ready
}
actual_top.tx_data   := echo.tx_data
actual_top.tx_strobe := echo.tx_strobe
actual_top.rx_ready  := echo.room
```

- Host→device bytes (`rx_data`/`rx_valid`) feed the echo server; device→host bytes (`tx_data`/`tx_strobe`) come back.
- **`rx_ready := echo.room`** is the whole of the receive-side flow control: the echo server's FIFO `room` signal (see [echo-fifo.md](echo-fifo.md)) tells the USB core to stop popping — the core NAKs bulk-OUT instead. The echo server never drops a byte (see [echo-server.md](echo-server.md)).

## The power-on reset and `usb_dp_pu`

```text
mod por of usb::UsbPor { it.clock := clock }
usb_dp_pu := por.por_done
```

Unwired platform ports default to low. `usb_dp_pu` low = D+ 1.5 kΩ pull-up off = the host **never sees a Full-Speed device attach** — the file's own comment warns that a wrapper leaving it unwired produces an image no host can ever see. The export wires `usb_dp_pu := por.por_done`, so the pull-up is released exactly when the ~175 ms power-on count completes (why the pull-up is attach signalling: [fomu-top.md](fomu-top.md); the signalling itself: [usb-primer.md](usb-primer.md)).

`por_done` also feeds `actual_top.por_done`; inside `usb::UsbController` it is registered one more time (`por_rst`) to form the core-wide reset — so the pull-up rises one clk before the core leaves reset.

## The pads: bidirectionality stays inside the ext mod

```text
mod pads of usb::UsbPads {
    it.dp_tx          := actual_top.dp_tx
    it.dn_tx          := actual_top.dn_tx
    it.output_enable  := actual_top.output_enable
}
usb_dp := pads.usb_dp
usb_dn := pads.usb_dn
actual_top.dp_rx := pads.dp_rx
actual_top.dn_rx := pads.dn_rx
```

The pad-facing side of the core is deliberately **unidirectional** wires (`dp_tx`/`dn_tx`/`output_enable` out, `dp_rx`/`dn_rx` in); the electrical bidirectionality (drive vs. release the bus) lives entirely inside the `UsbPads` ext body's `SB_IO` cells (see [fomu-top.md](fomu-top.md)). From Virdant's point of view `usb_dp`/`usb_dn` are ordinary outgoing wires the export assigns to the `Top` ports.

## The activity LEDs

```text
led_red   := !actual_top.rxing
led_green := !actual_top.txing
led_blue  := true
```

The LEDs are **active low** on the PVT board, so the inversions light them and `led_blue := true` ties blue off. `rxing`/`txing` come from the USB core: they are the 1-clk `rx_valid`/`tx_strobe` strobes stretched by 23-bit countdown counters inside `usb::UsbUart` (reloaded to `0x7F_FFFF` ≈ 175 ms on every strobe), so sustained traffic reads as a solid light and a single packet is still visible. Only bulk data traffic lights them — EP0 control traffic does not pulse `rx_valid`/`tx_strobe`.

## The loopback wires

```text
wire uart_tx_line : Bit { it := echo.txd }
reg uart_rx_reg : Bit on clock { it <= uart_tx_line }
echo.rxd := uart_rx_reg
```

The UART lines are internal wires, not pins: `txd` (everything the host wrote, rendered at 115200 8N1) is looped into `rxd` through a one-clk register. **This assignment is the loopback jumper that makes the device an echo server.**

The register is load-bearing: without it, the raw wire forms one long combinational path through the UART (sender data mux → `txd` → `rxd` → receiver state/baud counter), which dominates the critical path. One clk of loop latency is 1/417 of a UART bit time — invisible to the 8N1 receiver's mid-bit sampling.

Cut the loop (drive `echo.rxd` from a pin instead) to talk to an external UART — the echo server needs no changes ([echo-server.md](echo-server.md)).

## Key takeaways

- A platform export (`export mod Top for ice40::Fomu`) is board-level wiring: it instantiates `UsbPor`, `UsbController`, `UsbPads`, and `EchoServer`, and pins the ports for the Fomu board.
- `usb_dp_pu := por.por_done` must be wired explicitly — the platform default (low) means the host never sees the device attach.
- The pad-facing core wires are unidirectional; the only bidirectional nets live inside the `UsbPads` ext body.
- `rx_ready := echo.room` is the entire receive-side flow control between the echo server and the USB core.
- LEDs: red = receiving (`rxing`), green = sending (`txing`), blue tied off; both activity signals are stretched ~175 ms so traffic is visible.
- The loopback jumper (`echo.txd` → registered → `echo.rxd`) is what makes the device an echo server; the register breaks the UART's combinational loop path.
