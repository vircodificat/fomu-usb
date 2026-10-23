# The platform export `Top` — wiring the core to the echo server

**Source:** `src/top.vir:4` · **Back:** [Home.md](Home.md) · **Parts:** [fomu-top.md](fomu-top.md) · [echo-server.md](echo-server.md) · [build-flow.md](build-flow.md)

`src/top.vir` is the Virdant half of the Fomu top level. Where the ext module `\top::FomuTop` ([fomu-top.md](fomu-top.md)) owns the pads, POR, and the USB core, this file is a **platform export**: pure wiring, no logic.

## What a platform export is

```text
export mod Top for ice40::Fomu { ... }
```

`export mod ... for <platform>` is Virdant's way of saying *this module targets a concrete board* (here `ice40::Fomu`): it declares the board-level ports (`clock`, `usb_dp`, pins…) and the synthesis flow pins/places them for that platform (see [build-flow.md](build-flow.md)). All the actual behavior lives in the modules it instantiates.

## What it instantiates

- **`actual_top`** of `FomuTop` (`src/top.vir:18-26`) — the ext module. Only the **board pins** (clock, usb_dp/usb_dn, usb_dp_pu, LEDs) cross this boundary.
- **`echo`** of `echo::EchoServer` (`src/top.vir:31-39`) — the byte-level consumer.

`src/top.vir:58-79` is the matching `ext mod FomuTop` declaration — the Virdant-side view of the Verilog module's ports, including its **byte-stream interface** (`rx_data`/`rx_valid`/`tx_ready` out, `tx_data`/`tx_strobe`/`rx_ready` in) that the platform export consumes below.

## Wiring the echo server into the USB stream

```text
mod echo of echo::EchoServer {
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

## The loopback wires

`src/top.vir:47-49`:

```text
wire uart_tx_line : Bit { it := echo.txd }
wire uart_rx_line : Bit { it := uart_tx_line }
echo.rxd := uart_rx_line
```

The UART lines are internal wires, not pins: `txd` (everything the host wrote, rendered at 115200 8N1) is wired straight into `rxd`. **This one assignment is the loopback jumper that makes the device an echo server.** Cut it and bring the two wires to real pins to talk to an external UART instead — the echo server needs no changes ([echo-server.md](echo-server.md)).

## Why `usb_dp_pu` must be explicitly wired

`src/top.vir:8-12`:

> the platform default is low (= disconnected), so a wrapper that leaves it unwired produces an image no host can ever see (the P0 found when the first upload build tied it to 1'b0).

Unwired platform ports default to low. `usb_dp_pu` low = D+ 1.5 kΩ pull-up off = the host **never sees a Full-Speed device attach** (see [fomu-top.md](fomu-top.md) for why the pull-up is attach signalling, [usb-primer.md](usb-primer.md) for the signalling itself). The export wires `usb_dp_pu := it.usb_dp_pu` so the ext module's POR-gated pull-up reaches the actual pin.

## The ext mod declaration

`src/top.vir:58` — `ext mod FomuTop { ... }` — is how Virdant references a module whose **body lives in Verilog** (`ext/fomu_top.sv`): it declares the port list (`incoming`/`outgoing`, `Word[8]` byte ports) so Virdant code can instantiate and wire it, while the implementation is vendored from ext/ (see [build-flow.md](build-flow.md)). The `//>` comments are Virdant's doc-comment syntax.

## Key takeaways

- A platform export (`export mod Top for ice40::Fomu`) is board-level wiring: it instantiates `FomuTop` (pads/POR/USB core) and `EchoServer` and pins the ports for the Fomu board.
- `rx_ready := echo.room` is the entire receive-side flow control between the echo server and the USB core.
- `src/top.vir:47-49` is the loopback jumper (`txd → rxd`); cut it (and repin the wires) for a real external UART.
- `usb_dp_pu` must be wired explicitly — the platform default (low) means the host never sees the device attach.
- `ext mod FomuTop` declares the Verilog ext module's ports to Virdant; its body is `ext/fomu_top.sv`.