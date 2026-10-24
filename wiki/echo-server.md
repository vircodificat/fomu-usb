# `EchoServer` — bridging the USB byte stream and raw UART

**Source:** `src/echo.vir` (`mod EchoServer`) · **Back:** [Home.md](Home.md) · **Dep:** [echo-fifo.md](echo-fifo.md) · [uart-vir.md](uart-vir.md)

`EchoServer` is the Virdant bridge between the USB core's byte stream and two raw UART lines, `txd`/`rxd`. Bytes from the host are *rendered* onto `txd` at 115200 8N1; bytes read from `rxd` are pushed back to the host unmodified. The loop that turns this into an **echo** lives at the top level (`src/top.vir`) — see [top-vir.md](top-vir.md).

## The data path

```
host --USB--> usb_rx_data/usb_rx_valid --> EchoFifo --> UartSender --> txd
                                                                | (1-clk registered
                                                                |  loopback, src/top.vir)
host <--USB-- tx_data/tx_strobe <-- hold slot <-- UartReceiver <-- rxd
```

- **Host → line:** every byte the USB core pops (`usb_rx_valid` strobe) is pushed into the [EchoFifo](echo-fifo.md); the head byte is handed to `UartSender` when it can take it, and serialized onto `txd`.
- **Line → host:** `UartReceiver` deserializes `rxd`; a completed byte lands in the **hold slot**; the slot is pushed into the USB core's tx FIFO (`tx_data`/`tx_strobe`) when that FIFO has room.

## The top-level loopback register

`src/top.vir`:

```text
wire uart_tx_line : Bit { it := echo.txd }
reg uart_rx_reg : Bit on clock {
    it <= uart_tx_line
}
echo.rxd := uart_rx_reg
```

The loopback is **registered**: `rxd` follows `txd` with one clock of delay. The register breaks what would otherwise be a single long combinational path straight through the UART (sender's data mux → `txd` → `rxd` → receiver state/counter logic), which would dominate the critical path. One clock of loop latency is 1/417 of a UART bit time — invisible to the receiver's mid-bit sampling. That single loopback connection is what makes the device an echo server: cut it (and bring the two wires to real pins) and the same `EchoServer` becomes a USB↔external-UART bridge with no changes here. (Note `UartReceiver` samples `rxd` with no input synchronizer, so an external source would need one — see [uart-vir.md](uart-vir.md).)

## End-to-end flow control

Nothing in this chain can drop a byte:

- **`room`** is the FIFO's room signal (`count < 12`, see [echo-fifo.md](echo-fifo.md)) driven straight back to the USB core as `rx_ready`. While the queue fills, the core stops popping and **NAKs bulk-OUT packets** — the host retries, nothing is lost. (NAK vs other handshake PID: [usb-primer.md](usb-primer.md).)
- **`tx_drained`** is `!hold_valid`: the sender may start the next byte only once the previously *received* byte has been handed to the USB core. This guarantees the **hold slot is empty before the sender starts**, so `receiver.data_valid` can never overwrite an untransmitted byte — the two events can never coincide.
- **`usb_tx_ready`**: the core's registered tx-FIFO room signal (one clk stale). Bytes are pushed into the USB core's tx FIFO (`do_push`) only when it is high; otherwise `hold_valid` stays set and `tx_drained` stays low, throttling the whole upstream chain.

## The hold slot registers

```text
reg hold_data : Word[8] on clock
reg hold_valid : Bit on clock
```

A one-byte skid buffer between the receiver and the USB core:

- `hold_valid` is set when `receiver.data_valid` strobes (a byte completed on `rxd`), cleared when `do_push` fires (the byte entered the USB tx FIFO); both never true together by construction (see `tx_drained` above).
- `hold_data` captures `receiver.data` on the strobe and holds it until then.
- `tx_data := hold_data`, `tx_strobe := do_push` — the slot **is** the interface to the USB tx FIFO.

## The pop handshake with the sender

`fifo.pop_data` is wired to `sender.data`, and the sender is loaded via a registered pop strobe:

```text
wire fifo_pop : Bit {
    it := (fifo.count != 0w5) && sender.ready && tx_drained && !pop_r
}
reg pop_r : Bit on clock {
    initial it <= false
    it <= fifo_pop
}
sender.data_valid := pop_r
```

`EchoFifo`'s read is **registered**: asserting `pop_en` at cycle N consumes the head, and the popped byte appears on `pop_data` during cycle N+1 (see [echo-fifo.md](echo-fifo.md)). The sender's load strobe is therefore `pop_r` — `fifo_pop` delayed by one clock — which pairs the strobe with exactly that one-clk-later byte. The `!pop_r` term keeps exactly one byte in flight between the pop and the load: without it, the still-idle sender would trigger a second pop one cycle after the first and strand the byte it just popped. The pairing is safe because `EchoServer` pops at most one byte per sender frame (~4170 clocks apart) and never pops an empty queue.

## Key takeaways

- Data path: USB rx → `EchoFifo` → `UartSender` → `txd`; `rxd` → `UartReceiver` → hold slot → USB tx → host.
- The **loopback lives at the top level** (`src/top.vir`) and is **registered** — one clock of latency, which breaks the through-the-UART critical path without disturbing mid-bit sampling; cut it to talk to a real external UART.
- `room` NAKs bulk-OUT while the queue fills; `tx_drained` (`!hold_valid`) guarantees the hold slot is empty before the sender starts, so overwrite and completion can never coincide; pushes wait for `usb_tx_ready`.
- The hold slot is a one-byte register pair (`hold_data`/`hold_valid`) — set by `receiver.data_valid`, cleared by `do_push` into the USB tx FIFO.
- The sender is loaded by `pop_r`, one clock after `fifo_pop`, matching `EchoFifo`'s registered `pop_data`; `!pop_r` keeps exactly one byte in flight.
