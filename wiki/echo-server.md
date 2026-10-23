# `EchoServer` — bridging the USB byte stream and raw UART

**Source:** `src/echo.vir:53` · **Back:** [Home.md](Home.md) · **Dep:** [echo-fifo.md](echo-fifo.md) · [uart-vir.md](uart-vir.md)

`EchoServer` is the Virdant bridge between the USB core's byte stream and two raw UART lines, `txd`/`rxd`. Bytes from the host are *rendered* onto `txd` at 115200 8N1; bytes read from `rxd` are pushed back to the host unmodified. The loop that turns this into an **echo** lives at the top level (`src/top.vir:47-49`) — see [top-vir.md](top-vir.md).

## The data path

```
host --USB--> usb_rx_data/usb_rx_valid --> EchoFifo --> UartSender --> txd
                                                                | (loopback wire,
                                                                |  src/top.vir:47-49)
host <--USB-- tx_data/tx_strobe <-- hold slot <-- UartReceiver <-- rxd
```

- **Host → line:** every byte the USB core pops (`usb_rx_valid` strobe) is pushed into the [EchoFifo](echo-fifo.md); the head byte is handed to `UartSender` when it can take it, and serialized onto `txd`.
- **Line → host:** `UartReceiver` deserializes `rxd`; a completed byte lands in the **hold slot**; the slot is pushed into the USB core's tx FIFO (`tx_data`/`tx_strobe`) when that FIFO has room.

## The top-level loopback wire

`src/top.vir:47-49`:

```text
wire uart_tx_line : Bit { it := echo.txd }
wire uart_rx_line : Bit { it := uart_tx_line }
echo.rxd := uart_rx_line
```

That single assignment — `rxd` wired straight to `txd` — is the jumper that makes the device an echo server. Cut it (and bring the two wires to real pins) and the same `EchoServer` becomes a USB↔external-UART bridge with no changes here.

## End-to-end flow control

Nothing in this chain can drop a byte:

- **`room`** (`src/echo.vir:134`) is the FIFO's room signal (`count < 12`, see [echo-fifo.md](echo-fifo.md)) driven straight back to the USB core as `rx_ready`. While the queue fills, the core stops popping and **NAKs bulk-OUT packets** — the host retries, nothing is lost. (NAK vs other handshake PID: [usb-primer.md](usb-primer.md).)
- **`tx_drained`** (`src/echo.vir:130-132`) is `!hold_valid`: the sender may start the next byte only once the previously *received* byte has been handed to the USB core. This guarantees the **hold slot is empty before the sender starts**, so `receiver.data_valid` can never overwrite an untransmitted byte — the two events can never coincide.
- **`usb_tx_ready`**: bytes are pushed into the USB core's tx FIFO (`do_push`, `src/echo.vir:121-123`) only when that FIFO has room; otherwise `hold_valid` stays set and `tx_drained` stays low, throttling the whole upstream chain.

## The hold slot registers

```text
reg hold_data : Word[8] on clock   // src/echo.vir:104-110
reg hold_valid : Bit on clock      // src/echo.vir:112-119
```

A one-byte skid buffer between the receiver and the USB core:

- `hold_valid` is set when `receiver.data_valid` strobes (a byte completed on `rxd`), cleared when `do_push` fires (the byte entered the USB tx FIFO); both never true together by construction (see `tx_drained` above).
- `hold_data` captures `receiver.data` on the strobe and holds it until then.
- `tx_data := hold_data`, `tx_strobe := do_push` — the slot **is** the interface to the USB tx FIFO.

## The pop handshake with the sender

`src/echo.vir:87-90`:

```text
wire fifo_pop : Bit {
    it := (fifo.count != 0w5) && sender.ready && tx_drained
}
sender.data_valid := fifo_pop
```

The head byte is loaded into `UartSender` (`fifo.pop_data` is wired to `sender.data`, `src/echo.vir:81`) only when the queue is non-empty, the sender is idle, and the hold slot is drained. Because `EchoFifo`'s `pop_data` is the head valid on the same edge as `pop_en` ([echo-fifo.md](echo-fifo.md)), the load is combinationally coincident with the pop.

## Key takeaways

- Data path: USB rx → `EchoFifo` → `UartSender` → `txd`; `rxd` → `UartReceiver` → hold slot → USB tx → host.
- The **loopback wire lives at the top level** (`src/top.vir:47-49`); cut it to talk to a real external UART.
- `room` NAKs bulk-OUT while the queue fills; `tx_drained` (`!hold_valid`) guarantees the hold slot is empty before the sender starts, so overwrite and completion can never coincide; pushes wait for `usb_tx_ready`.
- The hold slot is a one-byte register pair (`hold_data`/`hold_valid`) — set by `receiver.data_valid`, cleared by `do_push` into the USB tx FIFO.