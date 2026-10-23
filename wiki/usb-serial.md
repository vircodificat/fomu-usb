# `usb_serial` — EP1 bulk protocol

`usb_serial` (`ext/usb_uart.sv:1475`) is the protocol brain for the CDC-ACM
data endpoints: **EP1 OUT** (host → device data) and **EP1 IN**
(device → host data), both **bulk** endpoints. It owns the *data toggles*,
decides **ACK vs NAK** for every transaction, and tells the core top
([`usb_uart`](usb-uart.md)) what to transmit. It never touches the FIFOs or
the wire itself — all data movement (FIFOs, snapshot register, tx mux) lives
in `usb_uart` (`ext/usb_uart.sv:1467`–`1474`).

Prerequisites: [usb-primer.md](usb-primer.md) (tokens, PIDs, transfers),
[usb-rx-packet.md](usb-rx-packet.md) (what `dt_valid`/`dt_pid` mean),
[usb-ep0.md](usb-ep0.md) (SET_CONFIGURATION, which resets the toggles).

## What a bulk transfer is

USB has four transfer types; this device uses **control** (EP0, see
[usb-ep0.md](usb-ep0.md)) and **bulk**. Bulk is the "give me lots of data,
error-checked, bandwidth-flexible" type:

- The host **polls**: an `OUT` token means "data is coming to you", an `IN`
  token means "give me data" (see [usb-primer.md](usb-primer.md)).
- A bulk transaction = token → (optional) DATA packet → handshake.
- **No guaranteed bandwidth**: if the device has nothing to say it answers
  `NAK` and the host retries later. That is exactly how USB flow control is
  *supposed* to work — no data is ever lost, only deferred.
- Every DATA packet carries a **data toggle** in its PID: `DATA0` or `DATA1`.
  Max packet size on this core is 64 bytes (`ext/usb_uart.sv:1538`).

## Data toggles: DATA0 / DATA1

Every endpoint maintains a toggle, one bit per direction. The rule:

- Sender flips its toggle **only after the receiver ACKs**.
- Receiver accepts a data packet **only if its PID matches the expected
  toggle**; a mismatched packet is a retransmission duplicate and is dropped.
- Toggles reset to "expect DATA0" on **bus reset** and **SET_CONFIGURATION**
  (USB 2.0 §9.1.1.5) — hence the `bus_reset`/`set_cfg` clear at
  `ext/usb_uart.sv:1517`.

In this module `tog_tx`/`tog_rx` are the toggles
(`ext/usb_uart.sv:1507`): `0` means the *next* data PID must be DATA0.
`0 → 1 → 0` is the "I received/sent a fresh packet" heartbeat of the whole
link; if you ever lose sync, the host and device will disagree on the toggle
and every subsequent packet is dropped — this is one of the classic
"device works then goes silent" USB bugs.

## Handshakes: ACK / NAK / (no STALL here)

- **ACK** — "accepted; flip your toggle."
- **NAK** — "I'm busy/full; retry. Do **not** flip the toggle."
- **STALL** — "unsupported." This CDC-ACM core never STALLs EP1; the
  endpoints are always supported. (An EP2 STALL-ish case: EP2 IN/OUT is
  always NAKed — see [usb-uart.md](usb-uart.md).)

Both directions of flow control fall out of this:

- **TX**: `IN` token + empty tx FIFO → NAK (`ext/usb_uart.sv:1547`).
- **RX**: `OUT` data + rx FIFO too full → NAK (`ext/usb_uart.sv:1577`).

## OUT packets → rx FIFO (host → device)

1. `tk_out` (an `OUT` token to EP1, router-gated) sets `armed_out`
   (`ext/usb_uart.sv:1554`) — a data packet is now expected. While armed,
   `ser_busy` is high so the core's data router steers the next data packet
   here and not to EP0 (`ext/usb_uart.sv:1844`).
2. The data packet arrives as a 1-clock `dt_valid` strobe with `dt_pid`,
   `dt_len`, `dt_crc_ok`. The accept/reject decision is **registered one
   clock later** in `dt_acc`/`dt_rej` (`ext/usb_uart.sv:1562`–`1568`) so the
   room check (an add+compare against the rx FIFO count) doesn't have to
   land in the arrival cycle. One clock is free: the device turnaround
   budget is ~30–50 clocks at 48 MHz.
3. **Accept** requires all of: armed, `dt_valid`, CRC good, PID matches the
   expected `tog_rx` toggle, **and `rx_room`** — the whole packet must fit
   the rx FIFO (64-deep, `ext/usb_uart.sv:1710`).
   - Accept → respond **ACK** (`ext/usb_uart.sv:1572`), flip `tog_rx`, and
     the core streams the payload bytes into the rx FIFO
     (`ext/usb_uart.sv:1938`).
   - Anything else → **NAK, no toggle flip** (`ext/usb_uart.sv:1577`). The
     host will retransmit the *same* toggle and the same bytes.

## IN packets ← tx FIFO (device → host)

1. `tk_in` and the tx FIFO has data → **arm a new packet**: snapshot
   `min(count, 64)` bytes from the tx FIFO into `usb_uart`'s 64-byte
   snapshot register, respond DATA0/DATA1 per `tog_tx`, and raise
   `resp_fresh` (`ext/usb_uart.sv:1536`–`1545`).
2. `pkt_pending` remembers an unacknowledged snapshot
   (`ext/usb_uart.sv:1508`). Until the host ACKs, every subsequent `IN`
   retransmits the **same snapshot bytes with the same PID**
   (`ext/usb_uart.sv:1531`) — byte-identical retransmission, so the host
   can never dedupe the wrong packet.
3. `hs_ack` (an ACK for *our* in-flight packet, router-gated) clears
   `pkt_pending` and flips `tog_tx` (`ext/usb_uart.sv:1582`).
4. `ser_ack_wait` (= `pkt_pending`) tells the handshake router this module
   owns the next ACK (`ext/usb_uart.sv:1848`).

## Flow control: `rx_ready` → NAK

End-to-end, no byte loss:

1. The byte-stream consumer (e.g. the UART echo server,
   [echo-server.md](echo-server.md)) pulls `rx_ready` **low** when its own
   buffer is full (`ext/usb_uart.sv:1691`).
2. The core stops popping the rx FIFO → the rx FIFO fills.
3. Next bulk-OUT data packet fails the `rx_room` test → **NAK**.
4. The host retransmits. The data waits inside the host's own stack until
   the device is ready — USB's built-in retry does all the queuing.

No CRC layer, no software retry loop, no lost bytes — just a toggle bit and
a NAK.

## Ports

| Port | Dir | Width | Meaning |
|---|---|---|---|
| `clk` | in | 1 | 48 MHz core clock |
| `rst` | in | 1 | sync reset, active high |
| `bus_reset` | in | 1 | bus reset detected (rx PHY) |
| `set_cfg` | in | 1 | SET_CONFIGURATION completed |
| `tk_in` / `tk_out` | in | 1 | router-gated IN/OUT token to EP1 |
| `dt_valid` | in | 1 | router-gated data packet (armed OUT to EP1) |
| `dt_pid` / `dt_len` | in | 4 / 7 | data PID and byte length |
| `dt_crc_ok` | in | 1 | data CRC16 residual check |
| `hs_ack` | in | 1 | our in-flight packet was ACKed |
| `txfifo_empty` | in | 1 | tx FIFO empty |
| `txfifo_count` | in | 7 | bytes waiting in tx FIFO |
| `rx_room` | in | 1 | whole packet fits the rx FIFO |
| `tx_room` | in | 7 | free slots in tx FIFO |
| `resp_req` | out | 1 | response request (1-clk pulse) |
| `resp_pid` / `resp_len` | out | 4 / 7 | response PID / length |
| `resp_fresh` | out | 1 | snapshot len bytes from tx FIFO |
| `ser_busy` | out | 1 | OUT data packet expected |
| `ser_ack_wait` | out | 1 | our data packet is in flight |

State: `tog_tx`, `tog_rx`, `pkt_pending`, `pkt_len`, `armed_out`,
`dt_acc`, `dt_rej` (`ext/usb_uart.sv:1507`–`1511`).

## Key takeaways

- Bulk endpoints use **data toggles** (DATA0/DATA1) to detect lost and
  duplicated packets; toggles flip only on ACK and reset on bus reset /
  SET_CONFIGURATION.
- **ACK** = "accepted, flip toggle"; **NAK** = "retry, keep toggle";
  toggling on a NAK corrupts the stream.
- Unacknowledged IN data lives in a **snapshot** register so
  retransmissions are byte- and PID-identical until ACKed
  (`ext/usb_uart.sv:1531`).
- OUT accept requires CRC-good + correct toggle + `rx_room`; otherwise NAK —
  which is how a full consumer (`rx_ready` low) throttles the host with zero
  byte loss.
- The core never STALLs EP1; it only ACKs or NAKs.
- Extra clocks spent registering decisions (`dt_acc`, `tok_v_r`) are free —
  the device turnaround budget (~30–50 clks) dwarfs them.

Next: [`usb_uart`](usb-uart.md) — the core top that connects this module to
the FIFOs, router, arbiter and PHYs.