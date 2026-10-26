# `endpoint::Serial` — EP1 bulk protocol

`endpoint::Serial` (`src/endpoint.vir`) is the protocol brain for the CDC-ACM data
endpoints: **EP1 OUT** (host → device data) and **EP1 IN**
(device → host data), both **bulk** endpoints. It owns the *data toggles*,
decides **ACK vs NAK** for every transaction, and tells the core top
([`usb::UsbUart`](usb-uart.md)) what to transmit. It never touches the
memories or the wire itself — all data movement (the FIFOs, the snapshot
RAM, the tx byte fetch, the rx push) lives in `usb::UsbUart`.

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
  Max packet size on this core is 64 bytes: the rx FIFO (`usb::RxFifo`, a
  64×8 ext body over `usb_fifo16x8`, `ext/usb_uart.sv`) is exactly that
  deep, and the IN arm size caps at `min(count, 64)`.

## Data toggles: DATA0 / DATA1

Every endpoint maintains a toggle, one bit per direction. The rule:

- Sender flips its toggle **only after the receiver ACKs**.
- Receiver accepts a data packet **only if its PID matches the expected
  toggle**; a mismatched packet is a retransmission duplicate and is NAKed.
- Toggles reset to "expect DATA0" on **bus reset** and **SET_CONFIGURATION**
  (USB 2.0 §9.1.1.5).

In this module `tog_tx`/`tog_rx` are the toggles (the protocol registers in
`endpoint::Serial`): `0` means the *next* data PID must be DATA0
(`exp_pid` picks `DATA0`/`DATA1` from `tog_rx`).
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

- **TX**: `IN` token + no pending packet + empty tx FIFO → NAK (the
  nothing-to-send arm of `resp_pid_r`).
- **RX**: `OUT` data rejected (`dt_rej`) → NAK.

## OUT packets → rx FIFO (host → device)

1. `tk_out` (an `OUT` token to EP1, router-gated) sets `armed_out` — a data
   packet is now expected. While armed, `ser_busy` is high so the core's
   data router delivers the next data packet here (unless EP0 is claiming
   it — see [usb-uart.md](usb-uart.md)).
2. The data packet arrives as a 1-clock `dt_valid` strobe with `dt_pid` and
   `dt_crc_ok` — the **arrival-cycle** strobe, *not* delayed a clk like
   EP0's, because the accept decision below is itself registered. The
   accept/reject decision lands one clock later in `dt_acc`/`dt_rej`, so the
   room check (an add+compare against the rx FIFO count, done in the core
   top: `rx_room_ok` = `rxfifo.count + dt_len <= 64`) doesn't have to land
   in the arrival cycle. One clock is free: the device turnaround budget is
   ~30–50 clocks at 48 MHz.
3. **Accept** (`dt_acc`) requires all of: armed, `dt_valid`, CRC good, PID
   matches the expected `tog_rx` toggle, **and `rx_room`** — the whole
   packet must fit the 64-deep rx FIFO.
   - Accept → respond **ACK**, flip `tog_rx`, and the core streams the
     payload bytes from the packet assembler's payload RAM (`usb::RxRam`,
     64×8) into the rx FIFO one byte per clk (the rx push in
     `usb::UsbUart`).
   - Anything else (`dt_rej`) → **NAK, no toggle flip**. The host will
     retransmit the *same* toggle and the same bytes.

`dt_len` is wired to the module but unused: the core's `rx_room` test
already includes the packet length, and the core's rx push counts the
packet's bytes itself — the whole packet is accepted or rejected on
`rx_room` alone.

**Zero-length packets**: a bulk-OUT ZLP passes the same checks (`rx_room`
is trivially satisfied) and is ACKed with the toggle flip, but moves no
data — the core's rx push only runs for a non-zero `dt_len`. On the IN side
the device never *generates* a ZLP: arming requires a non-empty tx FIFO, so
an `IN` token with nothing queued (and no pending packet) is NAKed.

## IN packets ← tx FIFO (device → host)

1. `tk_in`, no unacknowledged packet, and the tx FIFO has data → **arm a
   new packet**: pulse `resp_req` with the data PID per `tog_tx`,
   `resp_len = min(txfifo_count, 64)`, and the `resp_fresh` strobe. The
   core top's snapshot loader then pops those bytes from the tx FIFO into
   the 64-byte snapshot RAM (`usb::SnapRam`) one byte per clk — byte 0
   isn't needed until ~72 clks after the arm, so the load always wins the
   race.
2. `pkt_pending` remembers an unacknowledged snapshot. Until the host ACKs,
   every subsequent `IN` retransmits the **same snapshot bytes with the
   same PID** (`resp_len = pkt_len`, no `resp_fresh` → the core replays the
   snapshot RAM without re-popping the FIFO) — byte-identical
   retransmission, so the host can never dedupe the wrong packet.
3. `hs_ack` (an ACK for *our* in-flight packet, router-gated) clears
   `pkt_pending` and flips `tog_tx`.
4. `ser_ack_wait` (= `pkt_pending`) tells the handshake router this module
   owns the next ACK.

## Flow control: `rx_ready` → NAK

End-to-end, no byte loss:

1. The byte-stream consumer (the UART echo server,
   [echo-server.md](echo-server.md)) drives `rx_ready` with its echo FIFO's
   `room` signal — low once the 16-deep EchoFifo holds 12 bytes.
2. The core stops popping the rx FIFO (the pop in `usb::UsbUart` requires
   `rx_ready`; it also keeps tx FIFO headroom, because every popped byte
   must land in the tx FIFO through the loopback echo) → the rx FIFO fills.
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
| `dt_pid` | in | 4 | data PID |
| `dt_len` | in | 7 | data payload byte count (unused here) |
| `dt_crc_ok` | in | 1 | data CRC16 residual check |
| `hs_ack` | in | 1 | our in-flight packet was ACKed |
| `txfifo_empty` | in | 1 | tx FIFO empty |
| `txfifo_count` | in | 7 | bytes waiting in tx FIFO |
| `rx_room` | in | 1 | whole packet fits the rx FIFO (core-side add+compare) |
| `tx_room` | in | 7 | free slots in tx FIFO (unused here) |
| `resp_req` | out | 1 | response request (1-clk pulse) |
| `resp_pid` / `resp_len` | out | 4 / 7 | response PID / length |
| `resp_fresh` | out | 1 | 1-clk strobe: a NEW packet was armed (starts the snapshot load) |
| `ser_busy` | out | 1 | OUT data packet expected (= `armed_out`) |
| `ser_ack_wait` | out | 1 | our data packet is in flight (= `pkt_pending`) |

State: `tog_tx`, `tog_rx`, `pkt_pending`, `pkt_len`, `armed_out`,
`dt_acc`, `dt_rej` (the protocol registers in `endpoint::Serial`); the response
registers `resp_req_r`/`resp_pid_r`/`resp_len_r`/`resp_fresh_r` hold the
latched response the core's arbiter picks up.

## Key takeaways

- Bulk endpoints use **data toggles** (DATA0/DATA1) to detect lost and
  duplicated packets; toggles flip only on ACK and reset on bus reset /
  SET_CONFIGURATION.
- **ACK** = "accepted, flip toggle"; **NAK** = "retry, keep toggle";
  toggling on a NAK corrupts the stream.
- Unacknowledged IN data lives in the **snapshot RAM** so retransmissions
  are byte- and PID-identical until ACKed.
- OUT accept requires CRC-good + correct toggle + `rx_room`; otherwise NAK —
  which is how a full consumer (`rx_ready` low) throttles the host with zero
  byte loss.
- ZLPs: accepted (and ACKed) on the OUT side but move no data; never
  generated on the IN side.
- The core never STALLs EP1; it only ACKs or NAKs.
- Extra clocks spent registering decisions (`dt_acc`/`dt_rej`, and the
  core's registered token strobes) are free — the device turnaround budget
  (~30–50 clks) dwarfs them. EP0 delays its data strobe instead; Serial
  keeps the arrival-cycle strobe because its own decision is already
  registered.

Next: [`usb::UsbUart`](usb-uart.md) — the core top that connects this
module to the FIFOs, router, arbiter and PHYs.
