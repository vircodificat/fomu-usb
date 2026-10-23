# usb_tx_phy — bits back to pads

`usb_tx_phy` is the transmitter physical layer of the Fomu USB device: the
mirror of [`usb_rx_phy`](usb-rx-phy.md). It takes a packet description (PID,
optional payload length) and drives the raw `dp`/`dn` pads with a complete
USB Full-Speed packet: **sync pattern, NRZI-encoded, bit-stuffed data, and
EOP** — plus the output-enable signal that actually connects the driver to
the pads only while transmitting. It corresponds to the "packet records ->
pads" stage of the file-header inventory (`ext/usb_uart.sv:18`) and is
driven by the transmit arbiter inside [`usb_uart`](usb-uart.md). Background
on NRZI, stuffing, sync and EOP is in the [USB primer](usb-primer.md); see
also [system-overview.md](system-overview.md) for the big picture.

As with the whole core, there is a single 48 MHz clock and **4 clk cycles per
USB bit time** (`ext/usb_uart.sv:7`). The module header
(`ext/usb_uart.sv:713`–`ext/usb_uart.sv:731`) is an accurate summary of
everything below and is worth re-reading after this page.

## Ports

| Port | Dir | Width | Meaning |
| --- | --- | --- | --- |
| `clk` | in | 1 | 48 MHz system clock (4 cycles per USB bit time) |
| `rst` | in | 1 | synchronous reset, active high |
| `tx_start` | in | 1 | 1-clk pulse: latch `tx_pid`/`tx_len` and begin the packet |
| `tx_pid` | in | 4 | PID nibble (the upper complement is generated here) |
| `tx_len` | in | 7 | payload byte count, 0..64 (0 ⇒ zero-length packet) |
| `tx_byte` | in | 8 | payload byte from the producer, combinational at `tx_addr` |
| `tx_addr` | out | 7 | byte index currently being consumed |
| `tx_addr_next` | out | 7 | byte index the *next* bit time will consume (preload hint) |
| `tx_busy` | out | 1 | high while a packet is in flight |
| `tx_done` | out | 1 | 1-clk pulse when the bus is released |
| `dp`, `dn` | out | 1 | pad drive levels (valid only while `oe` is high) |
| `oe` | out | 1 | output enable: drive the pads |

`tx_pid`/`tx_len`/`tx_byte` come from the transmit arbiter in
[`usb_uart`](usb-uart.md), which serializes responses from `usb_ep0`,
`usb_serial`, and the ROM (descriptors).

## The bit-time engine

The module is a small FSM (`ext/usb_uart.sv:747`–`ext/usb_uart.sv:748`) with
five states:

```
S_IDLE -> S_GUARD -> S_DATA -> S_EOP0 -> S_EOPJ -> S_IDLE
```

Two counters time everything:

- `btc` (2 bits) — the clk phase inside the current bit time, 0..3. Every
  bit time lasts exactly 4 clks; the level for the *next* bit time is
  computed at the `btc == 3` edge of the current one
  (`ext/usb_uart.sv:899`–`ext/usb_uart.sv:932`), i.e. one clk before it
  drives — so the pad level register `lvl` never glitches mid-bit-time.
- `s` (10 bits) — the **raw** (pre-stuffing) bit index of the next bit time
  to be computed (`ext/usb_uart.sv:754`). This is the packet's identity in
  bit order.

### The raw bit stream: `f(s)`

A combinational function maps `s` to the next raw bit
(`ext/usb_uart.sv:801`–`ext/usb_uart.sv:817`):

| `s` range | bits | source |
| --- | --- | --- |
| 0–7 | sync | `00000001`, bit 7 is the trailing 1 (`ext/usb_uart.sv:804`) |
| 8–15 | PID | `{~pid[3:0], pid[3:0]}` LSB-first — the check complement is generated here (`ext/usb_uart.sv:806`–`ext/usb_uart.sv:809`) |
| 16 … payload_end−1 | payload | `tx_byte_r[(s-16) & 7]`, LSB first per byte |
| payload_end … payload_end+15 | CRC16 | wire bits `~crc[15-i]` (data PIDs only, `ext/usb_uart.sv:812`–`ext/usb_uart.sv:814`) |

`payload_end_r`/`total_bits_r` are precomputed at `tx_start`
(`ext/usb_uart.sv:876`–`ext/usb_uart.sv:881`) rather than recomputed per bit —
note `{3'b0, tx_len, 3'b0}` is just `tx_len * 8` as a shift. Handshake PIDs
(`pid[1:0] == 2'b00`, detected as `!is_data` via `is_data = pid_r[1:0]==2'b11`,
`ext/usb_uart.sv:764`) send only sync + PID + EOP.

The result is pipelined one clk into `raw_bit_r`
(`ext/usb_uart.sv:818`–`ext/usb_uart.sv:819`) — a timing measure: the big
`f(s)` select chain and the stuff/NRZI tail logic land in different clk
cycles.

### The payload producer pipeline

`tx_addr` is a pure bit-slice of `s`: `{(s-16)[9:3]}`, no state terms
(`ext/usb_uart.sv:835`–`ext/usb_uart.sv:836`) — outside `S_DATA` the value is
a don't-care, which keeps the producer mux (descriptor ROM / snapshot /
EP0 data — all inside [`usb_uart`](usb-uart.md)) off the critical path.
`tx_addr_next` (`s-9 >> 3`, `ext/usb_uart.sv:841`–`ext/usb_uart.sv:842`) is a
one-byte-ahead index that is stable for all eight bit times of a byte, so the
*producer* can prefetch with it.

`tx_byte_r` (`ext/usb_uart.sv:791`–`ext/usb_uart.sv:794`) latches the
producer's byte **once per payload byte**, when `s[2:0]==0` and `s >= 16` —
the engine is a one-bit-time lookahead, so this happens during the last bit
time of the previous byte; the registered producer then holds this byte for
all eight of its bit times.

## NRZI encoding

NRZI encoding is one line in the FSM: the drive level toggles (or not)
based on the raw bit (`ext/usb_uart.sv:914`):

```systemverilog
lvl <= raw_bit_r ? lvl : ~lvl;
```

A raw `1` keeps the level (no transition), a raw `0` toggles it
(transition) — the exact inverse of the receiver's rule in
[`usb_rx_phy`](usb-rx-phy.md). `lvl` is the current drive level, 0 = J,
1 = K (`ext/usb_uart.sv:757`), and the pad mapping is
(`ext/usb_uart.sv:845`–`ext/usb_uart.sv:846`):

```systemverilog
assign dp = (state == S_EOP0) ? 1'b0 : (lvl ? 1'b0 : 1'b1);
assign dn = (state == S_EOP0) ? 1'b0 : (lvl ? 1'b1 : 1'b0);
```

(`S_EOP0` forces both low — SE0 — see EOP below.)

### Sync generation

The sync byte `00000001` is produced by `f(0..7)` and encoded through the
same NRZI rule. From idle J, the first sync bit (`0`) toggles the line to
K — that happens in the `S_GUARD` exit
(`ext/usb_uart.sv:885`–`ext/usb_uart.sv:895`):

```systemverilog
s     <= 10'd1;     // bit time 0 (K) computed here
lvl   <= ~lvl;      // sync bit 0 is a 0: J -> K
```

…and the remaining six `0`s are all *no-transition* bit times, followed by
the final `1`'s transition — exactly the pattern
[`usb_rx_phy`](usb-rx-phy.md) detects as SOP on any receiving device.

## Bit stuffing

Stuffing is inserted by a 3-bit run counter `ones` over **raw** bits
(`ext/usb_uart.sv:916`–`ext/usb_uart.sv:924`):

- `ones` counts consecutive raw `1`s; a raw `0` clears it.
- When `ones == 5` and the current raw bit is also `1` (the sixth `1`),
  `stuff_pending` is set and `ones` resets — the *next* bit time will be a
  **stuffed 0**, regardless of what `f(s)` says.
- At the next `btc==3` edge, the `stuff_pending` branch
  (`ext/usb_uart.sv:905`–`ext/usb_uart.sv:910`) inserts a full bit time with
  a forced level toggle (`lvl <= ~lvl` — a `0` on the wire), does **not**
  consume `s`, and clears the run counter.
- The run counter starts fresh entering the PID (`s == 8`,
  `ext/usb_uart.sv:915`–`ext/usb_uart.sv:916`) and — as the header notes
  (`ext/usb_uart.sv:722`–`ext/usb_uart.sv:724`) — the sync pattern is
  exempt: `ones` is zeroed at `S_GUARD` exit (`ext/usb_uart.sv:892`).

Symmetry note: `cur_stuff` marks the *current* bit time as stuffed, and the
CRC engine is **not** advanced during it (`adv` gate at
`ext/usb_uart.sv:825`–`ext/usb_uart.sv:826`) — stuffed bits are overhead,
never part of the data.

## CRC16

Data packets append a 16-bit CRC. The engine instantiates
[`usb_crc16`](usb-crc.md) directly (`ext/usb_uart.sv:821`–`ext/usb_uart.sv:828`),
reset by `tx_start`, advanced once per payload raw bit at `btc==2` (two clks
before the level for that bit's *own* bit time is computed), fed from
`raw_bit_r`. As the header observes (`ext/usb_uart.sv:724`–`ext/usb_uart.sv:726`),
this means the LFSR's final state is stable several clks before the first
CRC wire bit is needed — a pipelining nicety, not a coincidence. Wire order
is complemented and bit-reversed: `~crc[15-i]` (`ext/usb_uart.sv:813`).

## EOP generation and output enable

When the raw stream is exhausted (`s_lt_end_r` low — the
`s < total_bits_r` compare pre-registered one clk,
`ext/usb_uart.sv:870`), the FSM moves to EOP (`ext/usb_uart.sv:926`–`ext/usb_uart.sv:931`):

- `S_EOP0`: **two bit times of SE0** (both pads driven 0 — the pad
  assigns above force it), counted by `ecnt`
  (`ext/usb_uart.sv:934`–`ext/usb_uart.sv:944`);
- `S_EOPJ`: **one bit time of J** (`lvl <= 1'b0`,
  `ext/usb_uart.sv:940`) — the bus returns to idle through the J, and only
  after that full bit time does the driver release.

Output enable is deliberately conservative: `oe_r` rises only at the
`S_GUARD` → `S_DATA` transition (`ext/usb_uart.sv:895`) — the pads are
driven starting with sync bit 0's K — and drops at the end of `S_EOPJ`
(`ext/usb_uart.sv:945`–`ext/usb_uart.sv:951`), in the same clk as the
`tx_done` pulse:

```systemverilog
oe_r   <= 1'b0;        // release the bus
done_r <= 1'b1;
state  <= S_IDLE;
```

So the device tri-states the pads exactly one bit time after the EOP's SE0,
never before, never outside a packet.

## The guard and the turnaround budget

Before SOP, `S_GUARD` burns **8 clks = 2 bit times** with the bus released
(`ext/usb_uart.sv:885`–`ext/usb_uart.sv:897`). This is the inter-packet gap
the device must leave after the host's packet before it starts responding.
The header does the arithmetic (`ext/usb_uart.sv:727`–`ext/usb_uart.sv:730`):
the response SOP lands roughly **11 clks ≈ 2.75 bit times** after the
triggering packet's EOP, comfortably inside the USB 2.0 device turnaround
window — and the guard also guarantees the PHY's own
[`usb_rx_phy`](usb-rx-phy.md) has gone idle-ward (SE0 ended) before the
transmitter grabs the bus, avoiding any drive contention on the pads.

## Handshake with the usb_uart transmit arbiter

The arbiter in [`usb_uart`](usb-uart.md) multiplexes responses from the EP0
control machine, the EP1 bulk engine, and the ROM. Its contract with this
module is:

- `tx_busy == 0` ⇒ the arbiter may pulse `tx_start` with `tx_pid`/`tx_len`
  valid and the payload source (`tx_byte` producer) selected; the FSM
  latches all three at that pulse (`ext/usb_uart.sv:873`–`ext/usb_uart.sv:884`)
  and `tx_busy` goes high (`state != S_IDLE`,
  `ext/usb_uart.sv:849`).
- Payload bytes are **pulled**: the engine presents `tx_addr` (and
  `tx_addr_next` for prefetch) and expects `tx_byte` to be the
  combinational read at that address, stable for a full byte — one byte per
  8 bit times.
- `tx_done` pulses for one clk at bus release; `tx_busy` is already low by
  then, so the arbiter can immediately arm the next response.

## Key takeaways

- `usb_tx_phy` builds complete packets as a **bit-time engine**: every bit
  time is 4 clks (`btc` 0..3), the next level is computed at the `btc==3`
  edge — one clk before it drives (`ext/usb_uart.sv:899`–`ext/usb_uart.sv:932`).
- NRZI encoding is `lvl <= raw_bit_r ? lvl : ~lvl`
  (`ext/usb_uart.sv:914`): raw `1` = no transition, raw `0` = toggle.
- Stuffing: a 3-bit run counter over raw `1`s; after six, a full stuffed-0
  bit time is inserted without consuming `s`, the CRC is not advanced
  during it, and the sync pattern is exempt
  (`ext/usb_uart.sv:905`–`ext/usb_uart.sv:924`).
- Packet layout via `f(s)`: sync `00000001` (s 0–7), complemented PID
  LSB-first (s 8–15), payload LSB-first (16…), complemented+reversed CRC16
  wire bits (data PIDs only) (`ext/usb_uart.sv:801`–`ext/usb_uart.sv:817`).
- EOP = 2 bit times SE0 + 1 bit time J, then release; `oe` spans exactly
  SOP's K through the EOP's J (`ext/usb_uart.sv:895`,
  `ext/usb_uart.sv:934`–`ext/usb_uart.sv:951`).
- The 8-clk guard puts the response SOP ~11 clks after the previous EOP —
  inside the USB turnaround window (`ext/usb_uart.sv:727`–`ext/usb_uart.sv:730`).
- Arbiter contract: `tx_start` when idle → latch pid/len → pull payload
  via `tx_addr`/`tx_addr_next` → `tx_done` at release.

Related pages: [usb-primer.md](usb-primer.md) · [system-overview.md](system-overview.md) ·
[usb-rx-phy.md](usb-rx-phy.md) · [usb-crc.md](usb-crc.md) ·
[usb-fifo.md](usb-fifo.md) · [usb-uart.md](usb-uart.md)