# `usb::TxPhy` — bits back to pads

`usb::TxPhy` (the `TxPhy` mod in `src/usb.vir`) is the
transmitter physical layer of the Fomu USB device: the mirror of
[`usb::RxPhy`](usb-rx-phy.md). It takes a packet description (PID, optional
payload length) and drives the raw D+/D− levels with a complete USB
Full-Speed packet: **sync pattern, NRZI-encoded, bit-stuffed data, and EOP**
— plus the output-enable signal that actually connects the driver to the
pads only while transmitting. It is the "packet records -> pads" stage of
the bottom-up chain described in the file header of `src/usb.vir` and is
driven by the transmit arbiter inside [`usb_uart`](usb-uart.md). Background on NRZI, stuffing,
sync and EOP is in the [USB primer](usb-primer.md); see also
[system-overview.md](system-overview.md) for the big picture.

As with the whole core, there is a single 48 MHz clock and **4 clk cycles
per USB bit time** (the `TxPhy` doccomment header says exactly that). The
doccomment block that opens `usb::TxPhy` in `src/usb.vir` is an accurate
summary of everything below — including the load-bearing timeline
conventions — and is worth re-reading after this page.

## Ports

| Port | Dir | Width | Meaning |
| --- | --- | --- | --- |
| `clk` | in | 1 | 48 MHz system clock (4 cycles per USB bit time) |
| `rst` | in | 1 | synchronous reset (core reset from `UsbController`) |
| `tx_start` | in | 1 | 1-clk pulse: latch `tx_pid`/`tx_len` and begin the packet |
| `tx_pid` | in | 4 | PID nibble (the upper complement is generated here) |
| `tx_len` | in | 7 | payload byte count, 0..64 (0 ⇒ zero-length packet; a data PID still appends its 16 CRC wire bits) |
| `tx_byte` | in | 8 | producer payload byte: the core top's mux over registered snapshot-RAM / descriptor-ROM read stages (addressed through `tx_addr_next`) |
| `tx_addr` | out | 7 | informational byte index `(s-16)>>3`; unused by the core's instantiation — the fetch pipeline runs off `tx_addr_next` |
| `tx_addr_next` | out | 7 | byte index one payload byte ahead (`(s-9)>>3`), byte-stable across each byte's eight bit times — the fetch preload hint |
| `tx_busy` | out | 1 | high while a packet is in flight (any state but Idle); left unused by the core's instantiation, which tracks ownership itself |
| `tx_done` | out | 1 | 1-clk pulse when the bus is released |
| `dp`, `dn` | out | 1 | D+/D− drive levels (valid only while `output_enable` is high) |
| `output_enable` | out | 1 | pad drive enable |

`tx_pid`/`tx_len`/`tx_byte` come from the transmit arbiter in
[`usb_uart`](usb-uart.md), which serializes responses from [`usb::Ep0`](usb-ep0.md)
(descriptors and status — with `GET_STATUS` reading as zero bytes),
[`usb::Serial`](usb-serial.md) (the bulk-IN snapshot RAM), and the
always-NAK EP2. The drive
levels leave the core top as `dp_tx`/`dn_tx`/`output_enable` and reach the
wires through the SB_IO tristate pad cells in `ext/usb_pads.sv`
([`fomu-top`](fomu-top.md)) — one shared output enable for the pair, since
the core drives both lines together or neither.

## The bit-time engine

The module is a small FSM (the `TxPhyState` enum) with five states:

```
Idle -> Guard -> Data -> Eop0 -> EopJ -> Idle
```

Two counters time everything:

- `btc` (2 bits) — the clk phase inside the current bit time, 0..3. Every
  bit time lasts exactly 4 clks; the decisions that shape the *next* bit
  time fire when `bit_end_r` is high — a flag registered one clk ahead
  from `btc == 2`, so it is high on the final clk of the current bit time.
  Registering it keeps the phase compare out of every decision cone.
- `s` (10 bits) — the **raw** (pre-stuffing) bit index of the *next* bit
  time. `s` advances only on raw bit times (stuffed bit times do not
  consume it); it starts at 1 because bit time 0's encoding — the J→K sync
  transition — is computed at the Guard→Data edge.

The timeline convention the header calls load-bearing: **the level DURING
bit time T encodes raw bit f(S_T − 1)** — the transition for a `0` bit
appears at that bit time's start, because the level register `lvl` is
updated at the edge that ends the previous bit time and never changes
mid-bit-time.

### The raw bit stream: `f(s)`

A combinational wire `raw_bit` maps `s` to the next raw bit:

| `s` range | bits | source |
| --- | --- | --- |
| 0–7 | sync | `0`, except `s == 7` → `1` — the trailing 1 |
| 8–15 | PID | lower half: `pid_r[s mod 8]`; upper half: `~pid_r[(s mod 8) - 4]` — the check complement is generated here |
| 16 … payload_end−1 | payload | `tx_byte_r[s mod 8]`, LSB first per byte |
| payload_end … payload_end+15 | CRC16 | `crc_shift[15]` (data PIDs only) |

One index serves the PID and payload selectors because `(s-16) mod 8 ==
s mod 8`.

The packet geometry is precomputed at `tx_start` rather than recomputed per
bit: `payload_end_r = 16 + 8*tx_len` (payload raw bits are `f(16) ..
f(payload_end_r - 1)`), plus `payload_end_m1_r = payload_end_r - 1` and
`total_bits_r = 8*tx_len + 32` for a data PID (payload + CRC) or
`8*tx_len + 16` otherwise. A data PID is `pid[1:0] == 2'b11` (`is_data`);
handshake and token PIDs send only sync + PID + EOP.

The result is pipelined one clk into `raw_bit_r` — a timing measure: `f(s)`
is evaluated against the registered `s` (stable for the whole 4-clk bit
time), and the compare/select chain and the stuff/NRZI tail logic land in
different clk cycles.

### The payload producer pipeline

The producer side is a three-stage fetch pipeline in the core top:

1. `tx_addr_next` = `(s-9) >> 3` — a pure bit-slice of `s`, byte-stable
   across all eight bit times of a byte, so a registered copy can address
   the source one byte ahead without flipping mid-byte. (For `s < 9` the
   subtraction wraps in `Word[10]` and lands on 126/127, which are never
   read.)
2. `byte_rd_addr` — `tx_addr_next` registered one clk.
3. The source's own registered read stage (the snapshot RAM and descriptor
   ROM read registers live inside their mods), followed by the producer
   mux: snapshot RAM for serial data, zero bytes for `GET_STATUS`,
   descriptor ROM for everything else. The mux sees registered sources
   only.

The PHY then re-times the producer byte into `tx_byte_r`, loaded **once per
payload byte**: when `state == Data`, `btc == 0`, `s mod 8 == 0`, and
`s >= 16`. The engine reads one bit time ahead, so this fires during the
last bit time of the previous byte; the registered producer then holds this
byte for all eight of its bit times. (`tx_addr` itself is informational —
the core's instantiation marks it unused.)

## NRZI encoding

NRZI encoding is one line in the FSM: the drive level toggles (or not)
based on the raw bit, at the `bit_end_r` edge that ends a raw bit time:

```
lvl <= raw_bit_r ? lvl : ~lvl
```

A raw `1` keeps the level (no transition), a raw `0` toggles it
(transition) — the exact inverse of the receiver's rule in
[`usb_rx_phy`](usb-rx-phy.md). `lvl` is the current drive level, 0 = J,
1 = K, and the pad mapping is:

```
dp = (state == Eop0) ? 0 : (lvl ? 0 : 1)
dn = (state == Eop0) ? 0 : lvl
```

(`Eop0` forces both low — SE0 — see EOP below.)

### Sync generation

The sync pattern `00000001` is produced by `f(0..7)` and encoded through
the same NRZI rule. From idle J, the first sync bit (`0`) toggles the line
to K — that happens at the Guard→Data edge, where the FSM also sets
`s <= 1` (bit time 0's encoding is computed here). Each of the seven `0`s
then toggles the level at its bit-time start, producing the alternating
**KJKJKJKK** pattern, and the final `1` holds the level (the trailing KK) —
exactly the pattern [`usb_rx_phy`](usb-rx-phy.md) detects as SOP on any
receiving device.

## Bit stuffing

Stuffing is inserted by a 3-bit run counter `ones` over **raw** bits:

- `ones` counts consecutive raw `1`s; a raw `0` clears it.
- When `ones == 5` and the current raw bit is also `1` (the sixth `1`),
  `stuff_pending` is set and `ones` resets — the *next* bit time will be a
  **stuffed 0**, regardless of what `f(s)` says.
- At that bit time's end edge, the `stuff_pending` branch inserts a full
  bit time with a forced level toggle (`lvl <= ~lvl` — a `0` on the wire),
  does **not** consume `s`, and clears the run counter.
- The run counter restarts fresh entering the PID (`s == 8`), and the sync
  pattern is exempt: `ones` is zeroed at the Guard→Data edge.

Symmetry note: `cur_stuff` marks the *current* bit time as stuffed, and the
CRC engine is **not** advanced during it (`cur_stuff` appears inverted in
the CRC advance enable) — stuffed bits are overhead, never part of the
data.

## CRC16

Data packets append a 16-bit CRC. The engine instantiates
[`usb::Crc16`](usb-crc.md) directly, re-seeded by `tx_start` (`rst` of the
instance is tied to `tx_start`), advanced once per payload raw bit at
`btc == 2` of every non-stuffed bit time whose `s` is a payload raw-bit
index (`16 <= s < payload_end_r`), and fed from `raw_bit_r`. Because the
engine runs one bit time ahead, each payload raw bit r is consumed at the
`btc == 2` cycle of bit time r−1 — the bit time whose end decides r's NRZI
level — so the LFSR's final state is stable before the first CRC wire bit
is needed.

The wire bits are **shifted out** of a dedicated 16-bit register
`crc_shift` instead of muxed live from the LFSR: it is loaded with the
final wire-ordered CRC at the edge that consumes the last payload raw bit
(armed by `crc_load_r`, which fires when `s == payload_end_r - 1`), then
shifted right once per consumed raw bit, so the wire bit is simply
`crc_shift[15]` — MSB first. The wire convention is `crc_wire[i] = ~crc[i]`
(`crc_wire = crc16.crc ^ 65535`); stuffed bit times do not shift, and a
harmless re-load on a following stuffed edge captures an unchanged value.

## EOP generation and output enable

When the raw stream is exhausted (`s_lt_end_r` low — the
`s < total_bits_r` compare, registered one clk ahead like every big
decision cone), the FSM moves to EOP:

- `Eop0`: **two bit times of SE0** (both pads driven 0 — the pad mapping
  above forces it regardless of `lvl`), counted by `ecnt` (bit times 0
  and 1);
- `EopJ`: **one bit time of J** — `lvl` is forced low entering it, the bus
  returns to idle through the J, and only after that full bit time does
  the driver release.

Output enable is deliberately conservative: `oe_r` rises at the
Guard→Data edge — the pads are driven starting with sync bit 0's K — and
drops at the end of `EopJ`, in the same clk as the `tx_done` pulse:

```
oe_r   <= 0;        // release the bus
done_r <= 1;
state  <= Idle;
```

So the device tri-states the pads exactly one bit time after the EOP's
SE0, never before, never outside a packet.

## The guard and the turnaround budget

Before SOP, `Guard` burns **8 clks = 2 bit times** with the bus released
(`gcnt` counts, and `guard_last_r` — registered one clk ahead from
`gcnt == 6` — marks the final cycle). This is the inter-packet gap the
device must leave after the host's packet before it starts responding; the
doccomments throughout the core treat such response latencies as "well
inside the device turnaround budget", and this guard is the PHY-side half
of that margin. It also guarantees the PHY's own
[`usb_rx_phy`](usb-rx-phy.md) has gone idle-ward (SE0 ended) before the
transmitter grabs the bus, avoiding any drive contention on the pads.

## Handshake with the usb_uart transmit arbiter

The arbiter in [`usb_uart`](usb-uart.md) multiplexes responses from the
EP0 control machine, the EP1 bulk engine (`usb_serial`, via the snapshot
RAM), and the descriptor ROM — EP2 exists only to be NAKed. Its contract
with this module is:

- When idle, the arbiter may pulse `tx_start` with `tx_pid`/`tx_len`
  valid and the payload source selected; the FSM latches pid and length at
  that pulse and `tx_busy` goes high. The arbiter itself tracks ownership
  in a `tx_owner` register (none / ep0 / serial / ep2) keyed on
  `tx_start` and cleared by `tx_done` — which is why the core's
  instantiation leaves `tx_busy` unconnected.
- Payload bytes are **pulled**: the engine presents `tx_addr_next` (the
  one-byte-ahead preload hint), the fetch pipeline registers it into the
  source read addresses, and the resulting `tx_byte` is re-timed into
  `tx_byte_r` once per payload byte — one byte per 8 bit times.
- `tx_done` pulses for one clk at bus release; the arbiter can immediately
  arm the next response.

## Key takeaways

- `usb::TxPhy` builds complete packets as a **bit-time engine**: every bit
  time is 4 clks (`btc` 0..3), and the decisions shaping the next bit time
  fire on `bit_end_r` — the final clk of the current one.
- The level during bit time T encodes raw bit f(S_T − 1): NRZI is
  `lvl <= raw_bit_r ? lvl : ~lvl` — raw `1` = no transition, raw `0` =
  toggle at the next bit time's start.
- Stuffing: a 3-bit run counter over raw `1`s; after six, a full stuffed-0
  bit time is inserted without consuming `s`, the CRC is not advanced
  during it, and the sync pattern is exempt.
- Packet layout via `f(s)`: sync `00000001` (s 0–7), complemented PID
  LSB-first (s 8–15), payload LSB-first (16…), CRC16 wire bits shifted
  MSB-first out of `crc_shift` (data PIDs only).
- The CRC wire convention is `crc_wire[i] = ~crc[i]`; `crc_shift` is
  loaded with the final wire CRC at the last payload raw bit and shifted
  right, bit 15 first.
- EOP = 2 bit times SE0 + 1 bit time J, then release; `output_enable`
  spans exactly SOP's K through the EOP's J.
- The 8-clk guard (2 bit times of released bus) is the PHY-side half of
  the device turnaround margin.
- Arbiter contract: `tx_start` when idle → latch pid/len → pull payload
  via the `tx_addr_next` fetch pipeline → `tx_done` at release.

Related pages: [usb-primer.md](usb-primer.md) · [system-overview.md](system-overview.md) ·
[usb-rx-phy.md](usb-rx-phy.md) · [usb-crc.md](usb-crc.md) ·
[usb-fifo.md](usb-fifo.md) · [usb-uart.md](usb-uart.md)
