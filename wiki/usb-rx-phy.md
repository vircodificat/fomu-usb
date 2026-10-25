# `usb::RxPhy` — pads to bits

`usb::RxPhy` (the `RxPhy` mod in `src/usb.vir`) is the receiver physical layer
of the Fomu USB device. It turns the two raw pad signals (`dp`, `dn`) into a
clean stream of **decoded** (NRZI-decoded, bit-unstuffed) serial bits, plus the
packet boundary signals that the rest of the core needs: start-of-packet,
end-of-packet, per-packet error, and bus reset. It is the "pad -> bit stream"
stage in the bottom-up chain described in the file header of `src/usb.vir`, fed
by the pad cells of `usb::UsbPads` (body in `ext/usb_pads.sv`) and sitting
directly under [`usb::RxPacket`](usb-rx-packet.md), which turns this bit stream
into packet records. If the USB terms below (NRZI, bit stuffing, sync, SE0...)
are new to you, read the [USB primer](usb-primer.md) first; the [system
overview](system-overview.md) shows where this block fits in the whole core.

There is no DLL, no PLL, and no clock crossing anywhere in this design: the
whole core runs on one **48 MHz** clock, and because USB Full-Speed bits are
**12 Mbit/s**, one bit time is nominally **4 clk cycles** (the first doc
comment of `usb::RxPhy`). That 4× oversampling is what makes this PHY simple
enough to read in one sitting.

## What a USB wire carries

USB does **not** put bits on the wire as a naive 0/1 voltage level. Two
transformations happen first:

- **NRZI encoding** — a `0` bit is a *transition* on the wire, a `1` bit is
  *no transition*. So the line level itself carries no bit value; only
  level *changes* do.
- **Bit stuffing** — after six consecutive `1`s on the wire (i.e. six bit
  times with no transition), the transmitter inserts a forced `0`
  (transition), guaranteeing edges frequently enough for the receiver to
  stay aligned. The receiver must **drop** this stuffed bit.

The receiver PHY therefore has to: recover the bit clock from transitions,
sample each bit mid-bit, NRZI-decode, un-stuff, and detect the packet
boundaries — sync pattern at the start, SE0 (both lines low) at the end.
That is exactly what this mod does, as its own doc comments summarize.

## Ports

| Port | Dir | Width | Meaning |
| --- | --- | --- | --- |
| `clk` | in | 1 | 48 MHz system clock (4 cycles per USB bit time) |
| `rst` | in | 1 | synchronous reset (core reset from `UsbController`; deliberately **not** asserted on bus reset — the bus-reset detector lives here) |
| `dp`, `dn` | in | 1 | raw D+/D− pad samples (SB_IO `D_IN_0` via `usb::UsbPads`; asynchronous to `clk`) |
| `bit_o` | out | 1 | decoded (unstuffed) data bit |
| `bit_valid` | out | 1 | 1-clk strobe: `bit_o` is good this cycle |
| `sop` | out | 1 | 1-clk pulse: J→K transition seen from idle (sync) |
| `eop` | out | 1 | 1-clk pulse: SE0 held ~1.75 bit times |
| `active` | out | 1 | inside a packet (from `sop` until `eop`) |
| `pkt_err` | out | 1 | sticky per-packet error flag (SE1, or stuff violation), cleared at `sop` |
| `bus_reset` | out | 1 | 1-clk pulse: SE0 held ≥ 2.5 µs |

`bit_o`/`bit_valid`/`sop`/`eop`/`bus_reset` are all registered, single-cycle
strobes (`sop_r`, `eop_r`, `bit_v_r`, `br_r` — default low, pulsed by their
event), so downstream logic (e.g. `usb::RxPacket`) just watches for one-clk
pulses.

## Signal conditioning: synchronizers and edge history

The raw pads are asynchronous to `clk` — the host's bit rate is only specified
to ±0.25%, which is why clock recovery below re-aligns on every transition —
so each line goes through a canonical two-flop synchronizer (the
`builtin::Sync` instances `sync_dp` and `sync_dn`). Everything downstream
reads only the synchronized versions:

```
wire sdp = sync_dp.out;   // synchronized D+
wire sdn = sync_dn.out;   // synchronized D-
```

A NB worth knowing: `builtin::Sync` has no reset, so the synchronizers track
the pads from cycle 0 — including through the ~175 ms power-on reset. The
detector state does not free-run then, though: `rst` parks every reg (the
SE0 timer `se0_cnt` included), so no `bus_reset` can pulse while the core is
held in reset.

`dp_p`/`dn_p` hold the *previous clk cycle's* synchronized sample, which gives
a cheap edge detector:

```
wire trans = (sdp != dp_p) || (sdn != dn_p);
```

…i.e. any change on either line is a transition. `lev_d`/`lev_n` hold the
*previous bit's sampled level* — the NRZI reference, see below.

The three fundamental line states are derived combinationally:

| state | dp | dn | name |
| --- | --- | --- | --- |
| `se0` | 0 | 0 | SE0 — both low (EOP / reset / idle driver-off seen by host) |
| J | 1 | 0 | idle / `0` level (D+ pulled up on a FS device) |
| K | 0 | 1 | the opposite polarity |
| `se1` | 1 | 1 | SE1 — illegal, both high |

The polarity convention ("J = dp=1, dn=0; idle = J") is baked into the reset
values of `lev_d`/`lev_n` (J = D+ high, D− low) and the `k`/`se0`/`se1` wires.

## Clock recovery: 4× oversampling with a 2-bit phase counter

There is no separate received clock. Instead, a 2-bit phase counter `ph`
tracks *where inside the current bit time* we are, and any wire transition
re-aligns it:

```
reg ph : Word[2] on clk {
    case rst       => 0w2
    case sop_edge  => 0w2
    case jk        => mux(trans, 0w2, it + 1w2)   // re-align on edge
    else           => it
}
```

The mid-bit sample fires when `ph` has counted to **1** (the `sample` wire,
`jk && ph == 1w2`). Because the re-align zeroes `ph` one clk after a
transition reaches the synchronized pads, the phase-1 capture lands 2–3 clks
after the switch reaches `sdp` — inside the bit's visibility window for
*every* achievable quantized window width (3, 4 or 5 clks), so exactly one
sample lands per bit window whether a given bit time quantizes to 4n−1, 4n or
4n+1 clks. That's the same trick a UART uses with 4× oversampling, except
here every data transition also *resets* `ph` to 0, so drift and duty-cycle
distortion are continuously corrected rather than accumulating. A transition
can only occur at a bit boundary or a stuffed-bit boundary, so re-anchoring
on it is always legal.

Note that `ph` only advances while in-packet at a J/K level (`jk`); the SOP
edge also zeroes it, so every packet starts sampling from a known phase.

## SOP: detecting the sync pattern

A packet begins, from idle, with a **sync pattern**: the byte `00000001`
sent LSB-first, i.e. seven `0`s then a `1`. In NRZI, seven `0`s from the
idle J state means the line starts with a transition to **K**, then no
transitions for six bit times, then a final transition back.

Rather than literally shift in the 8 bits, this PHY detects the start the
cheap way: while idle (`!act`), a K level — idle expects J — is declared SOP:

```
wire sop_edge = !act && k;
```

The `sop_edge` case then arms the packet in one shot: `act` is set, `ph` is
zeroed, and `perr` is cleared (new packet, clean slate). The rest of the sync
byte is not checked; the downstream `usb::RxPacket` counts bits from SOP and
skips the first 8 (the sync) before the PID — see
[usb_rx_packet](usb-rx-packet.md).

## NRZI decode

The core of the decode is one line:

```
wire bit_val = (sdp == lev_d) && (sdn == lev_n);
```

`1` if the sampled line level is *identical* to the previous bit's level
(no transition), `0` if it changed. `lev_d`/`lev_n` are parked at the idle-J
level by `rst` and updated to the fresh sample at every mid-bit sample; they
are never re-seeded at SOP, which is harmless — the first sync samples decode
against whatever the previous packet left, and the sync byte is skipped
downstream anyway.

This is textbook NRZI: no transition encodes `1`, transition encodes `0`.

This logic lives in its own mod, `usb::NrziDecoder` (instantiated as
`nrzi_decoder` inside `RxPhy`). It owns `lev_d`/`lev_n` and the captured
bit, and exposes two outputs with a distinction worth internalizing:
`bit_live` is the *combinational* decode of the line levels as they are
right now -- only meaningful at sample instants, and consumed at the
sample edge by same-edge decision logic (the destuffer, `error`).
`bit_sampled` is the *registered* capture of that decode on the sample
edge -- the stable, per-bit-time answer that downstream layers read
(exposed through `RxPhy`'s `bit_o` port). The parent keeps ownership of
the pad synchronizers and the clock recovery that generates the
`sample` strobe.

## Bit unstuffing

Transmitters insert a `0` after six consecutive `1`s. The receiver must
recognize that run and *drop the next bit*. Here, a 3-bit counter
`stuff_cnt` counts consecutive decoded `1`s at the mid-bit samples:

- a decoded `1` increments `stuff_cnt`; when the count reaches 5 *and the
  current bit is also 1* (the sixth one), `drop_state` is armed;
- a decoded `0` clears the run;
- when `drop_state` is set, the **next** mid-bit sample is the stuffed bit:
  it is discarded, not emitted (`bit_valid` stays low), the counters reset,
  and — importantly — if that bit decodes to `1` it is a **protocol
  violation**: a stuffed `0` may never NRZI-decode to `1`, so the packet
  is marked bad (`perr`).

The counter is 3 bits wide with just enough headroom: the drop sample zeroes
it, so it stops at 6 and never wraps. Note the asymmetry: the run counter only
advances at mid-bit samples (and only in-packet), and the drop sample itself
zeroes it — the stuffed bit is a forced transition (a decoded `0`), so it
naturally resets the run.

This logic also lives in its own mod, `usb::BitDestuffer` (instantiated as
`unstuff` inside `RxPhy`). It owns `stuff_count`/`drop_state` and the
`bit_valid` gate, takes the decoder's live decode (`bit_live`) as input,
and raises a combinational `violation` output at the offending sample;
the parent's `error` register latches that term (alongside the SE1/abort
term) until the next SOP or reset.

## EOP and bus reset: the SE0 timer

End-of-packet is SE0 held for ~1.75 bit times; a bus reset is SE0 held for
**≥ 2.5 µs = 120 clks** at 48 MHz (USB 2.0 §7.1.7.5). One 7-bit counter
serves both — the standalone `se0_cnt` reg, which just counts consecutive
SE0 clks:

```
reg se0_cnt : Word[7] on clk {
    case rst => 0w7
    case se0 => it + 1w7
    else     => 0w7
}
```

- **EOP** (`eop_now = act && se0 && se0_cnt == 7`): while `act`, the first
  SE0 is *not* immediately EOP — the PHY stays armed, and only once `se0_cnt`
  has counted 7 consecutive SE0 clks (~1.75 bit times) does it pulse `eop`
  and drop `act`. This is the "SE0 must persist" requirement made concrete:
  a one-bit-time glitch on both lines does not terminate the packet.
- **bus_reset** runs in parallel on the *same* counter, independent of
  `act`, via the compare `se0_cnt == 119` (120 SE0 clks ≈ 2.5 µs). The 7-bit
  counter *wraps* rather than saturating, so a sustained reset SE0 re-pulses
  `bus_reset` every 128 clks — and the EP0 FSM downstream treats each pulse
  identically.
- The deliberate distinction is worth noting: EOP is judged inside the
  active path (`eop_now` requires `act`), bus reset by the standalone timer
  (which runs whenever `rst` is low, in-packet or not).

## SE1: the illegal state

If both lines are high (`se1`), the bus state is illegal for Full-Speed. The
PHY immediately abandons the packet: `abort = act && se1` drops `act` and
sets `pkt_err` in the same cycle. Downstream, a flagged packet is silently
discarded (see [usb-rx-packet](usb-rx-packet.md)).

## One-clk-vs-bit timing, as implemented

A few timing subtleties worth calling out because they are *as implemented*,
not as an idealized spec:

- **All outputs are registered strobes** (`sop_r`, `eop_r`, `bit_v_r`,
  `br_r` default low, set in their events' branches; `bit_r` holds the last
  decoded value). Downstream logic never needs to do edge detection.
- **Phase-1 capture** means `bit_valid`/`bit_o` pulse 2–3 clks after the
  bit's transition reaches the synchronized pads, and SOP is detected from
  synchronized pads, so it is *2 synchronizer cycles late* relative to the
  pad — but since every downstream consumer works in the same `clk` domain
  and only counts strobes, the absolute delay is irrelevant; only *strobe
  ordering* matters, and the PHY guarantees SOP is pulsed before the
  packet's first `bit_valid`.
- **`eop` co-insides with dropping `act`**: `eop_now` both sets `eop_r` and
  clears `act`, so `active` goes low in the same cycle the `eop` pulse
  fires — a consumer watching `active` alone would see the packet end one
  cycle "for free".
- **`se0_cnt == 7`** means the EOP pulse fires after ~1.75 bit times of SE0
  (the strobe appears on the 8th consecutive SE0 clk) — right around the
  minimum legal EOP width, with little margin. In this single-clock FPGA
  design with short pad routes this is acceptable; a discrete-transceiver
  design would typically add margin.
- **`bit_r` is captured at every sample, including the dropped stuff bit** —
  the stuffing-violation check reads that same value at the drop sample, so
  `bit_o` is meaningful even in the cycle `bit_valid` is suppressed.

## What the next layer sees

Per packet, the consumer ([`usb::RxPacket`](usb-rx-packet.md), and through
it the [usb_uart](usb-uart.md) core top) sees:

- a `sop` pulse;
- a sequence of `bit_valid`/`bit_o` strobes, one per *decoded* bit (stuffed
  bits already removed — nothing downstream ever sees a stuffed bit, which
  is why the CRC engines in [usb_crc.md](usb-crc.md) can be fed directly);
- either an `eop` pulse (good or bad termination — check `pkt_err`) or a
  `pkt_err`-flagged early abort;
- occasionally a `bus_reset` pulse, which resets the whole device-side
  protocol state.

## Key takeaways

- `usb::RxPhy` = synchronizers + 4× oversampling clock recovery + NRZI
  decode + bit unstuff + SOP/EOP/reset detection, all in one 48 MHz clock
  domain (`4 clk = 1 USB bit`).
- Clock recovery is a 2-bit phase counter re-aligned to 0 on *any* `dp/dn`
  transition (and at SOP); the phase-1 capture (`sample = jk && ph == 1`)
  lands inside the bit's visibility window for every achievable quantized
  window width, so exactly one sample lands per bit.
- SOP is simply a K level seen while idle (`sop_edge = !act && k`); the rest
  of the sync byte is skipped by bit counting downstream.
- NRZI decode is "same level as previous bit ⇒ `1`"
  (`bit_val = (sdp == lev_d) && (sdn == lev_n)`); unstuffing drops the bit
  after six decoded `1`s and flags a violation if it decodes to `1`.
- EOP = SE0 counted for 7 clks (`se0_cnt == 7`); bus reset = SE0 counted for
  120 clks (`se0_cnt == 119`, re-pulsing every 128 clks on sustained SE0) —
  same 7-bit counter, two thresholds.
- All outputs are 1-clk registered strobes; `pkt_err` is sticky within a
  packet (SE1 or stuff violation) and clears at SOP; `active` spans exactly
  SOP→EOP.

Related pages: [usb-primer.md](usb-primer.md) · [system-overview.md](system-overview.md) ·
[usb-rx-packet.md](usb-rx-packet.md) · [usb-tx-phy.md](usb-tx-phy.md) ·
[usb-crc.md](usb-crc.md) · [usb-uart.md](usb-uart.md)