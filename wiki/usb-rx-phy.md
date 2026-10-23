# usb_rx_phy — pads to bits

`usb_rx_phy` is the receiver physical layer of the Fomu USB device. It turns the
two raw pad signals (`dp`, `dn`) into a clean stream of **decoded** (NRZI-decoded,
bit-unstuffed) serial bits, plus the packet boundary signals that the rest of the
core needs: start-of-packet, end-of-packet, per-packet error, and bus reset.
It is the "pad -> bit stream" stage in the bottom-up chain described in the file
header (`ext/usb_uart.sv:16`) and sits directly under
[`usb_rx_packet`](usb-rx-packet.md), which turns this bit stream into packet
records. If the USB terms below (NRZI, bit stuffing, sync, SE0...) are new to
you, read the [USB primer](usb-primer.md) first; the [system
overview](system-overview.md) shows where this block fits in the whole core.

There is no DLL, no PLL, and no clock crossing anywhere in this design: the whole
core runs on one **48 MHz** clock, and because USB Full-Speed bits are
**12 Mbit/s**, one bit time is exactly **4 clk cycles** (`ext/usb_uart.sv:7`).
That 4× oversampling is what makes this PHY simple enough to read in one sitting.

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
That is exactly what this module does, as its own header summarizes
(`ext/usb_uart.sv:171`–`ext/usb_uart.sv:180`).

## Ports

| Port | Dir | Width | Meaning |
| --- | --- | --- | --- |
| `clk` | in | 1 | 48 MHz system clock (4 cycles per USB bit time) |
| `rst` | in | 1 | synchronous reset, active high |
| `dp`, `dn` | in | 1 | raw D+/D− pad inputs (no local reset terms on them) |
| `bit_o` | out | 1 | decoded (unstuffed) data bit |
| `bit_valid` | out | 1 | 1-clk strobe: `bit_o` is good this cycle |
| `sop` | out | 1 | 1-clk pulse: J→K transition seen from idle (sync) |
| `eop` | out | 1 | 1-clk pulse: SE0 held ≥ 2 bit times |
| `active` | out | 1 | inside a packet (from `sop` until `eop`) |
| `pkt_err` | out | 1 | sticky per-packet error flag, cleared at `sop` |
| `bus_reset` | out | 1 | 1-clk pulse: SE0 held ≥ 2.5 µs |

`bit_o`/`bit_valid`/`sop`/`eop`/`bus_reset` are all registered, single-cycle
strobes (`ext/usb_uart.sv:304`–`ext/usb_uart.sv:310`), so downstream logic
(e.g. `usb_rx_packet`) just watches for one-clk pulses.

## Signal conditioning: synchronizers and edge history

```
ext/usb_uart.sv:195-197
    reg dp_s1, dp_s2, dn_s1, dn_s2;   // 2-FF pad synchronizers
    reg dp_p, dn_p;                   // previous-cycle sample (edge detect)
    reg lev_d, lev_n;                 // previous bit sample (NRZI decode)
```

The raw pads are asynchronous to `clk`, so each line goes through a two-flop
synchronizer (`dp_s1 -> dp_s2`, `ext/usb_uart.sv:228`–`ext/usb_uart.sv:229`).
Everything downstream reads only the synchronized versions:

```
ext/usb_uart.sv:205-206
    wire sdp = dp_s2;
    wire sdn = dn_s2;
```

`dp_p`/`dn_p` hold the *previous clk cycle's* sample
(`ext/usb_uart.sv:230`), which gives a cheap edge detector used for clock
recovery (`ext/usb_uart.sv:207`):

```
wire trans = (sdp != dp_p) || (sdn != dp_p);
```

…i.e. any change on either line is a transition. `lev_d`/`lev_n` hold the
*previous bit's sampled level* — the NRZI reference, see below.

The three fundamental line states are derived combinationally
(`ext/usb_uart.sv:208`–`ext/usb_uart.sv:210`):

| state | dp | dn | name |
| --- | --- | --- | --- |
| `se0` | 0 | 0 | SE0 — both low (EOP / reset / idle driver-off seen by host) |
| J | 1 | 0 | idle / `0` level (D+ pulled up on a FS device) |
| K | 0 | 1 | the opposite polarity |
| `se1` | 1 | 1 | SE1 — illegal, both high |

The polarity convention ("J = dp=1, dn=0; idle = J") is stated in the module
header (`ext/usb_uart.sv:180`).

## Clock recovery: 4× oversampling with a 2-bit phase counter

There is no separate received clock. Instead, a 2-bit phase counter `ph`
tracks *where inside the current bit time* we are, and any wire transition
re-aligns it (`ext/usb_uart.sv:273`–`ext/usb_uart.sv:276`):

```systemverilog
if (trans)
    ph <= 2'd0;      // clock recovery: re-align on edge
else
    ph <= ph + 2'd1;
if (ph == 2'd2) begin
    // mid-bit sample
    ...
end
```

At 48 MHz with 4 cycles per bit, the sample point `ph == 2` is the **middle**
of the bit time — maximum distance from both edges, maximum noise margin.
That's the same trick a UART uses with 4× oversampling, except here every
data transition also *resets* `ph` to 0, so drift and duty-cycle distortion
are continuously corrected rather than accumulating. A transition can only
occur at a bit boundary or a stuffed-bit boundary, so re-anchoring on it is
always legal.

Note the sample is taken on the *rising edge* where `ph` **equals** 2 —
`ph` is incremented in the same clocked block, so the sample happens on the
third cycle of each bit time.

## SOP: detecting the sync pattern

A packet begins, from idle, with a **sync pattern**: the byte `00000001`
sent LSB-first, i.e. seven `0`s then a `1`. In NRZI, seven `0`s from the
idle J state means the line starts with a transition to **K**, then no
transitions for six bit times, then a final transition back.

Rather than literally shift in the 8 bits, this PHY detects the start the
cheap way: while idle (`!act`), the first J→K transition — a K level where
idle expects J — is declared SOP (`ext/usb_uart.sv:246`–`ext/usb_uart.sv:257`):

```systemverilog
if (k) begin
    sop_r <= 1'b1;
    act   <= 1'b1;
    ph    <= 2'd0;
    lev_d <= 1'b1;      // previous level was idle J
    lev_n <= 1'b0;
    perr  <= 1'b0;      // new packet clears the error flag
end
```

The rest of the sync byte is not checked; the downstream
`usb_rx_packet` counts bits from SOP and skips the first 8 (the sync) before
the PID — see [usb_rx_packet](usb-rx-packet.md). `lev_d/lev_n` are
pre-seeded to the idle J level so the very first NRZI decode
("same level as last bit ⇒ `1`") is correct relative to idle.

## NRZI decode

The core of the decode is one line (`ext/usb_uart.sv:212`):

```systemverilog
wire bit_val = (sdp == lev_d) && (sdn == lev_n);
```

`1` if the sampled line level is *identical* to the previous bit's level
(no transition), `0` if it changed. `lev_d`/`lev_n` are then updated to
the fresh sample at the same mid-bit point (`ext/usb_uart.sv:279`–`ext/usb_uart.sv:280`).

This is textbook NRZI: no transition encodes `1`, transition encodes `0`.

## Bit unstuffing

Transmitters insert a `0` after six consecutive `1`s. The receiver must
recognize that run and *drop the next bit*. Here, a 3-bit counter
`stuff_cnt` counts consecutive decoded `1`s
(`ext/usb_uart.sv:290`–`ext/usb_uart.sv:296`):

- a decoded `1` increments `stuff_cnt`; when it reaches 5 *at the moment
  the current bit is also 1* (sixth one), `drop_state <= 1`
  (`ext/usb_uart.sv:292`–`ext/usb_uart.sv:293`);
- a decoded `0` clears the run;
- when `drop_state` is set, the **next** mid-bit sample is the stuffed bit:
  it is discarded, not emitted (`bit_valid` stays low), the counters reset,
  and — importantly — if that bit decodes to `1` it is a **protocol
  violation**: a stuffed `0` may never NRZI-decode to `1`, so the packet
  is marked bad (`ext/usb_uart.sv:282`–`ext/usb_uart.sv:288`):

```systemverilog
if (drop_state) begin
    // this decoded bit is the stuffed 0: drop it
    drop_state <= 1'b0;
    stuff_cnt  <= 3'd0;
    if (bit_val)
        perr <= 1'b1;    // 6 ones then a 1: violation
end else begin
    bit_v_r <= 1'b1;
    ...
end
```

Note the asymmetry: only *actually emitted* bits count toward the run
counter, and the stuffed bit itself is a forced transition (a decoded `0`)
so it naturally resets the run.

## EOP and bus reset: the SE0 timer

End-of-packet is **SE0 held for ≥ 2 bit times**; a bus reset is SE0 held
for **≥ 2.5 µs = 120 clks** at 48 MHz (USB 2.0 §7.1.7.5). One 7-bit counter
serves both (`ext/usb_uart.sv:238`–`ext/usb_uart.sv:244`):

```systemverilog
if (se0)
    se0_cnt <= se0_cnt + 7'd1;
else
    se0_cnt <= 7'd0;
if (se0_cnt == 7'd119)
    br_r <= 1'b1;        // bus_reset pulse
```

- **EOP** (`ext/usb_uart.sv:263`–`ext/usb_uart.sv:270`): while `act`, the first
  SE0 is *not* immediately EOP — the PHY stays armed, and only when
  `se0_cnt` reaches 7 (8 clks = 2 bit times) does it pulse `eop` and drop
  `act`. This is the "≥ 2 bit times" requirement made concrete: a one-bit-time
  glitch on both lines does not terminate the packet.
- **bus_reset** runs in parallel on the *same* counter, independent of
  `act`, via the compare at 119 (120 clks ≈ 2.5 µs). The counter saturates
  the count; a long reset keeps counting until the host releases the bus.
- The header notes the deliberate distinction (`ext/usb_uart.sv:263`–`ext/usb_uart.sv:266`):
  EOP is judged *inside* the active path, bus reset by the standalone timer.

## SE1: the illegal state

If both lines are high (`se1`, `ext/usb_uart.sv:259`–`ext/usb_uart.sv:262`),
the bus state is illegal for Full-Speed. The PHY immediately abandons the
packet: `pkt_err` is set and `act` drops. Downstream, a flagged packet is
silently discarded (see [usb-rx-packet](usb-rx-packet.md)).

## One-clk-vs-half-bit timing, as implemented

A few timing subtleties worth calling out because they are *as implemented*,
not as an idealized spec:

- **All outputs are registered strobes** (`sop_r`, `eop_r`, `bit_v_r`, `br_r`
  cleared every cycle at `ext/usb_uart.sv:233`–`ext/usb_uart.sv:236`, set in
  their events' branches). Downstream logic never needs to do edge detection.
- **Mid-bit sampling** means `bit_valid`/`bit_o` pulse on the *third* cycle
  of a nominal 4-cycle bit time, and SOP is detected from synchronized pads,
  so it is *2 synchronizer cycles late* relative to the pad — but since every
  downstream consumer works in the same `clk` domain and only counts strobes,
  the absolute delay is irrelevant; only *strobe ordering* matters, and the
  PHY guarantees SOP is pulsed before the packet's first `bit_valid`.
- **`eop` co-inside with dropping `act`** (`ext/usb_uart.sv:267`–`ext/usb_uart.sv:269`):
  `active` goes low in the same cycle the `eop` pulse fires, so a consumer
  watching `active` alone would see the packet end one cycle "for free".
- **`se0_cnt == 7`** means the EOP pulse fires exactly on the *8th*
  consecutive SE0 clk — i.e. exactly 2 bit times of SE0, the minimum legal
  EOP width, with no margin for error. In this single-clock FPGA design with
  short pad routes this is acceptable; a discrete-transceiver design would
  typically add margin.

## What the next layer sees

Per packet, the consumer ([`usb_rx_packet`](usb-rx-packet.md), and through
it the [usb_uart](usb-uart.md) router) sees:

- a `sop` pulse;
- a sequence of `bit_valid`/`bit_o` strobes, one per *decoded* bit (stuffed
  bits already removed — nothing downstream ever sees a stuffed bit, which
  is why the CRC engines in [usb_crc.md](usb-crc.md) can be fed directly);
- either an `eop` pulse (good or bad termination — check `pkt_err`) or a
  `pkt_err`-flagged early abort;
- occasionally a `bus_reset` pulse, which resets the whole device-side
  protocol state.

## Key takeaways

- `usb_rx_phy` = synchronizers + 4× oversampling clock recovery + NRZI
  decode + bit unstuff + SOP/EOP/reset detection, all in one 48 MHz clock
  domain (`4 clk = 1 USB bit`).
- Clock recovery is a 2-bit phase counter re-aligned to 0 on *any* `dp/dn`
  transition, sampled at `ph == 2` — mid-bit (`ext/usb_uart.sv:273`–`ext/usb_uart.sv:277`).
- SOP is simply a J→K level seen while idle (`ext/usb_uart.sv:250`); the
  rest of the sync byte is skipped by bit counting downstream.
- NRZI decode is "same level as previous bit ⇒ `1`"
  (`ext/usb_uart.sv:212`); unstuffing drops the bit after six decoded `1`s
  and flags a violation if it decodes to `1` (`ext/usb_uart.sv:282`–`ext/usb_uart.sv:287`).
- EOP = SE0 ≥ 8 clks (`se0_cnt == 7`); bus reset = SE0 ≥ 120 clks
  (`se0_cnt == 119`), same 7-bit counter, two thresholds
  (`ext/usb_uart.sv:238`–`ext/usb_uart.sv:244`, `ext/usb_uart.sv:263`–`ext/usb_uart.sv:270`).
- All outputs are 1-clk registered strobes; `pkt_err` is sticky within a
  packet and clears at SOP; `active` spans exactly SOP→EOP.

Related pages: [usb-primer.md](usb-primer.md) · [system-overview.md](system-overview.md) ·
[usb-rx-packet.md](usb-rx-packet.md) · [usb-tx-phy.md](usb-tx-phy.md) ·
[usb-crc.md](usb-crc.md) · [usb-uart.md](usb-uart.md)