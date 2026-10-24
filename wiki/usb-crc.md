# USB CRCs: `usb::Crc5` and `usb::Crc16`

Every USB packet ends with a CRC (Cyclic Redundancy Check) computed over its
contents, and every receiver re-computes it to decide whether the packet is
intact. This page covers the two tiny Virdant mods that do that math in the
fomu-usb core — [`usb::Crc5`](#usbcrc5-line-by-line) and
[`usb::Crc16`](#usbcrc16-line-by-line), both in `src/usb.vir` — plus the
subtle TX and RX wiring around them (`usb::TxPhy` and `usb::RxPacket`, also
in `src/usb.vir`). If you have never met a CRC or an LFSR before, the first
three sections are for you.

- What a CRC is and [why USB needs one](#why-usb-needs-a-crc)
- The [LFSR](#what-is-an-lfsr) trick that makes it nearly free in hardware
- The exact [TX wiring](#tx-side-complement--bit-reverse) (complement + bit-reverse)
- The exact [RX wiring](#rx-side-checking-by-residual) (residual check against
  `5'h0C` / `16'h800D`) and why computed-vs-received comparison cannot work

Related pages: [USB primer](usb-primer.md), [system overview](system-overview.md),
[rx PHY](usb-rx-phy.md), [rx packet](usb-rx-packet.md), [tx PHY](usb-tx-phy.md),
[UsbUart top](usb-uart.md), [testbenches](testbenches.md).

---

## Why USB needs a CRC

USB Full-Speed moves 12 Mbit/s over an unshielded twisted pair that sits
millimetres away from noisy things: your hand, a desk, a switching power
supply. A single flipped bit is undetectable on the wire itself — the
differential receiver sees a perfectly valid 0 or 1 — so the protocol must
assume bits get corrupted and detect it *after the fact*.

A corrupted packet is not a disaster in USB because the transaction layer
makes everything retryable: if a device sees a bad token it ignores it, if a
host sees bad data it sends no ACK and the device retransmits. But retries only
work if corruption is *detected*. That is the CRC's entire job: it is a few
bits of redundant summary appended to each packet, strong enough that random
bit errors almost never slip through.

Where it is used in USB (see [USB primer](usb-primer.md) for packet anatomy):

| CRC | Covers | Packet types | Polynomial | Seed |
|-----|--------|--------------|------------|------|
| CRC5 | 11 bits: address (7) + endpoint (4) | Tokens (SOF, SETUP, IN, OUT) | x^5 + x^2 + 1 | `5'h1F` |
| CRC16 | payload bytes (0–1023 bytes, 64 here) | Data (DATA0/DATA1) | x^16 + x^15 + x^2 + 1 | `16'hFFFF` |
| — | nothing | Handshakes (ACK/NAK/STALL) | — | — |

Handshake packets are just 8 bits (PID + its check nibble), so they carry no
CRC. Tokens are small and fixed-size, so a 5-bit CRC suffices; data payloads
can be up to a kilobyte, so they get a 16-bit one.

## The CRC idea in two minutes

Think of the message as one enormous binary number, `M`. To compute a W-bit
CRC with generator polynomial `G(x)`:

1. Multiply `M` by `x^W` — that is just appending W zero bits.
2. Divide by `G(x)` using **mod-2 arithmetic**: no carries, subtraction is
   exactly XOR. So "long division" is a sequence of shifts and XORs.
3. The remainder `R` (W bits) is the CRC.

The receiver does the same division over *(message + received remainder)*.
Arithmetic fact: since `M·x^W + R` is divisible by `G(x)` when nothing was
corrupted, the receiver's division lands on a predictable value — with plain
CRCs, exactly zero. Flip almost any bit anywhere and the division goes off the
rails, landing somewhere unpredictable.

Two properties make CRCs the standard choice:

- **Strong detection for the money.** Any single flipped bit is caught, any
  error *burst* (a run of flipped bits) up to W bits long is caught, and
  CRC16 — whose polynomial happens to factor as (x+1)(x^15+x+1) — catches
  every pattern with an *odd* number of flipped bits. For random noise that
  is far better than a simple checksum ("sum the bytes"), which is blind to
  many rearrangements.
- **Costs almost nothing in hardware** — see the next section.

USB adds two twists on the textbook scheme, both visible in our constants:

- **Seed = all ones** (`5'h1F`, `16'hFFFF`). The division starts from a
  nonzero remainder. Without it, a message beginning with many zeros would
  leave the register at zero for those bits — leading zeros would be
  "undetectable" errors. With an all-ones seed, even an all-zero payload
  immediately mixes with nonzero state.
- **The CRC is complemented (inverted) before transmission.** This guarantees
  the CRC of an all-zeros payload is nonzero, and — more importantly for us —
  it is exactly what makes the receiver-side [residual check](#rx-side-checking-by-residual)
  work with a *fixed* magic constant instead of a per-packet comparison.

## What is an LFSR?

A **LFSR** (Linear Feedback Shift Register) is a shift register whose next
input bit is a XOR of some of its current bits. It is the standard hardware
shape for CRCs: instead of storing the whole message and doing a big division,
you do the division *one message bit per clock* with W flip-flops and two or
three XOR gates. The register always holds the *partial remainder* of the
long division so far.

The generic CRC-style ("Fibonacci") LFSR used here, for a serial LSB-first
stream like USB's:

```
inv = din ^ r[W-1]            // fold incoming bit against the top bit
r[k] <= r[k-1]            for most k   // shift upward
r[k] <= r[k-1] ^ inv      where the polynomial has a term x^k
r[0] <= inv                             // the x^0 term
```

- `r[W-1]` is the bit about to "fall off the top" of the division window.
  XOR-ing it with the incoming bit gives `inv`, the long-division's next
  "quotient bit": if it is 1, the polynomial is XOR-ed (subtracted, in mod-2
  terms) into the register.
- **Each non-leading term of the polynomial becomes one XOR tap.** The leading
  term `x^W` is implicit in the shift itself.

In the Virdant source each LFSR is a single `reg` whose next state is written
as one concatenated word XOR-ed with a tap word — the taps are the 1-bits in
the second `word(...)`:

```
// Crc5 (W = 5): {r[3:0], inv} with the x^2 tap XORed into bit 2
word(it[4..0], inv) ^ word(0w2, inv, 0w2)

// Crc16 (W = 16): {r[14:0], inv} with the x^15 tap XORed into bit 15
// and the x^2 tap into bit 2
word(it[15..0], inv) ^ word(inv, 0w12, inv, 0w2)
```

- `word(a, b, …)` concatenates MSB-first, so `word(it[4..0], inv)` is the
  plain shift with `inv` entering at bit 0 (`r[0] <= inv`).
- The tap word has a single possible-1 bit per polynomial term: bit 2 for
  x^2 in both CRCs, bit 15 for x^15 in CRC16. Everything else is the plain
  shift.

So "CRC with polynomial x^5+x^2+1" literally compiles to: 5 flops, 2 XOR taps,
1 shift. That is the whole mod.

> Why does the tap look "mirrored" relative to the polynomial (bit k ↔ term
> x^k, but the stream is LSB-first)? Because this LFSR form implements the
> *reciprocal-polynomial* division that matches a bit-reversed serial stream.
> The mirroring and the TX bit-reversal cancel out — a full derivation is
> beyond this page; verify behavior empirically in the
> [testbenches](testbenches.md) instead.

## `usb::Crc5` line by line

Token CRC, USB 2.0 §8.3.5, polynomial x^5+x^2+1, seed `5'h1F`. Mod `Crc5` in
`src/usb.vir`; receive-side only — the device never transmits tokens, so
`usb::RxPacket` is its only user.

```virdant
mod Crc5 {
    incoming clk : Clock
    incoming rst : Bit
    incoming adv : Bit
    incoming din : Bit
    outgoing crc : Word[5] {
        it := r
    }

    // Feedback bit: din XOR the x^5 tap (r[4]).
    wire inv : Bit {
        it := din ^ r[4]
    }

    // The 5-bit shift register. Next state = {r[3:0], inv} with the
    // x^2 tap XORed into bit 2; seeds to 0x1F.
    reg r : Word[5] on clk {
        initial it <= 31w5
        it <= when {
            case rst => 31w5
            case adv => word(it[4..0], inv) ^ word(0w2, inv, 0w2)
            else     => it
        }
    }
}
```

Walking through it:

- **`adv`** — the LFSR is *idle unless strobed*. The core clock is 48 MHz and
  a USB-FS bit lasts 4 clocks, so the receiver produces a one-clock `bit_valid`
  strobe per decoded bit (see [rx PHY](usb-rx-phy.md)) and gates `adv` with
  it. Between strobes, `r` just holds.
- **`rst` seeds `5'h1F`** (`31w5` — the all-ones initial remainder). On RX the
  reset is actually `sop || rst || bus_reset` (wired in `usb::RxPacket`), i.e.
  re-seeded at every start of packet.
- **`inv = din ^ r[4]`** — incoming bit folded against the top of the partial
  remainder (the x^5 tap).
- **The `adv` line** — the shift plus the two polynomial taps (x^0 via `inv`
  at bit 0, x^2 via the tap word).

That is the entire token-CRC engine: 5 flip-flops, 2 XORs. The `crc` output is
the raw LFSR state; in this checker-only role the caller compares it against
the residual `5'h0C` (see [RX side](#rx-side-checking-by-residual)).

## `usb::Crc16` line by line

Data-payload CRC, polynomial x^16+x^15+x^2+1, seed `16'hFFFF`. Mod `Crc16` in
`src/usb.vir`; dual-role: checker in `usb::RxPacket`, generator for TX data
packets in `usb::TxPhy`. Structurally identical to `Crc5`, one width up.

| Port | Dir | Width | Meaning |
|------|-----|-------|---------|
| `clk` | in | 1 | 48 MHz core clock |
| `rst` | in | 1 | sync reset: seed `16'hFFFF` |
| `adv` | in | 1 | advance one serial bit |
| `din` | in | 1 | serial bit, wire order (LSB first) |
| `crc` | out | 16 | current LFSR state |

The interesting lines:

```virdant
// Feedback bit: din XOR the x^16 tap (r[15]).
wire inv : Bit {
    it := din ^ r[15]
}

// The 16-bit shift register. Next state = {r[14:0], inv} with
// the x^15 tap XORed into bit 15 and the x^2 tap into bit 2;
// seeds to 0xFFFF.
reg r : Word[16] on clk {
    initial it <= 65535w16
    it <= when {
        case rst => 65535w16
        case adv => word(it[15..0], inv) ^ word(inv, 0w12, inv, 0w2)
        else     => it
    }
}
```

- The tap word `word(inv, 0w12, inv, 0w2)` puts `inv` at bit 15 (x^15) and
  bit 2 (x^2), zeros elsewhere — the polynomial has no terms between x^2 and
  x^15, so the middle 12 bits are a plain shift.
- The three taps are exactly the non-leading terms: x^0 (`inv` into bit 0),
  x^2, x^15.
- Total cost: 16 flops, 3 XORs, plus small muxing for reset/hold. Two
  `Crc16` instances (one in `usb::RxPacket`, one in `usb::TxPhy`) plus one
  `Crc5` in `usb::RxPacket` is the whole CRC budget of the device.

## TX side: complement + bit-reverse

The device never transmits tokens (only the host does), so the TX path needs
only CRC16 — instantiated in `usb::TxPhy` as `mod crc16 of Crc16`:

```virdant
mod crc16 of Crc16 {
    it.clk := clk
    it.rst := tx_start
    it.adv := (state == #Data) && (btc == 2w2) && !cur_stuff
        && is_data && (s >= 16w10) && (s < payload_end_r)
    it.din := raw_bit_r
}
```

- `rst = tx_start` re-seeds per packet.
- `adv` runs **only over payload bits** (`16 <= s < payload_end_r`, one
  advance at `btc == 2` of each non-stuffed bit time). The TX knows its own
  payload length, so it knows exactly when to stop advancing — the LFSR then
  *holds* the remainder while the engine starts transmitting it.

The transmitted bits do not come from a live mux over the LFSR: `usb::TxPhy`
computes the complemented CRC as a word and loads it into a dedicated shift
register, `crc_shift`, at the edge that consumes the last payload raw bit:

```virdant
// The CRC16 wire bits, complemented (crc_wire[i] = ~crc[i]; the
// MSB-first wire order comes from shifting crc_shift right, bit
// 15 first).
wire crc_wire : Word[16] {
    it := crc16.crc ^ 65535w16
}
```

`crc_shift` then shifts one bit per consumed raw bit (stuffed bit times do
not shift), and `raw_bit`'s CRC branch is a single register read,
`crc_shift[15]` — the output tap. Since the shift walks the word from bit 15
down, the *stream* order is `crc_wire[i] = ~crc[15-i]` with `i = 0` the first
bit on the wire: the **complement + bit-reverse** convention documented in
the `ext/usb_uart.sv` header and in the `Crc16` doc comment in `src/usb.vir`.

- **Complement (`^ 65535w16`)** — USB mandates the transmitted CRC be the
  inverted remainder.
- **Bit-reverse (`15 - i`)** — the LFSR above is a LSB-first serial machine,
  so its internal bit order is mirrored relative to the bit order USB
  specifies for the CRC on the wire; shifting the complemented word out
  MSB-first un-mirrors it.

**Worked example** (verified in simulation, see [testbenches](testbenches.md)).
A CDC `SET_LINE_CODING` request carries a 7-byte data payload —
`00 C2 01 00 00 00 08` (115200 baud, 8 data bits, 1 stop, no parity; see
[usb-serial.md](usb-serial.md)):

1. Feed the 56 payload bits (each byte LSB first) through `usb::Crc16` with
   seed `16'hFFFF` → register ends at `0xEC27`.
2. Wire bits are `~crc[15], ~crc[14], …, ~crc[0]` → the byte sent first
   (the "low" CRC byte) is `0xC8`, followed by `0x1B`.
3. On the wire: `PID, payload…, 0xC8, 0x1B, EOP`.

## RX side: checking by residual

Here is the interesting design problem. The receiver instantiates both CRCs
inside `usb::RxPacket`:

```virdant
// CRC5 advances on token bits nb = 8..23 (data + received crc5
// wire bits; residual 5'h0C).  NOT gated by pkt_err: a bad
// packet's residual is never sampled.
mod crc5 of Crc5 {
    it.clk := clk
    it.rst := sop || rst || bus_reset
    it.adv := bit_valid && (st == #Body) && !sec
        && (nb >= 8w5) && (nb < 24w5)
    it.din := bit_i
}
// CRC16 advances on every data-section bit (payload + received
// crc16 wire bits; residual 16'h800D).
mod crc16 of Crc16 {
    it.clk := clk
    it.rst := sop || rst || bus_reset
    it.adv := bit_valid && (st == #Body) && sec
    it.din := bit_i
}
```

- `rst = sop || rst || bus_reset` — re-seed at every start of packet.
- `sec` is the section flag inside the `#Body` state: 0 = PID section,
  1 = token/data section.
- `nb` counts post-sync packet bits: PID byte 0..7, token data 8..18, CRC5
  wire bits 19..23 (parked at 24 in the data section). So CRC5 advances on
  bits 8…23: the 11 token bits **plus the 5 received CRC wire bits** —
  notice it does *not* stop at the token/CRC boundary.
- CRC16 advances on **every** data-section bit: payload bits *and* the 16
  received CRC wire bits, with no boundary at all.

### Why not compute one CRC and compare with the other?

The obvious textbook check — "compute the CRC over the payload, then compare
against the 16 received CRC bits" — has a fatal flaw on the receive side:
**the receiver does not know where the payload ends and the CRC begins.** A
data packet on the wire is just `SYNC, PID, bits…, bits…, EOP`; nothing in the
packet says how many bytes are payload. The boundary is only implicitly
defined by "the last 16 bits before EOP are CRC" — and EOP arrives *after*
all the bits. So there is no mid-packet moment where the receiver could say
"snapshot the computed CRC now" — by the time it knows the payload was over,
those bits have already streamed through.

The complement-the-CRC scheme has exactly the property needed to fix this.
If the receiver feeds **every** post-PID bit — payload *and* the received CRC
wire bits — through its own identically-seeded LFSR, then for an uncorrupted
packet the final register value is a **constant** that depends only on the
polynomial, the seed, and the CRC width — never on the payload. That constant
is called the *residual*: for an intact token, feeding all 16 post-PID bits
(11 data + 5 CRC) leaves the CRC5 register at `5'h0C`; for an intact data
packet, feeding every data-section bit leaves the CRC16 register at
`16'h800D`. This is not folklore — `tb/tb_crc.v` asserts it for a set of
known-good token vectors (see
[testbenches](testbenches.md)), and for CRC16 payloads from empty to the
64-byte bulk maximum.

So the check is just an equality against a constant, sampled at EOP. The
equalities live in their own registered flags in `usb::RxPacket`:

```virdant
// CRC5 residual compare (5'h0C), registered: the 5-bit equality
// tree ends at its own flop instead of riding tk_crc_ok's cone.
reg crc5_res_r : Bit on clk { ... else => c5 == 12w5 }

// CRC16 residual compare (16'h800D), registered likewise.
reg crc16_res_r : Bit on clk { ... else => c16 == 0x800dw16 }
```

and are sampled at EOP into the per-packet verdicts: `tk_crc_ok_r` captures
`crc5_res_r` on `eop && tok_cap` (token packets), `dt_crc_ok_r` captures
`crc16_res_r` on `eop && dat_cap` (data packets).

(Tokens have fixed length, so computed-vs-received *would* be possible for
CRC5; the design still uses the residual mechanism there for uniformity —
same hardware shape, same check, one less special case.)

Two implementation details worth stealing for your own designs:

- The equalities are **registered** (`crc5_res_r` / `crc16_res_r`): a 16-bit
  compare is a long XOR tree, and ending it at its own flip-flop keeps that
  tree out of the downstream timing path. The residual is only *sampled* at
  EOP, when the LFSR has consumed the whole packet, so mid-packet values are
  harmless.
- The `rst` term includes `bus_reset` so a bus reset re-seeds both LFSRs
  even mid-packet.

### Worked example: an IN token

Host sends `IN` to device address 5, endpoint 1. Post-PID bits on the wire
(LSB first): `1,0,1,0,0,0,0` (addr 5) then `1,0,0,0` (endp 1), then the
received CRC5 wire bits `0,0,1,1,0`:

1. Feed the first 11 bits through `usb::Crc5` (seed `5'h1F`) → register
   `5'b11001` (`0x19`). (The host computed the same value and transmitted its
   complement, reversed: `~crc[4..0] = 0,0,1,1,0`.)
2. Keep feeding the 5 received wire bits → 16 bits total, matching the
   `nb < 24` window.
3. Final register: `5'h0C`. Residual check passes, token is accepted.

Corrupt **one** payload bit anywhere and the final value lands elsewhere —
e.g. flipping a single bit in a data packet whose good residual is `0x800D`
gives `0x078E` — the check fails and the packet is discarded, which makes the
host retry. That is the whole error-control story of USB in one register.

## Key takeaways

- USB protects tokens with **CRC5** (x^5+x^2+1, seed `5'h1F`) and data payloads
  with **CRC16** (x^16+x^15+x^2+1, seed `16'hFFFF`); handshakes carry none.
- A CRC is polynomial long division in mod-2 (XOR) arithmetic; an **LFSR**
  performs it serially — one input bit per `adv` strobe — costing only W
  flops plus one XOR tap per non-leading polynomial term.
- Each polynomial term x^k maps to a literal XOR tap into bit k of the shift
  register: x^0 and x^2 in `Crc5`, x^0/x^2/x^15 in `Crc16` (the tap words in
  the `adv` next-state lines of `src/usb.vir`).
- **TX** advances the LFSR over payload bits only and sends the complemented
  CRC MSB-first out of `crc_shift`, i.e. `crc_wire[i] = ~crc[15-i]` as a
  stream — complement because the spec says so, bit-reverse because the
  serial LFSR's bit order is mirrored.
- **RX** cannot know the payload/CRC boundary before EOP, so it feeds *all*
  post-PID bits through the LFSR and compares the final state (the residual)
  against the fixed constants `5'h0C` / `16'h800D` (the `crc5` / `crc16`
  instances and `crc5_res_r` / `crc16_res_r` in `usb::RxPacket`).
- The residual is constant for every uncorrupted payload — verified by
  simulation in the [testbenches](testbenches.md) — and any single-bit error
  breaks it, which is what makes host retries safe.
