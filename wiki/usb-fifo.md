# `usb_fifo16x8`: the byte FIFO

Computers are full of places where one part of a system produces data at a
rate another part cannot match, and the standard cure is a **FIFO** — a
first-in, first-out queue that buffers the difference. This page explains
FIFOs from scratch and walks through `usb_fifo16x8` (`ext/usb_uart.sv`,
module at line 72), the synchronous byte-width FIFO that the USB core uses
as its endpoint buffers.

Related pages: [system overview](system-overview.md), [USB primer](usb-primer.md),
[usb_uart top](usb-uart.md), [usb_serial (EP1 bulk)](usb-serial.md),
[echo FIFO](echo-fifo.md), [rx packet](usb-rx-packet.md), [tx PHY](usb-tx-phy.md),
[testbenches](testbenches.md), [build flow](build-flow.md).

---

## What a FIFO is

A **FIFO** (First In, First Out) is the hardware equivalent of a queue at a
counter: bytes enter in order, leave in the same order, and the ones that
arrive when the consumer is busy wait inside. It is the standard cure for a
**rate mismatch**:

- **Producer faster than consumer** (for a while): the extra data waits in
  the FIFO instead of being lost.
- **Consumer idle while producer bursts**: the consumer drains the backlog at
  its leisure.
- **Flow control**: once the FIFO is *full* the producer can be told to stop
  (in USB, by NAK-ing — more below), and once *empty* the consumer knows there
  is nothing to do.

The classic implementation is a **circular buffer** — three pieces of state
plus memory:

```
mem[0..DEPTH-1]   the storage (here: bytes)
wptr              write pointer: where the next push lands
rptr              read pointer: where the next pop comes from
cnt               occupancy: how many valid entries are in there
```

A push writes `mem[wptr]` and bumps `wptr`; a pop reads `mem[rptr]` and bumps
`rptr`. The pointers count in a *ring*: after writing the last slot they wrap
back to 0, so a 16-deep FIFO uses a 4-bit pointer that overflows into itself
naturally. `cnt` tracks the difference (push without pop: +1; pop without
push: −1), and the two conditions everyone needs fall straight out of it:

- `full  = (cnt == DEPTH)`
- `empty = (cnt == 0)`

That is all a synchronous FIFO is. `usb_fifo16x8` is a compact, single-clock
version of exactly this.

## Why this design needs FIFOs

USB bulk transfers are **bursty**: the host may fire a 64-byte bulk-OUT packet
that arrives back-to-back at 12 Mbit/s, while the consumer on the board — the
[echo server](echo-fifo.md) — drains bytes at 115200 baud, roughly 100×
slower. The architecture (see [system overview](system-overview.md)) absorbs
this with the two endpoint FIFOs of the core top `usb::UsbUart`
(`src/usb.vir`):

- **`rxfifo`** — the **EP1 bulk-OUT buffer** (`mod rxfifo of RxFifo`,
  `src/usb.vir`): every byte the host sends to the device lands here until
  the byte-stream consumer takes it. 64 deep, stored in block RAM
  (`DEPTH=64, AW=6, RAM=1`).
- **`txfifo`** — the **EP1 bulk-IN buffer** (`mod txfifo of TxFifo`): bytes
  produced locally (echoed data) queue here until the host polls EP1 IN and
  the [TX engine](usb-tx-phy.md) can serialize them. Default parameters:
  16 deep × 8 bits wide, stored in flops.

Flow control closes the loop: when the consumer is slow, `rxfifo` fills; when
it can no longer accept a whole incoming packet, the [serial
endpoint](usb-serial.md) NAKs the host, and the host retries later. No bytes
are lost and nothing needs an "interrupt + emergency drain" path.

## Where the FIFOs live in the module tree

The FIFO itself is plain Verilog: `usb_fifo16x8` is an ext body in
`ext/usb_uart.sv` (module at line 72). Virdant cannot put parameters on an
`ext mod` declaration, so the two shapes the core needs are baked into two
thin wrapper ext bodies in the same file:

- `\usb::TxFifo` (line 147) — instantiates `usb_fifo16x8` at its defaults
  (`DEPTH=16, AW=4, RAM=0`): 16 deep, flop storage, combinational read head.
- `\usb::RxFifo` (line 165) — instantiates it with `DEPTH=64, AW=6, RAM=1`:
  64 deep, block-RAM storage, registered read head.

On the Virdant side (`src/usb.vir`) these appear as `ext mod TxFifo` /
`ext mod RxFifo`, and `usb::UsbUart` instantiates them as `mod txfifo` /
`mod rxfifo`. The wrappers add nothing — the behavior contract below is the
`usb_fifo16x8` header's.

## Parameters and ports

Module declaration (`ext/usb_uart.sv:72`–`86`):

```systemverilog
module usb_fifo16x8 #(
    parameter DEPTH = 16,
    parameter AW    = 4,
    parameter RAM   = 0
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        wr_en,
    input  wire [7:0]  wr_data,
    input  wire        rd_en,
    output wire [7:0]  rd_data,
    output wire [6:0]  count,
    output wire       full,
    output wire       empty
);
```

| Port | Dir | Width | Meaning |
|------|-----|-------|---------|
| `clk` | in | 1 | 48 MHz core clock (everything is synchronous to it) |
| `rst` | in | 1 | sync reset: clears `wptr`, `rptr`, `cnt` (memory contents are *not* cleared — fine, since the FIFO is empty) |
| `wr_en` | in | 1 | push one byte (`wr_data`) this clock. **Contract: caller must gate with `!full`** — see below |
| `wr_data` | in | 8 | byte to push |
| `rd_en` | in | 1 | pop one byte this clock. **Contract: caller must gate with `!empty`** — see below |
| `rd_data` | out | 8 | popped byte: combinational head (`RAM=0`) or head delayed one clock (`RAM=1`) |
| `count` | out | 7 | occupancy `cnt`, for room arithmetic by the caller (7 bits wide regardless of `DEPTH`) |
| `full` | out | 1 | `cnt == DEPTH` |
| `empty` | out | 1 | `cnt == 0` |

**Parameterization.** The data width is fixed at 8 bits — it is a *byte*
FIFO (the `16x8` in the name is "16 deep × 8 wide"), so there is no `WIDTH`
knob. What you can tune:

- **`DEPTH`** — number of slots (default 16). Both instantiation sites use
  the two fixed wrappers above: 16 deep on the tx side, 64 deep on the rx
  side (why, below).
- **`AW`** — address/pointer width, `log2(DEPTH)` (4 for 16, 6 for 64). Kept
  explicit rather than computed from `DEPTH` so pointer widths stay obvious
  at each instantiation.
- **`RAM`** — the storage-target selector, i.e. the "BRAM vs distributed"
  knob that other cores often call `TARGET` or `RAM_STYLE`:

| | `RAM=0` (distributed, flops) | `RAM=1` (block RAM) |
|---|---|---|
| storage | `(* no_ram *) reg [7:0] mem[...]` in LUTs/FFs | `(* ram_style = "block" *) reg [7:0] mem[...]` |
| `rd_data` | combinational: `assign rd_data = mem[rptr]` | registered read `rd_q <= mem[rptr]` — popped byte valid **one clock after** `rd_en` |
| cost | 8 × DEPTH flip-flops + per-byte write decode | one iCE40 BRAM block, no flop cost |
| used by | `txfifo` (16 deep = 128 flops — cheap) | `rxfifo` (64 deep = 512 flops — moved, with their decode cones, out of the fabric) |

Both variants are selected by a `generate` block (`g_ram` / `g_ff`,
`ext/usb_uart.sv:97`–`115`) so the pointer/count logic is shared verbatim.

## Push/pop semantics — read this part twice

The defining property of this FIFO is in its header (`ext/usb_uart.sv:60`–`66`):
**it has *no* internal over/underflow protection at all.**

```systemverilog
wire do_wr = wr_en;      // no cnt term here!
wire do_rd = rd_en;      // no cnt term here either!
```

A push while full or a pop while empty silently corrupts `cnt` and the
pointers. Both enables are the caller's responsibility. Why would anyone
design it that way? Timing. If the FIFO checked `full`/`empty` internally,
the `cnt` comparisons would sit in the write/read clock-enable cones — on
every cycle, for every byte. On the 64-deep bulk-OUT FIFO that cone is wide,
and at 48 MHz it is the kind of thing that makes nextpnr timing fail (the
header calls it "a hard requirement at 48 MHz"). Instead, the *caller* makes
the "do I have room?" decision — usually in a register's next-state — and
presents an already-gated `wr_en`/`rd_en`. The two instantiation sites in
`usb::UsbUart` (`src/usb.vir`) both comply, each in its own way:

- **`rxfifo`**: `wr_en` is the registered strobe `rx_push_run_d`, pre-gated
  with `!rxfifo.full` inside `rx_push_run`'s own next-state (and the accept
  already required `cnt + dt_len <= 64`); `rd_en` is the registered pop
  decision `rx_pop_d`, pre-gated with `!rxfifo.empty`.
- **`txfifo`**: the write gate is the registered `txfifo_wr_d`
  (`= tx_strobe && !txfifo.full`), so the full-compare stays out of every
  byte's write-enable cone. A registered gate is safe here (the overflow
  scenario — two pushes within 1 clk — cannot occur: the echo server pushes
  at most one byte per ~417 clks and is the FIFO's only writer). `rd_en` is
  the snapshot loader's pop `load_run` — the tx FIFO's *only* popper, and it
  pops exactly `min(count, 64)` bytes **counted at arm**, so it can never
  meet an empty FIFO mid-load (count only grows during the load).

This is a **contract**, not a suggestion: misuse corrupts the FIFO rather
than dropping a byte politely. If you reuse this module, gate your writes
*and* your pops.

**Simultaneous push+pop is supported** (`ext/usb_uart.sv:127`–`131`):

```systemverilog
case ({do_wr, do_rd})
    2'b10: cnt <= cnt + 7'd1;   // push only
    2'b01: cnt <= cnt - 7'd1;   // pop only
    default: ;                 // both (net 0) or neither
endcase
```

Both in the same clock is the common case mid-stream, and the ring makes it
safe — the pointers move past each other independently, `cnt` unchanged.

**`count` and room math.** Callers reason about *room*, not occupancy.
`usb::UsbUart` computes `tx_room = 16w7 - txfifo.count` and passes it to
`endpoint::Serial` (which currently leaves it unused — arm sizing reads
`txfifo.count` directly), and the whole-packet admission check computes
`rx_sum = rxfifo.count + rxpkt.dt_len` and accepts an OUT packet only when
`rx_sum <= 64` (`rx_room_ok`, `src/usb.vir`) — see below.

### The one-clock read latency of `RAM=1`

With `RAM=1` the read port is *free-running*: `rd_q <= mem[rptr]` happens
every clock regardless of `rd_en` (`ext/usb_uart.sv:104`), because block RAMs
have registered read ports — a combinational `mem[rptr]` would not map to a
BRAM primitive. Consequence: after your `rd_en`, the popped byte appears on
`rd_data` one clock *later*. The consumer must pair the data with a matching
valid strobe delayed the same way. The core does exactly this on the rx side:
`rx_valid_r` and `rx_valid_rr` delay the pop decision so `rx_valid_rr` lines
up with the BRAM's output held in `rx_data_d` (`src/usb.vir`). Forgetting
this pairing is the classic bug when switching a FIFO from flops to BRAM.

## Line-by-line tour

- `ext/usb_uart.sv:87`–`88` — state: two `AW`-bit pointers and a 7-bit
  count (7 bits is enough for any `DEPTH` up to 127).
- `ext/usb_uart.sv:94`–`95` — the ungated internal strobes `do_wr`/`do_rd`
  discussed above.
- `ext/usb_uart.sv:97`–`115` — `generate` split between BRAM storage
  (`g_ram`, free-running registered read) and flop storage (`g_ff`,
  combinational read).
- `ext/usb_uart.sv:117`–`133` — the pointer/count sequencer. On reset all
  three registers clear; otherwise each pointer does a wrapping `+1`
  (`wptr <= wptr + 1` written as a sign-extended constant, lines 123–126)
  when its strobe fires, and the `case` updates `cnt`. Note reset clears
  pointers and count but not `mem` — harmless because `cnt==0` means no
  slot is considered valid.
- `ext/usb_uart.sv:135`–`137` — outputs: `count = cnt`,
  `full = (cnt == DEPTH[6:0])`, `empty = (cnt == 7'd0)`.

## How `usb::UsbUart` uses the two wrappers

```systemverilog
// src/usb.vir, mod rxfifo of RxFifo
mod rxfifo of RxFifo {
    it.clk     := clk
    it.rst     := rst || bus_reset
    it.wr_en   := rx_push_run_d
    it.wr_data := rxpkt.dt_rd_data
    it.rd_en   := rx_pop_d
}

// src/usb.vir, mod txfifo of TxFifo
mod txfifo of TxFifo {
    it.clk     := clk
    it.rst     := rst || bus_reset
    it.wr_en   := txfifo_wr_d
    it.wr_data := tx_data
    it.rd_en   := load_run
}
```

**Why `rxfifo` is 64 deep.** USB bulk endpoints ACK a packet only if they
can take *all* of it — a partial accept would lose bytes. A maximum-size
bulk-OUT packet is 64 bytes, so the buffer must be able to absorb 64 bytes
at once; a 16-deep FIFO can never accept one under this
"whole-packet-must-fit" rule. 64 also matches what a real device's endpoint
buffer holds. Going BRAM (`RAM=1`) takes 512 flops and their per-byte
write-decode cones out of the scarce iCE40 fabric (see
[build flow](build-flow.md) for how Yosys honors the `ram_style` attribute).

The rule is enforced by the caller, not the FIFO: `rx_room_ok` in
`usb::UsbUart` (`src/usb.vir`) adds the packet length to the current fill —
`rx_sum = rxfifo.count + rxpkt.dt_len <= 64` — and passes it to
`endpoint::Serial` as `rx_room`, which registers the accept/reject decision one
clk after the data packet arrives: accept (ACK) only on a good CRC, the
expected PID, and room for the whole packet; NAK otherwise, and the host
retries. Bytes never drop.

**Why `txfifo` stays 16 deep.** The IN side is drained by the host, which
is usually fast, and the snapshot loader's admission control (`min(count,
64)`) sizes the payload into its arm decision; 16 flop-based slots are cheap
and keep the read combinational, which suits the TX engine's byte-fetch
timing ([usb_serial](usb-serial.md), [tx PHY](usb-tx-phy.md)).

**End-to-end flow control** — the part that makes the whole design lossless:

1. Host bulk-OUT burst → [rx packet](usb-rx-packet.md) hands good payload
   bytes to `usb::UsbUart`'s rx push, which streams them into `rxfifo` one
   byte per clk (from the packet payload RAM's registered read port).
2. The byte-stream consumer pops at its own pace (`rx_ready` throttling;
   with the echo server as consumer, that pace is the UART's 115200-baud
   drain rate); if it stalls, `rxfifo.count` grows.
3. The serial endpoint refuses a new packet when `rx_sum > 64` — it NAKs
   instead, and the host retries. Bytes never drop.

## Key takeaways

- A FIFO is a circular buffer (memory + read pointer + write pointer + count)
  that absorbs producer/consumer rate mismatch; `full`/`empty` fall straight
  out of the count.
- `usb_fifo16x8` is parameterized by `DEPTH`, pointer width `AW`, and `RAM`
  — the BRAM-vs-distributed storage-target knob; data width is fixed at 8
  bits.
- **No internal over/underflow protection**: the caller must pre-gate
  `wr_en` with `!full` *and* `rd_en` with `!empty`. The enables carry no
  `cnt` terms by design, so the count compares never sit in a write/read
  clock-enable cone — a hard requirement at 48 MHz on the 64-deep FIFO.
- Simultaneous push+pop is supported; `count` is exposed so callers can do
  room math (whole-packet-fits admission `rx_sum <= 64` on the rx side,
  `min(count, 64)` arm sizing on the tx side).
- With `RAM=1`, `rd_data` is valid **one clock after** `rd_en` — pair it
  with an equally delayed valid strobe (`rx_valid_rr` / `rx_data_d` in
  `usb::UsbUart`).
- The core instantiates it through two fixed-shape ext wrappers in
  `ext/usb_uart.sv` (`\usb::TxFifo` 16×8 flops, `\usb::RxFifo` 64×8 BRAM):
  a 64-deep BRAM OUT buffer sized to atomically absorb a maximum bulk
  packet, and a 16-deep flop IN buffer; NAK-on-no-room gives end-to-end
  flow control with no byte loss.
