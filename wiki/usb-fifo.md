# `usb_fifo16x8`: the byte FIFO

Computers are full of places where one part of a system produces data at a
rate another part cannot match, and the standard cure is a **FIFO** — a
first-in, first-out queue that buffers the difference. This page explains
FIFOs from scratch and walks through `usb_fifo16x8` at `ext/usb_uart.sv:105`,
the synchronous byte-width FIFO that `usb_uart` uses as its endpoint buffers
and echo path.

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

## Why this project needs FIFOs

USB bulk transfers are **bursty**: the host may fire a 64-byte bulk-OUT packet
that arrives back-to-back at 12 Mbit/s, while the UART side drains bytes at
115200 baud — roughly 100× slower. The architecture (see
[system overview](system-overview.md)) absorbs this with two FIFOs inside
`usb_uart` plus a top-level [echo FIFO](echo-fifo.md):

- **`u_rxfifo`** (`ext/usb_uart.sv:1700`) — the **EP1 bulk-OUT buffer**: every
  byte the host sends to the device lands here until the UART consumer takes
  it. 64 deep, stored in block RAM (parameters `DEPTH=64, AW=6, RAM=1`).
- **`u_txfifo`** (`ext/usb_uart.sv:1713`) — the **EP1 bulk-IN buffer**: bytes
  produced locally (echoed data, line status) queue here until the host polls
  EP1 IN and the [TX engine](usb-tx-phy.md) can serialize them. Default
  parameters: 16 deep × 8 bits wide, stored in flops.

Flow control closes the loop: when the UART consumer is slow, `u_rxfifo`
fills; when it can no longer accept a whole incoming packet, the endpoint
NAKs the host, and the host retries later (`ext/usb_uart.sv:1685`–`1691`).
No bytes are lost and nothing needs an "interrupt + emergency drain" path.

## Parameters and ports

Module declaration (`ext/usb_uart.sv:105`–`119`):

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
| `rd_en` | in | 1 | pop one byte this clock. **Self-protecting**: ignored when the FIFO is empty |
| `rd_data` | out | 8 | popped byte: combinational head (`RAM=0`) or head delayed one clock (`RAM=1`) |
| `count` | out | 7 | occupancy `cnt`, for room arithmetic by the caller |
| `full` | out | 1 | `cnt == DEPTH` |
| `empty` | out | 1 | `cnt == 0` |

**Parameterization.** The data width is fixed at 8 bits — it is a *byte*
FIFO (the `16x8` in the name is "16 deep × 8 wide"), so there is no `WIDTH`
knob. What you can tune:

- **`DEPTH`** — number of slots (default 16). Both instantiations use this
  default except the bulk-OUT side, which uses 64 (why, below).
- **`AW`** — address/pointer width, `log2(DEPTH)` (4 for 16, 6 for 64). Kept
  explicit rather than computed from `DEPTH` so pointer widths stay obvious
  at each instantiation.
- **`RAM`** — the storage-target selector, i.e. the "BRAM vs distributed"
  knob that other cores often call `TARGET` or `RAM_STYLE`:

| | `RAM=0` (distributed, flops) | `RAM=1` (block RAM) |
|---|---|---|
| storage | `reg [7:0] mem[...]` in LUTs/FFs (`(* no_ram *)` hint) | `(* ram_style = "block" *) reg [7:0] mem[...]` |
| `rd_data` | combinational: `assign rd_data = mem[rptr]` (`ext/usb_uart.sv:143`) | registered read `rd_q <= mem[rptr]` — popped byte valid **one clock after** `rd_en` (`ext/usb_uart.sv:134`–`136`) |
| cost | 8 × DEPTH flip-flops + per-byte write decode | one iCE40 BRAM block, no flop cost |
| used by | `u_txfifo` (16 deep = 128 flops — cheap) | `u_rxfifo` (64 deep = 512 flops — the header notes moving these and their decode cones out of the fabric, `ext/usb_uart.sv:88`–`103`) |

Both variants are selected by a `generate` block (`ext/usb_uart.sv:127`–`145`)
so the pointer/count logic is shared verbatim.

## Push/pop semantics — read this part twice

Two asymmetries in the flags are deliberate and documented in the module
header (`ext/usb_uart.sv:96`–`103`):

**1. `wr_en` is *not* self-protecting — the caller must gate it with
`!full`.** Inside the FIFO:

```systemverilog
wire do_wr = wr_en;                      // no cnt term here!
wire do_rd = rd_en && (cnt != 7'd0);     // ext/usb_uart.sv:124-125
```

Why would anyone design it that way? Timing. If the FIFO checked `full`
internally, the `cnt == DEPTH` comparison would sit on the write path — the
clock-enable cone of every single byte write, on every cycle. On the 64-deep
bulk-OUT FIFO that cone is wide, and at 48 MHz it is the kind of thing that
makes nextpnr timing fail (the header calls it "a 48 MHz killer",
`ext/usb_uart.sv:97`–`100`). Instead, the *caller* registers the "do I have
room?" decision one clock ahead and presents an already-gated `wr_en`:

- `u_rxfifo`: gated upstream in the register that produces `rx_push_run_d`
  (`ext/usb_uart.sv:1702`) — the full check lives in that register's D-input
  instead of riding the write.
- `u_txfifo`: gated inline, `wr_en(tx_strobe && !txfifo_full)`
  (`ext/usb_uart.sv:1723`) — a 16-deep full compare is short, and the comment
  at `ext/usb_uart.sv:1715`–`1722` explains why a *registered* gate would be
  stale here (bytes armed at 15 land at 16 with an in-flight push).

This is a **contract**, not a suggestion: pushing into a full FIFO overwrites
the oldest byte and corrupts `cnt`. If you reuse this module, gate your writes.

**2. `rd_en` *is* self-protecting.** `do_rd` includes the `cnt != 0` term
(`ext/usb_uart.sv:125`). The pointer/count registers are cheap to re-enable,
so the FIFO can afford to swallow a stray pop — it only feeds `rptr`/`cnt`,
never the memory write port.

**Simultaneous push+pop is supported** (`ext/usb_uart.sv:157`–`161`):

```systemverilog
case ({do_wr, do_rd})
    2'b10: cnt <= cnt + 7'd1;   // push only
    2'b01: cnt <= cnt - 7'd1;   // pop only
    default: ;                 // both (net 0) or neither
endcase
```

Both in the same clock is the common case mid-stream (the RX pushes while the
consumer pops), and the ring makes it safe — the pointers move past each
other independently.

**`count` and room math.** Callers mostly reason about *room*, not
occupancy: `tx_room = 7'd16 - txfifo_cnt`
(`ext/usb_uart.sv:1726`) tells the producer how many more bytes fit, and the
bulk-OUT admission check computes `rxfifo_cnt + dt_len <= 64` to decide
whether a whole incoming packet fits before ACK-ing it
(`ext/usb_uart.sv:1710`–`1711`).

### The one-clock read latency of `RAM=1`

With `RAM=1` the read port is *free-running*: `rd_q <= mem[rptr]` happens
every clock regardless of `rd_en` (`ext/usb_uart.sv:134`), because block RAMs
have registered read ports — a combinational `mem[rptr]` would not map to a
BRAM primitive. Consequence: after your `rd_en`, the popped byte appears on
`rd_data` one clock *later*. The consumer must pair the data with a matching
valid strobe delayed the same way — `usb_uart` does exactly this, delaying
`rx_valid` through two registers so `rx_valid_rr` lines up with the BRAM's
output (`ext/usb_uart.sv:1691`–`1696`). Forgetting this pairing is the
classic bug when switching a FIFO from flops to BRAM.

## Line-by-line tour

- `ext/usb_uart.sv:120`–`121` — state: two `AW`-bit pointers and a 7-bit
  count (7 bits is enough for any `DEPTH` up to 127).
- `ext/usb_uart.sv:124`–`125` — the gated internal strobes `do_wr`/`do_rd`
  discussed above.
- `ext/usb_uart.sv:127`–`145` — `generate` split between BRAM storage
  (`g_ram`, free-running registered read) and flop storage (`g_ff`,
  combinational read).
- `ext/usb_uart.sv:147`–`163` — the pointer/count sequencer. On reset all
  three registers clear; otherwise each pointer does a wrapping `+1`
  (`wptr <= wptr + 1` written as a sign-extended constant, lines 153–156)
  when its strobe fires, and the `case` updates `cnt`. Note reset clears
  pointers and count but not `mem` — harmless because `cnt==0` means no
  slot is considered valid.
- `ext/usb_uart.sv:165`–`167` — outputs: `count = cnt`,
  `full = (cnt == DEPTH[6:0])`, `empty = (cnt == 7'd0)`.

## How `usb_uart` uses it (EP buffers)

```systemverilog
// ext/usb_uart.sv:1700
usb_fifo16x8 #(.DEPTH(64), .AW(6), .RAM(1)) u_rxfifo(
    .clk(clk), .rst(rst || bus_reset),
    .wr_en(rx_push_run_d), .wr_data(dt_rd_data),
    .rd_en(rx_pop_d), .rd_data(rxfifo_rd),
    .count(rxfifo_cnt), .full(rxfifo_full), .empty(rxfifo_empty));

// ext/usb_uart.sv:1713
usb_fifo16x8 u_txfifo(
    .clk(clk), .rst(rst || bus_reset),
    .wr_en(tx_strobe && !txfifo_full), .wr_data(tx_data),
    .rd_en(snap_pop), .rd_data(txfifo_rd),
    .count(txfifo_cnt), .full(txfifo_full), .empty(txfifo_empty));
```

**Why `u_rxfifo` is 64 deep.** USB bulk endpoints ACK a packet only if they
can take *all* of it — a partial accept would lose bytes. A maximum-size
bulk-OUT packet is 64 bytes, so the buffer must be able to absorb 64 bytes
at once; a 16-deep FIFO can never accept one under the "whole packet must
fit" rule (`ext/usb_uart.sv:88`–`92`, and the room check at
`ext/usb_uart.sv:1710`). 64 also matches what a real device's endpoint buffer
holds. Going BRAM (`RAM=1`) takes 512 flops and their per-byte write-decode
cones out of the scarce iCE40 fabric (see [build flow](build-flow.md) for how
Yosys honors the `ram_style` attribute).

**Why `u_txfifo` stays 16 deep.** The IN side is drained by the host, which
is usually fast, and the echo path's admission control sizes the headroom
into its pop decision; 16 flop-based slots are cheap and keep the read
combinational, which suits the TX engine's byte-fetch timing
([usb_serial](usb-serial.md), [tx PHY](usb-tx-phy.md)).

**End-to-end flow control** — the part that makes the whole design lossless:

1. Host bulk-OUT burst → [rx packet](usb-rx-packet.md) hands good payload
   bytes to `usb_uart`, which pushes them into `u_rxfifo`.
2. The UART consumer pops at its own pace (`rx_ready` throttling,
   `ext/usb_uart.sv:1691`); if it stalls, `rxfifo_cnt` grows.
3. The serial endpoint refuses a new packet when `rxfifo_cnt + dt_len > 64`
   (`rx_room_ok`, `ext/usb_uart.sv:1710`–`1711`) — it NAKs instead, and the
   host retries. Bytes never drop.

## Key takeaways

- A FIFO is a circular buffer (memory + read pointer + write pointer + count)
  that absorbs producer/consumer rate mismatch; `full`/`empty` fall straight
  out of the count.
- `usb_fifo16x8` is parameterized by `DEPTH`, pointer width `AW`, and `RAM`
  — the BRAM-vs-distributed storage-target knob; data width is fixed at 8
  bits.
- **`wr_en` must be pre-gated with `!full` by the caller** — putting the full
  check inside the write path would ride every byte's clock-enable cone and
  kill timing on the 64-deep FIFO; **`rd_en` is self-protecting** and ignored
  when empty.
- Simultaneous push+pop is supported; `count` is exposed so callers can do
  room math (`tx_room = 16 - count`, whole-packet-fits admission).
- With `RAM=1`, `rd_data` is valid **one clock after** `rd_en` — pair it with
  an equally delayed valid strobe (`rx_valid_rr`,
  `ext/usb_uart.sv:1691`–`1696`).
- `usb_uart` uses two instances as the EP1 bulk buffers
  (`ext/usb_uart.sv:1700`, `1713`): a 64-deep BRAM OUT buffer sized to
  atomically absorb a maximum bulk packet, and a 16-deep flop IN buffer;
  NAK-on-full gives end-to-end flow control with no byte loss.
