# Timing cones — why 48 MHz is tight on this design

This page is for readers who have written a bit of RTL and met a timing
report before, but want to connect the report's vocabulary — *critical
path*, *cone*, *clock-enable*, *Fmax* — to actual code and to **this**
project's numbers. It explains the single most useful concept in timing
closure work (the **cone**), why this design point is legitimately
tight, and why the usual escape hatch (pipelining) only goes so far.

Where the build flow itself lives: [build-flow.md](build-flow.md).
Where the core sits in the system: [system-overview.md](system-overview.md),
[usb-uart.md](usb-uart.md).

## The design point: one clock, and it's load-bearing

The whole core runs on a single 48 MHz domain — no PLL, no second clock,
no CDC ([usb-uart.md](usb-uart.md)). The constraint is pinned in
`fomu.pcf` (`set_frequency clock 48.0`), and the RTL states it in the
`usb::RxPhy` header comment in `src/usb.vir` ("48 MHz domain, 4 clks per
FS bit"). That number is pinned by the USB protocol, not by taste:
Full-Speed is 12 Mbit/s, and 48 MHz gives **exactly 4 clock cycles per
wire bit** — you can see the budget in the code as the bit-phase counter
`ph : Word[2]` in `usb::RxPhy` (`src/usb.vir`), which counts 0..3 within
each bit and is re-aligned to 0 on every line transition.

That 4-cycles-per-bit budget has a consequence that shapes everything
else: the PHY datapath (NRZI decode, bit destuffing, serial CRC,
sync/EOP generation) is a **fixed-latency** pipeline — it uses precisely
its 4 cycles per bit, with no idle cycles to hand out. Any extra
register inserted anywhere has to be absorbed by the *protocol's* spare
cycles, and the control plane (EP0 enumeration, token routing, CRC
checks) barely has any. See [usb-rx-phy.md](usb-rx-phy.md) and
[usb-rx-packet.md](usb-rx-packet.md) for how those cycles are spent.

So the target isn't "slow, fabric is comfortable" — it's "slow, but
with no spare latency in the protocol." That's the tight spot.

## What is a cone?

A **cone** is the set of all combinational logic that feeds one point
in the netlist — literally the fan-in triangle you'd draw by tracing
backwards from a signal through every gate and mux it depends on,
stopping at registers or primary inputs:

```
  rq6[7:0]   other regs...
      \        \      \
   [ LUT ]---[ LUT ]---[ LUT ]        ← decode / compare logic
          \     |     /
           [ LUT ]---[ LUT ]
                \   |
                 [ LUT ]                ← final gate
                    |
                   [D]──▶ p_action flop   ← apex of this cone
```

Timing analysis works *per cone*: when nextpnr reports a "critical
path", it is reporting the longest register-to-register traversal
through one cone — register output → some sequence of LUT levels and
routing → register input. The clock period must cover the worst cone in
the design, plus clock-to-out, routing, and setup.

### The cone types that matter here

- **D-cone** — everything feeding a flop's data input. The classic
  critical path. "This flop's D-cone is 21 ns deep" means the design
  cannot change state faster than that.
- **Clock-enable (CE) cone** — everything feeding a flop's *enable*.
  On iCE40 there is no dedicated enable hardware: the enable folds
  into the LUT that drives the flop's D input, so the enable cone
  **becomes part of the D-cone**. This is the classic trap: "just gate
  this write" can silently widen the path you were trying to shorten.
  This design's own archived report shows it: the critical path in
  `build/nextpnr.log` ends not on a D pin but on the `CEN`
  (clock-enable) pin of a flop — see the excerpt below.
- **Reset cone** — same idea for gated resets; not fixable by
  pipelining, since the reset must reach the flop in the same cycle by
  definition.

## Reading this project's timing report

`make synth` (the `synth` target in the `Makefile`) runs yosys
`synth_ice40`, then `nextpnr-ice40 --up5k --package uwg30 --freq 48`,
and tees the log into `build/nextpnr.log`. Each run prints two verdict
lines — one after placement, one after routing — and they can disagree
by several MHz:

```
Info: Max frequency for clock 'clock$SB_IO_IN_$glb_clk': 44.68 MHz (FAIL at 48.00 MHz)
...
Info: Max frequency for clock 'clock$SB_IO_IN_$glb_clk': 50.04 MHz (PASS at 48.00 MHz)
```

The clock's full period is 48 MHz → **20.83 ns**. Every
register-to-register cone in the design must fit inside it.

The register-to-register critical path in the archived
`build/nextpnr.log` (trimmed; names are as mangled by synthesis):

```
  1.4  1.4  Source actual_top.core.txfifo.u.cnt_..._LC.O
  4.8  6.2    Net actual_top.core.serial.txfifo_count[3]
  1.3  7.5  Source ...pkt_len_..._LC.O
  ...         (arm-sizing / packet-length decode levels)
  3.5 16.2    Net ... (11,21) -> (17,22)
               Sink ...pkt_pending_..._LC.I2
  1.2 17.4  Source ...pkt_pending_..._E_LC.O
  2.4 19.9    Net ...pkt_pending_..._E
               Sink ...pkt_pending_..._DFFLC.CEN   ← clock-enable pin!
  0.1 20.0  Setup                                 (5.7 ns logic + 14.3 ns routing)
```

Read structurally: the source is the TX FIFO's read-pointer counter
(`cnt` inside `usb_fifo16x8`, the ext body behind the `usb::TxFifo`
wrapper in `ext/usb_uart.sv`); the middle levels are the arm-sizing
decode in `endpoint::SerialInData` (`src/endpoint.vir`) — `load_len :=
min(txfifo_count, 64)` and `arm_new := tk_in && !pkt_pending &&
!txfifo_empty` — and the sink is the **clock-enable** of the
`pkt_pending` TX-arm register. "Did an IN token arrive, and how many
bytes can we send?" must be answered in the same cycle the token strobe
is high, so the FIFO-occupancy compare lands on the enable cone of a
register whose write is gated by that very decision. Same-cycle
decision path and CE cone, in one report line — and at ~20.0 ns against
a 20.83 ns budget, with essentially no margin.

The EP0 request decode ([usb-ep0.md](usb-ep0.md)) is the other
structurally deep corner: its next-state logic reads latched request
fields through priority chains, and it keeps trading the top spot in
the report with the TX arm path as the netlist changes.

The report also lists two *cross-domain* paths, which are not
register-to-register cones at all:

```
Max delay <async> -> posedge clock: 4.67 ns     (input pads → first flop)
Max delay posedge clock -> <async>: 16.66 ns    (last flop → output pads)
```

These are I/O timing checks (pad-to-setup and clock-to-out). They are
usually fine here — the ~16.7 ns clock-to-out goes to the USB pads,
which the protocol tolerates — and they are worth distinguishing from
the internal cones because nextpnr reports them separately.

## Why pipelining doesn't simply fix it

Pipelining inserts a register to split a deep cone into two shallow
ones. It genuinely works — the archived `build/synth_*.log` reports
show the design's post-route estimate being pushed above the 48 MHz
target by exactly such restructurings — but the remaining paths are the
ones pipelining is *structurally* unable to help. Three categories:

### (a) Same-cycle decision paths

The deep cones above feed **FSM next-state logic**. A next-state
decision is a feedback loop by definition: the result is needed *this
cycle* to compute the state entered *next cycle*. You cannot pipeline
inside a cycle. The registered-accept pattern in the EP0 request
pipeline (`req_acc_r` / `sto_acc_r` in `endpoint::Ep0ReqDecode`,
`src/endpoint.vir` — see the "registered data-stage accepts" comment block) is the correct fix —
it splits "did the packet decode?" from "what does the FSM do about
it?" across two clock edges — but each such split needs the protocol to
have a spare cycle in that window. EP0's timeline is mostly budgeted;
the comment notes the turnaround slack (~30–50 clks) that made those
particular splits free.

### (b) Clock-enable cones

"Only write this flop when X" requires X to be known **in the same
cycle as the data** — there is no pipelining a clock enable. On iCE40
the enable folds into the D-driving LUT, so gating a write *widens* the
path it sits on — which is why the critical path above ends on a `CEN`
pin. The fix pattern is to restructure so the enable is a single
registered bit rather than a decoded expression.

### (c) The fixed-latency serializer

The RX/TX PHY already uses exactly its 4 cycles per bit (per-bit
captures at the mid-bit sample point, bit-phase re-alignment on every
transition). A pipeline that is already exactly as deep as its deadline
allows has no split points left; adding depth means missing the bit
boundary. See [usb-rx-phy.md](usb-rx-phy.md),
[usb-tx-phy.md](usb-tx-phy.md).

**The one-line summary:** pipelining buys throughput by adding latency;
this design's problems are mostly *latency-budget* problems in a
protocol that has already spent nearly all of its spare cycles.

## The sharing hazard: cone independence

Synthesis tools merge cones that share substructure — usually good for
area, but it can undo your splits. This is why the `endpoint::Ep0` header
comment (`src/usb.vir`) calls its structure out explicitly: every
register's next-state is a priority chain over *mutually exclusive*
branches, so the tool can share the decode logic without the branches'
cones feeding each other, and the registered-accept pipeline
(`req_acc_r` / `sto_acc_r`) keeps each chain's cone shallow by
construction. Reshape those chains into nested, dependent conditions and
the shared logic stops being free: cones that looked independent get
glued together and the critical path jumps.

Lesson: on this flow, cone **independence** is sometimes worth more
than cone **depth**, and every "obvious" fix must be measured, not
assumed.

## A toy example in Virdant

Here is the whole phenomenon in ~10 lines. A counter that only ticks
when enabled:

```virdant
mod Counter {
    incoming clock : Clock
    outgoing count : Word[4]

    wire enable : Bit {
        it := (cnt != 12w4) && load_pending
    }

    reg cnt : Word[4] on clock {
        initial it <= 0w4
        it <= when {
            case enable => it + 1w4
            else        => it
        }
    }

    count := cnt
}
```

The `when { case enable => ... }` block is a priority mux — fine in
simulation. But the `enable` signal is computed **combinationally** off
registered state (`cnt` itself, plus whatever `load_pending` decodes).
On iCE40 the enable folds into the LUT driving `cnt`'s D input, so the
compare-against-12 cone and the `load_pending` decode cone **join** the
increment cone. The flop's D-cone is now: decode `enable` (2+ levels)
*plus* the 4-bit increment — and if `enable` itself sits on the design's
critical path, you have just lengthened it.

The pipelined fix registers the enable one cycle early:

```virdant
    wire enable : Bit {
        it := (cnt != 12w4) && load_pending
    }

    reg enable_r : Bit on clock {
        initial it <= false
        it <= enable
    }

    reg cnt : Word[4] on clock {
        initial it <= 0w4
        it <= when {
            case enable_r => it + 1w4
            else          => it
        }
    }
```

Now `cnt`'s enable cone is a single registered bit — depth 1. The cost
is one cycle of latency: the counter reacts to `enable` one clock
later. Whether that cycle is affordable is a *protocol* question, not a
synthesis question — exactly the judgement this project makes in its
EP0 pipeline (`endpoint::Ep0ReqDecode`, `src/endpoint.vir`): "the FSM reacts to the router
strobes one clk after arrival ... the +1 clk is free against the
~30-50 clk turnaround budget."

(Virdant syntax notes: `Word[4]` is a 4-bit word, `0w4` a 4-bit zero
literal, `it <=` assigns inside a `reg ... on clock` block, and the
`when { case ... else ... }` block is the structured priority mux. See
[top-vir.md](top-vir.md) and [echo-server.md](echo-server.md) for real
instances in this repo.)

## The flow amplifies tightness into a seed lottery

Two facts about this project's tool flow matter (the flow itself is
documented in [build-flow.md](build-flow.md)):

1. **`--freq` does not change the FPGA's clock.** It is purely a
   nextpnr *target* for static timing analysis — and additionally a
   weight for the simulated-annealing placer. The fabric runs at
   whatever 48 MHz signal the board gives it; Fmax is an emergent
   property of the placed netlist, not a programmed setting.
2. **Timing knowledge reaches the placer only through constraints.**
   `make synth` passes `--freq 48`; the pin constraints in `fomu.pcf`
   also carry `set_frequency clock 48.0` for STA. Within that,
   placement is annealing with a seed — and the same netlist lands on a
   different Fmax depending on the seed. A design with 20% margin
   wouldn't notice; a design hovering at ~45–50 MHz against a 48 MHz
   target flips between PASS and FAIL on dice.

That is why the working rule is: when you change the netlist, re-run
`make synth` and compare the verdict lines in the fresh
`build/nextpnr.log` — a single lucky PASS is not closure.

## Calibration: is this normal?

- **Tight at 48 MHz with a bit-banged FS device core on UP5K: yes,
  normal.** Comparable cores in this lineage (valentyusb) also close at
  48 MHz on the same part, and only after similar decode pipelining.
- **Bouncing pass/fail at the boundary on every edit: the real
  symptom.** The archived reports under `build/` show it concretely:
  successive netlist revisions land at 45.23, 48.34, 45.36, 45.63 and
  50.04 MHz against the same 48 MHz target (`build/synth_*.log`,
  `build/nextpnr.log`). With effectively ~0–5% margin, any added logic
  decides the outcome, and the iteration protocol becomes "every
  netlist edit is a re-spin of closure."

The sustainable exits, all still available: register more of the deep
decode cones (the EP0 request decode and the TX arm / `pkt_pending`
enable are the recurring deep corners), pass `--freq` to the placer,
or make seed search part of the bitstream script. A bigger architectural
lever — a PLL giving a dual-domain split (PHY at 96 MHz = 8
cycles/bit, control plane at 48 MHz, phase-aligned) — exists but
collides with the core's single-clock-domain design rule and is a
restructure of a working, sim-green design; it is not the first move.

## Where this is in the tree

| What | Where |
|---|---|
| Clock frequency / pins | `fomu.pcf` (`set_frequency clock 48.0`) |
| 4 clks per USB bit | `src/usb.vir` — `usb::RxPhy` header comment and the `ph` bit-phase counter (0..3) |
| Build flow / synth target | `Makefile` (`synth`), [build-flow.md](build-flow.md) |
| Latest timing report | `build/nextpnr.log` (verdict lines at "Max frequency"; paths at "Critical path report for clock") |
| Registered-accept pipeline comment | `src/endpoint.vir`, `endpoint::Ep0ReqDecode` — "registered data-stage accepts" block and the `endpoint::Ep0ReqDecode` header |
| The critical path's decode logic | `src/endpoint.vir`, `endpoint::SerialInData` — `arm_new` / `load_len` decode into `pkt_pending`'s clock-enable |

## See also

- [build-flow.md](build-flow.md) — how the build and its timing report are produced
- [usb-uart.md](usb-uart.md) — the core top that owns the deep cones
- [usb-ep0.md](usb-ep0.md) — the EP0 control endpoint whose request decode is one of the deep corners
- [usb-rx-phy.md](usb-rx-phy.md) / [usb-tx-phy.md](usb-tx-phy.md) — the fixed-latency 4-cycles-per-bit serializer
