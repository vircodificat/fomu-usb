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

The whole core runs on a single 48 MHz domain ([usb-uart.md](usb-uart.md),
[FOMU_UART_OVER_USB_PLANv2.md §"Single clock domain"]). That number is
pinned by the USB protocol, not by taste: Full-Speed is 12 Mbit/s, and
48 MHz gives **exactly 4 clock cycles per wire bit** (`ext/usb_uart.sv:7`).

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
  This project hit it twice (see HANDOFF.md: the `load_run` D-cone
  feeding `li`'s own clock-enable at 21.3 ns, and the flat-D sticky
  `bad` whose enable cone ingested the `bc == 66` compare through the
  carry chain).
- **Reset cone** — same idea for gated resets; not fixable by
  pipelining, since the reset must reach the flop in the same cycle by
  definition.

## Reading this project's timing report

`make synth` (see `Makefile`, the `synth` target) runs
`nextpnr-ice40 --freq 48` and archives the log in
`build/nextpnr.log`. The verdict line looks like:

```
ERROR: Max frequency for clock 'clock$SB_IO_IN_$glb_clk': 43.13 MHz (FAIL at 48.00 MHz)
```

The clock's full period is 48 MHz → **20.83 ns**. Every
register-to-register cone in the design must fit inside it.

The register-to-register critical path (the failing cone) in the
current tree, from `build/nextpnr.log`, reads:

```
  1.4   1.4  Source actual_top.u_core.u_ep0.rq6_..._DFFLC.O
  1.8   3.2    Net rq6[7] (14,9) -> (14,9)
               Sink p_wlen0_..._LC.I0        (ext/usb_uart.sv:1105)
  1.3   4.4  Source p_wlen0_..._LC.O
  ...         (p_wlen0 → p_is_cls → p_valid decode levels)
  1.8  17.1    Net ...[3] (14,11) -> (13,11)
               Sink p_valid_..._LC.I3
  0.9  18.0  Source p_valid_..._LC.O
  0.9  20.6  Source p_valid_..._1_LC.O
  1.8  22.4    Net ... (13,11) -> (12,11)
               Sink p_action_..._LC.I3       ← D-cone of the p_action flop
```

Six-to-seven LUT4 levels plus routing, ~21 ns against a 20.83 ns
budget. The source is a *registered* EP0 request field (`rq6`), the
sink is the D-cone of the `p_action` one-hot route register — i.e. the
**request-decode cone** of the EP0 control endpoint
([usb-ep0.md](usb-ep0.md)). This corner of the FSM is structurally the
deepest thing in the design and keeps reappearing at the top of the
report after every netlist change.

The report also lists two *cross-domain* paths, which are not
register-to-register cones at all:

```
Max delay <async> -> posedge clock: 4.54 ns     (input pads → first flop)
Max delay posedge clock -> <async>: 15.38 ns    (last flop → output pads)
```

These are I/O timing checks (pad-to-setup and clock-to-out). They are
usually fine here — the 15.4 ns clock-to-out goes to the USB pads,
which the protocol tolerates — and they are worth distinguishing from
the internal cones because nextpnr reports them separately.

## Why pipelining doesn't simply fix it

Pipelining inserts a register to split a deep cone into two shallow
ones. It genuinely works — this project's M6 closing history is ~15
pipelining fixes that moved Fmax from ~34 MHz to a best of 48.78 MHz —
but the remaining paths are the ones pipelining is *structurally*
unable to help. Three categories:

### (a) Same-cycle decision paths

The failing cone is decode feeding **FSM next-state logic**. A
next-state decision is a feedback loop by definition: the result is
needed *this cycle* to compute the state entered *next cycle*. You
cannot pipeline inside a cycle. The registered-accept pattern in the
EP0 request pipeline (`req_acc_r` / `rcd_acc_r` / `sto_acc_r`,
`ext/usb_uart.sv` — see the "Request pipeline" comment block) is the
correct fix — it splits "did the packet decode?" from "what does the
FSM do about it?" across two clock edges — but each such split needs
the protocol to have a spare cycle in that window. EP0's timeline is
mostly budgeted; the comment notes the turnaround slack (~30–50 clks)
that made those particular splits free.

### (b) Clock-enable cones

"Only write this flop when X" requires X to be known **in the same
cycle as the data** — there is no pipelining a clock enable. On iCE40
the enable folds into the D-driving LUT, so gating a write *widens* the
path it sits on. The fix pattern is to restructure so the enable is a
single registered bit rather than a decoded expression (the
"flat-D sticky" style fixes in HANDOFF.md).

### (c) The fixed-latency serializer

The RX/TX PHY already uses exactly its 4 cycles per bit (per-bit
captures, precomputed caps — several M6 fixes were spent making those 4
cycles *just enough*). A pipeline that is already exactly as deep as
its deadline allows has no split points left; adding depth means
missing the bit boundary. See [usb-rx-phy.md](usb-rx-phy.md),
[usb-tx-phy.md](usb-tx-phy.md).

**The one-line summary:** pipelining buys throughput by adding latency;
this design's problems are mostly *latency-budget* problems in a
protocol that has already spent nearly all of its spare cycles.

## The sharing hazard: cone independence

Synthesis tools merge cones that share substructure — usually good for
area, but it can undo your splits. HANDOFF.md documents a measured
regression: a "flat priority-mux rewrite" of the response mux made
things *slower* because the wide mux became a new abc9 sharing hub.
Conversely, the rxpkt `if/else-if` priority tree was shared across
*several unrelated* cones (token, data and handshake D-cones *and* the
captured fields' clock enables — `hs_pid`'s enable was literally
`hs_valid`'s D-cone); flattening it, since the branches are mutually
exclusive, fixed all of them at once.

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
EP0 pipeline (`ext/usb_uart.sv`): "the FSM reacts to the registered
accepts one clk later; ... +1 clk is free: the turnaround budget is
~30–50 clks."

(Virdant syntax notes: `Word[4]` is a 4-bit word, `0w4` a 4-bit zero
literal, `it <=` assigns inside a `reg ... on clock` block, and the
`when { case ... else ... }` block is the structured priority mux. See
[top-vir.md](top-vir.md) and [echo-server.md](echo-server.md) for real
instances in this repo.)

## The flow amplifies tightness into a seed lottery

Two facts about this project's tool flow matter (documented at length
in HANDOFF.md and [build-flow.md](build-flow.md)):

1. **`--freq` does not change the FPGA's clock.** It is purely a
   nextpnr *target* for static timing analysis — and additionally a
   weight for the simulated-annealing placer. The fabric runs at
   whatever 48 MHz signal the board gives it; Fmax is an emergent
   property of the placed netlist, not a programmed setting.
2. **vir's flow does not pass `--freq`** — it relies on the PCF's
   `set_frequency clock 48.0` (`fomu.pcf`), which constrains STA but
   not the placer. So placement is timing-blind, and the same netlist
   lands anywhere from 44.7 to 48.8 MHz depending on seed. A design
   with 20% margin wouldn't notice; a design at 43–49 MHz against a
   48 MHz target flips between PASS and FAIL on dice.

That is why HANDOFF.md insists: measure with **vir's exact command**
(after `make build`), not with `make synth`'s `--freq 48` invocation —
the two flows differ in the placer's knowledge of the constraint even
though both target the same clock.

## Calibration: is this normal?

- **Tight at 48 MHz with a bit-banged FS device core on UP5K: yes,
  normal.** Comparable cores in this lineage (valentyusb) also close at
  48 MHz on the same part, and only after similar decode pipelining.
- **Bouncing pass/fail at the boundary on every edit: the real
  symptom.** With effectively ~0–5% margin, any added logic decides the
  outcome, and the iteration protocol becomes "every netlist edit is a
  re-spin of closure."

The sustainable exits, all still available: register more of the EP0
decode cone (the highest-value corner, since it keeps reappearing as
the critical path), pass `--freq` to the placer, or make seed search
part of the bitstream script. A bigger architectural lever — a PLL
giving a dual-domain split (PHY at 96 MHz = 8 cycles/bit, control plane
at 48 MHz, phase-aligned) — exists but collides with the plan's
"single clock domain" hard rule and is a restructure of a working,
sim-green design; it is not the first move.

## Where this is in the tree

| What | Where |
|---|---|
| Clock frequency / pins | `fomu.pcf` (`set_frequency clock 48.0`) |
| 4 clks per USB bit | `ext/usb_uart.sv:7` |
| Build flow / synth target | `Makefile` (`synth`), [build-flow.md](build-flow.md) |
| Latest timing report | `build/nextpnr.log` (critical path at "Critical path report for clock") |
| Fix history & M6 closure | HANDOFF.md (M6 section) |
| EP0 request pipeline comment | `ext/usb_uart.sv`, "Request pipeline (48 MHz timing...)" |
| The failing cone's source code | `ext/usb_uart.sv` — `p_wlen0`/`p_is_cls`/`p_valid` decode around line 1105, `p_action` route register |

## See also

- [build-flow.md](build-flow.md) — how the build and its timing report are produced
- [usb-uart.md](usb-uart.md) — the core top that owns the failing cone
- [usb-ep0.md](usb-ep0.md) — the EP0 control endpoint whose request decode is the critical path
- [usb-rx-phy.md](usb-rx-phy.md) / [usb-tx-phy.md](usb-tx-phy.md) — the fixed-latency 4-cycles-per-bit serializer
- HANDOFF.md — the full M6 closure history, fix by fix
