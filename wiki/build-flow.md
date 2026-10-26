# Build, Test, Synth — the fomu-usb flow

This page walks the whole toolchain: what `vir build` emits, what each
Makefile target does, how simulation and synthesis run, what every
build artifact is — and the project's one hard rule: **there is no
upload target, and there must never be one**. Toolchain presence is
checked by `make tools`.

The system you are building is described in
[system-overview.md](system-overview.md); the RTL itself lives in the
per-module pages. If you have never used an FPGA flow, read the
"Beginner's framing" section first.

## Beginner's framing: RTL → bitstream

An FPGA is a sea of configurable look-up tables (LUTs), flip-flops
(FFs), and block RAMs. "Programming" one means *configuring* it:

```
src/*.vir + ext/*.sv   RTL (behavioral description: registers, FSMs, logic)
        │  vir build / hand-written
        ▼
build/*.sv             Verilog netlist-of-behavior (same level, one language)
        │  yosys synth_ice40            ── SYNTHESIS
        ▼                        (maps registers→FFs, logic→LUTs,
                                 fifos/ROM→BRAM; optimizes)
build/top.json         technology-mapped netlist (UP5K cells)
        │  nextpnr-ice40 --up5k        ── PLACE & ROUTE
        ▼                        (assigns cells to physical LUTs,
                                 routes wires between them,
                                 checks the 48 MHz timing)
build/top.asc          ASCII bitstream (FPGA configuration image)
        │  (the operator converts/flashes it — NOT this Makefile)
        ▼
hardware
```

Simulation (iverilog) is the *other* track: same RTL (`ext/*.sv` +
the Virdant-generated `build/*.sv`), run as software against
testbenches ([testbenches.md](testbenches.md)), before you ever touch
synthesis. This project does both, every `make`.

## The source layout

Two kinds of RTL, compiled by two tools:

- **`src/*.vir`** — the design is written in Virdant: `src/top.vir`
  (the platform wrapper), `src/usb.vir` (the USB core: `usb::UsbPor`,
  `usb::RxPhy/RxPacket`, `usb::TxPhy`, wrapped by
  `usb::UsbController`), `src/crc.vir` (`usb::Crc5/Crc16`),
  `src/endpoint.vir` (the endpoint protocol mods `endpoint::Ep0` /
  `endpoint::Serial`), `src/echo.vir` (`echo::EchoServer`), and
  `src/uart.vir` (`uart::UartSender/UartReceiver`).
- **`ext/*.sv`** — hand-written Verilog *ext bodies* for the few mods
  Virdant cannot express, because they need `inout` nets, BRAM
  inference, or initializer ROMs: `ext/usb_pads.sv` (`usb::UsbPads`,
  two iCE40 SB_IO tristate pads — the only `inout` in the design),
  `ext/usb_uart.sv` (the shared `usb_fifo16x8` primitive plus the ext
  bodies `usb::TxFifo`, `usb::RxFifo`, and the descriptor ROM
  `usb::DescRom`), and `ext/echo_fifo.sv` (`echo::EchoFifo`).

`Virdant.toml` sets the program top: `[prog] top = "top::Top"`.

## `vir build`

`vir build` (run by `make build`):

- compiles `src/*.vir` — since `src/top.vir` carries a real
  `export mod Top for ice40::Fomu` platform export, **`vir build`
  emits the wrapper `module Top` into `build/top.sv`**
  (instantiating the ext mods `\usb::UsbPor`, `\usb::UsbController`
  and `\usb::UsbPads` plus the echo server) plus a manifest,
- compiles the rest of the Virdant design to `build/usb.sv`
  (`\usb::UsbPor` through `\usb::UsbController`), `build/echo.sv`
  (`\echo::EchoServer`) and `build/uart.sv`
  (`\uart::UartSender/UartReceiver`), plus `build/builtin.sv` /
  `build/ice40.sv` for the library cells,
- and **copies the ext sources itself** into `build/ext/`.

Two rules follow:

- `make build` *checks that `build/top.sv` is non-empty* — the
  emitted wrapper is the proof that the platform export is intact.
  Never copy `ext/*.sv` into `build/` by hand: `vir` does it, and a
  stale or duplicate manual copy breaks the flow (module
  re-definition).

## Makefile targets

From `Makefile` (`make all` = build + test + synth):

| target | does |
|---|---|
| `build` | `vir build` + wrapper-exists check |
| `test` | build + all testbenches (below) |
| `test-crc` | compiles `tb/tb_crc.v` against `ext/usb_uart.sv` + `build/builtin.sv` + `build/usb.sv` (+ `tb/sb_ram_sim.v`), runs it (iverilog/vvp) |
| `test-usb-phy` | `tb/tb_usb_phy.v` + `tb/usb_host_bfm.v` (a host model *included* into the TB, never compiled alone) + the same ext/build sources |
| `test-usb-serial` | full-device TB: all ext bodies + `build/builtin.sv build/usb.sv build/echo.sv build/uart.sv` + `tb/sb_io_sim.v` (SB_IO behavioural model) + `tb/sb_ram_sim.v` + BFM + `tb/tb_usb_serial.v` |
| `synth` | yosys + nextpnr (below) |
| `tools` | assert iverilog/yosys/nextpnr/`vir` exist |
| `clean` | delete `build/*.vvp`, `build/top.json`, `build/top.asc`, `build/*.log`, `build/html` |
| `html` | render `wiki/*.md` to `build/html/*.html` with pandoc (dark stylesheet `wiki/style.css` inlined into every page) |

Simulation detail: everything runs under **iverilog** with
`-g2012 -gspecify`, compiled to a `.vvp` image in `build/` and executed
with `vvp`. The three testbenches are the project's *primary
acceptance*: [testbenches.md](testbenches.md) covers what each one
exercises (CRC known-answer vectors, raw PHY waveforms, full
enumeration + loopback echo).

## Synthesis and place & route

`make synth` (same RTL `vir upload` would build — but does **not**
flash; see below):

1. **yosys**: `read_verilog` the ext bodies + `build/builtin.sv
   build/usb.sv build/echo.sv build/uart.sv` + the wrapper
   `build/top.sv`, then `synth_ice40 -top Top -json build/top.json`.
   Log teed into `build/yosys.log`. This maps the design onto iCE40
   primitives: SB_LUTs, SB_DFFs, and the FIFOs/descriptor ROM into
   SB_RAM40_4K block RAMs.

2. **nextpnr-ice40**: `--up5k --package uwg30 --json build/top.json
   --pcf fomu.pcf --asc build/top.asc --freq 48`. Log teed into
   `build/nextpnr.log`.

   - `--up5k --package uwg30` matches the Fomu's iCE40 UP5K in the
     ultrathin package the board uses;
   - `--pcf fomu.pcf` binds the platform wrapper's top-level ports to
     physical pins — clock F4, usb_dp A1, usb_dn A2, usb_dp_pu A4,
     LEDs B5/A5/C5, plus the platform's standard touch*/flash* pins
     (inputs dangling / outputs tied in the wrapper, but they *must*
     be constrained or nextpnr errors out);
   - `--freq 48` requests 48 MHz timing analysis; the Fomu clock
     makes this trivially closable.

## Build artifacts

| artifact | producer | what it is |
|---|---|---|
| `build/top.sv` | `vir build` | the emitted platform wrapper `module Top` + manifest — proves the `export mod Top for ice40::Fomu` in `src/top.vir` is intact |
| `build/usb.sv`, `build/echo.sv`, `build/uart.sv`, `build/builtin.sv`/`build/ice40.sv`, `build/ext/*` | `vir build` | Virdant-compiled USB core, echo server and UART, the library cell definitions, and `vir`'s own copy of the ext sources (don't duplicate by hand) |
| `build/tb_*.vvp` | iverilog | compiled simulation images, run by `vvp` |
| `build/top.json` | yosys | technology-mapped netlist (UP5K cells) |
| `build/yosys.log`, `build/nextpnr.log` | yosys / nextpnr | synthesis & PnR reports (LUT/FF/BRAM counts, timing) |
| `build/top.asc` | nextpnr | ASCII bitstream — the final FPGA configuration image |
| `build/html/*.html` | pandoc | the rendered wiki, one HTML page per Markdown file |

## The no-upload rule

The Makefile header states it as a hard rule: there is **no
upload/flash target** in this Makefile and there must never be one.

`vir upload` *does* build a bitstream (the platform export exists),
but **flashing and hardware bring-up are the operator's call** —
nothing in this repo runs it. Rationale:

- flashing touches real hardware the build system can't verify;
- bring-up steps (which image, which uploader, safe-to-flash checks)
  are operator decisions;
- the bitstream and logs (`make synth`) are the *complete* deliverable
  of the build flow.

If you want to see the timing/resource numbers the flasher would see:
`make synth` and read `build/nextpnr.log` — same design, same flow,
minus the hardware step.

## Key takeaways

- The flow is the standard FPGA pipeline: RTL (`src/*.vir` +
  `ext/*.sv`) → `vir build` (emits `build/top.sv` wrapper + compiled
  Virdant + its *own* copy of ext sources) → yosys `synth_ice40`
  (`build/top.json`) → nextpnr UP5K uwg30 @ 48 MHz with `fomu.pcf`
  (`build/top.asc`).
- `make test` (iverilog, `-g2012 -gspecify`, run via `vvp`) is the
  project's primary acceptance; three testbenches: CRC, PHY, full
  serial enumeration+echo.
- `make build` asserts the emitted wrapper exists; never hand-copy
  `ext/*.sv` into `build/` — `vir` does it, and duplicates break the
  build.
- There is deliberately **no upload/flash target**: the bitstream
  (`build/top.asc`) is the end of the Makefile's responsibility;
  flashing is the operator's call.

*Next: [testbenches.md](testbenches.md) for what the simulations
actually check, or [system-overview.md](system-overview.md) for what
it all implements.*
