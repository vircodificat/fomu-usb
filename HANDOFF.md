# HANDOFF — fomu-usb: UART-over-USB PoC

State as of the end of this working session. Read together with
`FOMU_UART_OVER_USB_PLANv2.md` (the plan; §0 hard rules are binding).

## TL;DR

- **M0–M5 done and green**: `vir build` green, all three iverilog
  testbenches pass the plan's §7 acceptance sequence (enumeration,
  negative address test, 512-byte bulk loopback with data toggles +
  NAK-retry, CDC control traffic, SOF interleave, mid-stream bus reset +
  re-enumeration).
- **Platform export landed mid-session** (in-tree when this session
  started): `src/top.vir` now carries `export mod Top for ice40::Fomu`,
  the ext module was renamed `Top` → `\top::FomuTop`, and `vir upload`
  builds the full 15-pin Fomu image.  **`vir upload` fails at nextpnr
  `--freq 48` timing** (the M6 gap).
- **P0 RESOLVED**: the platform export initially left `usb_dp_pu`
  unwired (wrapper tied it low = hardware-dead image); it — and the
  heartbeat LEDs — are now wired through the export (verified in
  build/top.sv).  No P0 hardware flag remains.
- **M6 (timing)**: dt_data payload capture off the fabric (previous
  round, killed the `vir upload` critical path) + the registered shared
  qualify in `usb_rx_packet` (this round, removed the `*_cap` chain
  from the critical path).  Tests all green; **still short of 48 MHz**:
  wrapper (vir-equivalent flow) 41.5 MHz, ext-core band 40.3–43.9
  (abc9 ≈ same).  Next known target + options below.

## Hard rules (from plan §0 — still binding)

1. No new dependencies; no migen/litex/cocotb/Python in the flow.
   valentyusb is reading material only. (A one-off Python snippet was used
   to *generate* the descriptor ROM `initial` block inside
   `ext/usb_uart.sv`; the committed artifact is pure Verilog-2005.)
2. All RTL is vendored plain Verilog in `ext/`.
3. Simulation is iverilog + Verilog testbenches only.
4. NEVER run any flashing command (`vir upload`, `dfu-util`, `iceprog`).
   `vir upload` now *builds* (platform export) — building is fine,
   flashing is still the operator's.
5. Do not modify `~/projects/virdant`.
6. Single clock domain, 48 MHz, 4 clks per USB bit.

## Flow

- `make build` — `vir build`; **build/top.sv is now EXPECTED non-empty**
  (the platform wrapper `module Top` — the old "must stay empty" check
  guarded the pre-platform era and was inverted).
- `make test` — compiles + runs tb_crc, tb_usb_phy, tb_usb_serial.
  **Run it unfiltered** (its output is the log); use `head`, not `tail —
  failures print immediately, retries grind for minutes after.
- `make synth` — yosys `synth_ice40` + nextpnr-ice40 on the SAME design
  `vir upload` builds (wrapper + ext core; `synth_ice40` WITHOUT -abc9 —
  vir's flow uses abc9, see the manual commands in the M6 section).
  Reports archived in `build/`.
- There is deliberately NO upload target.

## Platform export state (P0 resolved this session)

`src/top.vir` now exports `Top for ice40::Fomu` wiring **clock,
usb_dp, usb_dn, usb_dp_pu, led_red/green/blue** through to the ext core
(`\top::FomuTop`).  The first cut of the export wired only
clock/usb_dp/usb_dn, so the wrapper tied `usb_dp_pu = 1'b0` (platform
default = disconnected) — a hardware-dead image; that P0 is fixed and
verified in build/top.sv (`assign usb_dp_pu = \actual_top.usb_dp_pu ;`).
Residual notes:
- The tristate caveat in top.vir's comments (usb_dp/dn as `outgoing`)
  still stands until Virdant platform tristate support lands.
- top.vir's header comment block ("real top pending platform support /
  ext mod is the top / Verilog Top dies") is stale — the live export
  supersedes it.
- `ext/fomu_top.sv` module is now `\top::FomuTop` (escaped identifier).
  In yosys scripts the top name is `top::FomuTop` (no backslash!).
- `fomu.pcf` now constrains the full Fomu (PVT) pin set (touch1-4 =
  E4/D5/E5/F5, flash = C1/D1/F1/E1, cross-checked against
  ~/repos/fomu-workshop/pcf/fomu-pvt.pcf) — nextpnr errors on
  unconstrained IOs from the wrapper's 15 ports.
- `tb/tb_usb_serial.v` instantiates `\top::FomuTop` DIRECTLY (the wrapper
  has no POR_CYCLES parameter and ties the pins the TB watches).

## What was built (updated this session)

- `ext/usb_uart.sv` (~1900 lines) — module inventory per plan §4:
  `usb_crc5`, `usb_crc16`, `usb_fifo16x8` (`RAM=1` = block-RAM storage +
  registered read), `usb_rx_phy`, `usb_rx_packet`, `usb_tx_phy`,
  `usb_ep0`, `usb_serial`, `usb_uart` (router/arbiter/FIFOs/loopback/ROM).
- **`usb_rx_packet` payload capture is OFF the fabric** (this session's
  RTL move): `dt_ram` 64×8 block RAM (payload bytes 0..63, one write
  port with REGISTERED address `bc`, sync read via `dt_rd_addr` →
  `dt_rd_data`) + `dt_head` (flopped packet bytes 0..7 — the EP0
  request / line-coding window).  The old 512-flop `dt_data` put the
  bc→64-way byte-enable decode (riding the bc+1 carry chain) AND the
  64:1 read mux into the rx-push in the critical path.  `usb_ep0`
  consumes `dt_head[63:0]` (rq* latch, line-coding capture); `usb_uart`
  streams accepted payloads from `dt_rd_data` (read index = rx-push
  `push_i`; the 1-stage `rx_push_run_d` strobe pairs index and byte —
  write timing identical to the old mux design).
- **`usb_uart` architectural state (all comment-documented)**: EP0
  4-stage request pipeline (latch → EP_PARSE decode flags → EP_SETUP
  one-hot `p_action` → EP_ARM apply); SET_ADDRESS/SET_CONFIGURATION
  ownership-tagged side effects (`pend_addr`/`pend_cfg`); registered
  token router (`tok_v_r`); `lc_cap_r` registered line-coding store;
  rxfifo in block RAM with the 3-stage loopback pipe (`txfifo_cnt<13`
  headroom); rxpkt `nb[4:0]`/`n3[2:0]`/`bc` counters + per-bit
  `tok_cap/sof_cap/hs_cap/dat_cap` precompute regs computed from the
  REGISTERED shared qualify `q_pid_r`/`q_dat_r` (same-cycle pkt_err
  re-checked inline; same-cycle `bad` sets covered by the inline PID
  check / the ≥8-clk gap to the next byte boundary — see the decl
  comment); `dat_bc_ok_r` registered byte-count admission.

### Byte fetch pipeline (tx side, unchanged — still the confusing part)

    s (bit index, 1-bit-time lookahead in usb_tx_phy)
      -> tx_addr_next = (s-9)>>3   byte-stable: equals b for ALL of byte b-1
      -> byte_rd_addr (reg)
      -> snap_q / desc_q / ep0_small_q (sync-read BRAM / registered mux)
      -> tx_byte_r  loaded ONCE per payload byte (s[2:0]==0, s>=16)
      -> raw_bit_r -> engine level

Key invariants: `tx_byte_r` loads only at byte boundaries (reloading
every bit time races the next byte's fetch); the engine consumes bit 0 of
byte b at the END of byte b-1's bit 7, so the load condition is
`s[2:0]==0`.

### Descriptor ROM

`desc_rom`: 256×8 initialized BRAM (sync read). Pages: device@0 (18 B),
config@18 (75 B), str0@93 (4), str1@97 (10), str2@107 (28); sel 5/6
(line coding / status) bypass via `ep0_small_q`.  **Keep the `initial`
block and `rom_page()` in sync** (generator lives in git history).

## Deliberate deviations from the plan (all intentional)

| Plan said | Built instead | Why |
|---|---|---|
| CDC without interrupt endpoint | EP2 IN notif (descriptor-only, always NAKed), IAD, EF/02/01, 75-B config | Linux `cdc_acm` refuses bNumEndpoints==0 comm ifaces (child-B audit) |
| rx FIFO 16 deep | 64 deep, block RAM | whole-64-B-packet atomic absorb |
| `desc_byte()` case function | initialized BRAM + fetch pipeline | 128-deep case ≈ 20 ns LUT chain |
| (implied) flopped payload capture (`dt_data`) | **payload RAM (BRAM) + flopped `dt_head` shadow** | 512 payload flops + enable decode + 64:1 read mux were the critical path at 48 MHz (this session) |
| one-cycle EP0 parse | 4-stage pipeline | request tree + routing + datapath never fit one cycle |
| combinational token router | registered `tok_v_r` | dev_addr-compare fanout was critical path |

Also: EP2 response pid NAK not ACK (registered, one clk later).

## Verification status (as of HEAD)

- `make build` — green (wrapper emitted, manifest written).
- `make test` — **all three TBs ALL TESTS PASSED** (tb_usb_phy's T3/T5
  now exercise BOTH the head window (`dt_head`, shadowed at dt_valid)
  and the payload RAM (streamed out via the `snap_dt` read task)).
- `make synth` — places at **29% LC (1579 ext / 1542 wrapper), 4 BRAM**
  (snap, desc_rom, rxfifo, dt_ram), 15 SB_IO (wrapper); **FAILS
  `--freq 48`**: see the M6 numbers below.

## M6 status and history

Start of previous session: ~34 MHz, 3690 LC (69%). This session, after
the payload-off-fabric move, on the **ext core** (`-top top::FomuTop`,
7-pin pcf — the design comparable to all earlier numbers):

| flow | band across seeds | best |
|---|---|---|
| default (make synth) | 41.0–44.7 MHz | **44.73 (seed 13)** |
| abc9 (vir's flow) | 43.1–44.7 MHz | 44.73 |
| SA placer st10 | — | 41.7 (worse than HeAP refinement) |

On the **wrapper** (what `vir upload` actually builds): default
**41.51** after the qualify fix (was 40.60; the user's failing run:
39.96).  NB: before this session's P0 fix the wrapper tied usb_dp_pu/
leds constant, pruning those cones — after the fix they are live, so
wrapper vs ext-core numbers are now near-equivalent designs.

History of fixes (each re-tested): (1) EP0 route/apply split +
`p_action`; (2) FIFO caller-gated wr_en; (3) rxfifo→BRAM, 3-stage
loopback, `<13` headroom (after a data-loss regression); (4) rxpkt
`nb`/`n3`/`bc` + per-bit `*_cap` precompute; (5) `lc_cap_r` registered
store; (6) registered token router; (7) **dt_data → dt_ram (BRAM) +
dt_head (this session)** — killed the `vir upload` critical path
(bc → carry chain → 64-way byte-enable decode → 512 flops, 25 ns).

### Remaining critical path (next round's target)

The `*_cap` chain is FIXED (gone from the critical path after the
`q_pid_r`/`q_dat_r` registered qualify).  The new worst path is the
**EP0 data-accept cone**: `usb_rx_packet`'s registered EOP outputs
(`dt_len`, `dt_crc_ok`) fan into `usb_ep0`'s accept decisions and
register enables (`pend_cfg`/`pend_addr`/`dev_addr` CEN, `st`/`tog`/
`resp_sel`), ~21–24 ns (7–8 ns logic / 15–16 ns routing).  Candidate
next move: register EP0's data-accept decision one clk after
dt_valid (the +1 clk is free — turnaround budget ~30–50 clks; the
usb_serial module already does exactly this with dt_acc/dt_rej).
Beyond that: relaxed `--freq` (user sign-off, plan deviation).

## Bugs found and fixed (don't re-derive these)

Previous sessions: rxpkt `pid_r` freeze / token `n` / `tk_sh` bit-18 /
data-byte commit `{bit_i, bshift[7:1]}` / `bc`-counts-CRC (`dt_len =
bc-2`); rxphy `act` held through EOP; BFM crc5 taps; BFM `wait_idle`;
TB 4-bit `tog` (`~4'b0001` truthy); EP0 stale-`rq*`/stale-`sel` reads;
TB drain accumulator reset; iverilog gotchas (task array ports,
`{a,b}[6:0]` SV-only).
- **#9**: EP0 parse split latched `addr_pending`/`cfg_pending`
  unconditionally → first control read wiped `dev_addr` (ownership tags
  `pend_addr`/`pend_cfg`).
- **#10**: registered txfifo write gate was off-by-one (byte armed at
  cnt 15 lands at 16) → reverted to the comb gate; pop headroom re-derived
  (`<13`).
- iverilog `$time` in expressions is in time UNITS (ns) while `%t`
  displays precision units (ps) — a `$time > N` filter written from `%t`
  output is 1000× off.
- yosys `-top` for an escaped Verilog id (`\top::FomuTop `) is
  `top::FomuTop` — WITH the backslash it is "not found".

## Loose ends

- M6 decision: `--freq 48` still fails (ext-core band 40.3–43.9,
  wrapper 41.5).  Next round = the EP0 data-accept cone (above), then
  relaxed `--freq` needs user sign-off.
- `FOMU_UART_OVER_USB_PLANv2.md` §2/§4/§9 predate: EP2/IAD/EF-class,
  rx-FIFO BRAM depth, the 4-stage EP0 pipeline, the platform export
  (§9 said the ext Top "dies" when the wrapper lands — it half-did).
- top.vir's own header comments are stale (see above).
- `tb/tb_usb_serial.v` never polls EP2 (notif endpoint NAK loop).
- `build/` is gitignored; `/tmp/dbg/` scratch is disposable.
- Green work committed (2 commits: platform-wiring+flow, then the RTL
  move).

## Tool versions/paths

- `iverilog` (`-g2012`), `vvp` on PATH; `yosys`, `nextpnr-ice40` on PATH
  (yosyshq 0.56); `vir` at `~/.local/virdant/bin/vir` (0.4.0).
- Reference repos (read-only): `~/repos/valentyusb` (semantics oracle),
  `~/repos/fomu-workshop` (migen SB_IO lowering, fomu-pvt.pcf pin map).