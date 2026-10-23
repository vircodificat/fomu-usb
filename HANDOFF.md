# HANDOFF — fomu-usb: UART-over-USB PoC

State as of the end of this working session. Read together with
`FOMU_UART_OVER_USB_PLANv2.md` (the plan; §0 hard rules are binding).

## TL;DR

- **M0–M5 done and green**: `vir build` green (ext-mod Top, empty `top.sv`),
  all three iverilog testbenches pass the plan's §7 acceptance sequence
  (enumeration, negative address test, 512-byte bulk loopback with data
  toggles + NAK-retry, CDC control traffic, SOF interleave, mid-stream bus
  reset + re-enumeration).
- **M6 (synthesis) — stopped and referred to the user** (plan next-step 4).
  After this session's five timing rounds, `nextpnr-ice40` closes at
  **36–41 MHz across seeds (best 41.0, typical ~40)** vs the 48 MHz
  target. Everything committed (see below) is TEST-GREEN and
  synthesizable; the *only* failure is the `--freq 48` timing check.
  Remaining options need a user decision (see "M6 decision" below).
- This session fixed a **real functional bug** shipped by the previous
  session's EP0 parse split (device wiped its own address after the first
  control read — see "Bugs found" #9), then landed four more timing
  restructures, all regression-tested.

## Hard rules (from plan §0 — still binding)

1. No new dependencies; no migen/litex/cocotb/Python in the flow.
   valentyusb is reading material only. (A one-off Python snippet was used
   to *generate* the descriptor ROM `initial` block inside
   `ext/usb_uart.sv`; the committed artifact is pure Verilog-2005.)
2. All RTL is vendored plain Verilog in `ext/`.
3. Simulation is iverilog + Verilog testbenches only.
4. NEVER run any flashing command (`vir upload`, `dfu-util`, `iceprog`).
5. Do not modify `~/projects/virdant`.
6. Single clock domain, 48 MHz, 4 clks per USB bit.

## Flow

- `make build` — `vir build`; must stay green; `build/top.sv` must stay empty.
- `make test` — compiles + runs tb_crc, tb_usb_phy, tb_usb_serial.
  **Run it unfiltered** (its output is the log); use `head`, not `tail —
  failures print immediately, retries grind for minutes after.
- `make synth` — yosys `synth_ice40` + nextpnr-ice40 place (streams via
  `tee`; `pipefail` propagates nextpnr's timing failure). Reports archived
  in `build/`.
- There is deliberately NO upload target.

## What was built (updated this session)

- `ext/usb_uart.sv` (~1900 lines) — the whole core, module inventory as in
  plan §4: `usb_crc5`, `usb_crc16`, `usb_fifo16x8` (now with a `RAM`
  parameter: `RAM=1` = block-RAM storage + registered read), `usb_rx_phy`,
  `usb_rx_packet`, `usb_tx_phy`, `usb_ep0`, `usb_serial`,
  `usb_uart` (router/arbiter/FIFOs/loopback/ROM).
- `ext/fomu_top.sv`, `tb/sb_io_sim.v`, `tb/usb_host_bfm.v`,
  `tb/tb_crc.v`, `tb/tb_usb_phy.v`, `tb/tb_usb_serial.v` — unchanged
  this session (still the M5 versions, all green).

### Architectural changes this session (all comment-documented in RTL)

1. **EP0 request pipeline is now 4 stages**: EP_RCV_REQ latch → EP_PARSE
   decode flags → EP_SETUP route flags into a one-hot `p_action` register
   → EP_ARM apply (datapath + state). The request case tree never touches
   a datapath register enable combinationally.
2. **SET_ADDRESS/SET_CONFIGURATION side effects are ownership-tagged**
   (`pend_addr`, `pend_cfg` latched in EP_PARSE with the request that
   owns them) and only applied by that transfer's own status stage.
3. **`usb_fifo16x8.do_wr` trusts the caller** (`wr_en` must be pre-gated
   with `!full` at the *producing register*): the full compare must not
   ride every byte's clock-enable cone.
4. **rxfifo is block RAM** (`RAM=1`): 512 flops + their per-byte write
   decodes moved out of the fabric; read port is registered, so the
   loopback pipe is 3 stages (`rx_pop_d → rx_valid_r → rx_valid_rr`,
   `rx_data_d` pairs with `rx_valid_rr`).
5. **rxpkt bit counters split**: 10-bit `n` replaced by `nb[4:0]`
   (PID/token-section index) + `n3[2:0]` (data-section bit-in-byte,
   wraps) + `bc` (bytes) — all position compares are 2–3 LUTs, no carry
   chains. Capture decisions precomputed per-bit into `tok_cap/sof_cap/
   hs_cap/dat_cap` registers (EOP-edge strobes are 2-input gates).
   `dat_bc_ok_r` registers the bc admission compare.
6. **Token router registered** (`tok_v_r`): the dev_addr+pid+endp decode
   sits in one register's D-cone; consumers see clean strobes 1 clk later
   (tk_pid/tk_endp hold between packets so this is exact).
7. **`usb_ep0` line_coding store moved to its own always block**, pulsed
   by a registered `lc_cap_r` (sampled with the accept compare, applied
   +1 clk when dt_data is stable) — the 56 byte-enables no longer close
   over the whole RCV_DATA accept tree (that tree went through a global
   clock-enable buffer and was the critical path).

### Byte fetch pipeline (unchanged, still the confusing part)

    s (bit index, 1-bit-time lookahead in usb_tx_phy)
      -> tx_addr_next = (s-9)>>3   byte-stable: equals b for ALL of byte b-1
      -> byte_rd_addr (reg)
      -> snap_q / desc_q / ep0_small_q (sync-read BRAM / registered mux)
      -> tx_byte_r  loaded ONCE per payload byte (s[2:0]==0, s>=16)
      -> raw_bit_r -> engine level

Key invariants (each cost a debug session):
- `tx_byte_r` loads only at byte boundaries; reloading every bit time
  races the next byte's fetch.
- The engine consumes bit 0 of byte b at the END of the previous byte's
  bit 7 (s is a lookahead index), so the load condition is `s[2:0]==0`.

### Descriptor ROM

`desc_rom`: 256×8 **initialized BRAM** (sync read). Contiguous pages:
device@0 (18 B), config@18 (75 B), str0@93 (4), str1@97 (10), str2@107
(28). `sel` 5 (line coding, dynamic) and 6 (status) bypass the ROM via
`ep0_small_q`. Page starts duplicated in `rom_page()` in usb_uart —
**keep the `initial` block and `rom_page()` in sync** (the generator for
the initial block lives in git history, not in the repo).

## Deliberate deviations from the plan (all intentional)

| Plan said | Built instead | Why |
|---|---|---|
| CDC **without** interrupt endpoint, `bDeviceClass=0x02` | EP2 IN notification endpoint (descriptor-only, always NAKed), IAD, `bDeviceClass=EF/02/01`, 75-byte config | Child-B audit (P0): Linux `cdc_acm` refuses a CDC config with `bNumEndpoints==0` on the comm interface (`drivers/usb/class/cdc-acm.c`); byte values cross-checked against tinyusb's hardware-proven tables |
| rx FIFO 16 deep | **64 deep, in block RAM** (`usb_fifo16x8 #(.DEPTH(64), .RAM(1))`) | A 64-byte max bulk-OUT packet must be absorbed atomically; 16 can never accept one under the whole-packet-fits rule. BRAM: 512 flops out of the fabric (49% util), registered read |
| `desc_byte()` case function | initialized BRAM + fetch pipeline | 128-deep case ≈ 20 ns LUT chain; BRAM read is a register |
| (implied) one-cycle EP0 parse | 4-stage: latch → decode → route(`p_action`) → apply | request case tree + routing + datapath ops never fit one 48 MHz cycle |
| combinational token router | registered `tok_v_r` strobes | dev_addr compare fanout was the critical path (48 MHz) |

Also fixed earlier: EP2 response pid was ACK, must be **NAK** (registered,
one clk later, to keep the token-decode cone out of the tx engine state).

## Verification status (as of HEAD, this session)

- `make build` — green (ext mod emits no body; manifest skipped).
- `make test` — **all three TBs ALL TESTS PASSED** at HEAD.
- `make synth` — places at **49% LC (2605/5280), 3 BRAM** (snap, desc_rom,
  rxfifo), 7 SB_IO; **FAILS `--freq 48`**: 36–41 MHz over seeds (best
  41.04 with `--placer sa --starttemp 10`, ~40 typical), default seed
  39.56. abc9 ≈ same (35–36). See "M6 history".

## M6 status and history (this session)

Start of session: ~34 MHz (abc9 34.49 / default 33.49), 3690 LC (69%).

| # | Fix | After |
|---|---|---|
| 0 | EP0 parse-split **functional bug fix** (see bugs #9) | tests green again |
| 1 | EP_SETUP → route/apply split (`p_action`, EP_ARM) + arm-time `send_zlp_r`/`last_r` | 38.3 |
| 2 | FIFO `do_wr` caller-gated; full compare into `rx_push_run` D | 36.7* |
| 3 | rxfifo → BRAM, 3-stage loopback pipe, **headroom `<13`** (bug #10) | 38.5 |
| 4 | rxpkt `nb`/`n3`/`bc` counters + `*_cap` precompute | 38.5* |
| 5 | `lc_cap_r` registered line_coding store | 40.8 |
| 6 | token router registered (`tok_v_r`) | ~40 (36–41 band) |

\* seed-dependent; the band across seeds/ placers is ±2 MHz, so single
measurements are noisy. Seed sweep (default flow, current HEAD): best
41.04 (`sa --starttemp 10`), 40.8, 39-40 typical, worst 36.

### M6 decision (needs the user)

Timing is now routing-dominated (~70% of the worst path), not
logic-depth-dominated: each fix relocates the bottleneck and buys ~1–2
MHz. Options, roughly in cost order:

a. **Accept a relaxed `--freq`** (e.g. 40) as a documented plan
   deviation and treat nextpnr's models as pessimistic vs the real
   UP5K + hand retiming on hardware. Plan §0's "48 MHz" is about the
   *clock*, not nextpnr closure — hardware bring-up is the user's.
b. One more targeted round: `dt_data` (512 flops, 44→~40% util) into
   BRAM — kills the 64:1 byte-mux cones into `rx_push_byte` and the `rq*`
   latches; needs a deeper rx-push pipe (verify the room/admission
   invariants again — see bug #10's lesson).
c. Systematic placer/router search (seed × placer × `--tmg-ripup`
   sweeps, scripted; each run ~10 s).
d. Architectural rethink of remaining wide cones (the tx-path byte mux,
   the serial accept adder) — the diminishing-returns path.

The handoff rule was: STOP and present the gap — this is that report.

## Bugs found and fixed (don't re-derive these)

Previous session (unchanged):
- `usb_rx_packet`: `pid_r` kept shifting after the PID byte (froze it);
  token `n` never advanced past 8; token bits shifted into `tk_sh` past
  bit 18; data byte assembly used the delay-line's stale bit (commit
  `{bit_i, bshift[7:1]}`); `bc` counted payload+CRC so `dt_len = bc-2`.
- `usb_rx_phy`: dropped `act` on the FIRST SE0 cycle — EOP never fired.
- BFM `crc5_step` had the poly taps at the wrong positions.
- BFM `wait_idle` missing → host drove over the device's EOP tail.
- TB `ctrl_read`: 4-bit `tog` reg — `~4'b0001` is truthy, toggle never
  advanced (make toggles 1 bit).
- EP0 parsed the request from `rq*` registers on the same clk it latched
  them (stale reads); later, from `nq*` combinational views — but sized
  `remain` from the stale `sel` (added `new_sel`).
- TB bulk drains reset the accumulator per call — 432 "lost" bytes were
  accounting, not data (drain_in is append-based).
- iverilog gotchas: task array ports unsupported; functions need ≥1
  input; `{a,b}[6:0]` part-select is SV-only.

This session:
- **#9 (P0, from the parse split)**: `EP_PARSE` latched
  `addr_pending`/`cfg_pending` UNCONDITIONALLY, and status completion
  applied them unconditionally — so the GET_DESCRIPTOR(config) transfer
  (wValue lo byte = 0) latched `addr_pending=0` and **wiped dev_addr at
  its status stage** (device went deaf mid-enumeration). Fix: per-transfer
  ownership tags `pend_addr`/`pend_cfg` latched in EP_PARSE, applied only
  by the owning transfer's status stage.
- **#10 (self-inflicted, caught by TB)**: a REGISTERED txfifo write gate
  (`tx_wr_d <= tx_strobe && !full`) is off-by-one vs a comb gate at the
  write edge: a byte armed at cnt=15 lands at cnt=16 → count corruption.
  Reverted to the comb gate; instead the *pop-decision* headroom was
  resized for the 3-stage pipe (`txfifo_cnt < 13`: full 16 − 3 in flight
  − 1 margin). Lesson: when pipelining a strobe, re-derive every
  count-coupled invariant.
- iverilog: `$time` in expressions scales to the module time unit (ns),
  while `%t` display uses the precision unit (ps) — a `$time > N` filter
  written from `%t` output is silently off by 1000× (cost a debug detour).

## Subagent history (why the repo looks like it does)

Two child agents were used early (previous session): a reviewer (USB
semantics audit — its P0 finding about the CDC notification endpoint is
applied; residuals 0x0C/0x800D, wire orders, PIN_TYPE 101001, Fomu pins
all confirmed) and a worker (cancelled mid-run, none of its files
merged). All subagent work was later stopped at the user's request; do
not relaunch without being asked.

## Loose ends

- `FOMU_UART_OVER_USB_PLANv2.md` still describes the pre-audit design
  (§4 endpoint map, §2 FIFO depths, §4 module inventory). Update §4 for
  the EP2/IAD/EF-class layout, the rx-FIFO BRAM deviation, and the
  4-stage EP0 pipeline — or the doc will contradict the RTL.
- M6 decision (see above) — user sign-off needed for any `--freq` change.
- `tb/tb_usb_serial.v` never polls EP2 — an IN poll to EP2 expecting
  NAK would close the loop on the notif-endpoint behavior.
- Debug scratch files from this session live in `/tmp/dbg/` (`tb_dbg.v`,
  `dbg*.log`) — disposable. `build/` is gitignored.
- Plan §3 promised no `.yosys.sv`/`.verilator.sv` split — still true.
- Everything green is COMMITTED at HEAD (this was a loose end; done).
  The commit includes the updated HANDOFF.md.

## Tool versions/paths

- `iverilog` (`-g2012`), `vvp` on PATH; `yosys`, `nextpnr-ice40` on PATH
  (yosyshq builds); `vir` at `~/.local/virdant/bin/vir` (0.4.0).
- Reference repos (read-only): `~/repos/valentyusb` (semantics oracle),
  `~/repos/fomu-workshop` (migen SB_IO lowering, fomu-pvt.pcf).