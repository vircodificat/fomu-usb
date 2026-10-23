# Fomu: UART-over-USB PoC — Plan v2

**Goal.** A hand-written, vendored, plain-Verilog USB Full-Speed device core
that enumerates on the host as a CDC-ACM serial port (`/dev/ttyACM0`) and
echoes bytes in hardware (loopback), built in this repo (`fomu-usb`) as a
simulation + synthesis proof of concept. Fomu has no bridge chip and no
physical UART pins — D+ (A1), D− (A2) and the D+ pull-up enable (A4) are the
only serial path — so any "UART" on this board is necessarily a soft USB
device in the fabric.

This plan supersedes `~/projects/virdant-riscv-fomu/FOMU_UART_OVER_USB_PLAN.md`.
Everything happens in **this** repo; the port-back to `virdant-riscv-fomu` is
described in §9 and is explicitly *not* part of the PoC.

---

## 0. Hard rules (from the design interview)

1. **No new dependencies.** No migen, no litex, no cocotb, no Python anywhere
   in the flow. valentyusb (`~/repos/valentyusb`) is *reading material and
   cross-reference only* — every line we ship is hand-written Verilog we own.
2. **All RTL is vendored plain Verilog in `ext/`**, following the established
   ext-mod pattern (like `virdant-riscv-fomu`'s `ext/Mem.yosys.sv`).
3. **Simulation is iverilog + Verilog testbenches.** No cocotb.
4. **Acceptance = sim + synth only.** All iverilog TBs pass; yosys
   `synth_ice40` and nextpnr place the design cleanly at 48 MHz on UP5K.
   Hardware bring-up happens after the PoC, with the user.
5. **NEVER run `vir upload`, `dfu-util`, `iceprog` or any flashing command.**
   Flashing requires the user's physical intervention; if an upload ever
   seems needed, *ask first*. (The PoC makes this structurally impossible
   anyway — see §5.)
6. Virdant platform support for bidirectional/tristate pins does not exist
   yet; the real `export mod Top for ice40::Fomu` is the user's to write
   later. Until then the PoC top is an **ext mod Top** implemented entirely
   in Verilog. **Do not modify `~/projects/virdant`** — it is read-only
   reference for the compiler's behaviour.
7. The milestone consumer is **hardware loopback** (`rx → tx` in glue). No
   CPU, no Wishbone, no firmware in the USB path. The core still exposes a
   byte-stream FIFO interface internally so the later CPU integration
   (v3: `WbUsbUart` at `0x4000_0000`, making `echo.s` run over USB) is a
   wrapper change, not a redesign.

---

## 1. Why hand-writing is tractable here (scope framing)

Full USB FS from scratch is a months-long expert project — but only if you
need the whole protocol. Our scope cut (from the interview):

- Full-Speed only, single configuration, single interface, bus-powered.
- No suspend/resume, no remote wakeup, no SOF bookkeeping beyond ignoring
  SOF tokens, no isochronous/interrupt endpoints, no alternate settings.
- Enumeration: respond to `GET_DESCRIPTOR` (device/config/string),
  `SET_ADDRESS`, `SET_CONFIGURATION`, `GET_STATUS`, and the three CDC
  requests the host's `cdc_acm` driver actually sends
  (`SET_LINE_CODING`, `GET_LINE_CODING`, `SET_CONTROL_LINE_STATE`).
  Everything else → STALL (which is spec-legal for an unknown request).
- Two bulk endpoints (EP1 IN + EP1 OUT) with byte FIFOs, ACK/NAK only.

That reduces the protocol layer from "2–4 weeks" to a contained state
machine, because we never own more than one control transfer at a time and
never own retransmission policy (we ACK/NAK; the host retries). The
irreducible part is the PHY (clock recovery, NRZI, bit stuffing, CRC, EOP,
bus reset) — that is where the sim-first strategy pays: every bit-level
behaviour is observable in iverilog, which is impossible to do on hardware
without a USB analyzer.

What we reuse from valentyusb (as reference, not source): block
decomposition (`rx/`, `tx/`, `sm/transfer`), the descriptor byte tables
(`cpu/dummyusb.py`), the known-answer vectors in its `*_test.py` unit tests
(CRC, bit-stuff, NRZI), and the *shape* of its Python host model
(`sim/test-common.py`) which I port to Verilog BFM tasks. valentyusb is
BSD-3-Clause; we cite it in the ext file headers as design reference.

---

## 2. Design overview

**Single clock domain: 48 MHz.** The Fomu platform clock (pin F4) is
48 MHz; USB FS is 12 Mbps → 4 clock cycles per bit. The entire core is
sampled/driver logic on that one clock; no PLL, no CDC, no divided clocks.
(48 MHz is also far below what UP5K fabric handles; timing closure should be
trivial.)

**Pin plumbing (the part that needs no Virdant platform support):**
inside the ext `Top`, D+/D− go through explicit `SB_IO` cells — the
iCE40-native tristate pad cell — with the configuration proven on real Fomus
by migen's iCE40 tristate lowerer, which valentyusb's `IoBuf` rides on
(`~/repos/fomu-workshop/litex/deps/migen/migen/build/lattice/common.py`):

```verilog
SB_IO #(.PIN_TYPE(6'b101001))   // PIN_OUTPUT_TRISTATE + PIN_INPUT
       sb_io_dp (
    .PACKAGE_PIN(usb_dp),       // top-level port, PCF-constrained to A1
    .OUTPUT_ENABLE(dp_oe),      // active-high drive enable from the core
    .D_OUT_0(dp_tx),            // what to drive when oe
    .D_IN_0(dp_rx)              // pad readback into the core
);
```

`usb_dp_pu` is a plain output (no `SB_IO` needed): driven low during
power-on reset (~175 ms counter), then high to connect to the host. LED
heartbeat proves life. This is exactly the wiring the future Virdant `Top`
will express via `ice40::SbIo` once platform tristate exists — the Verilog
version is the reference implementation for it.

**Byte-stream interface** (internal module boundary, preserved for port-back):

```
usb_uart (the core, without pad glue):
  in : clock, reset, dp_rx, dn_rx
  out: dp_tx, dn_tx, dp_oe        // one oe for the pair (never drives one line alone)
  out: rx_data[7:0], rx_valid     // FIFO non-empty strobe: one byte held until rx_ready? no —
                                 // PoC: rx_valid pulses one cycle per byte; reader latches same cycle
  in : tx_data[7:0], tx_strobe   // write strobe when tx_ready is high
  out: tx_ready                  // FIFO has room
```

For the PoC the loopback is literally
`tx_data = rx_data; tx_strobe = rx_valid;` in the ext `Top` — same clock,
FIFO write is a single-cycle event, nothing lost. (Documented caveat for
v3: a one-cycle strobe means the future Virdant consumer must either be
combinational or we add `rx_ready`/`Valid` framing then.)

FIFOs: 16 deep × 8 bit, simple LUT RAM/regs — at FS a 64-byte bulk packet
arrives byte-per-byte over ≥64 µs, so 16 bytes of buffering plus NAK when
full is plenty for a loopback.

---

## 3. Repository layout (all in `~/projects/fomu-usb/`)

```
Virdant.toml          [prog] top = "top::Top"  (unchanged)
fomu.pcf              static pin constraints for the PoC synth flow
                      (copied from build/fomu-usb.pcf — the platform-emitted
                       PCF — subset to the ports Top actually declares)
Makefile              build / test / synth targets (NO upload target)
src/
  top.vir             ext mod Top declaration (port contract)
                      + the original `export mod Top for ice40::Fomu`
                        hello-world block, commented out with a pointer to
                        this plan (platform tristate support pending)
ext/
  usb_uart.sv         the USB core: rx PHY, tx PHY, CRC, EP0, bulk serial
                      (hand-written Verilog-2005; multiple modules in one
                       file — see §4)
  fomu_top.sv         module Top: SB_IO×2 glue, POR + pull-up timing,
                      loopback wiring, LED heartbeat
tb/
  sb_io_sim.v         behavioural SB_IO model for iverilog (pad = oe ? D_OUT_0 : 1'bz)
  usb_host_bfm.v      pin-level USB host model: packet encode/decode tasks
  tb_usb_phy.v        PHY unit/integration TB
  tb_crc.v            CRC5/CRC16 known-answer TB
  tb_usb_serial.v     full-device TB: enumeration + loopback echo
```

Naming note: because nothing Virdant-emitted instantiates the ext `Top`,
`ext/fomu_top.sv` defines a plain `module Top(...)`. (Had it been
instantiated, the emitter would require the escaped path name
``\top::Top ``, as in `ext/Mem.yosys.sv`'s ``\mem::Mem ``.) The
`src/top.vir` `ext mod Top` port list must mirror `module Top`'s port list
exactly — it is the human/machine contract until the real platform top
replaces it. `vir build` must stay green with this arrangement (it emits an
empty `top.sv`; ext mods emit no bodies — verified: `build/ice40.sv` is
0 bytes).

No `.yosys.sv`/`.verilator.sv` ifdef split is needed in this repo: the PoC
synth flow (§6) and the iverilog TBs both compile the same `ext/*.sv` files
directly; the split pattern becomes relevant only at port-back if the
`virt` simulator needs to model the core.

---

## 4. The core: `ext/usb_uart.sv` module inventory

Written as Verilog-2005 (yosys `read_verilog` and iverilog both accept it).
Module list, bottom-up; each has a unit TB where marked:

| Module | Responsibility |
|---|---|
| `usb_crc5` / `usb_crc16` | serial LFSR CRC generators/checkers (polynomials x⁵+x²+1; x¹⁶+x¹⁵+x²+1). TB: port the known-answer vectors from valentyusb's `rx/crc_test.py` + `tx/crc_test.py`. |
| `usb_fifo16x8` | 16×8 sync FIFO, `rx`/`tx` sides. |
| `usb_rx_phy` | 2-FF pad sync → 4× oversampling clock recovery (re-align 2-bit phase counter on any transition; sample mid-bit), NRZI decode, bit unstuffing (delete 0 after six 1s), sync pattern detect (SOP), EOP detect (SE0 ≥ 2 bits), **bus reset detect (SE0 ≥ 2.5 µs)**, SE1 → error/ignore. Output: bit-serial stream + packet boundary flags. |
| `usb_rx_packet` | PID capture & PID check (`pid[3:0] == ~pid[7:4]`); token decode (11-bit addr+EP, CRC5 verify); data capture to byte buffer with CRC16 verify. Emits per-packet records: `(valid, pid, addr, endp, data[N], len)`. TB: `tb_usb_phy` drives raw waveforms. |
| `usb_tx_phy` | Response serializer: PID + payload + CRC; bit stuffing; NRZI encode; sync prefix; EOP; drives `dp_tx/dn_tx/oe` with correct timing (2-bit inter-packet guard; ACK turnaround within spec window after EOP detect). TB: waveform capture + decode in `tb_usb_phy`. |
| `usb_ep0` | Control-transfer state machine (spec §8.5.5): SETUP → (optional DATA stage, direction from `bmRequestType`) → STATUS. Descriptor ROM (device 18 B, config ~54 B CDC-ACM, strings). Implements: `GET_DESCRIPTOR`, `SET_ADDRESS` (address register latched at status stage per spec), `SET_CONFIGURATION`, `GET_STATUS` (2 zero bytes), `GET_LINE_CODING` (return stored 7-byte line coding), `SET_LINE_CODING` (accept & discard data stage), `SET_CONTROL_LINE_STATE` (accept). Anything else → STALL. |
| `usb_serial` | Bulk endpoints: EP1 OUT → `rx_fifo` (ACK on good CRC; NAK if full). EP1 IN ← `tx_fifo` (send up to 32 B when armed; NAK if empty; wait host ACK, retransmit-armed if no ACK). Data-toggle tracking per direction, reset on bus reset & `SET_CONFIGURATION`. |
| `usb_uart` | Top of the core: token router (address filter after `SET_ADDRESS`), EP0 vs serial mux, `rx/tx` FIFO byte interface (§2), pull-up request, reset (POR + bus-reset) distribution. |

Endpoint/traffic map:

- **EP0**: control, max packet 64.
- **EP1 OUT** (bulk, host→device) and **EP1 IN** (bulk, device→host), max
  packet 64. CDC-ACM descriptor set follows valentyusb `dummyusb.py`'s
  tables (proven accepted by Linux/macOS/Windows hosts on Fomu hardware):
  `bDeviceClass=0x02`, CDC control interface (`class 0x02`, subclass ACM)
  with **no** interrupt endpoint + CDC data interface (`class 0x0A`) with
  the two bulk endpoints.
- **SOF**: decoded and ignored (it exercises the address filter in the TB).
- VID/PID: `0x1209` (Folknest, the same vendor range Fomu uses) with a
  private PID — propose `0x1209:0x7A10`, product string `"Fomu UART PoC"`.
  Single `localparam` block makes this trivially changeable; note in the
  file that a production device must register a real VID/PID.

Bus-state constants: idle = J; SOP = J→K; reset/EOP = SE0. The D+ pull-up
makes idle J when the device is connected and not driving.

---

## 5. Virdant side (`src/top.vir`)

```virdant
import ice40

// The real Fomu top is pending platform support for bidirectional
// (tristate) pins — see FOMU_UART_OVER_USB_PLANv2.md §5. Until then the
// top is a Verilog ext mod (ext/fomu_top.sv) and this declaration is the
// port contract for it. Port names match the Fomu platform pin names so
// the future real Top keeps the same PCF lines.
ext mod Top {
    //> 48 MHz board clock (pin F4).
    incoming clock : Clock
    //> USB D+ pad (pin A1) — driven via SB_IO inside the ext module.
    outgoing usb_dp : Bit
    //> USB D− pad (pin A2) — driven via SB_IO inside the ext module.
    outgoing usb_dn : Bit
    //> USB D+ pull-up enable (pin A4); low = disconnected from host.
    outgoing usb_dp_pu : Bit
    //> Heartbeat (active-low on PVT).
    outgoing led_red : Bit
    outgoing led_green : Bit
    outgoing led_blue : Bit
}

// export mod Top for ice40::Fomu { ... }   // pending platform tristate
//                                          // support; see plan §5/§9
```

`vir build` with this arrangement emits an empty `top.sv` and succeeds
(ext mods emit no bodies — confirmed in
`virdant/src/verilog/conversion.rs`; `emit_manifest` skips when the
`[prog] top` module has no platform `for` clause). **Consequence:** `vir
bitstream`/`vir upload` fail by construction on this repo
("Top module does not implement a platform"), so there is *no path* by
which tooling here can touch hardware — an extra guarantee of rule 5.

---

## 6. Flows (Makefile)

- **`make build`** — `vir build`. Virdant-side green check only.
- **`make test`** — iverilog TBs (see §7). Runs `iverilog -g2012` over
  `ext/*.sv tb/*.sv`, executes each TB, fails on any error count > 0.
  This is the PoC's primary acceptance.
- **`make synth`** — direct yosys + nextpnr (we cannot use `vir bitstream`
  — no platform `for` on the ext Top — and do not want `vir upload` to ever
  exist as a target):
  ```
  yosys -p "read_verilog ext/usb_uart.sv ext/fomu_top.sv; synth_ice40 -top Top -json build/top.json"
  nextpnr-ice40 --up5k --package uwg30 --json build/top.json \
      --pcf fomu.pcf --asc build/top.asc --freq 48
  ```
  Success = clean exit, no nextpnr timing violation at 48 MHz. Record
  utilization from both reports. (Packaging to `.bin`/DFU is deliberately
  out of scope of the PoC.)

`fomu.pcf` (committed; copied from the platform-emitted
`build/fomu-usb.pcf`, subset to Top's ports):

```
set_io clock F4
set_io usb_dp A1
set_io usb_dn A2
set_io usb_dp_pu A4
set_io led_red B5
set_io led_green A5
set_io led_blue C5
set_frequency clock 48.0
```

Tool availability to verify at milestone M0: `iverilog`, `yosys`,
`nextpnr-ice40` on PATH; `vir` at `~/.local/virdant/bin/vir`.

---

## 7. Verification plan (iverilog, no cocotb)

**`tb/sb_io_sim.v`** — 10-line behavioural `SB_IO`
(`assign pad = OUTPUT_ENABLE ? D_OUT_0 : 1'bz; D_IN_0 = pad;`). iverilog
needs it; yosys treats `SB_IO` as a native cell and never sees this file.

**`tb/usb_host_bfm.v`** — the one nontrivial TB piece (~500 lines of
Verilog tasks): a pin-level USB host. Drives `dp/dn` one 48 MHz cycle at a
time (4 cycles per bit): bus reset (SE0 ≥ 10 ms — sim runs this; at
20.833 ns/cycle ≈ 480k cycles, fine for iverilog), sync/token/data/handshake
packet transmission (stuffing + NRZI + CRC built from the same logic shape
as valentyusb's `sim/test-common.py`, but hand-written Verilog), and
capture/decode of device responses with CRC verification. Tasks:
`host_reset`, `host_send_token(pid,addr,ep)`, `host_send_data0/1(bytes)`,
`host_expect_handshake(pid)`, `host_poll_in(...)`, `host_wait_connect`
(waits for `usb_dp_pu` rise).

**`tb/tb_crc.v`** — known-answer vectors for CRC5/CRC16, ported from
valentyusb's `usbcore/rx/crc_test.py` / `tx/crc_test.py`.

**`tb/tb_usb_phy.v`** — wire the BFM's pin driver straight into
`usb_rx_phy`/`usb_rx_packet`: send token+data packets, assert decoded
PID/addr/EP/bytes/CRC pass; then loop the core's `usb_tx_phy` output back
to the BFM's pin sampler and assert byte round-trips at the waveform level
(incl. stuffing edge cases: six consecutive 1s, 64-byte max packets).

**`tb/tb_usb_serial.v`** — the full-device acceptance TB, instantiating
`module Top` (with `sb_io_sim` for the pads). Sequence:

1. Reset; wait for `usb_dp_pu` to rise (connect).
2. Enumerate: `GET_DESCRIPTOR(device)` — expect 18 bytes, check
   bDeviceClass=0x02, VID/PID; `SET_ADDRESS(42)`;
   `GET_DESCRIPTOR(config)` — expect full CDC-ACM descriptor set;
   `GET_DESCRIPTOR(string)`s; `SET_CONFIGURATION(1)`.
3. Negative test: token to old address (0) → device must not respond
   (timeout, no line activity).
4. Bulk loopback: write 512 bytes (8× 64-byte max-packet packets, both
   toggle phases) → poll IN until all 512 bytes return byte-identical and
   in order; check DATA0/DATA1 toggling on both directions.
5. CDC control traffic mid-stream: `SET_LINE_CODING`, `GET_LINE_CODING`
   round-trip, `SET_CONTROL_LINE_STATE` — accepted (status stage OK).
6. Interleave SOF tokens between every transaction (address-filter
   exercise).
7. Bus reset mid-stream → device returns to address 0 → re-enumerate →
   echo still works (frame this as the final sim acceptance).

All TBs print a `PASS`/`FAIL` summary and `$fatal` on mismatch so
`make test` propagates a nonzero exit.

**Cross-checking the host model itself:** the BFM is our oracle, so it
gets validated against the same vectors the core does (it shares the
CRC/stuffing TBs via includes) and its packet shapes are diffed by eye
against valentyusb's `test-common.py` captures and the USB 2.0 spec
waveform figures (§7.1.9, §7.1.10, §8.3). A wrong host model that
happily talks to a wrong device is the classic sim-first hazard; the
mitigation is strict spec-section citations in the BFM comments.

---

## 8. Milestones & effort

| # | Milestone | Exit criterion | Est. |
|---|---|---|---|
| M0 | Repo skeleton: `Makefile`, `fomu.pcf`, `src/top.vir` ext-mod Top + commented export, empty ext/tb files; tool check | `make build` green (`vir build` OK, empty `top.sv`) | 0.25 d |
| M1 | CRC5/16 + FIFO + `tb_crc` green | `tb_crc` passes with ported valentyusb vectors | 0.5 d |
| M2 | `usb_rx_phy` + `usb_rx_packet`; `tb_usb_phy` RX half green | BFM-driven tokens/data decode correctly | 1 d |
| M3 | `usb_tx_phy` + TX half of `tb_usb_phy` | waveform-level round-trip, stuffing edges | 1 d |
| M4 | `usb_ep0` + descriptor ROM + `usb_serial` bulk endpoints | `tb_usb_serial` steps 1–4 pass (full enumeration + echo) | 1.5 d |
| M5 | CDC control requests, SOF interleaving, negative address test, mid-stream reset re-enumeration (steps 5–7) | `make test` fully green | 1 d |
| M6 | `make synth` clean: yosys warnings clean, nextpnr 48 MHz no violation, utilization ≲1.5 k LC expected | reports archived in `build/` | 0.5 d |

Total ≈ **6 working days** of implementation; the risk register (§10)
explains where the slack lives. All milestones are sim-verifiable except
M6 (synth), and nothing requires hardware or user intervention.

---

## 9. Port-back to `virdant-riscv-fomu` (after the PoC, after user intervention)

Recorded so the PoC doesn't paint us into a corner; not part of this effort.

1. Platform grows bidirectional/tristate port support (user's work), or the
   documented `SbIo` pattern (`PACKAGE_PIN` ← top-level outgoing `usb_dp`/
   `usb_dn`, `D_OUT_0`/`OUTPUT_ENABLE` ← core, `D_IN_0` → core) is blessed.
2. In the target repo: real `export mod Top for ice40::Fomu` with two
   `@parameter("PIN_TYPE", 0b101001)` `ice40::SbIo` instances, pull-up
   release timing, and loopback glue
   (`usb.tx_data := usb.rx_data; usb.tx_strobe := usb.rx_valid`).
3. `src/usb.vir` declares `ext mod UsbUart` with the byte-stream + pad
   interface; `ext/usb_uart.sv`'s `usb_uart` module renamed to the emitted
   instantiation name ``\usb::UsbUart `` (same pattern as
   `ext/Mem.yosys.sv`'s ``\mem::Mem ``). Optionally split into
   `.yosys.sv`/`.verilator.sv` if the `virt` flow must model it.
4. The PoC's ext `Top` file dies; the PCF goes back to being
   platform-emitted; `vir bitstream`/`vir upload` return to the flow —
   still never `vir upload` without the user.
5. v3 (separate discussion): `WbUsbUart` presenting the existing
   `WbUart`-compatible RBR/THR/LSR register interface backed by the byte
   FIFOs, so `sw/uart_demo/echo.s` and friends run unchanged over
   `/dev/ttyACM0`.

---

## 10. Risks & mitigations

| Risk | Mitigation |
|---|---|
| **BFM/host-model fidelity** — a subtly wrong host model greenlights a subtly wrong device | Shared CRC/stuffing primitives between core and BFM, spec-section citations in BFM comments, cross-check packet construction against valentyusb's `test-common.py` (which talks to real hosts) |
| **FS electrical bring-up surprises** (D+/D− swap, pull-up timing, SE0 threshold, PIN_TYPE placement) | Deferred by design to bring-up with the user; the plan's sim already fixes everything observable in waveforms. Bring-up checklist: check `lsusb -v` shows FS (not LS — LS means D± swapped), `dmesg` CDC-ACM bind, pull-up release ≥100 ms after config |
| **yosys/iverilog semantic drift** (latches, X-optimism) | RTL restricted to Verilog-2005, no `initial` state reliance, always-@(*) full assignments; yosys warnings clean is an M6 exit criterion |
| **Resource/timing** | Trivial at 48 MHz; expectation ≲1.5 k LC on a 5280-LC UP5K |
| **ext-mod port contract drift** between `src/top.vir` and `ext/fomu_top.sv` | Both files carry a cross-reference comment; M6 re-runs `vir build` as a consistency check |

**Out of scope (explicit):** CPU/Wishbone path, interrupts, DTR/RTS
semantics, suspend/resume, USB 2.0 high-bandwidth anything, DFU, a real
VID/PID registration, hardware bring-up steps, and every flashing command.
