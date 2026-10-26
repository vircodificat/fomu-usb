# Testbenches: simulating the Fomu USB device

Before you ever flash a bitstream, you want proof your USB device works. This project's entire verification story lives in `tb/` and is run by `make test` — three self-checking Icarus Verilog (`iverilog`) testbenches that build from "known-answer unit tests" all the way up to a full-device acceptance test where a simulated USB host enumerates the device and pumps bulk traffic through it.

This page walks through each piece of test infrastructure: what it checks, how it passes/fails, and how to run it. If you're new to USB itself, start with [usb-primer.md](usb-primer.md); for where these blocks sit in the design, see [system-overview.md](system-overview.md).

## Why testbenches (and why this level of effort)?

A testbench is Verilog code that exists only in simulation: it generates stimulus for your design (the DUT, "device under test"), watches the outputs, and decides pass/fail. Three ideas recur throughout `tb/`:

- **Self-checking tests.** No eyeballing waveforms — the TB compares results against expected values and prints `PASS`/`FAIL` lines, ending with a summary line like `TB_CRC: ALL TESTS PASSED`. That makes it scriptable in CI (`make test` fails if any test fails).
- **Known-answer tests.** The lowest layers (CRC) are tested against *vectors computed by an independent implementation* (valentyusb's Python), so a matching result means both are right, not just self-consistent.
- **BFMs (Bus Functional Models).** Higher up, instead of hand-crafting every wire transition, the TB instantiates a *model of a USB host* (`tb/usb_host_bfm.v`) that speaks real USB at the raw D+/D− level. The TB then says "send a SETUP token to address 42" instead of toggling nets bit by bit.

## `tb_crc.v` — CRC known-answer tests

The simplest TB, and the foundation: it verifies the two serial CRC LFSRs, `usb::Crc5` (tokens/SOF) and `usb::Crc16` (data packets) — Virdant mods in `src/usb.vir`, compiled Verilog in `build/usb.sv` — described in [usb-crc.md](usb-crc.md).

Structure:

- A free-running 48 MHz clock (`always #10.416 clk = ~clk;`) — the same clock the real core runs on.
- One instance each of `\usb::Crc5` and `\usb::Crc16`, both fed by the *same* serial `din`/`adv` stream, so one feed checks both LFSRs at once.
- Small tasks that drive bits LSB-first: `send_bit`, `seed`, `send_bytes`.

What it checks (four kinds of vectors, all cross-checked against valentyusb):

1. **CRC5 token generator** (`check_crc5_token`): feed `addr(7)+ep(4)` LSB-first; the *wire* CRC — complemented and bit-reversed from the LFSR state — must equal valentyusb's `crc5_token` values (e.g. `addr=92, ep=0 -> 5'h1C`).
2. **CRC5 checker residual** (`check_crc5_residual`): feed the 11 token bits *plus the 5 wire CRC bits*; the LFSR must land on the magic residual `5'h0C`. This is exactly what a receiver does: run the CRC over everything and check the leftover state.
3. **CRC16 generator** (`check_crc16`): feed payload bytes LSB-first; the two wire bytes (low then high) must match valentyusb's CRC-16/USB output. Vectors include the empty payload, `"Fomu"`, 9 bytes, a 64-byte maximum-length bulk packet, and more.
4. **CRC16 checker residual** (`check_crc16_residual`): payload + 16 wire bits → LFSR state `16'h800D`.

Pass/fail: every mismatch increments the `errors` counter and prints a `FAIL:` line with got vs. expected; the final block prints `TB_CRC: ALL TESTS PASSED` or `TB_CRC: %0d FAILURES`.

Run it:

```
make test-crc
```

which compiles `ext/usb_uart.sv`, `tb/sb_ram_sim.v`, `build/builtin.sv`, `build/usb.sv` and `tb/tb_crc.v` with iverilog and runs the result with `vvp` (see the `test-crc` target in the `Makefile`).

## `tb_usb_phy.v` — PHY loopback / encode tests

The next level up: whole packets across the raw D+/D− nets, exercising the receive PHY ([usb-rx-phy.md](usb-rx-phy.md)) and transmit PHY ([usb-tx-phy.md](usb-tx-phy.md)).

Wiring:

- `pullup(usb_dp)` / `pulldown(usb_dn)` model the device's 1.5 kΩ D+ pull-up: when *nobody* drives the nets, the bus sits at idle **J** (dp=1, dn=0), just like real Full-Speed USB.
- The core's RX chain (`\usb::RxPhy` + `\usb::RxPacket`) hangs straight on those nets.
- The core's `\usb::TxPhy` drives the *same* nets through a tri-state (`assign usb_dp = dut_oe ? dut_dp : 1'bz;`) — exactly what the SB_IO pad does on real hardware.
- The host BFM is a second driver on those nets.

Because both the BFM's TX and the core's TX are tri-stated onto one net pair, this TB closes a **loopback**: BFM-sent packets must be decoded correctly by the core's RX, and core-sent packets must be captured byte-identically by the BFM's *independent* receiver — a genuine cross-check, since the two sides share no RTL.

The scenarios (T1–T9):

| Test | What it proves |
|---|---|
| T1 | SETUP token to addr=3 ep=0 decodes with right PID/addr/EP/CRC |
| T2 | SOF frame 1234 decodes |
| T3 | 8-byte DATA0 (a real `GET_DESCRIPTOR` shape) decodes; the flopped head window (bytes 0..7, what `endpoint::Ep0` latches) *and* the payload RAM both match via the packet RAM read port |
| T4 | A deliberately corrupted-CRC data packet must **not** pass (`host_send_data_corrupt`) — a negative test |
| T5 | TX round-trip: 16-byte DATA1 captured by the BFM *and* decoded by the core's own RX |
| T6 | TX zero-length packet (ZLP) survives |
| T7 | TX handshake (STALL) captured |
| T8 | TX stuffing torture: 64 × `0xFF` = 512 consecutive raw 1s |
| T9 | TX max-length 64-byte mixed pattern |

Two testbench idioms worth stealing:

- **Sticky capture of 1-clock pulses.** The DUT raises `tk_valid`/`dt_valid` for a single clock — easy to miss while your task is busy waiting. So an `always @(posedge clk)` block latches every pulse plus its shadow fields into `s_*` regs, and the `wait_tk`/`wait_dt`/`wait_cap` tasks poll the sticky flag with a timeout.
- **Streaming a payload RAM.** The RX packet payload lives in a RAM with a read port (`dt_rd_addr`/`dt_rd_data`); the `snap_dt` task walks all 64 addresses and copies the bytes into a big shift register for comparison.

Run it:

```
make test-usb-phy
```

(the `test-usb-phy` target compiles `ext/usb_uart.sv`, `tb/sb_ram_sim.v`, `build/builtin.sv`, `build/usb.sv`, `tb/usb_host_bfm.v` (as an include), and `tb/tb_usb_phy.v`.)

## `usb_host_bfm.v` — a USB host as a Verilog module

The Bus Functional Model is the workhorse of both higher testbenches. A BFM is a module that models *the protocol behavior of the other side of the link* — here, a USB host — so the TB can speak protocol-level ("send a data packet") instead of bit-level. `usb_host_bfm.v` is **never synthesized and never compiled standalone**; it's an include file of tasks/macros, and both TBs instantiate it as a module on the dp/dn nets.

### TX: driving raw D+/D− (NRZI, bit-stuffing, CRC)

The BFM builds packets from scratch with its own *behavioural* implementations — deliberately independent of the RTL, so agreement between BFM and core is meaningful (the primitives themselves are known-answer-verified in `tb_crc.v`).

- **Pad drivers** are tri-state, just like the core's: strong while transmitting, `z` otherwise, letting the pull-up hold idle J.
- **One bit time = 4 clocks** at 48 MHz (`drive_sym_4`) — the Full-Speed 12 Mbit/s rate.
- **NRZI**: a raw 0 toggles the line level, a raw 1 holds it (`nrzi_send`). The sync pattern is the NRZI of `00000001` from idle, i.e. the classic `kjkjkjkk` (`send_sync`).
- **Bit stuffing**: a stuffed 0 is inserted after any run of six raw 1s across the whole post-sync stream (`stuffed_send`), with the run counter reset per packet.
- **CRC5/CRC16** are behavioural functions (`crc5_step`/`crc16_step`) with the same polynomial/seed as the RTL; wire bits are complemented bit-reversals.
- **EOP** is SE0 for two bit times then one J bit, then release (`send_eop`).

On top of those primitives sit the public tasks a TB calls:

- `host_send_token(pid, addr, ep)` — sync + PID + 11 token bits + CRC5 + EOP;
- `host_sof(frame)` — token-shaped: an 11-bit frame number + CRC5;
- `host_send_data0/1` — payload comes from the public `host_data_bytes[]`/`host_data_len` globals the TB fills first;
- `host_send_data_corrupt(pid)` — same, but the final CRC wire bit is inverted, for negative tests;
- `host_send_ack/nak/stall`;
- `host_reset` — SE0 held for 10 ms of simulated time (480k clocks), a real USB bus reset;
- `host_idle(cycles)` and `wait_idle` — `wait_idle` waits for the bus to settle back at J before driving, so the BFM never collides with the device's EOP tail.

### RX: a passive, clock-recovering receiver

While the host isn't transmitting, the BFM *listens*. It's not fed a clock-enable by the DUT — it does its own 4× oversampling clock recovery: any dp/dn edge resets a 2-bit phase counter, and it samples mid-bit. That means it can decode *any* transmitter with a sane bit period, not just this core. Decoded bits are NRZI-unstuffed (six 1s must be followed by a dropped 0, else the packet is marked bad), assembled into PID + payload bytes (payload + 16-bit CRC = 66 legal bytes; a 67th is flagged as an overflow), and CRC-checked by the residual trick: everything post-PID through CRC16 must leave the LFSR at `16'h800D`.

When a packet completes (EOP = SE0 held long enough), the BFM publishes a **capture** into public regs — `cap_valid` (a 1-clock pulse), `cap_pid`, `cap_is_data`, `cap_len`, `cap_crc_ok`, `cap_bytes[0:63]` — which is how the TBs read back what the device said.

The BFM also self-checks: `host_expect_handshake`/`host_expect_data` wait with a timeout and set the public `host_fail` flag on mismatch; both TBs fold `host.host_fail` into their final summary so the BFM's own expectations gate pass/fail too.

## `tb_usb_serial.v` — full-device acceptance test

The big one: the whole CDC-ACM device ([usb-serial.md](usb-serial.md)) — real USB core, real descriptors, real echo application — against the host BFM doing a *realistic enumeration and data session* (the full scenario is summarized in the header comment of `tb/tb_usb_serial.v`).

- The DUT is not the platform wrapper (its generated `module Top` has no parameters to play with); instead the TB instantiates the **pieces of the topology that `export mod Top` in `src/top.vir` wires**, directly:
  - `\usb::UsbPor` — the power-on reset, a plain Virdant mod whose count of 8,400,000 cycles (~175 ms at 48 MHz — the USB attach delay before the D+ pull-up is released) is **inlined, with no parameter to shorten it**, so this sim eats the full 8.4M-cycle attach delay. `por_done` is wired to `usb_dp_pu`, exactly as `src/top.vir` does.
  - `\usb::UsbController` — the core itself (`build/usb.sv`), with its byte-stream side wired to the echo server.
  - `\usb::UsbPads` — the ext SB_IO pad pair (`ext/usb_pads.sv`), the only `inout` in the design (behaviourally modelled by `tb/sb_io_sim.v`, below).
- The echo path is the real one: the generated `\echo::EchoServer` (from `src/echo.vir`) consumes the core's byte-stream interface and renders bytes onto a UART at 115200 8N1 — and the TB wires `uart_rxd = uart_txd`, the *loopback jumper* (in `src/top.vir` it is registered; here it is a plain assign — one clk of loop latency, invisible to the 8N1 receiver), so bytes that go out the "UART" come right back and get echoed to the host. This exercises the UART path from [usb-uart.md](usb-uart.md) too.

The scenario (S1–S7):

1. **Connect.** Wait for the DUT's `usb_dp_pu` (pull-up enable) — the device signaling "I'm here". This is where the ~175 ms POR count is paid for: the sim waits out the full 8,400,000 clocks before the pull-up appears.
2. **Enumerate** like a real OS driver: `GET_DESCRIPTOR(device)` at address 0 (checking `idVendor 0x1209`, `idProduct 0x7A10`, `bDeviceClass 0xEF`/IAD...), `SET_ADDRESS(42)`, `GET_DESCRIPTOR(config)` (75 bytes — checking the IAD, CDC/ACM comm interface, notification interrupt EP, and the two bulk EPs), string descriptors, `SET_CONFIGURATION(1)`.
3. **Negative test:** a token to the *old* address 0 must be met with silence.
4. **Bulk loopback:** 512 bytes in 64-byte packets, with SOF tokens interleaved before every transaction, DATA0/DATA1 toggle checked in both directions, NAK-retry (device FIFO full → host drains the echo and retries, exactly like a real driver), and a final drain; all 512 echoed bytes must come back in order.
5. **CDC control:** `SET_LINE_CODING` and `GET_LINE_CODING` are *stripped* — the echo server never reads line coding — so both must **STALL**; `SET_CONTROL_LINE_STATE` is accepted. (S5b adds a regression case: bulk loopback must keep working *while* EP0 is stalled.)
6. (SOFs throughout — see step 4.)
7. **Bus reset mid-stream** (`host.host_reset`, 10 ms of SE0) → device must return to address 0 and become unconfigured → re-enumerate at address 7 → echo still works.

The TB builds all of this from control-transfer helpers (`ctrl_setup`/`ctrl_read`/`ctrl_write`/`ctrl_no_data`) and bulk helpers (`bulk_out_try`/`bulk_in_try`/`drain_in`) that layer on the BFM's packet tasks — so the *host side* implements real USB driver state (toggles, retries, status stages) rather than just poking the DUT.

Run it:

```
make test-usb-serial
```

(this is the longest-running sim: the full 8.4M-cycle POR attach delay, plus a 10 ms bus reset alone is ~480k clock edges.)

## `sb_io_sim.v` — making SB_IO exist in simulation

`SB_IO` is an iCE40 *primitive*: a real hard pad cell on the FPGA. `ext/usb_pads.sv` instantiates two of them (PIN_TYPE `6'b101001`) because they are the only `inout` in the design. yosys/nextpnr know `SB_IO` natively — but `iverilog` does not, so any TB that elaborates `usb::UsbPads` would fail.

`tb/sb_io_sim.v` is the fix: a behavioural stand-in with the same port list (`PACKAGE_PIN`, `OUTPUT_ENABLE`, `D_OUT_0`, `D_IN_0`) that models the tristate in two `assign`s:

```verilog
assign PACKAGE_PIN = OUTPUT_ENABLE ? D_OUT_0 : 1'bz;
assign D_IN_0 = PACKAGE_PIN;
```

(strong drive while enabled, floating otherwise, and the input always tracks the pad.) Because it's a plain Verilog module named `SB_IO`, iverilog happily elaborates the unmodified ext pads; because yosys sees a *definition* only when you pass it this file (the synth flow never does), there's no conflict. Note `tb_usb_phy.v` doesn't need it — it models the tristate with its own `assign`s and instantiates only `usb::RxPhy`/`usb::TxPhy`/`usb::RxPacket` — but `tb_usb_serial.v` compiles the real `\usb::UsbPads` (which contains SB_IO instances), so the `test-usb-serial` target includes `tb/sb_io_sim.v` in that build only.

## `sb_ram_sim.v` — making SB_RAM40_4K exist in simulation

Same pattern, block-RAM flavour: the USB core's packet/payload RAMs (`usb::RxRam`, `usb::SnapRam` in `src/usb.vir`) lower onto `SB_RAM40_4K` cells, which yosys/nextpnr treat as native but iverilog again cannot elaborate. `tb/sb_ram_sim.v` is a behavioural stand-in that the `test-crc`, `test-usb-phy` and `test-usb-serial` targets all include, because every one of them compiles `build/usb.sv`.

## Running the tests

```
make test            # build + all three TBs (the primary acceptance gate)
make test-crc        # CRC known-answer vectors
make test-usb-phy    # PHY loopback / encode tests
make test-usb-serial # full-device acceptance: enumeration + bulk echo
```

Each target first runs `tools` (which verifies `iverilog`, `yosys`, `nextpnr-ice40` and `vir` are on PATH), compiles with `iverilog -g2012 -gspecify`, runs the sim with `vvp`, and passes/fails on the TB's own exit summary. There is deliberately **no hardware/upload target** anywhere in the Makefile — simulation and synthesis ([build-flow.md](build-flow.md)) are automated; flashing is the operator's call.

## Key takeaways

- Testbenches are Verilog that exists only in simulation: they generate stimulus, check outputs, and print pass/fail — this project's TBs are fully **self-checking** and end with an `ALL TESTS PASSED` summary line, so `make test` works as a CI gate.
- **Known-answer tests** at the bottom of the stack (CRC vectors vs. valentyusb) anchor everything above them; a **BFM** (bus functional model) at the top lets you write protocol-level scenarios (`send a SETUP token`, `NAK-retry this packet`) instead of wire-level ones.
- `usb_host_bfm.v` is a complete *second USB implementation in behaviour*: its own NRZI, bit-stuffing, CRC5/16, 4× oversampling clock recovery, and packet capture — so core-vs-BFM agreement is a real cross-check, not a tautology.
- A little simulation plumbing goes a long way: `pullup`/`pulldown` on the nets for idle-J, tri-state `assign`s for shared driving, sticky flags to catch 1-clock pulses, and behavioural `sb_io_sim.v`/`sb_ram_sim.v` models so iverilog can elaborate a design that instantiates the `SB_IO`/`SB_RAM40_4K` primitives.
- Negative tests matter as much as positive ones: corrupt-CRC packets must fail the check, tokens to stale addresses must be ignored, line-coding requests must STALL, and a bus reset must knock the device back to a re-enumerable state.
- Run them with `make test`, or individually with `make test-crc` / `test-usb-phy` / `test-usb-serial`. Note the full-device sim is slow by design: the inlined 8.4M-cycle POR (~175 ms at 48 MHz) plus 10 ms bus resets are all simulated honestly.
