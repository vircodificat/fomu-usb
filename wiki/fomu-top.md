# `\top::FomuTop` — pad glue, power-on reset, and the LED heartbeat

**Source:** `ext/fomu_top.sv:23` · **Back:** [Home.md](Home.md) · **Next:** [echo-fifo.md](echo-fifo.md)

`\top::FomuTop` is the hand-written Verilog half of the Fomu top level: everything Virdant cannot express lives here — the tristate USB pads, the power-on reset, and the pin drivers. The Virdant half ([top-vir.md](top-vir.md)) instantiates this module and hangs the [echo server](echo-server.md) off its byte-stream ports. See [system-overview.md](system-overview.md) for the big picture.

## Why this module is ext Verilog

Plain Verilog-2005 was chosen because nothing Virdant-emitted instantiates it and because it needs iCE40 primitives (`SB_IO`) that Virdant has no syntax for. See [build-flow.md](build-flow.md) for how ext modules get vendored into the build.

## Power-on reset (~175 ms)

```verilog
parameter [23:0] POR_CYCLES = 24'd8_400_000   // ~175 ms @ 48 MHz
```

A 24-bit counter counts up to `POR_CYCLES` and then sticks at done:

- `por_cnt` / `por_done` (`ext/fomu_top.sv:46-55`)
- `core_rst = ~por_done` — the USB core is held in reset the whole time
- `usb_dp_pu = por_done` — **the D+ 1.5 kΩ pull-up is released only after POR**

The pull-up timing is not cosmetic — it is **attach signalling**. USB is a wired-OR bus: the host sees a device on a Full-Speed port only when D+ is pulled up. If the device asserted the pull-up before its core came out of reset, the host would start enumerating a device that cannot answer. Holding `usb_dp_pu` low through POR guarantees the host's attach detection happens only once the core is alive. `POR_CYCLES` is a parameter so a testbench can shorten power-on; the host sees ~175 ms of "cable unplugged" first. (Full-Speed vs Low-Speed attach is D+ vs D− — see [usb-primer.md](usb-primer.md).)

## SB_IO tristate pads

USB needs a **bidirectional** pad: the device drives D± during packet transmission and releases the pair to receive. On iCE40 that is a `SB_IO` cell with PIN_TYPE `6'b101001` (tristate output + input on D_IN_0):

```verilog
SB_IO #(.PIN_TYPE(6'b101001)) sb_io_dp (
    .PACKAGE_PIN(usb_dp), .OUTPUT_ENABLE(oe),
    .D_OUT_0(dp_tx), .D_IN_0(dp_rx));
```

- `PACKAGE_PIN` is `inout` on the module — the pin itself.
- `OUTPUT_ENABLE` (`oe` from the core) selects drive vs release; **one `oe` is shared for the D+/D− pair**, since USB always drives or releases both lines together.
- This is exactly the lowering migen performs for a 1-bit `Tristate` on iCE40, and what valentyusb rides on real Fomu hardware.

## Byte-stream pass-through ports

The module itself contains **no byte-level logic**. It exposes the USB core's stream directly:

| Port | Dir | Meaning |
|---|---|---|
| `rx_data` | out | byte from the host |
| `rx_valid` | out | 1-clk strobe per byte |
| `tx_ready` | out | tx FIFO has room |
| `tx_data` | in | byte toward the host |
| `tx_strobe` | in | 1-clk push strobe |
| `rx_ready` | in | low = consumer full: stop popping |

The consumer of this stream is the [echo server](echo-server.md), wired in [top-vir.md](top-vir.md). The CDC-ACM layer that makes it a serial port is the core's business — see [usb-serial.md](usb-serial.md).

## LED heartbeat

`hb_cnt` free-runs; `led_red = por_done ? hb_cnt[22] : 1` blinks at ~5.7 Hz once POR finishes and is **off** during POR (active-low on the PVT board). The LED is your one-bit diagnostic: no blink → no 48 MHz clock or stuck in POR. Green/blue are tied off.

## The `inout` vs `outgoing Bit` port contract

`src/top.vir:6-7` declares `usb_dp`/`usb_dn` as `outgoing Bit`, but the Verilog module needs them `inout` for the `SB_IO` `PACKAGE_PIN`. That mismatch is deliberate: **Virdant cannot express a bidirectional pad yet**, so the platform export drives the pins one-way and the ext module owns the real tristate. The real `export mod Top for ice40::Fomu` would replace this file and fix the contract.

## Key takeaways

- POR (~175 ms) holds the core in reset **and** D+ pull-up low; releasing the pull-up *is* USB attach signalling.
- D± go through `SB_IO` cells (PIN_TYPE `6'b101001`) with one shared `OUTPUT_ENABLE` for the pair.
- The module is pure glue: USB core's byte stream straight to ports; the echo server hangs off them in Virdant.
- `usb_dp`/`usb_dn` are `inout` in Verilog but `outgoing Bit` in Virdant — Virdant has no bidirectional-pad syntax yet.
- Red LED blinks ~5.7 Hz after POR — your "is it alive" bit.