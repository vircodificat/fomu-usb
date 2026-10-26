# Fomu board glue — pads, power-on reset, attach, and the LEDs

**Source:** `ext/usb_pads.sv` + `usb::UsbPor` in `src/usb.vir` + pin map in `fomu.pcf` · **Back:** [Home.md](Home.md) · **Next:** [echo-fifo.md](echo-fifo.md)

This page is the board-level half of the top level: the tristate USB pads, the power-on reset / attach delay, and the LEDs. Everything here exists because the FPGA fabric or the platform cannot do it in plain Virdant — and it is all wired together by the platform export ([top-vir.md](top-vir.md)). See [system-overview.md](system-overview.md) for the big picture.

## The pin map (`fomu.pcf`)

| Pin | Signal | Role |
|---|---|---|
| `F4` | `clock` | 48 MHz board clock (`set_frequency clock 48.0`) |
| `A1` | `usb_dp` | USB D+ (inout, via `SB_IO` cells) |
| `A2` | `usb_dn` | USB D− (inout, via `SB_IO` cells) |
| `A4` | `usb_dp_pu` | D+ 1.5 kΩ pull-up enable — **this is how the device "attaches"** |
| `B5` / `A5` / `C5` | `led_red/green/blue` | LEDs, active low; red = receiving, green = sending |

(The `touch*`/`flash*` pins in the pcf are the platform's standard set; they must be constrained or nextpnr errors out, but the design leaves them dangling.)

## SB_IO tristate pads (`ext/usb_pads.sv`)

USB needs a **bidirectional** pad: the device drives D± during packet transmission and releases the pair to receive. On iCE40 that is a `SB_IO` cell with PIN_TYPE `6'b101001` (tristate output + input on D_IN_0):

```verilog
SB_IO #(.PIN_TYPE(6'b101001)) sb_io_dp (
    .PACKAGE_PIN(usb_dp), .OUTPUT_ENABLE(output_enable),
    .D_OUT_0(\usb.dp_tx ), .D_IN_0(\usb.dp_rx ));
```

`ext/usb_pads.sv` is the body of the `usb::UsbPads` ext mod (declared in `src/usb.vir`) and contains exactly two such cells (one per line) — no clock, no reset, no state. This is exactly the lowering migen performs for a 1-bit `Tristate` on iCE40.

- `PACKAGE_PIN` is `inout` in Verilog — the pin itself.
- `OUTPUT_ENABLE` (the core's `output_enable`) selects drive vs release; **one `oe` is shared for the D+/D− pair**, since USB always drives or releases both lines together (while low, the pads float to the idle J state held by the host and the D+ pull-up).
- The design's **only bidirectional nets** live inside this module. From Virdant's view every wire is unidirectional: the core drives via the `usb` socket's `dp_tx`/`dn_tx` channels gated by `output_enable`, samples via `dp_rx`/`dn_rx` (the `usb::UsbTxRx` socket) — the `inout` is the Verilog-side plumbing that gets the `SB_IO` `PACKAGE_PIN` nets out to the `Top` ports and the PCF pins. Virdant cannot express a bidirectional pad yet, so the bidirectionality never crosses the Virdant boundary.

## Power-on reset / attach delay (`usb::UsbPor` in `src/usb.vir`)

A 24-bit counter counts up to **8_400_000 cycles (~175 ms @ 48 MHz)** and then raises `por_done`, sticky:

```text
mod UsbPor {
    incoming clock  : Clock
    outgoing por_done : Bit
    // por_cnt: counts while !por_done_r, holds at 8_400_000
    // por_done_r: set when por_cnt >= 8_400_000, never cleared
}
```

The count **is the USB attach delay**: how long the device waits before releasing the D+ pull-up and attaching to the host. It is inlined — there is no parameter — so the sim testbench eats the full ~175 ms attach delay too (see [testbenches.md](testbenches.md)).

`por_done` fans out two ways in `src/top.vir`:

- `usb_dp_pu := por_done` — **the D+ 1.5 kΩ pull-up is released only after POR**.
- into `usb::UsbController`, where it is registered one more clk (`por_rst`) and inverted to form `core_rst` — the USB core is held in reset through the whole count plus one clk.

### Why the pull-up timing is not cosmetic

The pull-up is **attach signalling**. USB is a wired-OR bus: the host sees a device on a Full-Speed port only when D+ is pulled up (D− would be the low-speed pull-up — see [usb-primer.md](usb-primer.md)). If the device asserted the pull-up before its core came out of reset, the host would start enumerating a device that cannot answer. Holding `usb_dp_pu` low through POR guarantees the host's attach detection happens only once the core is alive — and because the core's reset (`por_rst`) is registered one clk behind `por_done`, the pull-up rises one clk before the core leaves reset; the core still cannot drive the bus until then (`output_enable` is gated by the in-reset core).

The same unwiring trap exists one level up: **unwired platform ports default to low**, and `usb_dp_pu` low = pull-up off = the host never sees the device. `src/top.vir`'s own comment warns that a wrapper leaving it unwired produces an image no host can ever see.

## Activity LEDs (`src/top.vir` + `usb::UsbUart`)

The LEDs are wired at the top level, mapped onto the core's activity signals:

```text
led_red   := !actual_top.rxing     // active low on the PVT board
led_green := !actual_top.txing
led_blue  := true                  // tied off
```

`rxing`/`txing` come from `usb::UsbUart` inside the core: two free-running 23-bit countdown counters, reloaded to `0x7F_FFFF` (~175 ms) on every `rx`/`tx` pipe payload strobe and counting down to rest. The 1-clk strobes are far too fast to see; the stretch makes sustained traffic read as a solid light and a single packet still visible. Deliberately **no reset** on these counters (power-on zero is enough), and EP0 control traffic does not light them — only bulk data pulses `rx_valid`/`tx_strobe`.

Red lights while the USB is **receiving** (host → device), green while **sending** (device → host). The LED is your one-bit diagnostic: no light under traffic → check the clock, POR, and the pull-up wiring first.

## Key takeaways

- D± go through two `SB_IO` cells (`PIN_TYPE 6'b101001`, one shared `OUTPUT_ENABLE`) in `ext/usb_pads.sv` — the design's only bidirectional nets.
- The Virdant pad-facing wires are unidirectional (the `usb` socket's cosi channels out, soci channels in); the `inout` is Verilog-side plumbing only.
- `usb::UsbPor` counts 8_400_000 cycles (~175 ms) with no parameter; `por_done` releases the D+ pull-up (attach) and, registered once more, forms the core reset.
- Attach = releasing the 1.5 kΩ D+ pull-up (pin A4); getting that wiring wrong makes the device invisible.
- LEDs are active low: red = rxing, green = txing, blue off; the ~175 ms stretch makes activity visible.
