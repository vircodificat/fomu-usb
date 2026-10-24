// UsbPads — ext body for src/usb.vir (module \usb::UsbPads).
// Part of ~/projects/fomu-usb (see FOMU_UART_OVER_USB_PLANv2.md §2/§5).
// Hand-written Verilog-2005, vendored as an ext mod (plan §0 rules 1-3).
//
// The minimal `inout` module: the ONLY bidirectional thing in the
// design.  Two iCE40 SB_IO tristate pad cells (PIN_TYPE 6'b101001),
// exactly the lowering migen performs for a 1-bit Tristate on iCE40
// and which valentyusb rides on real Fomu hardware:
//   migen/build/lattice/common.py (LatticeiCE40TristateImpl):
//     Instance("SB_IO", p_PIN_TYPE=0b101001, io_PACKAGE_PIN=io,
//              i_OUTPUT_ENABLE=oe, i_D_OUT_0=o, o_D_IN_0=i)
// No clock, no reset, no state: pure pad plumbing.  Everything around
// this — POR, pull-up enable, the USB core itself — is unidirectional
// and lives in Virdant (src/usb.vir) or the parameterized POR ext mod
// (ext/usb_por.sv).
//
// Port contract note: the Virdant ext mod declaration (src/usb.vir)
// types the pad side `outgoing Bit` because Virdant cannot express a
// bidirectional pad; the `inout` below is the Verilog-side plumbing
// that gets the SB_IO PACKAGE_PIN nets out to the Top ports / PCF
// pins.  The platform wrapper reads these nets with plain `assign`s
// (`assign usb_dp = \pads.usb_dp ;`), exactly the proven glue the old
// \top::UsbController used — the bidirectionality never crosses the
// Virdant boundary.
`timescale 1ns/1ps

module \usb::UsbPads (
    // Pad side: the physical USB wires (PCF pins A1/A2).
    inout  wire usb_dp,
    inout  wire usb_dn,
    // Core side, into the pads.
    input  wire dp_tx,         // D+ drive level (valid while output_enable)
    input  wire dn_tx,         // D- drive level (valid while output_enable)
    input  wire output_enable, // tristate control: drive the bus when high
    // Core side, out of the pads.
    output wire dp_rx,         // D+ sample (SB_IO D_IN_0)
    output wire dn_rx          // D- sample (SB_IO D_IN_0)
);
    // One OUTPUT_ENABLE for the pair: the core drives both lines
    // together (or neither — while oe is low the pads float to the
    // idle J state held by the host and the D+ 1.5k pull-up).
    SB_IO #(.PIN_TYPE(6'b101001)) sb_io_dp (
        .PACKAGE_PIN   (usb_dp),
        .OUTPUT_ENABLE (output_enable),
        .D_OUT_0       (dp_tx),
        .D_IN_0        (dp_rx)
    );
    SB_IO #(.PIN_TYPE(6'b101001)) sb_io_dn (
        .PACKAGE_PIN   (usb_dn),
        .OUTPUT_ENABLE (output_enable),
        .D_OUT_0       (dn_tx),
        .D_IN_0        (dn_rx)
    );
endmodule
