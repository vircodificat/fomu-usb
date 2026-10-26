// UsbPads -- ext body of usb::UsbPads (src/usb.vir): the design's only
// bidirectional nets.  Two iCE40 SB_IO tristate pad cells (PIN_TYPE
// 6'b101001), exactly the lowering migen performs for a 1-bit Tristate
// on iCE40 (migen/build/lattice/common.py, LatticeiCE40TristateImpl:
//   Instance("SB_IO", p_PIN_TYPE=0b101001, io_PACKAGE_PIN=io,
//            i_OUTPUT_ENABLE=oe, i_D_OUT_0=o, o_D_IN_0=i))
// No clock, no reset, no state: pure pad plumbing.  Everything around
// this -- the POR (usb::UsbPor), the D+ pull-up enable (usb_dp_pu,
// driven by the POR in src/top.vir), the USB core itself -- is
// unidirectional and lives in Virdant (src/usb.vir, src/top.vir).
//
// Port contract note: the Virdant ext mod declaration (src/usb.vir)
// types the pad side `outgoing Bit` because Virdant cannot express a
// bidirectional pad; the `inout` below is the Verilog-side plumbing
// that gets the SB_IO PACKAGE_PIN nets out to the Top ports (wired in
// src/top.vir's export mod Top: usb_dp := pads.usb_dp etc.) and
// constrained to the physical pins in fomu.pcf (usb_dp = A1,
// usb_dn = A2).  The bidirectionality never crosses the Virdant
// boundary.
`timescale 1ns/1ps

module \usb::UsbPads (
    // Pad side: the physical USB wires (PCF pins A1/A2).
    inout  wire usb_dp,
    inout  wire usb_dn,
    // Core side, into the pads (the usb::UsbTxRx socket channels,
    // flattened by vir into escaped \usb.* port names).
    input  wire \usb.dp_tx ,         // D+ drive level (valid while output_enable)
    input  wire \usb.dn_tx ,         // D- drive level (valid while output_enable)
    input  wire \usb.output_enable , // tristate control: drive the bus when high
    // Core side, out of the pads.
    output wire \usb.dp_rx ,         // D+ sample (SB_IO D_IN_0)
    output wire \usb.dn_rx           // D- sample (SB_IO D_IN_0)
);
    // One OUTPUT_ENABLE for the pair: the core drives both lines
    // together (or neither — while oe is low the pads float to the
    // idle J state held by the host and the D+ 1.5k pull-up).
    SB_IO #(.PIN_TYPE(6'b101001)) sb_io_dp (
        .PACKAGE_PIN   (usb_dp),
        .OUTPUT_ENABLE (\usb.output_enable ),
        .D_OUT_0       (\usb.dp_tx ),
        .D_IN_0        (\usb.dp_rx )
    );
    SB_IO #(.PIN_TYPE(6'b101001)) sb_io_dn (
        .PACKAGE_PIN   (usb_dn),
        .OUTPUT_ENABLE (\usb.output_enable ),
        .D_OUT_0       (\usb.dn_tx ),
        .D_IN_0        (\usb.dn_rx )
    );
endmodule
