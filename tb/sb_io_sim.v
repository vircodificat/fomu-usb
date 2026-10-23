// Fomu UART-over-USB PoC — behavioural SB_IO model for iverilog testbenches.
// Part of ~/projects/fomu-usb (see FOMU_UART_OVER_USB_PLANv2.md §7).
// iverilog sees this file ONLY; yosys/nextpnr treat SB_IO as a native cell
// and never see it.  Wiring mirrors how migen lowers a 1-bit Tristate on
// iCE40 (PIN_TYPE 6'b101001):
//   migen/build/lattice/common.py (LatticeiCE40TristateImpl):
//     Instance("SB_IO", p_PIN_TYPE=0b101001,
//              io_PACKAGE_PIN=io, i_OUTPUT_ENABLE=oe,
//              i_D_OUT_0=o, o_D_IN_0=i)
`timescale 1ns/1ps

module SB_IO #(
    parameter [5:0] PIN_TYPE = 6'b000000,
    parameter       PULLUP   = 1'b0
)(
    inout  wire PACKAGE_PIN,
    input  wire OUTPUT_ENABLE,
    input  wire D_OUT_0,
    output wire D_IN_0
);
    // Tristate pad: strong drive only while OUTPUT_ENABLE; otherwise the pad
    // floats (driven by nobody) — D_IN_0 always tracks the pad.
    assign PACKAGE_PIN = OUTPUT_ENABLE ? D_OUT_0 : 1'bz;
    assign D_IN_0 = PACKAGE_PIN;
endmodule