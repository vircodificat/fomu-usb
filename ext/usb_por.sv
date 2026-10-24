// UsbPor — ext body for src/usb.vir (module \usb::UsbPor).
// Part of ~/projects/fomu-usb (see FOMU_UART_OVER_USB_PLANv2.md §2/§5).
// Hand-written Verilog-2005, vendored as an ext mod (plan §0 rules 1-3).
//
// Power-on reset: counts to POR_CYCLES at 48 MHz (~175 ms at the
// hardware default), then releases por_done.  The top wires por_done
// to the D+ pull-up enable (attach) and — registered one clk, inside
// the Virdant UsbController — to the core reset.
//
// Why an ext mod: POR_CYCLES must be overridable at simulation time
// (tb/tb_usb_serial.v shortens it to a few hundred cycles), which
// requires a hand-written Verilog `parameter` on a module the TB
// instantiates directly.  A plain Virdant mod would bake the count
// into generated code the TB cannot reach.  (Virdant's @parameter
// annotations parameterize ext mods at *elaboration* time — the value
// is fixed in the .vir source — which is not what the TB needs.)
`timescale 1ns/1ps

module \usb::UsbPor #(
    parameter POR_CYCLES = 24'd8_400_000   // ~175 ms @ 48 MHz
)(
    input  wire clock,
    output wire por_done
);
    reg [23:0] por_cnt = 24'd0;
    reg        por_done_r = 1'b0;
    always @(posedge clock) begin
        if (!por_done_r) begin
            if (por_cnt >= POR_CYCLES)
                por_done_r <= 1'b1;
            else
                por_cnt <= por_cnt + 24'd1;
        end
    end
    assign por_done = por_done_r;
endmodule
