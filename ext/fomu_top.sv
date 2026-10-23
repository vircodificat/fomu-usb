// Fomu UART-over-USB PoC — ext mod Top: pad glue, POR, loopback, LED.
// Part of ~/projects/fomu-usb (see FOMU_UART_OVER_USB_PLANv2.md §2/§5).
// Hand-written Verilog-2005, vendored as an ext mod (plan §0 rules 1-3).
//
// Plain `module Top` (nothing Virdant-emitted instantiates it — plan §3):
// D+/D- go through explicit SB_IO cells (iCE40-native tristate pad,
// PIN_TYPE 6'b101001), exactly the lowering migen performs for a 1-bit
// Tristate on iCE40 and which valentyusb rides on real Fomu hardware:
//   migen/build/lattice/common.py (LatticeiCE40TristateImpl):
//     Instance("SB_IO", p_PIN_TYPE=0b101001, io_PACKAGE_PIN=io,
//              i_OUTPUT_ENABLE=oe, i_D_OUT_0=o, o_D_IN_0=i)
//
// Port contract note: src/top.vir declares usb_dp/usb_dn `outgoing Bit`
// because Virdant cannot express a bidirectional pad yet (plan §5); the
// Verilog Top needs them `inout` for the SB_IO PACKAGE_PIN.  The future
// real `export mod Top for ice40::Fomu` replaces this file (plan §9).
//
// Timing: POR holds the core in reset and usb_dp_pu low for ~175 ms
// (POR_CYCLES at 48 MHz), then releases the pull-up to attach to the host.
// POR_CYCLES is overridable so testbenches can shorten power-on.
`timescale 1ns/1ps

module \top::FomuTop #(
    parameter [23:0] POR_CYCLES = 24'd8_400_000   // ~175 ms @ 48 MHz
)(
    input  wire clock,        // 48 MHz board clock (pin F4)
    inout  wire usb_dp,       // USB D+ (pin A1)
    inout  wire usb_dn,       // USB D- (pin A2)
    output wire usb_dp_pu,    // D+ 1.5k pull-up enable (pin A4)
    output wire led_red,      // active low (pin B5)
    output wire led_green,    // active low (pin A5)
    output wire led_blue,     // active low (pin C5)
    // Byte-stream interface (plan §2).  The byte-level consumer lives
    // in Virdant (the echo server, instantiated by the platform top);
    // this ext module only exposes the USB core's FIFO stream.
    output wire [7:0] rx_data,   // byte from the host
    output wire       rx_valid,  // 1-clk strobe per byte
    output wire       tx_ready,  // tx FIFO has room
    input  wire [7:0] tx_data,   // byte toward the host
    input  wire       tx_strobe, // 1-clk push strobe
    input  wire       rx_ready   // low = consumer full: stop popping
);
    // ------------------------------------------------------------------
    // Power-on reset: count up to POR_CYCLES, then release core + pull-up
    // ------------------------------------------------------------------
    reg [23:0] por_cnt = 24'd0;
    reg        por_done = 1'b0;
    always @(posedge clock) begin
        if (!por_done) begin
            if (por_cnt >= POR_CYCLES)
                por_done <= 1'b1;
            else
                por_cnt <= por_cnt + 24'd1;
        end
    end
    // Attach: pull-up released only after POR (plan §2)
    assign usb_dp_pu = por_done;

    // The core reset is REGISTERED one clk behind por_done: the por_done
    // flop must sit next to the usb_dp_pu pad (its dominant load), and a
    // pad-adjacent flop sourcing the core-wide reset forced the placer to
    // route the reset OUT of the IO region and back into the fabric
    // (the 4.9 ns first hop of the vir-flow critical path, then a ~6-LUT
    // chain through the EP0 register reset muxes to resp_pid's clock
    // enable).  A dedicated flop with no IO load is placed in the core
    // region instead.  +1 clk of reset assertion at a ~175 ms POR is
    // free; the core still leaves reset before it can drive the bus
    // (oe is gated by the in-reset core).
    reg por_rst = 1'b0;
    always @(posedge clock) por_rst <= por_done;
    wire core_rst = ~por_rst;   // sync, active high

    // ------------------------------------------------------------------
    // The core.  The byte stream goes straight out to the ports; the
    // Virdant echo server (see src/echo.vir / the platform top) sits
    // between rx and tx.
    // ------------------------------------------------------------------
    wire       dp_tx, dn_tx, oe;
    wire       dp_rx, dn_rx;

    usb_uart u_core(
        .clk(clock),
        .rst(core_rst),
        .dp_rx(dp_rx),
        .dn_rx(dn_rx),
        .dp_tx(dp_tx),
        .dn_tx(dn_tx),
        .oe(oe),
        .rx_data(rx_data),
        .rx_valid(rx_valid),
        .tx_data(tx_data),
        .tx_strobe(tx_strobe),
        .rx_ready(rx_ready),
        .tx_ready(tx_ready),
        .bus_reset_o(),
        .connected_o()
    );

    // ------------------------------------------------------------------
    // Pads: SB_IO tristate cells, one OUTPUT_ENABLE for the pair
    // ------------------------------------------------------------------
    SB_IO #(.PIN_TYPE(6'b101001)) sb_io_dp (
        .PACKAGE_PIN(usb_dp),
        .OUTPUT_ENABLE(oe),
        .D_OUT_0(dp_tx),
        .D_IN_0(dp_rx)
    );
    SB_IO #(.PIN_TYPE(6'b101001)) sb_io_dn (
        .PACKAGE_PIN(usb_dn),
        .OUTPUT_ENABLE(oe),
        .D_OUT_0(dn_tx),
        .D_IN_0(dn_rx)
    );

    // ------------------------------------------------------------------
    // LED heartbeat (proves life; active low on PVT): red blinks after
    // POR at ~5.7 Hz, off during POR; green/blue off.
    // ------------------------------------------------------------------
    reg [23:0] hb_cnt = 24'd0;
    always @(posedge clock) hb_cnt <= hb_cnt + 24'd1;
    assign led_red   = por_done ? hb_cnt[22] : 1'b1;
    assign led_green = 1'b1;
    assign led_blue  = 1'b1;
endmodule
