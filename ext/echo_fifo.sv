// EchoFifo -- ext body for src/echo.vir (module \echo::EchoFifo).
//
// A 16x8 circular FIFO.  Virdant cannot express indexed memories inside
// a plain `mod` (the same gap that keeps the USB core's RAMs in ext/),
// so the storage primitive lives here while the echo server (flow
// control, sender/receiver wiring, the hold slot) stays in Virdant.
//
// Interface contract (mirrored by the docstring on ext mod EchoFifo in
// src/echo.vir):
//   push_valid: push_data enters the tail on this clk edge.  No push-
//               side backpressure; the room output throttles upstream.
//   pop_en:     the head is consumed on this clk edge; the popped byte
//               appears on pop_data on the NEXT cycle (registered read,
//               see below).  The echo server pairs its one-clk-delayed
//               sender load strobe (pop_r in src/echo.vir) with exactly
//               that byte; it pops at most one byte per sender frame
//               (~4170 clks apart) and never pops an empty queue.
//   pop_data:   registered head byte.  Captured on every clk edge, so
//               it tracks the current head between pops and presents
//               the just-popped byte for exactly the one cycle after a
//               pop.  Keeping the 16:1 mux behind its own flop keeps
//               it off the echo server's single-cycle load path.
//   count:      number of bytes currently stored (0..16, saturating).
//   room:       high while count < 12 -- flow control for the USB
//               core's rx pop path (rx_ready in src/usb.vir).  At the
//               12 threshold 4 slots remain free, enough for the bytes
//               still in flight in the core's pop pipeline
//               (rx_pop_d -> rx_valid_r -> rx_valid_rr) after the
//               throttle takes effect.
//
// Internal behavior: 16 slots, wrap-around write and read pointers
// (4 bits), and a saturating count.  A byte is pushed to slot wr_ptr
// and popped from slot rd_ptr; both pointers advance and wrap
// independently, so bytes come out in exactly the order they went in,
// with no stale slots and no loss, at any fill level.  A simultaneous
// push and pop leaves count unchanged.

`default_nettype none

module \echo::EchoFifo (
    input  wire        clock,
    input  wire [7:0]  push_data,
    input  wire        push_valid,
    input  wire        pop_en,
    output wire [7:0]  pop_data,
    output wire [4:0]  count,
    output wire        room
);

    // 16 slots of 8 bits each.
    reg [7:0] slots [0:15];

    reg [3:0] wr_ptr;
    reg [3:0] rd_ptr;
    reg [4:0] count_q;

    initial begin
        wr_ptr  = 4'd0;
        rd_ptr  = 4'd0;
        count_q = 5'd0;
    end

    always @(posedge clock) begin
        if (push_valid) begin
            slots[wr_ptr] <= push_data;
            wr_ptr <= wr_ptr + 4'd1;
        end
        if (pop_en) begin
            rd_ptr <= rd_ptr + 4'd1;
        end
        // count_q saturates at both ends: a simultaneous push and pop
        // leaves it unchanged.
        case ({push_valid, pop_en})
            2'b10: count_q <= (count_q == 5'd16) ? 5'd16 : count_q + 5'd1;
            2'b01: count_q <= (count_q == 5'd0)  ? 5'd0  : count_q - 5'd1;
            default: count_q <= count_q;
        endcase
    end

    // The head byte is always slots[rd_ptr] while count_q != 0.  The
    // read is spelled out as an explicit 16:1 case mux: portable
    // Verilog-2001 style, and it maps to simple distributed logic on
    // the iCE40 synthesis/pnr path.
    reg [7:0] rd_mux;
    always @(*) begin
        case (rd_ptr)
            4'd0:  rd_mux = slots[0];
            4'd1:  rd_mux = slots[1];
            4'd2:  rd_mux = slots[2];
            4'd3:  rd_mux = slots[3];
            4'd4:  rd_mux = slots[4];
            4'd5:  rd_mux = slots[5];
            4'd6:  rd_mux = slots[6];
            4'd7:  rd_mux = slots[7];
            4'd8:  rd_mux = slots[8];
            4'd9:  rd_mux = slots[9];
            4'd10: rd_mux = slots[10];
            4'd11: rd_mux = slots[11];
            4'd12: rd_mux = slots[12];
            4'd13: rd_mux = slots[13];
            4'd14: rd_mux = slots[14];
            default: rd_mux = slots[15];
        endcase
    end

    // Registered read of the head mux: captured with the OLD rd_ptr on
    // every edge (non-blocking), so on a pop edge pop_data_q takes the
    // byte being consumed and presents it one clk later -- exactly when
    // the echo server's registered load strobe (pop_r in src/echo.vir)
    // fires.  Between pops (rd_ptr static) the capture keeps pop_data_q
    // equal to the current head.
    reg [7:0] pop_data_q;
    initial   pop_data_q = 8'd0;
    always @(posedge clock)
        pop_data_q <= rd_mux;

    assign pop_data = pop_data_q;
    assign count    = count_q;
    // room: high while fewer than 12 bytes are stored (>= 4 free
    // slots); drives rx_ready in src/usb.vir to throttle the core's rx
    // pop path, with enough headroom left for the bytes still in
    // flight in the core's pop pipeline.
    assign room     = (count_q < 5'd12);

endmodule

`default_nettype wire
