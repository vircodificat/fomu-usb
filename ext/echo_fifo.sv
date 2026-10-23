// EchoFifo — ext body for src/echo.vir (module \echo::EchoFifo).
//
// A 16x8 circular FIFO.  Virdant cannot express indexed memories inside
// a plain `mod` (the same limitation that keeps the USB core in ext/),
// so the storage primitive lives here while the echo server (flow
// control, sender/receiver wiring, the hold slot) stays in Virdant.
//
// Interface contract (mirrors the old shift-register version, but is
// actually a correct FIFO):
//   push_valid: push_data enters the tail on this clk edge.
//   pop_en:     pop_data (the head) is consumed this cycle; the next
//               byte becomes the head on the clk edge.
//   pop_data is the head byte, valid whenever count != 0.  It changes
//   only on the edge that consumes it, so a pop strobe and the byte it
//   delivers are coincident (the sender loads pop_data combinationally
//   on the same edge).
//   count: number of bytes currently stored (0..16).
//   room:  count < 12 -- flow control for the USB core's rx pop path,
//          sized so the couple of in-flight bytes in the core's
//          rx_pop_d -> rx_valid_r -> rx_valid_rr pipeline always fit
//          above the room threshold.
//
// Internal behavior: 16 slots, wrap-around write and read pointers
// (4 bits), and a saturating add/subtract count.  A byte is pushed to
// slot wr_ptr and popped from slot rd_ptr; both pointers advance and
// wrap independently, giving 16 slots of true FIFO order -- bytes come
// out in exactly the order they went in, with no stale slots and no
// loss, regardless of the current fill level.

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
        // count_q saturates: push and pop together is a no-op.
        case ({push_valid, pop_en})
            2'b10: count_q <= (count_q == 5'd16) ? 5'd16 : count_q + 5'd1;
            2'b01: count_q <= (count_q == 5'd0)  ? 5'd0  : count_q - 5'd1;
            default: count_q <= count_q;
        endcase
    end

    // The head byte is always slots[rd_ptr] when count_q != 0.  We can't
    // use a variable index into a reg array port directly in portable
    // Verilog-2001 style here (iverilog handles it, but nextpnr's ice40
    // path prefers a distributed mux), so spell it out as a 16:1 mux.
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

    assign pop_data = rd_mux;
    assign count    = count_q;
    assign room     = (count_q < 5'd12);

endmodule

`default_nettype wire
