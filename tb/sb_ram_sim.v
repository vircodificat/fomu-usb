// Fomu UART-over-USB PoC — behavioural SB_RAM40_4K model for iverilog
// testbenches.  iverilog sees this file ONLY; yosys/nextpnr treat
// SB_RAM40_4K as a native cell and never see it.  Semantics mirror the
// yosys ice40 cells_sim.v model (MODE 0 = 256x16 only, the only mode
// this design uses):
//   - registered read on posedge RCLK, gated by RE && RCLKE;
//   - per-bit write mask: a SET MASK bit is NOT written (MODE 0 uses
//     MASK directly);
//   - read-during-write to the same address returns the OLD byte
//     (both ports nonblocking on the same edge);
//   - uninitialized entries read X (SB_RAM40_4K has no implicit zero
//     init; the design relies on fresh-X for SnapRam's never-written
//     upper half).
`timescale 1ns/1ps

module SB_RAM40_4K #(
    parameter WRITE_MODE = 0,
    parameter READ_MODE  = 0,
    parameter INIT_FILE  = ""
)(
    output [15:0] RDATA,
    input         RCLK,
    input         RCLKE,
    input         RE,
    input  [10:0] RADDR,
    input         WCLK,
    input         WCLKE,
    input         WE,
    input  [10:0] WADDR,
    input  [15:0] MASK,
    input  [15:0] WDATA
);
    reg [15:0] memory [0:255];
    reg [15:0] rdata_q;

    // INIT_FILE ($readmemh hex image), for initialized ROMs (DescRom).
    initial begin
        if (INIT_FILE != "")
            $readmemh(INIT_FILE, memory);
    end

    always @(posedge WCLK) begin
        if (WE && WCLKE) begin
            if (!MASK[ 0]) memory[WADDR[7:0]][ 0] <= WDATA[ 0];
            if (!MASK[ 1]) memory[WADDR[7:0]][ 1] <= WDATA[ 1];
            if (!MASK[ 2]) memory[WADDR[7:0]][ 2] <= WDATA[ 2];
            if (!MASK[ 3]) memory[WADDR[7:0]][ 3] <= WDATA[ 3];
            if (!MASK[ 4]) memory[WADDR[7:0]][ 4] <= WDATA[ 4];
            if (!MASK[ 5]) memory[WADDR[7:0]][ 5] <= WDATA[ 5];
            if (!MASK[ 6]) memory[WADDR[7:0]][ 6] <= WDATA[ 6];
            if (!MASK[ 7]) memory[WADDR[7:0]][ 7] <= WDATA[ 7];
            if (!MASK[ 8]) memory[WADDR[7:0]][ 8] <= WDATA[ 8];
            if (!MASK[ 9]) memory[WADDR[7:0]][ 9] <= WDATA[ 9];
            if (!MASK[10]) memory[WADDR[7:0]][10] <= WDATA[10];
            if (!MASK[11]) memory[WADDR[7:0]][11] <= WDATA[11];
            if (!MASK[12]) memory[WADDR[7:0]][12] <= WDATA[12];
            if (!MASK[13]) memory[WADDR[7:0]][13] <= WDATA[13];
            if (!MASK[14]) memory[WADDR[7:0]][14] <= WDATA[14];
            if (!MASK[15]) memory[WADDR[7:0]][15] <= WDATA[15];
        end
    end

    always @(posedge RCLK) begin
        if (RE && RCLKE)
            rdata_q <= memory[RADDR[7:0]];
    end

    assign RDATA = rdata_q;
endmodule
