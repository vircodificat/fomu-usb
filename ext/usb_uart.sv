// Fomu UART-over-USB PoC — vendored USB Full-Speed CDC-ACM device core.
// Part of ~/projects/fomu-usb (see FOMU_UART_OVER_USB_PLANv2.md §4).
// Hand-written Verilog-2005, vendored as an ext mod (plan §0 rules 1-3).
// USB protocol constants cross-referenced against valentyusb (BSD-3-Clause,
// ~/repos/valentyusb) as reading reference only; all code here is original.
//
// Single clock domain: 48 MHz (4 clk cycles per USB FS bit).  No PLL, no CDC.
// Scope (plan §1): FS only, one configuration, EP0 control + EP1 IN/OUT
// bulk, no suspend/resume, no SOF bookkeeping.  EP2 IN interrupt exists in
// the descriptors only (CDC notification endpoint; Linux cdc_acm requires
// its presence but the core never arms it — every IN to EP2 is NAKed).
//
// Module inventory (bottom-up):
//   (usb_crc5 / usb_crc16, the serial CRC LFSRs, migrated to
//    src/usb.vir as usb::Crc5 / usb::Crc16 -- USB 2.0 §8.3.5)
//   usb_fifo16x8          16x8 sync FIFO
//   (usb_rx_phy, the RX PHY, migrated to src/usb.vir as usb::RxPhy)
//   (usb_rx_packet, the bit stream -> packet record assembler,
//    migrated to src/usb.vir as usb::RxPacket -- except its 64x8
//    payload block RAM, which stays here as \usb::RxRam: Virdant
//    cannot express an indexed memory, the same limitation that
//    keeps \echo::EchoFifo in ext/)
//   (usb_tx_phy, the TX PHY, migrated to src/usb.vir as usb::TxPhy)
//   (usb_ep0, the control transfer SM + CDC requests, migrated to
//    src/usb.vir as usb::Ep0)
//    src/usb.vir as usb::Ep0)
//   (usb_serial, the EP1 bulk endpoint protocol, migrated to
//    src/usb.vir as usb::Serial)
//   (usb_uart, the core top itself -- token router, response
//    arbiter, snapshot loader, rx push, loopback pipe -- migrated
//    to src/usb.vir as usb::UsbUart)
//   What remains here: the memory primitives Virdant cannot
//   express (indexed memories) + the fixed-parameter FIFO wrappers
//   the .vir top instantiates:
//     \usb::TxFifo / \usb::RxFifo   ext wrappers over usb_fifo16x8
//     \usb::RxRam                   rx packet payload RAM
//     \usb::SnapRam                 bulk-IN snapshot RAM
//     \usb::DescRom                 descriptor ROM (initialized BRAM)
//   usb_uart              core top: ROM, router, arbiter, FIFOs, loopback
//
// Bit/wire-order conventions used throughout (verified against valentyusb):
//  - every byte goes on the wire LSB first;
//  - PID byte = {~pid[3:0], pid[3:0]}; pid[1:0]: 01=token 11=data 10=handshake;
//  - token = sync + PID + addr[6:0] + endp[3:0] + crc5[4:0], all LSB first;
//  - data   = sync + PID + payload bytes + crc16 (low byte then high byte);
//  - CRC5 poly x^5+x^2+1 seed 0x1F; CRC16 poly x^16+x^15+x^2+1 seed 0xFFFF;
//  - TX CRC wire bits: crc_wire[i] = ~crc[W-1-i] (complement + bit reverse);
//  - RX CRC check by residual: feed the LFSR every post-PID bit including
//    the received CRC wire bits and compare the final state (token CRC5:
//    5'h0C; data CRC16: 16'h800D).  The receiver cannot know the payload/
//    CRC boundary before EOP, so computed-vs-received is not an option.

// ===========================================================================
// usb_fifo16x8 — synchronous FIFO, default 16-deep x 8-bit (plan §2).
// DEPTH=64 on the bulk-OUT rx side is a documented plan deviation: a
// max-size (64 B) bulk-OUT packet must be absorbed atomically, which a
// 16-deep FIFO can never accept under the "whole packet must fit" rule;
// 64 is also exactly what a real device's endpoint buffer holds.
// rd_data is the combinational head (RAM=0) or the head delayed one clk
// (RAM=1: block-RAM storage with a free-running registered read port —
// the 64-deep rx side uses it, taking 512 flops and their per-byte
// write-decode cones out of the fabric).  CONTRACT: wr_en must be
// pre-gated with !full by the caller (the write path must not read
// `cnt`, or the full compare rides every byte's clock-enable cone —
// a 48 MHz killer on the 64-deep rx FIFO; both instantiation sites
// gate it in the register that produces wr_en).  rd_en must likewise
// be caller-gated with !empty: the empty compare used to sit in
// do_rd, chaining off the count registers into the txfifo read
// pointer's clock enable (the vir-flow critical path, 23.1 ns).
// Both sites comply: the rx popper pre-gates with !rxfifo_empty, and
// the snapshot loader pops exactly min(count,64) bytes (counted at
// arm; it is the only txfifo popper, so count only grows during the
// load).  Simultaneous push+pop supported; with
// RAM=1 the popped byte is on rd_data one clk after rd_en.
// ===========================================================================
module usb_fifo16x8 #(
    parameter DEPTH = 16,
    parameter AW    = 4,
    parameter RAM   = 0
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        wr_en,
    input  wire [7:0]  wr_data,
    input  wire        rd_en,
    output wire [7:0]  rd_data,
    output wire [6:0]  count,
    output wire       full,
    output wire       empty
);
    reg [AW-1:0] wptr, rptr;
    reg [6:0] cnt;

    // wr_en is caller-gated with !full, rd_en with !empty (see header
    // CONTRACT): no `cnt` terms here.  The empty compare in do_rd once
    // chained off the count registers into the txfifo read pointer's
    // clock enable -- the vir-flow critical path (23.1 ns).  Both
    // callers gate their pops (rx: !rxfifo_empty in rx_pop_d; tx: the
    // snapshot loader pops min(count,64) and is the only popper).
    wire do_wr = wr_en;
    wire do_rd = rd_en;

    generate
        if (RAM != 0) begin : g_ram
            (* ram_style = "block" *) reg [7:0] mem [0:DEPTH-1];
            reg [7:0] rd_q;
            always @(posedge clk) begin
                if (do_wr)
                    mem[wptr] <= wr_data;
                rd_q <= mem[rptr];     // free-running registered read
            end
            assign rd_data = rd_q;
        end else begin : g_ff
            (* no_ram *) reg [7:0] mem [0:DEPTH-1];
            always @(posedge clk) begin
                if (do_wr)
                    mem[wptr] <= wr_data;
            end
            assign rd_data = mem[rptr];
        end
    endgenerate

    always @(posedge clk) begin
        if (rst) begin
            wptr <= {AW{1'b0}};
            rptr <= {AW{1'b0}};
            cnt  <= 7'd0;
        end else begin
            if (do_wr)
                wptr <= wptr + {{(AW-1){1'b0}}, 1'b1};
            if (do_rd)
                rptr <= rptr + {{(AW-1){1'b0}}, 1'b1};
            case ({do_wr, do_rd})
                2'b10: cnt <= cnt + 7'd1;
                2'b01: cnt <= cnt - 7'd1;
                default: ;
            endcase
        end
    end

    assign count    = cnt;
    assign full     = (cnt == DEPTH[6:0]);
    assign empty    = (cnt == 7'd0);
endmodule

// ===========================================================================
// \usb::RxRam -- the 64x8 payload block RAM, the ext body of
// usb::RxRam (src/usb.vir).  What remains of usb_rx_packet after its
// migration to Virdant (usb::RxPacket, cycle-exact): Virdant cannot
// express an indexed memory, so the payload store stays vendored
// Verilog here, instantiated from the .vir side.
//
// One write port (registered address wa + one enable -- no decode
// tree, no per-byte enable fanout) and one registered sync read port
// (present ra, the byte is on rd the NEXT clk) that replaces the 64:1
// read mux a flopped payload would put on the 48 MHz critical path.
// Writes beyond byte 63 never happen (usb::RxPacket pre-gates we with
// !(bc >= 64); over-length packets set bad there, so dt_valid never
// fires and no consumer ever sees a truncated payload).  Read-during-
// write to one address returns the OLD byte (nonblocking): the rx
// push of packet N reads packet N's bytes (written long before the
// push starts), never a byte of a concurrently-arriving packet N+1.
// Port contract identical to the old dt_ram (dt_wr / dt_rd_addr /
// dt_rd_data); (* ram_style = "block" *) exactly as before, so yosys
// maps it to the same SB_RAM.
// ===========================================================================
module \usb::RxRam (
    input  wire       clk,
    input  wire       we,
    input  wire [5:0] wa,
    input  wire [7:0] wd,
    input  wire [5:0] ra,
    output reg  [7:0] rd
);
    (* ram_style = "block" *) reg [7:0] mem [0:63];

    always @(posedge clk) begin
        if (we)
            mem[wa] <= wd;
        rd <= mem[ra];    // registered sync read; read-during-write = OLD byte
    end
endmodule
// ===========================================================================
// \usb::TxFifo / \usb::RxFifo -- ext bodies of usb::TxFifo / usb::RxFifo
// (src/usb.vir): thin fixed-parameter wrappers over usb_fifo16x8
// (Virdant ext mods cannot carry parameters).  The behavior contract
// (caller-gated wr_en/rd_en, count terms never in the FIFO's own
// enables) is the usb_fifo16x8 header's; the wrappers add nothing.
// ===========================================================================
module \usb::TxFifo (
    input  wire       clk,
    input  wire       rst,
    input  wire       wr_en,
    input  wire [7:0] wr_data,
    input  wire       rd_en,
    output wire [7:0] rd_data,
    output wire [6:0] count,
    output wire       full,
    output wire       empty
);
    usb_fifo16x8 u (   // defaults: DEPTH=16, AW=4, RAM=0 (flops, comb head)
        .clk(clk), .rst(rst),
        .wr_en(wr_en), .wr_data(wr_data),
        .rd_en(rd_en), .rd_data(rd_data),
        .count(count), .full(full), .empty(empty));
endmodule

module \usb::RxFifo (
    input  wire       clk,
    input  wire       rst,
    input  wire       wr_en,
    input  wire [7:0] wr_data,
    input  wire       rd_en,
    output wire [7:0] rd_data,
    output wire [6:0] count,
    output wire       full,
    output wire       empty
);
    usb_fifo16x8 #(.DEPTH(64), .AW(6), .RAM(1)) u (   // BRAM, registered head
        .clk(clk), .rst(rst),
        .wr_en(wr_en), .wr_data(wr_data),
        .rd_en(rd_en), .rd_data(rd_data),
        .count(count), .full(full), .empty(empty));
endmodule

// ===========================================================================
// \usb::SnapRam -- the bulk-IN snapshot RAM, ext body of usb::SnapRam
// (src/usb.vir).  64x8 block RAM: one write port (the snapshot loader's
// index, one enable) and one registered sync read (the tx byte fetch
// index).  The 7-bit ports index a 64-entry array with Verilog
// out-of-range semantics (write ignored, read X) -- exactly the old
// `snap[li] <= txfifo_rd; snap_q <= snap[byte_rd_addr]`: the tx fetch
// index can lead the consumed bytes during the packet tail, and a
// leading fetch must read X, not an aliased byte.
// ===========================================================================
module \usb::SnapRam (
    input  wire       clk,
    input  wire       we,
    input  wire [6:0] wa,
    input  wire [7:0] wd,
    input  wire [6:0] ra,
    output reg  [7:0] rd
);
    (* ram_style = "block" *) reg [7:0] mem [0:63];

    always @(posedge clk) begin
        if (we)
            mem[wa] <= wd;
        rd <= mem[ra];    // registered sync read
    end
endmodule

// ===========================================================================
// \usb::DescRom -- the descriptor ROM, ext body of usb::DescRom
// (src/usb.vir).  256x8 initialized block RAM, registered sync read
// (ra -> rd next clk).  Contiguous pages: sel0 device@0 (18 B), sel1
// config@18 (75 B), sel2 str0@93, sel3 str1@97 (10 B), sel4 str2@107
// (28 B); sel5 (line coding) and sel6 (status) are dynamic/trivial and
// bypass the ROM.  The initial block is the vendored usb_uart's, moved
// verbatim (a BRAM read replaces the desc_byte() case cascade: a
// 128-deep case is ~20 ns of LUT chain, a BRAM read is a register).
// ===========================================================================
module \usb::DescRom (
    input  wire       clk,
    input  wire [7:0] ra,
    output reg  [7:0] rd
);
    (* ram_style = "block" *) reg [7:0] desc_rom [0:255];
    initial begin
            desc_rom[8'd0] = 8'h12;
            desc_rom[8'd1] = 8'h01;
            desc_rom[8'd2] = 8'h00;
            desc_rom[8'd3] = 8'h02;
            desc_rom[8'd4] = 8'hEF;
            desc_rom[8'd5] = 8'h02;
            desc_rom[8'd6] = 8'h01;
            desc_rom[8'd7] = 8'h40;
            desc_rom[8'd8] = 8'h09;
            desc_rom[8'd9] = 8'h12;
            desc_rom[8'd10] = 8'h10;
            desc_rom[8'd11] = 8'h7A;
            desc_rom[8'd12] = 8'h00;
            desc_rom[8'd13] = 8'h01;
            desc_rom[8'd14] = 8'h01;
            desc_rom[8'd15] = 8'h02;
            desc_rom[8'd16] = 8'h00;
            desc_rom[8'd17] = 8'h01;
            desc_rom[8'd18] = 8'h09;
            desc_rom[8'd19] = 8'h02;
            desc_rom[8'd20] = 8'h4B;
            desc_rom[8'd21] = 8'h00;
            desc_rom[8'd22] = 8'h02;
            desc_rom[8'd23] = 8'h01;
            desc_rom[8'd24] = 8'h01;
            desc_rom[8'd25] = 8'h80;
            desc_rom[8'd26] = 8'h32;
            desc_rom[8'd27] = 8'h08;
            desc_rom[8'd28] = 8'h0B;
            desc_rom[8'd29] = 8'h00;
            desc_rom[8'd30] = 8'h02;
            desc_rom[8'd31] = 8'h02;
            desc_rom[8'd32] = 8'h02;
            desc_rom[8'd33] = 8'h00;
            desc_rom[8'd34] = 8'h00;
            desc_rom[8'd35] = 8'h09;
            desc_rom[8'd36] = 8'h04;
            desc_rom[8'd37] = 8'h00;
            desc_rom[8'd38] = 8'h00;
            desc_rom[8'd39] = 8'h01;
            desc_rom[8'd40] = 8'h02;
            desc_rom[8'd41] = 8'h02;
            desc_rom[8'd42] = 8'h00;
            desc_rom[8'd43] = 8'h01;
            desc_rom[8'd44] = 8'h05;
            desc_rom[8'd45] = 8'h24;
            desc_rom[8'd46] = 8'h00;
            desc_rom[8'd47] = 8'h10;
            desc_rom[8'd48] = 8'h01;
            desc_rom[8'd49] = 8'h05;
            desc_rom[8'd50] = 8'h24;
            desc_rom[8'd51] = 8'h01;
            desc_rom[8'd52] = 8'h00;
            desc_rom[8'd53] = 8'h01;
            desc_rom[8'd54] = 8'h04;
            desc_rom[8'd55] = 8'h24;
            desc_rom[8'd56] = 8'h02;
            desc_rom[8'd57] = 8'h02;
            desc_rom[8'd58] = 8'h05;
            desc_rom[8'd59] = 8'h24;
            desc_rom[8'd60] = 8'h06;
            desc_rom[8'd61] = 8'h00;
            desc_rom[8'd62] = 8'h01;
            desc_rom[8'd63] = 8'h07;
            desc_rom[8'd64] = 8'h05;
            desc_rom[8'd65] = 8'h82;
            desc_rom[8'd66] = 8'h03;
            desc_rom[8'd67] = 8'h08;
            desc_rom[8'd68] = 8'h00;
            desc_rom[8'd69] = 8'h10;
            desc_rom[8'd70] = 8'h09;
            desc_rom[8'd71] = 8'h04;
            desc_rom[8'd72] = 8'h01;
            desc_rom[8'd73] = 8'h00;
            desc_rom[8'd74] = 8'h02;
            desc_rom[8'd75] = 8'h0A;
            desc_rom[8'd76] = 8'h00;
            desc_rom[8'd77] = 8'h00;
            desc_rom[8'd78] = 8'h00;
            desc_rom[8'd79] = 8'h07;
            desc_rom[8'd80] = 8'h05;
            desc_rom[8'd81] = 8'h81;
            desc_rom[8'd82] = 8'h02;
            desc_rom[8'd83] = 8'h40;
            desc_rom[8'd84] = 8'h00;
            desc_rom[8'd85] = 8'h00;
            desc_rom[8'd86] = 8'h07;
            desc_rom[8'd87] = 8'h05;
            desc_rom[8'd88] = 8'h01;
            desc_rom[8'd89] = 8'h02;
            desc_rom[8'd90] = 8'h40;
            desc_rom[8'd91] = 8'h00;
            desc_rom[8'd92] = 8'h00;
            desc_rom[8'd93] = 8'h04;
            desc_rom[8'd94] = 8'h03;
            desc_rom[8'd95] = 8'h09;
            desc_rom[8'd96] = 8'h04;
            desc_rom[8'd97] = 8'h0A;
            desc_rom[8'd98] = 8'h03;
            desc_rom[8'd99] = 8'h46;
            desc_rom[8'd100] = 8'h00;
            desc_rom[8'd101] = 8'h6F;
            desc_rom[8'd102] = 8'h00;
            desc_rom[8'd103] = 8'h6D;
            desc_rom[8'd104] = 8'h00;
            desc_rom[8'd105] = 8'h75;
            desc_rom[8'd106] = 8'h00;
            desc_rom[8'd107] = 8'h20;
            desc_rom[8'd108] = 8'h03;
            desc_rom[8'd109] = 8'h46;
            desc_rom[8'd110] = 8'h00;
            desc_rom[8'd111] = 8'h6F;
            desc_rom[8'd112] = 8'h00;
            desc_rom[8'd113] = 8'h6D;
            desc_rom[8'd114] = 8'h00;
            desc_rom[8'd115] = 8'h75;
            desc_rom[8'd116] = 8'h00;
            desc_rom[8'd117] = 8'h20;
            desc_rom[8'd118] = 8'h00;
            desc_rom[8'd119] = 8'h55;
            desc_rom[8'd120] = 8'h00;
            desc_rom[8'd121] = 8'h41;
            desc_rom[8'd122] = 8'h00;
            desc_rom[8'd123] = 8'h52;
            desc_rom[8'd124] = 8'h00;
            desc_rom[8'd125] = 8'h54;
            desc_rom[8'd126] = 8'h00;
            desc_rom[8'd127] = 8'h20;
            desc_rom[8'd128] = 8'h00;
            desc_rom[8'd129] = 8'h50;
            desc_rom[8'd130] = 8'h00;
            desc_rom[8'd131] = 8'h6F;
            desc_rom[8'd132] = 8'h00;
            desc_rom[8'd133] = 8'h43;
            desc_rom[8'd134] = 8'h00;
            desc_rom[8'd135] = 8'h00;
            desc_rom[8'd136] = 8'h00;
            desc_rom[8'd137] = 8'h00;
            desc_rom[8'd138] = 8'h00;
            desc_rom[8'd139] = 8'h00;
            desc_rom[8'd140] = 8'h00;
            desc_rom[8'd141] = 8'h00;
            desc_rom[8'd142] = 8'h00;
            desc_rom[8'd143] = 8'h00;
            desc_rom[8'd144] = 8'h00;
            desc_rom[8'd145] = 8'h00;
            desc_rom[8'd146] = 8'h00;
            desc_rom[8'd147] = 8'h00;
            desc_rom[8'd148] = 8'h00;
            desc_rom[8'd149] = 8'h00;
            desc_rom[8'd150] = 8'h00;
            desc_rom[8'd151] = 8'h00;
            desc_rom[8'd152] = 8'h00;
            desc_rom[8'd153] = 8'h00;
            desc_rom[8'd154] = 8'h00;
            desc_rom[8'd155] = 8'h00;
            desc_rom[8'd156] = 8'h00;
            desc_rom[8'd157] = 8'h00;
            desc_rom[8'd158] = 8'h00;
            desc_rom[8'd159] = 8'h00;
            desc_rom[8'd160] = 8'h00;
            desc_rom[8'd161] = 8'h00;
            desc_rom[8'd162] = 8'h00;
            desc_rom[8'd163] = 8'h00;
            desc_rom[8'd164] = 8'h00;
            desc_rom[8'd165] = 8'h00;
            desc_rom[8'd166] = 8'h00;
            desc_rom[8'd167] = 8'h00;
            desc_rom[8'd168] = 8'h00;
            desc_rom[8'd169] = 8'h00;
            desc_rom[8'd170] = 8'h00;
            desc_rom[8'd171] = 8'h00;
            desc_rom[8'd172] = 8'h00;
            desc_rom[8'd173] = 8'h00;
            desc_rom[8'd174] = 8'h00;
            desc_rom[8'd175] = 8'h00;
            desc_rom[8'd176] = 8'h00;
            desc_rom[8'd177] = 8'h00;
            desc_rom[8'd178] = 8'h00;
            desc_rom[8'd179] = 8'h00;
            desc_rom[8'd180] = 8'h00;
            desc_rom[8'd181] = 8'h00;
            desc_rom[8'd182] = 8'h00;
            desc_rom[8'd183] = 8'h00;
            desc_rom[8'd184] = 8'h00;
            desc_rom[8'd185] = 8'h00;
            desc_rom[8'd186] = 8'h00;
            desc_rom[8'd187] = 8'h00;
            desc_rom[8'd188] = 8'h00;
            desc_rom[8'd189] = 8'h00;
            desc_rom[8'd190] = 8'h00;
            desc_rom[8'd191] = 8'h00;
            desc_rom[8'd192] = 8'h00;
            desc_rom[8'd193] = 8'h00;
            desc_rom[8'd194] = 8'h00;
            desc_rom[8'd195] = 8'h00;
            desc_rom[8'd196] = 8'h00;
            desc_rom[8'd197] = 8'h00;
            desc_rom[8'd198] = 8'h00;
            desc_rom[8'd199] = 8'h00;
            desc_rom[8'd200] = 8'h00;
            desc_rom[8'd201] = 8'h00;
            desc_rom[8'd202] = 8'h00;
            desc_rom[8'd203] = 8'h00;
            desc_rom[8'd204] = 8'h00;
            desc_rom[8'd205] = 8'h00;
            desc_rom[8'd206] = 8'h00;
            desc_rom[8'd207] = 8'h00;
            desc_rom[8'd208] = 8'h00;
            desc_rom[8'd209] = 8'h00;
            desc_rom[8'd210] = 8'h00;
            desc_rom[8'd211] = 8'h00;
            desc_rom[8'd212] = 8'h00;
            desc_rom[8'd213] = 8'h00;
            desc_rom[8'd214] = 8'h00;
            desc_rom[8'd215] = 8'h00;
            desc_rom[8'd216] = 8'h00;
            desc_rom[8'd217] = 8'h00;
            desc_rom[8'd218] = 8'h00;
            desc_rom[8'd219] = 8'h00;
            desc_rom[8'd220] = 8'h00;
            desc_rom[8'd221] = 8'h00;
            desc_rom[8'd222] = 8'h00;
            desc_rom[8'd223] = 8'h00;
            desc_rom[8'd224] = 8'h00;
            desc_rom[8'd225] = 8'h00;
            desc_rom[8'd226] = 8'h00;
            desc_rom[8'd227] = 8'h00;
            desc_rom[8'd228] = 8'h00;
            desc_rom[8'd229] = 8'h00;
            desc_rom[8'd230] = 8'h00;
            desc_rom[8'd231] = 8'h00;
            desc_rom[8'd232] = 8'h00;
            desc_rom[8'd233] = 8'h00;
            desc_rom[8'd234] = 8'h00;
            desc_rom[8'd235] = 8'h00;
            desc_rom[8'd236] = 8'h00;
            desc_rom[8'd237] = 8'h00;
            desc_rom[8'd238] = 8'h00;
            desc_rom[8'd239] = 8'h00;
            desc_rom[8'd240] = 8'h00;
            desc_rom[8'd241] = 8'h00;
            desc_rom[8'd242] = 8'h00;
            desc_rom[8'd243] = 8'h00;
            desc_rom[8'd244] = 8'h00;
            desc_rom[8'd245] = 8'h00;
            desc_rom[8'd246] = 8'h00;
            desc_rom[8'd247] = 8'h00;
            desc_rom[8'd248] = 8'h00;
            desc_rom[8'd249] = 8'h00;
            desc_rom[8'd250] = 8'h00;
            desc_rom[8'd251] = 8'h00;
            desc_rom[8'd252] = 8'h00;
            desc_rom[8'd253] = 8'h00;
            desc_rom[8'd254] = 8'h00;
            desc_rom[8'd255] = 8'h00;
    end
    always @(posedge clk) rd <= desc_rom[ra];
endmodule
