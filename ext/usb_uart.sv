// Fomu UART-over-USB — vendored USB Full-Speed CDC-ACM device core.
// Hand-written Verilog-2005 ext bodies for src/usb.vir: the USB core
// itself (Crc5/Crc16, RxPhy, RxPacket + RxRam, TxPhy, Ep0, Serial and
// the UsbUart top -- token router, response arbiter, snapshot loader,
// rx push, loopback pipe) lives in src/usb.vir.  USB protocol constants
// cross-referenced against valentyusb (BSD-3-Clause, ~/repos/valentyusb)
// as a reading reference only; all code here is original.
//
// Single clock domain: 48 MHz (4 clock cycles per USB FS bit), no PLL, no
// CDC.  FS only, one configuration, EP0 control + EP1 IN/OUT bulk; no
// suspend/resume, no SOF bookkeeping (SOF tokens are captured and
// ignored).  The CDC notification endpoint EP2 IN exists in the
// descriptors only (Linux cdc_acm requires its presence); the core never
// arms it -- every EP2 token is NAKed (usb::UsbUart in src/usb.vir).
//
// Module inventory (this file):
//   usb_fifo16x8      generic synchronous FIFO, default 16-deep x 8-bit
//                     (the rx side instantiates it 64-deep with BRAM
//                     storage; see its banner below)
//   \usb::TxFifo      ext body of usb::TxFifo  -- 16x8, flops, comb head
//   \usb::RxFifo      ext body of usb::RxFifo  -- 64x8, BRAM, registered head
//   \usb::DescRom     ext body of usb::DescRom -- 256x8 initialized BRAM
//
// Why these stay in ext: \usb::TxFifo / \usb::RxFifo must bake their
// shape (Virdant ext mod declarations cannot carry parameters), and
// \usb::DescRom needs a Verilog `initial` block for its contents (a
// plain mod over an ice40::SbRam40_4k cell would need an INIT_FILE hex
// image instead).  The other indexed memories -- the bulk-IN snapshot
// RAM (usb::SnapRam) and the bulk-OUT payload RAM (usb::RxRam) -- are
// plain Virdant mods over ice40::SbRam40_4k cells in src/usb.vir (no
// ext body); the pads are ext/usb_pads.sv and the echo FIFO is
// ext/echo_fifo.sv.
//
// Bit/wire-order conventions used throughout the core (verified against
// valentyusb):
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
// usb_fifo16x8 — synchronous FIFO, default 16-deep x 8-bit.
// DEPTH=64 on the bulk-OUT rx side (usb::RxFifo): a max-size (64 B)
// bulk-OUT packet must be absorbed atomically under the core's
// whole-packet-must-fit admission rule, which a 16-deep FIFO could
// never accept; 64 B is also exactly what a real device's endpoint
// buffer holds.
// rd_data is the combinational head (RAM=0) or the head delayed one
// clock (RAM=1: block-RAM storage with a free-running registered read
// port -- the 64-deep rx side uses it, taking 512 flops and their
// per-byte write-decode cones out of the fabric).  With RAM=1 the
// popped byte is on rd_data one clock after rd_en.
// CONTRACT: wr_en must be pre-gated with !full by the caller, and
// rd_en with !empty.  The FIFO has no internal protection: a push
// while full or a pop while empty corrupts cnt and the pointers.  By
// design the write/read enables here carry no `cnt` terms at all, so
// the count compares never sit in a write/read clock-enable cone --
// a hard requirement at 48 MHz.  Both instantiation sites comply
// (usb::UsbUart in src/usb.vir): the rx pusher pre-gates with
// !rxfifo.full and the rx popper with !rxfifo.empty, and the snapshot
// loader pops exactly min(count,64) bytes counted at arm (it is the
// tx FIFO's only popper, so count only grows during the load).
// Simultaneous push+pop is supported (both pointers advance, cnt
// unchanged).  `count` is 7 bits wide regardless of DEPTH.
// ===========================================================================
module usb_fifo16x8 #(
    parameter DEPTH = 16,
    parameter AW    = 4,
    parameter RAM   = 0
)(
    input  wire        clock,
    input  wire        reset,
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

    // do_wr/do_rd carry no `cnt` terms: the caller owns both gates
    // (wr_en pre-gated with !full, rd_en with !empty -- see the header
    // CONTRACT), so the count compares never sit in the write/read
    // enable paths.
    wire do_wr = wr_en;
    wire do_rd = rd_en;

    generate
        if (RAM != 0) begin : g_ram
            (* ram_style = "block" *) reg [7:0] mem [0:DEPTH-1];
            reg [7:0] rd_q;
            always @(posedge clock) begin
                if (do_wr)
                    mem[wptr] <= wr_data;
                rd_q <= mem[rptr];     // free-running registered read
            end
            assign rd_data = rd_q;
        end else begin : g_ff
            (* no_ram *) reg [7:0] mem [0:DEPTH-1];
            always @(posedge clock) begin
                if (do_wr)
                    mem[wptr] <= wr_data;
            end
            assign rd_data = mem[rptr];
        end
    endgenerate

    always @(posedge clock) begin
        if (reset) begin
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
// \usb::TxFifo / \usb::RxFifo -- ext bodies of usb::TxFifo / usb::RxFifo
// (src/usb.vir): thin fixed-parameter wrappers over usb_fifo16x8
// (Virdant ext mods cannot carry parameters).  The behavior contract
// (caller-gated wr_en/rd_en, count terms never in the FIFO's own
// enables) is the usb_fifo16x8 header's; the wrappers add nothing.
// ===========================================================================
module \usb::TxFifo (
    input  wire       clock,
    input  wire       reset,
    input  wire       wr_en,
    input  wire [7:0] wr_data,
    input  wire       rd_en,
    output wire [7:0] rd_data,
    output wire [6:0] count,
    output wire       full,
    output wire       empty
);
    usb_fifo16x8 u (   // defaults: DEPTH=16, AW=4, RAM=0 (flops, comb head)
        .clock(clock), .reset(reset),
        .wr_en(wr_en), .wr_data(wr_data),
        .rd_en(rd_en), .rd_data(rd_data),
        .count(count), .full(full), .empty(empty));
endmodule

module \usb::RxFifo (
    input  wire       clock,
    input  wire       reset,
    input  wire       wr_en,
    input  wire [7:0] wr_data,
    input  wire       rd_en,
    output wire [7:0] rd_data,
    output wire [6:0] count,
    output wire       full,
    output wire       empty
);
    usb_fifo16x8 #(.DEPTH(64), .AW(6), .RAM(1)) u (   // BRAM, registered head
        .clock(clock), .reset(reset),
        .wr_en(wr_en), .wr_data(wr_data),
        .rd_en(rd_en), .rd_data(rd_data),
        .count(count), .full(full), .empty(empty));
endmodule

// ===========================================================================
// \usb::DescRom -- the descriptor ROM, ext body of usb::DescRom
// (src/usb.vir).  256x8 initialized block RAM, registered sync read
// (ra presented this clock -> rd next clock).  Contiguous pages, addressed
// by usb::UsbUart's rom_page map (src/usb.vir): sel0 device@0 (18 B),
// sel1 config@18 (75 B), sel2 str0@93 (4 B), sel3 str1@97 (10 B),
// sel4 str2@107 (32 B -- bLength 0x20; the string data is followed by
// zero pad bytes up to the declared length).  GET_STATUS (sel6)
// bypasses the ROM: the tx producer mux in usb::UsbUart returns zero
// bytes for it; no sel5 exists (usb::Ep0 implements no line-coding
// request).  The ROM stays an ext body because its contents come from
// the Verilog `initial` block below; a plain mod over an
// ice40::SbRam40_4k cell would need an INIT_FILE hex image instead.
// ===========================================================================
module \usb::DescRom (
    input  wire       clock,
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
    always @(posedge clock) rd <= desc_rom[ra];
endmodule
