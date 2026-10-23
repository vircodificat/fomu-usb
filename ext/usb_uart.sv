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
//   usb_crc5 / usb_crc16  serial CRC LFSRs          (USB 2.0 §8.3.5)
//   usb_fifo16x8          16x8 sync FIFO
//   usb_rx_phy            pad -> bit stream         (USB 2.0 §7.1.x)
//   usb_rx_packet         bit stream -> packet records
//   usb_tx_phy            packet records -> pads
//   usb_ep0               control SM + CDC requests (USB 2.0 §8.5.5)
//   usb_serial            EP1 OUT/IN bulk protocol (toggles, ACK/NAK)
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
// usb_crc5 — serial CRC5 LFSR (USB 2.0 §8.3.5, poly x^5+x^2+1, seed 0x1F)
// ===========================================================================
module usb_crc5(
    input  wire       clk,
    input  wire       rst,     // sync reset (active high): seed 5'h1F
    input  wire       adv,     // advance one serial bit
    input  wire       din,     // serial bit, wire order (LSB first)
    output wire [4:0] crc
);
    reg [4:0] r;
    wire inv = din ^ r[4];
    always @(posedge clk) begin
        if (rst)
            r <= 5'h1F;
        else if (adv) begin
            r[0] <= inv;
            r[1] <= r[0];
            r[2] <= r[1] ^ inv;
            r[3] <= r[2];
            r[4] <= r[3];
        end
    end
    assign crc = r;
endmodule

// ===========================================================================
// usb_crc16 — serial CRC16 LFSR (x^16+x^15+x^2+1, seed 0xFFFF)
// ===========================================================================
module usb_crc16(
    input  wire         clk,
    input  wire         rst,     // sync reset: seed 16'hFFFF
    input  wire         adv,
    input  wire         din,
    output wire [15:0]  crc
);
    reg [15:0] r;
    wire inv = din ^ r[15];
    always @(posedge clk) begin
        if (rst)
            r <= 16'hFFFF;
        else if (adv) begin
            r[0]   <= inv;
            r[1]   <= r[0];
            r[2]   <= r[1] ^ inv;
            r[14:3] <= r[13:2];
            r[15]  <= r[14] ^ inv;
        end
    end
    assign crc = r;
endmodule

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
// gate it in the register that produces wr_en).  rd_en is
// self-protecting (checked against empty here; it only feeds the
// pointer/count registers).  Simultaneous push+pop supported; with
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

    // wr_en is caller-gated with !full (see header): no `cnt` term here.
    wire do_wr = wr_en;
    wire do_rd = rd_en && (cnt != 7'd0);

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
// usb_rx_phy — pad -> unstuffed decoded bit stream + packet/reset flags.
//
// 4x oversampling clock recovery: a 2-bit phase counter re-aligns to 0 on
// ANY dp/dn transition; bits are sampled mid-bit (phase==2).  NRZI: same
// level as the previous bit sample -> 1, change -> 0.  Bit unstuffing: the
// bit after six consecutive decoded 1s is dropped (a stuffed 0); a 1 there
// is a violation.  EOP = SE0 held >= 2 bit times (8 clks).  Bus reset =
// SE0 held >= 2.5 us (120 clks @ 48 MHz; USB 2.0 §7.1.7.5).  SE1 is an
// error: the packet is flagged bad and abandoned.
// J = (dp=1,dn=0), K = (dp=0,dn=1), idle = J (D+ pull-up).
// ===========================================================================
module usb_rx_phy(
    input  wire clk,
    input  wire rst,
    input  wire dp,               // raw pads
    input  wire dn,
    output wire bit_o,            // decoded (unstuffed) bit
    output wire bit_valid,        // 1-clk strobe per decoded bit
    output wire sop,              // 1-clk pulse: J->K from idle
    output wire eop,              // 1-clk pulse: SE0 >= 2 bit times
    output wire active,           // in packet (sop..eop)
    output wire pkt_err,          // per-packet error (cleared at sop)
    output wire bus_reset         // 1-clk pulse: SE0 >= 2.5 us
);
    reg dp_s1, dp_s2, dn_s1, dn_s2;   // 2-FF pad synchronizers
    reg dp_p, dn_p;                   // previous-cycle sample (edge detect)
    reg lev_d, lev_n;                 // previous bit sample (NRZI decode)
    reg [1:0] ph;
    reg act, perr;
    reg [2:0] stuff_cnt;
    reg drop_state;
    reg [6:0] se0_cnt;
    reg sop_r, eop_r, bit_v_r, bit_r, br_r;

    wire sdp = dp_s2;
    wire sdn = dn_s2;
    wire trans = (sdp != dp_p) || (sdn != dn_p);
    wire se0   = (sdp == 1'b0) && (sdn == 1'b0);
    wire se1   = (sdp == 1'b1) && (sdn == 1'b1);
    wire k     = (sdp == 1'b0) && (sdn == 1'b1);
    // NRZI decode of the mid-bit sample
    wire bit_val = (sdp == lev_d) && (sdn == lev_n);

    always @(posedge clk) begin
        if (rst) begin
            dp_s1 <= 1'b0; dp_s2 <= 1'b0;
            dn_s1 <= 1'b0; dn_s2 <= 1'b0;
            dp_p <= 1'b0; dn_p <= 1'b0;
            lev_d <= 1'b1; lev_n <= 1'b0;
            ph <= 2'd0;
            act <= 1'b0; perr <= 1'b0;
            stuff_cnt <= 3'd0; drop_state <= 1'b0;
            se0_cnt <= 7'd0;
            sop_r <= 1'b0; eop_r <= 1'b0;
            bit_v_r <= 1'b0; bit_r <= 1'b0; br_r <= 1'b0;
        end else begin
            // synchronizer + edge history
            dp_s1 <= dp; dp_s2 <= dp_s1;
            dn_s1 <= dn; dn_s2 <= dn_s1;
            dp_p <= sdp; dn_p <= sdn;

            // defaults: single-cycle pulses
            sop_r   <= 1'b0;
            eop_r   <= 1'b0;
            bit_v_r <= 1'b0;
            br_r    <= 1'b0;

            // SE0 timer: EOP (2 bit times) and bus reset (2.5 us)
            if (se0)
                se0_cnt <= se0_cnt + 7'd1;
            else
                se0_cnt <= 7'd0;
            if (se0_cnt == 7'd119)
                br_r <= 1'b1;

            if (!act) begin
                // idle: wait for the J->K sync edge
                stuff_cnt  <= 3'd0;
                drop_state <= 1'b0;
                if (k) begin
                    sop_r <= 1'b1;
                    act   <= 1'b1;
                    ph    <= 2'd0;
                    lev_d <= 1'b1;      // previous level was idle J
                    lev_n <= 1'b0;
                    perr  <= 1'b0;
                end
            end else begin
                if (se1) begin
                    // illegal bus state: abandon packet
                    perr <= 1'b1;
                    act  <= 1'b0;
                end else if (se0) begin
                    // SE0: EOP after >= 2 bit times (8 clks) — stay armed
                    // until then (act drops WITH the eop pulse); bus reset
                    // is detected by the standalone SE0 timer below.
                    if (se0_cnt == 7'd7) begin
                        eop_r <= 1'b1;
                        act <= 1'b0;
                    end
                end else begin
                    // J or K level: track the bit period
                    if (trans)
                        ph <= 2'd0;      // clock recovery: re-align on edge
                    else
                        ph <= ph + 2'd1;
                    if (ph == 2'd2) begin
                        // mid-bit sample: NRZI decode + unstuff
                        lev_d <= sdp;
                        lev_n <= sdn;
                        bit_r <= bit_val;
                        if (drop_state) begin
                            // this decoded bit is the stuffed 0: drop it
                            drop_state <= 1'b0;
                            stuff_cnt  <= 3'd0;
                            if (bit_val)
                                perr <= 1'b1;    // 6 ones then a 1: violation
                        end else begin
                            bit_v_r <= 1'b1;
                            if (bit_val) begin
                                stuff_cnt <= stuff_cnt + 3'd1;
                                if (stuff_cnt == 3'd5)
                                    drop_state <= 1'b1;
                            end else begin
                                stuff_cnt <= 3'd0;
                            end
                        end
                    end
                end
            end
        end
    end

    assign bit_o     = bit_r;
    assign bit_valid = bit_v_r;
    assign sop       = sop_r;
    assign eop       = eop_r;
    assign active    = act;
    assign pkt_err   = perr;
    assign bus_reset = br_r;
endmodule

// ===========================================================================
// usb_rx_packet — bit stream -> packet records.
//
// Bit counting: `nb` counts PID-section bits (sync 0..7 ignored, PID
// byte 0..7, token data 8..18, crc5 wire bits 19..23, i.e. up to 24);
// `n3` is the data-section bit-in-byte index (0..7, wrapping), with `bc`
// counting complete bytes (payload + the 2 trailing crc16 wire bytes,
// subtracted at EOP).  Two small counters instead of one 10-bit `n`:
// every position compare is then 2-3 LUTs, never a carry chain (48 MHz
// timing — the wide compares rode the capture cones last synthesis).  Emits 1-clk registered pulses tk_*/dt_*/hs_*;
// bad packets are silent.
// Payload capture (bytes, wire order LSB first): dt_head = packet bytes
// 0..7 (flopped — the EP0 request / line-coding window), dt_ram = payload
// bytes 0..63 in block RAM (sync read: present dt_rd_addr, the byte is
// on dt_rd_data the next clk).  Why the payload store is off the fabric:
// see the storage block below the port list.
// ===========================================================================
module usb_rx_packet(
    input  wire         clk,
    input  wire         rst,
    input  wire         bus_reset,
    input  wire         bit_i,
    input  wire         bit_valid,
    input  wire         sop,
    input  wire         eop,
    input  wire         pkt_err,
    output reg          tk_valid,
    output reg  [3:0]   tk_pid,
    output reg  [6:0]   tk_addr,
    output reg  [3:0]   tk_endp,
    output reg  [10:0]  tk_frame,
    output reg          tk_crc_ok,
    output reg          dt_valid,
    output reg  [3:0]   dt_pid,
    output reg  [6:0]   dt_len,
    output reg  [63:0]  dt_head,
    output reg  [7:0]   dt_rd_data,
    input  wire [5:0]   dt_rd_addr,
    output reg          dt_crc_ok,
    output reg          hs_valid,
    output reg  [3:0]   hs_pid
);
    localparam [1:0] ST_IDLE = 2'd0, ST_SYNC = 2'd1, ST_BODY = 2'd2;

    reg [1:0]  st;
    reg [4:0]  nb;           // PID/token-section bit index (0..24)
    reg [2:0]  n3;          // data-section bit-in-byte index (0..7)
    reg [7:0]  pid_r;
    reg [10:0] tk_sh;
    reg [7:0]  bshift;
    reg [6:0]  bc;
    reg        bad;
    reg        sec;        // 0 = PID section, 1 = data section
    // capture candidates: the (wide) end-of-packet capture conditions,
    // precomputed at the last relevant BIT clk and held until EOP (no
    // bit_valid clks fire during SE0/EOP, so a hold is safe; any further
    // decoded bit clears them, so over-long packets stay silent).  The
    // EOP-edge strobes then close over 2-3 inputs instead of the whole
    // bit-counting tree (48 MHz timing).
    reg        tok_cap, sof_cap, hs_cap, dat_cap;
    // registered shared qualify for the caps (one clk delayed): the
    // !bad && !pkt_err && st==ST_BODY prefix ANDed with the section.
    // Computing it ONCE here keeps the per-bit cap cones at 2-3 shallow
    // LUTs; a same-cycle pkt_err is re-checked inline (pkt_err is a
    // register), and same-cycle `bad` sets are covered: the nb==7 PID
    // check by hs_cap's own inline PID check, the bc==66 overflow by the
    // ≥8-clk gap to the next data-section byte boundary.  The caps are
    // consumed only at EOP (≥8 clks after the last bit), so the +1 clk
    // is free.
    reg        q_pid_r;     // qualify for the PID-section caps
    reg        q_dat_r;     // qualify for the data-section cap
    // registered byte-count admission for dat_cap (bc in 1..65: at
    // least 2 bytes total, and one more byte would still fit the
    // 64-byte payload RAM) — keeps the 7-bit compares out of dat_cap's cone
    reg        dat_bc_ok_r;
    // registered helpers (see the always blocks below): bc_is_66_r
    // decouples the (bc == 66) compare from the bc+1 carry chain (the
    // compare shared the adder's carry LUTs, chaining bc's D-cone into
    // dat_bc_ok_r's D and bad's enable cones — the critical path);
    // pid_bad_r / bc66_hit_r are registered per-cause error hits so the
    // sticky `bad` flag's D is a 3-input OR of REGISTERS (no clock
    // enable: an E-gated bad ingested the same compare tree); the CRC
    // residual compares are registered so the 16-bit equality tree ends
    // at its own register instead of riding dt_crc_ok's D-cone.
    reg        bc_is_66_r;
    reg        pid_bad_r;
    reg        bc66_hit_r;
    reg        crc5_res_r;
    reg        crc16_res_r;

    wire [4:0]  c5;
    wire [15:0] c16;

    // CRC5 advances on token bits nb = 8..23 (data + received crc5 wire
    // bits; residual check 5'h0C); CRC16 advances on every data-section
    // bit (payload + received crc16 wire bits; residual check 16'h800D).
    usb_crc5 u_crc5(
        .clk(clk),
        .rst(sop | rst | bus_reset),
        .adv(bit_valid && (st == ST_BODY) && !sec
             && (nb >= 5'd8) && (nb <= 5'd23)),
        .din(bit_i),
        .crc(c5));
    usb_crc16 u_crc16(
        .clk(clk),
        .rst(sop | rst | bus_reset),
        .adv(bit_valid && (st == ST_BODY) && sec),
        .din(bit_i),
        .crc(c16));

    // registered helpers: shallow independent cones, each ending at its
    // OWN register (see the declaration comment).  crc*_res_r are
    // computed every cycle — the residual is only SAMPLED at EOP (when
    // the LFSR has consumed the whole packet), so mid-packet values are
    // harmless.  bc_is_66_r is read one clk after bc reaches 66; bc
    // holds between byte completes, so the delay is exact.
    always @(posedge clk) begin
        if (rst || bus_reset) begin
            bc_is_66_r  <= 1'b0;
            pid_bad_r   <= 1'b0;
            bc66_hit_r  <= 1'b0;
            crc5_res_r  <= 1'b0;
            crc16_res_r <= 1'b0;
        end else begin
            bc_is_66_r  <= (bc[6:3] == 4'b0100) && (bc[2:0] == 3'b010);
            pid_bad_r   <= bit_valid && (st == ST_BODY) && !sec
                           && (nb == 5'd7)
                           && (pid_new[3:0] != ~pid_new[7:4]);
            bc66_hit_r  <= bit_valid && (st == ST_BODY) && sec
                           && (n3 == 3'd7) && (bc == 7'd66);
            crc5_res_r  <= (c5 == 5'h0C);
            crc16_res_r <= (c16 == 16'h800D);
        end
    end

    // bad: FLAT sticky error — plain D, no clock enable (see the decl
    // comment).  The +1 clk on the set (hits are registered) is covered
    // by the documented redundancy: q_pid_r/q_dat_r re-check pkt_err
    // inline, hs_cap re-checks the PID inline, and dat_bc_ok_r gates
    // the overflow via bc_is_66_r.
    always @(posedge clk) begin
        if (rst || bus_reset)
            bad <= 1'b0;
        else
            bad <= sop ? 1'b0
                 : (bad || (bit_valid && pkt_err)
                        || pid_bad_r || bc66_hit_r);
    end

    // PID capture value on this cycle
    wire [7:0] pid_new = {bit_i, pid_r[7:1]};

    // ------------------------------------------------------------------
    // Payload capture storage — OFF THE FABRIC (48 MHz timing): 512
    // payload flops put the bc->byte-enable decode (64-way, riding the
    // bc+1 carry chain) AND the 64:1 read mux (into the usb_uart rx
    // push) in the critical path.  Instead:
    //   dt_ram  64x8 block RAM: payload bytes 0..63.  ONE write port
    //           with a REGISTERED address (bc) and one enable — no
    //           decode tree, no per-byte enable fanout; the sync read
    //           (dt_rd_addr -> dt_rd_data) replaces the mux.
    //   dt_head 8 flopped bytes: packet bytes 0..7 — consumers that
    //           need several bytes AT ONCE (usb_ep0: the rq* request
    //           latch, the SET_LINE_CODING capture) can't use a RAM
    //           port, so the head window stays flopped (its enable
    //           decode is 3 bits, not 7).
    // Writes beyond byte 63 (over-length packets) are dropped — the
    // 512-flop version's out-of-range part-select writes did nothing;
    // those packets set bad (bc==66) and dt_valid stays 0, so no
    // consumer ever sees a dropped byte.  A bad packet's payload is
    // still written (same as the flop version).  Read-during-write to
    // one address returns the OLD byte (nonblocking): the rx push of
    // packet N reads packet N's bytes (written long before the push
    // starts), never a byte of a concurrently-arriving packet N+1.
    // ------------------------------------------------------------------
    (* ram_style = "block" *) reg [7:0] dt_ram [0:63];

    // byte completing on this bit clk (bit_i = the byte MSB,
    // bshift[7:1] = the earlier 7 bits, LSB first)
    wire [7:0] dt_byte_new = {bit_i, bshift[7:1]};

    // byte-complete write strobe (registered inputs only: n3/st/sec/bc
    // are regs; bit_valid/pkt_err are registered upstream in usb_rx_phy)
    wire dt_wr = bit_valid && !pkt_err && (st == ST_BODY) && sec
                 && (n3 == 3'd7);

    always @(posedge clk) begin
        if (dt_wr && !bc[6])            // bc >= 64: drop (no RAM slot)
            dt_ram[bc[5:0]] <= dt_byte_new;
        dt_rd_data <= dt_ram[dt_rd_addr];  // free-running registered read
    end

    // head shadow (bytes 0..7): flopped copy in its own always block so
    // the 64 byte clock enables stay out of the FSM's cones
    always @(posedge clk) begin
        if (rst || bus_reset)
            dt_head <= 64'd0;
        else if (dt_wr && (bc[6:3] == 4'd0))      // bc < 8: head window
            dt_head[8*bc[2:0] +: 8] <= dt_byte_new;
    end

    always @(posedge clk) begin
        if (rst || bus_reset) begin
            st      <= ST_IDLE;
            nb      <= 5'd0;
            n3      <= 3'd0;
            pid_r   <= 8'd0;
            tk_sh   <= 11'd0;
            bshift  <= 8'd0;
            bc      <= 7'd0;
            sec     <= 1'b0;
            tok_cap <= 1'b0;
            sof_cap <= 1'b0;
            hs_cap  <= 1'b0;
            dat_cap <= 1'b0;
            dat_bc_ok_r <= 1'b0;
            q_pid_r <= 1'b0;
            q_dat_r <= 1'b0;
        end else begin

            if (sop) begin
                st     <= ST_SYNC;
                nb     <= 5'd0;
                n3     <= 3'd0;
                pid_r  <= 8'd0;
                tk_sh  <= 11'd0;
                bshift <= 8'd0;
                bc     <= 7'd0;
                sec    <= 1'b0;
                tok_cap <= 1'b0;
                sof_cap <= 1'b0;
                hs_cap  <= 1'b0;
                dat_cap <= 1'b0;
                q_pid_r <= 1'b0;
                q_dat_r <= 1'b0;
            end else if (eop) begin
                // capture decision: precomputed at the last bit clk
                // (tok_cap & friends).  The strobes + captured fields
                // live in the FLAT always block below — this branch
                // only retires the FSM state and clears the caps.
                st <= ST_IDLE;
                tok_cap <= 1'b0;
                sof_cap <= 1'b0;
                hs_cap  <= 1'b0;
                dat_cap <= 1'b0;
            end else if (bit_valid) begin
                // per-bit capture precompute (registered inputs only):
                // latched at the last relevant bit and held through the
                // SE0 tail to EOP.  tok: nb==23 is the last token bit
                // (nb==24 would be at EOP); hs: nb==7 is the last PID bit
                // (nb==8 at EOP, PID check included — `bad` is not
                // visible yet); dat: byte-aligned last bit (n3==7 with
                // dat_bc_ok_r ⇔ 2+ bytes at EOP) and the 64-byte payload
                // limit holds (bc != 66 ⇔ one more byte would overflow
                // the 64-byte payload RAM)
                // shared qualify, REGISTERED one clk earlier (see decl)
                q_pid_r <= !bad && !pkt_err && (st == ST_BODY) && !sec;
                q_dat_r <= !bad && !pkt_err && (st == ST_BODY) && sec;
                tok_cap <= q_pid_r && !pkt_err
                           && (pid_r[1:0] == 2'b01) && (nb == 5'd23);
                sof_cap <= q_pid_r && !pkt_err
                           && (pid_r[1:0] == 2'b01) && (nb == 5'd23)
                           && (pid_r[3:0] == 4'h5);
                hs_cap  <= q_pid_r && !pkt_err
                           && ((pid_r[1:0] == 2'b00)
                               || (pid_r[1:0] == 2'b10))
                           && (nb == 5'd7)
                           && (pid_new[3:0] == ~pid_new[7:4]);
                dat_cap <= q_dat_r && !pkt_err
                           && (n3 == 3'd7) && dat_bc_ok_r;
                dat_bc_ok_r <= (bc != 7'd0) && !bc_is_66_r;
                if (!pkt_err) begin
                    case (st)
                    ST_IDLE: ;
                    ST_SYNC: begin
                        // ignore sync bits 0..7
                        if (nb == 5'd7) begin
                            st  <= ST_BODY;
                            nb  <= 5'd0;
                            n3  <= 3'd0;
                            sec <= 1'b0;
                        end else
                            nb <= nb + 5'd1;
                    end
                    ST_BODY: begin
                        if (!sec) begin
                            // PID byte (nb = 0..7), then token remainder.
                            // pid_r is FROZEN after the PID byte (it
                            // classifies the packet at EOP); token data
                            // bits are nb = 8..18 (the shift-up capture);
                            // crc5 wire bits nb = 19..23 feed the residual
                            // check via usb_crc5 and are not captured.
                            if (nb <= 5'd7)
                                pid_r <= pid_new;
                            if ((nb >= 5'd8) && (nb <= 5'd18))
                                tk_sh <= {bit_i, tk_sh[10:1]};
                            if (nb == 5'd7) begin
                                // PID check failure is registered upstream
                                // (pid_bad_r -> bad, +1 clk)
                                if (pid_new[1:0] == 2'b11) begin
                                    sec <= 1'b1;    // data section
                                    n3  <= 3'd0;
                                    nb  <= 5'd24;
                                end else begin
                                    nb <= 5'd8;    // token/handshake bits
                                end
                            end else
                                nb <= nb + 5'd1;
                        end else begin
                            // data section: byte COUNTING only — the
                            // payload STORE is off-fabric (dt_ram /
                            // dt_head above).  bc counts payload bytes
                            // AND the 2 trailing crc16 wire bytes
                            // (subtracted at EOP — same scheme the BFM
                            // receiver uses and T8/T9 proved on 64 B
                            // packets).  No delay line: the end is only
                            // known at EOP.  n3 wraps mod 8 (3 bits, no
                            // compare); bshift feeds dt_byte_new.
                            n3     <= n3 + 3'd1;
                            bshift <= {bit_i, bshift[7:1]};
                            if (n3 == 3'd7) begin
                                // byte complete (the store strobe is
                                // dt_wr, in the storage block above;
                                // the >64-byte overflow is registered
                                // upstream: bc66_hit_r -> bad)
                                bc <= bc + 7'd1;
                            end
                        end
                    end
                    endcase
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Output strobes + captured fields — FLAT equations, own always
    // block.  These used to live inside the FSM's if/else-if chain
    // above; synthesis then shared the priority-mux tree across
    // tk_valid/dt_valid/hs_valid's D-cones AND the captured fields'
    // clock enables (hs_pid's enable was literally the same LUTs as
    // hs_valid's D), chaining sop/eop/reset terms LUT-to-LUT from the
    // PHY into hs_pid's enable — the 23 ns critical path.  The caps are
    // MUTUALLY EXCLUSIVE by construction (tok: nb==23 in the PID
    // section; hs: nb==7 in the PID section; dat: data section), so the
    // else-if priority was redundant: each strobe is now one 2-input
    // AND of registers, and each capture enable is eop AND its cap.
    // Reading the caps AT the eop edge is safe (same edge clears them;
    // nonblocking semantics see the pre-edge values).
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst || bus_reset) begin
            tk_valid  <= 1'b0;
            dt_valid  <= 1'b0;
            hs_valid  <= 1'b0;
            tk_pid    <= 4'h0;
            tk_addr   <= 7'h0;
            tk_endp   <= 4'h0;
            tk_frame  <= 11'h0;
            tk_crc_ok <= 1'b0;
            dt_pid    <= 4'h0;
            dt_len    <= 7'h0;
            dt_crc_ok <= 1'b0;
            hs_pid    <= 4'h0;
        end else begin
            tk_valid <= eop && tok_cap;
            dt_valid <= eop && dat_cap;
            hs_valid <= eop && hs_cap;

            if (eop) begin
                if (tok_cap) begin
                    tk_pid    <= pid_r[3:0];
                    tk_crc_ok <= crc5_res_r;
                end
                if (tok_cap && sof_cap) begin
                    // SOF: 11-bit frame number
                    tk_frame <= tk_sh;
                end
                if (tok_cap && !sof_cap) begin
                    // token bits b8..b18 land at tk_sh[0..10]
                    // (shift-up capture): addr = tk_sh[6:0],
                    // endp = tk_sh[10:7]
                    tk_addr <= tk_sh[6:0];
                    tk_endp <= tk_sh[10:7];
                end
                if (dat_cap) begin
                    // data: bc = payload bits + 16 crc bits
                    dt_pid    <= pid_r[3:0];
                    dt_len    <= (bc >= 7'd2) ? (bc - 7'd2) : 7'd0;
                    dt_crc_ok <= crc16_res_r;
                end
                if (hs_cap) begin
                    // handshake: nothing after the PID byte
                    hs_pid <= pid_r[3:0];
                end
            end
        end
    end
endmodule

// ===========================================================================
// usb_tx_phy — packet records -> pads.
//
// A bit-time engine: every transmitted bit time is 4 clk cycles (btc 0..3).
// `s` is the raw bit index of the NEXT bit time to be computed; the level
// for it is registered at the btc==3 edge of the current bit time (one clk
// before it drives).  Raw stream: sync 00000001 (bits 0..7), PID byte
// (8..15), payload (16..16+8L-1, LSB first, pulled byte-by-byte via
// tx_byte/tx_addr), CRC16 wire bits (data pids only).  A stuffed-0 bit
// time is inserted after six consecutive raw 1s (the run counter counts
// raw bits only and is cleared entering the PID; the sync pattern is
// exempt).  The CRC16 LFSR is fed each payload raw bit at btc==0 of its
// bit time, so the final state is stable 3+ clks before the first CRC
// wire bit time is computed.  EOP: 2 bit times SE0 + 1 bit time J, then
// release.  Handshake pids send sync + PID + EOP only.  An 8-clk (2 bit
// times) guard precedes SOP; response SOP lands ~11 clks (~2.75 bit
// times) after the triggering packet's EOP — well inside the USB 2.0
// device turnaround window.
// ===========================================================================
module usb_tx_phy(
    input  wire        clk,
    input  wire        rst,
    input  wire        tx_start,   // 1-clk pulse: begin packet
    input  wire [3:0]  tx_pid,
    input  wire [6:0]  tx_len,     // payload byte count (0..64; 0 => ZLP)
    input  wire [7:0]  tx_byte,    // producer data (comb, at tx_addr)
    output wire [6:0]  tx_addr,    // byte index currently consumed
    output wire [6:0]  tx_addr_next,// byte index of the NEXT bit time's byte
    output wire        tx_busy,
    output wire        tx_done,    // 1-clk pulse at packet release
    output wire        dp,
    output wire        dn,
    output wire        oe
);
    localparam [2:0] S_IDLE = 3'd0, S_GUARD = 3'd1,
                     S_DATA = 3'd2, S_EOP0 = 3'd3, S_EOPJ = 3'd4;

    reg [2:0] state;
    reg [1:0] btc;            // clk phase within the current bit time
    reg [2:0] gcnt;           // guard clk counter
    reg [2:0] ecnt;           // EOP bit-time counter
    reg [9:0] s;              // raw bit index of the next bit time
    reg [3:0] pid_r;
    reg [6:0] len_r;
    reg       lvl;            // current drive level: 0 = J, 1 = K
    reg [2:0] ones;           // consecutive raw 1s (stuff run)
    reg       stuff_pending;  // next bit time is a stuffed 0
    reg       cur_stuff;      // current bit time is a stuffed 0
    reg       oe_r;
    reg       done_r;

    wire is_data = (pid_r[1:0] == 2'b11);
    // payload_end / total_bits are registered at packet start (the
    // {3'b0, len_r, 3'b0} adder would otherwise sit inside the
    // raw-bit comparison path)
    reg [9:0] payload_end_r, total_bits_r;
    // (s < total_bits_r) REGISTERED one cycle ahead: the 10-bit compare
    // is a carry chain, and it fed cur_stuff's D-cone every bit time
    // (the tx-engine critical path).  s changes only at btc==3 edges, so
    // the flag captured at the previous cycle is exact at the decision
    // edge — +0 clk latency, the compare cone now ends at its own
    // register.
    reg       s_lt_end_r;

    // producer byte pipeline: tx_addr leads the engine's read by one
    // bit time (s advances at the btc==3 edge, the byte is stable for
    // the whole 4-clk bit time), so one register stage splits the big
    // producer mux (64:1 snapshot / descriptor ROM) out of the
    // single-cycle engine path.  The engine reads tx_byte_r only.
    reg [7:0] tx_byte_r;
    // load once per payload byte.  The engine is a one-bit-time
    // lookahead: during bit time k, s = k+1, and bit 0 of byte b
    // (s = 16+8b) is CONSUMED at the end of the previous byte's last bit
    // time.  So the reload fires when s[2:0]==0 (s = 16+8b, during byte
    // b-1's bit 7); the registered producer (snap_q / desc_q /
    // ep0_small_q) holds THIS byte's data from the previous byte's fetch
    // window, and the single load per byte keeps it stable through all
    // eight bit times.
    always @(posedge clk)
        if (state == S_DATA && btc == 2'd0 && (s[2:0] == 3'd0)
                && (s >= 10'd16))
            tx_byte_r <= tx_byte;

    // raw bit f(s): sync, PID, payload (LSB first), CRC wire bits.
    // f(s) is evaluated against the REGISTERED s (stable for the whole
    // 4-clk bit time), then pipelined into raw_bit_r: the engine reads
    // raw_bit_r one clk later, which splits the compare/select chain
    // and the stuff/NRZI tail into separate cycles.
    reg raw_bit;
    always @* begin
        if (s < 10'd8)
            // sync = 00000001 (bit 7 is the trailing 1)
            raw_bit = (s == 10'd7);
        else if (s < 10'd16)
            // PID byte: {~pid[3:0], pid[3:0]}, LSB first
            raw_bit = (s[2:0] < 4'd4) ? pid_r[s[2:0]]
                                     : ~pid_r[s[2:0] - 4'd4];
        else if (is_data && (s < payload_end_r))
            raw_bit = tx_byte_r[(s - 10'd16) & 7];
        else if (is_data)
            // CRC16 wire bits: crc_wire[i] = ~crc[15-i]
            raw_bit = ~u_crc16_crc[15 - (s - payload_end_r)];
        else
            raw_bit = 1'b0;      // unreachable for handshake pids
    end
    reg raw_bit_r;
    always @(posedge clk) raw_bit_r <= raw_bit;

    wire [15:0] u_crc16_crc;
    usb_crc16 u_crc16(
        .clk(clk),
        .rst(tx_start),
        .adv(state == S_DATA && (btc == 2'd2) && !cur_stuff
             && is_data && (s >= 10'd16) && (s < payload_end_r)),
        .din(raw_bit_r),
        .crc(u_crc16_crc));

    // byte index requested from the producer.  Pure bit-slice of s (no
    // state term): the engine only reads payload bits while
    // 16 <= s < payload_end, which only happens in S_DATA, so the value
    // outside S_DATA is a don't-care — keeping the producer path short
    // matters (the snap/ROM mux + tx_byte_r stage is the critical path).
    wire [9:0] s_m16 = s - 10'd16;
    assign tx_addr = s_m16[9:3];
    // next byte's index, byte-stable: during bit time k, s = k+1, so
    // (s-9)>>3 equals b for EVERY bit time of byte b-1 (and 0 during the
    // PID byte) — a registered copy addresses the source one byte ahead
    // without flipping mid-byte
    wire [9:0] s_m9 = s - 10'd9;
    assign tx_addr_next = s_m9[9:3];

    // pad drive: J/K/SE0 (levels only matter while oe_r is set)
    assign dp = (state == S_EOP0) ? 1'b0 : (lvl ? 1'b0 : 1'b1);
    assign dn = (state == S_EOP0) ? 1'b0 : (lvl ? 1'b1 : 1'b0);
    assign oe = oe_r;

    assign tx_busy = (state != S_IDLE);
    assign tx_done = done_r;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            btc <= 2'd0;
            gcnt <= 3'd0;
            ecnt <= 3'd0;
            s <= 10'd0;
            pid_r <= 4'd0;
            len_r <= 7'd0;
            lvl <= 1'b0;
            ones <= 3'd0;
            stuff_pending <= 1'b0;
            cur_stuff <= 1'b0;
            oe_r <= 1'b0;
            done_r <= 1'b0;
            s_lt_end_r <= 1'b0;
        end else begin
            done_r <= 1'b0;
            s_lt_end_r <= (s < total_bits_r);

            case (state)
            S_IDLE: begin
                if (tx_start) begin
                    pid_r <= tx_pid;
                    len_r <= tx_len;
                    payload_end_r <= {3'b0, tx_len, 3'b0} + 10'd16;
                    total_bits_r  <= {3'b0, tx_len, 3'b0}
                                   + ((tx_pid[1:0] == 2'b11)
                                      ? 10'd32 : 10'd16);
                    state <= S_GUARD;
                    gcnt  <= 3'd0;
                end
            end
            S_GUARD: begin
                // 8 clks (2 bit times) inter-packet guard, bus released
                if (gcnt == 3'd7) begin
                    state <= S_DATA;
                    btc   <= 2'd0;
                    s     <= 10'd1;     // bit time 0 (K) computed here
                    lvl   <= ~lvl;      // sync bit 0 is a 0: J -> K
                    ones  <= 3'd0;
                    stuff_pending <= 1'b0;
                    cur_stuff <= 1'b0;
                    oe_r  <= 1'b1;
                end else
                    gcnt <= gcnt + 3'd1;
            end
            S_DATA: begin
                // bit-time engine: at btc==3 compute the next bit time
                btc <= btc + 2'd1;
                if (btc == 2'd3) begin
                    btc <= 2'd0;
                    cur_stuff <= 1'b0;
                    if (stuff_pending) begin
                        // stuffed 0 bit time (raw index not consumed)
                        lvl <= ~lvl;
                        stuff_pending <= 1'b0;
                        ones <= 3'd0;
                        cur_stuff <= 1'b1;
                    end else if (s_lt_end_r) begin
                        // raw bit time s (flag registered 1 clk ahead —
                        // exact, see the declaration comment)
                        lvl <= raw_bit_r ? lvl : ~lvl;
                        if (s == 10'd8)
                            ones <= 3'd0;    // run counter fresh at the PID
                        else if (raw_bit_r) begin
                            if (ones == 3'd5) begin
                                stuff_pending <= 1'b1;
                                ones <= 3'd0;
                            end else
                                ones <= ones + 3'd1;
                        end else
                            ones <= 3'd0;
                        s <= s + 10'd1;
                    end else begin
                        // raw stream complete: EOP (SE0, 2 bit times)
                        state <= S_EOP0;
                        ecnt  <= 3'd0;
                        btc   <= 2'd0;
                    end
                end
            end
            S_EOP0: begin
                btc <= btc + 2'd1;
                if (btc == 2'd3) begin
                    btc <= 2'd0;
                    if (ecnt == 3'd1) begin
                        state <= S_EOPJ;
                        lvl   <= 1'b0;     // J for the EOP's last bit time
                    end else
                        ecnt <= ecnt + 3'd1;
                end
            end
            S_EOPJ: begin
                btc <= btc + 2'd1;
                if (btc == 2'd3) begin
                    oe_r   <= 1'b0;        // release the bus
                    done_r <= 1'b1;
                    state  <= S_IDLE;
                end
            end
            endcase
        end
    end
endmodule

// ===========================================================================
// usb_ep0 — control transfer state machine (USB 2.0 §8.5.5) + CDC requests.
//
// Supported: GET_DESCRIPTOR (device/config/string), SET_ADDRESS (latched
// at status-stage completion), SET_CONFIGURATION, GET_STATUS (device),
// SET_LINE_CODING / GET_LINE_CODING (7 bytes), SET_CONTROL_LINE_STATE.
// Everything else -> STALL until the next SETUP.
//
// Data toggles: SETUP is always DATA0 and re-initializes the toggle; the
// device data stage starts at DATA1; the status-stage data packet is a
// zero-length DATA1 packet: the DEVICE sends it in response to an IN token
// (control write / no data stage) and ACKs the host's DATA1 ZLP after an
// OUT token (control read).  Descriptor bytes are pulled by usb_tx_phy
// from the usb_uart ROM via (resp_sel, resp_base + tx_addr).
//
// Request pipeline (48 MHz timing; each stage is one clk):
//   EP_RCV_REQ latch request  ->  EP_PARSE decode flags (rq* are
//   registered)  ->  EP_SETUP route flags into the one-hot p_action
//   ->  EP_ARM apply p_action (datapath + state).  The request case
//   tree never reaches a datapath register enable combinationally.
// Data-stage accepts are REGISTERED (req_acc_r / rcd_acc_r /
// sto_acc_r, one clk after the router's data strobe): the three
// accept condition trees otherwise all feed st's next-state mux
// directly, making it a 3-level LUT cone (the critical path).  The
// FSM reacts to the registered accepts one clk later; every field
// they read (dt_head/dt_pid/dt_len/dt_crc_ok) holds between packets,
// so the delayed reaction is bit-identical (+1 clk is free: the
// turnaround budget is ~30-50 clks).
// ===========================================================================
module usb_ep0(
    input  wire         clk,
    input  wire         rst,
    input  wire         bus_reset,
    // token strobes (router-gated: address + endpoint matched)
    input  wire         tk_setup,
    input  wire         tk_in,
    input  wire         tk_out,
    // data packet strobe (router-gated: EP0 expects this packet)
    input  wire         dt_valid,
    input  wire  [3:0]  dt_pid,
    input  wire  [6:0]  dt_len,
    // payload head window: packet bytes 0..7 (usb_rx_packet's flopped
    // shadow — EP0 only ever needs the request / line-coding window,
    // never the RAM bytes 8..63)
    input  wire [63:0]  dt_head,
    input  wire         dt_crc_ok,
    // handshake strobe (router-gated: EP0's response was ACKed)
    input  wire         hs_ack,
    // tx response request (1-clk pulse; params held)
    output reg          resp_req,
    output reg  [3:0]   resp_pid,
    output reg  [6:0]   resp_len,
    output reg  [2:0]   resp_sel,     // descriptor select (usb_uart ROM)
    output reg  [9:0]   resp_base,    // descriptor byte offset
    // device status
    output reg  [6:0]  dev_addr,
    output reg         configured,
    output reg  [55:0] line_coding,   // 7 bytes (GET/SET_LINE_CODING data)
    // router hints
    output wire        ep0_busy,     // EP0 expects a data packet
    output wire        ep0_wait_ack  // EP0's response is in flight
);
    localparam [3:0] PID_SETUP = 4'hD, PID_DATA0 = 4'h3, PID_DATA1 = 4'hB,
                     PID_ACK = 4'h2, PID_STALL = 4'hE;

    localparam [3:0] EP_IDLE     = 4'd0,   // no transfer: wait SETUP
                     EP_RCV_REQ  = 4'd1,   // SETUP data stage (DATA0, 8 B)
                     EP_PARSE    = 4'd8,   // decode the request (1 clk later)
                     EP_SETUP    = 4'd9,   // route flags -> action reg (2 clk)
                     EP_ARM      = 4'd10,  // apply action: datapath + st (3 clk)
                     EP_SEND     = 4'd2,   // wait IN token, then send data
                     EP_WAIT_ACK = 4'd3,   // response sent, wait host ACK
                     EP_RCV_DATA = 4'd4,   // OUT data stage (SET_LINE_CODING)
                     EP_STATUS_I = 4'd5,   // status: IN token -> DATA1 ZLP
                     EP_STATUS_O = 4'd6,   // status: OUT token -> host ZLP
                     EP_STALL    = 4'd7;   // STALL until next SETUP

    reg [7:0] rq0, rq1, rq2, rq3;
    reg [7:0] rq6, rq7;             // wLength bytes
    // decode-stage registers (EP_PARSE -> EP_SETUP pipeline: the
    // request case tree is too deep for one 48 MHz cycle)
    reg [2:0]  p_sel;    // descriptor select (from new_sel)
    reg        p_valid;  // decoded request is supported
    reg        p_is_desc, p_is_get_status, p_is_set_addr, p_is_set_cfg;
    reg        p_is_set_lc, p_is_get_lc, p_is_cls;
    reg [15:0] p_wlen;
    reg [6:0] addr_pending;
    reg        cfg_pending;
    // per-transfer ownership tags: the pending values may only be applied
    // at THIS transfer's status completion.  Without them any control
    // read's status stage (e.g. GET_DESCRIPTOR(config), wValue lo = 0)
    // latches addr_pending=0 and wipes dev_addr at completion.
    reg        pend_addr;   // this transfer is SET_ADDRESS
    reg        pend_cfg;    // this transfer is SET_CONFIGURATION
    // EP_SETUP compute-stage registers: the routed one-hot action (EP_ARM
    // applies it with shallow decodes only, so the request routing never
    // reaches datapath register enables combinationally)
    reg [2:0]  p_action;    // 0 STALL 1 STATUS_I 2 GET_STATUS 3 RCV_DATA
    // descriptor data-stage length, min(wLength, desc_len) PRECOMPUTED in
    // EP_SETUP: the 15-bit compare against desc_len(p_sel) was a full
    // carry chain ending at remain's clock enable (the vir-flow critical
    // path).  desc_len <= 75 < 128, so the compare only needs the low
    // bits: wLength >= 128 ⇒ the descriptor length wins outright.
    reg [6:0]  p_dremain_r;
                           // 4 SEND_LC 5 SEND_DESC
    reg        p_wlen0;     // request wLength == 0
    reg        p_desc_in;   // GET_DESCRIPTOR direction (rq0[7])
    // arm-time precomputed packet flags (the EP_WAIT_ACK decisions read
    // these 1-bit registers instead of comparing send_len/remain)
    reg        send_zlp_r;  // armed packet is the status ZLP (len 0)
    reg        last_r;      // armed packet completes the data stage
    // SET_LINE_CODING capture pulse, REGISTERED one clk after the
    // accepted data packet: dt_data is stable by then, and the 56
    // line_coding byte clock enables then see one REGISTER, not the
    // accept compare tree (that cone rode a global clock-enable buffer
    // and was the critical path)
    reg        lc_cap_r;
    // registered data-stage accepts (see the header comment): each is
    // an independent shallow cone (state decode + packet-field
    // compares) that lands on its OWN register — st's next-state then
    // keys off one clean registered bit instead of three condition
    // trees
    reg        req_acc_r;   // SETUP request packet accepted
    reg        rcd_acc_r;   // OUT data-stage packet accepted
    reg        sto_acc_r;   // status-stage ZLP accepted
    reg        stall_resp_r;  // STALL handshake response due (+1 clk)

    reg [3:0] st;
    reg       tog;                   // 1 => next device data pid = DATA1
    reg [6:0] remain;               // data stage bytes left to send
    reg [6:0] send_len;             // bytes in the in-flight packet
    reg [9:0] base;                 // current send offset
    reg [2:0] sel;                   // current descriptor select
    reg [6:0] expect_len;           // OUT data stage total to receive
    reg [6:0] recv_cnt;              // OUT data stage bytes received

    // line coding registers (7 bytes): dwDTERate[31:0], bCharFormat,
    // bParityType, bDataBits — packed [55:0], byte i at [8*i +: 8]
    // default 115200 8N1 (matches a real cdc_acm default)
    wire [55:0] lc_default = {8'h08, 8'h00, 8'h00, 8'h00,
                              8'h00, 8'hC2, 8'h01};

    // request bytes as seen THIS cycle (dt_head is stable during the
    // request packet's dt_valid strobe); rq* registers hold them for
    // later stages (EP_RCV_DATA)
    // wLength from the REGISTERED request bytes (the parse runs one
    // clk after the SETUP data, so the registered view is current)
    wire [15:0] w_length = {rq7, rq6};
    // min(w_length, 7) for the SET_LINE_CODING expect length (decode
    // stage; small compare)
    wire [6:0] p_wlen_next_7 =
        (w_length > 16'd7) ? 7'd7 : w_length[6:0];

    // GET_DESCRIPTOR select, computed from the registered request bytes
    // (the FSM must not read the previous request's `sel` when sizing
    // `remain` — non-blocking assignments are not visible yet)
    reg [2:0] new_sel;
    always @* begin
        new_sel = 3'd7;                      // invalid -> STALL
        case (rq3)
        8'h01: new_sel = 3'd0;               // device
        8'h02: new_sel = 3'd1;               // config
        8'h03: begin                         // string (rq2 = index)
            case (rq2)
            8'h00: new_sel = 3'd2;
            8'h01: new_sel = 3'd3;
            8'h02: new_sel = 3'd4;
            default: new_sel = 3'd7;
            endcase
        end
        endcase
    end

    assign ep0_busy = (st == EP_RCV_REQ) || (st == EP_RCV_DATA)
                   || (st == EP_STATUS_O);
    assign ep0_wait_ack = (st == EP_WAIT_ACK);

    // descriptor lengths — must match usb_uart's desc_byte ROM
    // sel: 0=device 1=config 2=str0 3=str1 4=str2 5=line_coding 6=status
    function [6:0] desc_len;
        input [2:0] s;
        begin
            case (s)
            3'd0: desc_len = 7'd18;
            3'd1: desc_len = 7'd75;
            3'd2: desc_len = 7'd4;
            3'd3: desc_len = 7'd10;
            3'd4: desc_len = 7'd32;
            3'd5: desc_len = 7'd7;
            default: desc_len = 7'd2;   // status
            endcase
        end
    endfunction

    // SET_LINE_CODING data capture: a REGISTERED pulse one clk after the
    // accepted OUT data packet (dt_head is stable by then; recv_cnt/rq1
    // are sampled at the same edge as the FSM's rcd_acc_r reaction —
    // pre-update values), so the 56 line_coding byte clock enables see
    // one REGISTER, not the accept compare tree.
    always @(posedge clk) begin
        if (rst || bus_reset) begin
            lc_cap_r     <= 1'b0;
        end else begin
            lc_cap_r     <= rcd_acc_r && (recv_cnt == 7'd0)
                           && (rq1 == 8'h20);
            if (lc_cap_r) begin
                // SET_LINE_CODING: store all 7 bytes (all in the head
                // window)
                line_coding[0*8 +: 8] <= dt_head[0*8 +: 8];
                line_coding[1*8 +: 8] <= dt_head[1*8 +: 8];
                line_coding[2*8 +: 8] <= dt_head[2*8 +: 8];
                line_coding[3*8 +: 8] <= dt_head[3*8 +: 8];
                line_coding[4*8 +: 8] <= dt_head[4*8 +: 8];
                line_coding[5*8 +: 8] <= dt_head[5*8 +: 8];
                line_coding[6*8 +: 8] <= dt_head[6*8 +: 8];
            end
        end
    end

    // registered data-stage accepts: independent shallow cones, one
    // register each (see the header comment)
    // NOTE: resp_pid/resp_len deliberately KEEP their clock enables
    // (per-branch constant D values).  A flat priority mux into their D
    // was tried and was a measured REGRESSION (vir-flow band 47.9-48.7
    // -> 44.2-46.0): the wide mux cone became a new abc9 sharing hub.
    // Their enable terms are all registered strobes, which is what
    // keeps the cones shallow.
    always @(posedge clk) begin
        if (rst || bus_reset) begin
            req_acc_r <= 1'b0;
            rcd_acc_r <= 1'b0;
            sto_acc_r <= 1'b0;
            stall_resp_r <= 1'b0;
        end else begin
            req_acc_r <= (st == EP_RCV_REQ) && dt_valid
                         && (dt_pid == PID_DATA0) && dt_crc_ok
                         && (dt_len == 7'd8);
            rcd_acc_r <= (st == EP_RCV_DATA) && dt_valid && dt_crc_ok
                         && (dt_pid == (tog ? PID_DATA1 : PID_DATA0));
            sto_acc_r <= (st == EP_STATUS_O) && dt_valid
                         && (dt_len == 7'd0) && dt_crc_ok;
            // the STALL response is the ONLY resp_* branch still keyed
            // on the raw token decode (tk_pid/tk_endp compares) — every
            // other branch keys off a registered strobe.  Left
            // unregistered it dragged the token-decode chain into
            // resp_pid's clock-enable cone (the last 21+ ns path).
            // SETUP preempts via the pre-case branch (different pid, so
            // the two can never fire together); a token at cycle t gets
            // its STALL at t+2, well inside the turnaround budget.
            stall_resp_r <= (st == EP_STALL) && (tk_in || tk_out);
        end
    end

    always @(posedge clk) begin
        if (rst || bus_reset) begin
            st          <= EP_IDLE;
            tog         <= 1'b0;
            remain      <= 7'd0;
            send_len    <= 7'd0;
            base        <= 10'd0;
            sel         <= 3'd0;
            expect_len  <= 7'd0;
            recv_cnt    <= 7'd0;
            addr_pending <= 7'd0;
            cfg_pending  <= 1'b0;
            pend_addr    <= 1'b0;
            pend_cfg     <= 1'b0;
            p_action     <= 3'd0;
            p_wlen0      <= 1'b0;
            p_desc_in    <= 1'b0;
            send_zlp_r   <= 1'b0;
            last_r       <= 1'b0;
            dev_addr     <= 7'd0;
            configured   <= 1'b0;
            resp_req     <= 1'b0;
            line_coding  <= lc_default;
        end else begin
            resp_req <= 1'b0;

            // SETUP aborts any transfer in progress (USB 2.0 §8.5.2.2)
            if (tk_setup) begin
                st       <= EP_RCV_REQ;
                tog      <= 1'b0;        // SETUP data stage is DATA0
                base     <= 10'd0;
                recv_cnt <= 7'd0;
                resp_req <= 1'b0;
            end else begin
                case (st)
                EP_IDLE: ;
                EP_RCV_REQ: begin
                    // SETUP data stage: DATA0 with the 8 request bytes.
                    // Only LATCH the request and ACK here; the parse runs
                    // one clk later from the registered bytes (the whole
                    // request-decode tree from dt_head is too long for
                    // one 48 MHz cycle)
                    if (req_acc_r) begin
                        rq0 <= dt_head[0*8 +: 8];
                        rq1 <= dt_head[1*8 +: 8];
                        rq2 <= dt_head[2*8 +: 8];
                        rq3 <= dt_head[3*8 +: 8];
                        rq6 <= dt_head[6*8 +: 8];
                        rq7 <= dt_head[7*8 +: 8];
                        resp_req  <= 1'b1;         // ACK the request
                        resp_pid  <= PID_ACK;
                        resp_len  <= 7'd0;
                        resp_sel  <= 3'd0;
                        resp_base <= 10'd0;
                        st        <= EP_PARSE;
                    end
                end
                EP_PARSE: begin
                    // decode stage: the request case tree (rq1 class,
                    // descriptor sub-case, sub-sub-case) is split across
                    // two cycles; here only ONE-hot flags + the raw
                    // wLength register.  Lengths/states are computed in
                    // EP_SETUP from these REGISTERED flags.
                    p_sel      <= new_sel;
                    p_wlen     <= {rq7, rq6};
                    p_is_desc      <= (rq1 == 8'h06) && (new_sel != 3'd7);
                    p_is_get_status<= (rq1 == 8'h00) && (rq0 == 8'h80);
                    p_is_set_addr  <= (rq1 == 8'h05);
                    p_is_set_cfg   <= (rq1 == 8'h09)
                                      && ((rq2 == 8'h01) || (rq2 == 8'h00));
                    p_is_set_lc    <= (rq1 == 8'h20) && (rq0 == 8'h21);
                    p_is_get_lc    <= (rq1 == 8'h21) && (rq0 == 8'hA1);
                    p_is_cls       <= (rq1 == 8'h22) && (rq0 == 8'h21);
                    p_valid        <= ((rq1 == 8'h06) && (new_sel != 3'd7))
                                   || ((rq1 == 8'h00) && (rq0 == 8'h80))
                                   || (rq1 == 8'h05)
                                   || ((rq1 == 8'h09)
                                       && ((rq2 == 8'h01) || (rq2 == 8'h00)))
                                   || ((rq1 == 8'h20) && (rq0 == 8'h21))
                                   || ((rq1 == 8'h21) && (rq0 == 8'hA1))
                                   || ((rq1 == 8'h22) && (rq0 == 8'h21));
                    // side effects latched here, tagged per request so
                    // only the owning transfer's status stage applies them
                    addr_pending <= rq2[6:0];           // SET_ADDRESS value
                    cfg_pending  <= (rq2 == 8'h01);    // SET_CONFIGURATION
                    pend_addr    <= (rq1 == 8'h05);
                    pend_cfg     <= (rq1 == 8'h09)
                                    && ((rq2 == 8'h01) || (rq2 == 8'h00));
                    expect_len   <= (p_wlen_next_7);
                    st <= EP_SETUP;
                end
                EP_SETUP: begin
                    // compute stage: route the REGISTERED decode flags into
                    // a one-hot action register.  This is the only place
                    // the request routing tree runs, and it lands on ONE
                    // register (not on datapath clock enables), keeping
                    // the tree out of every other register's cone.
                    p_wlen0   <= (w_length == 16'd0);
                    p_desc_in <= rq0[7];
                    p_dremain_r <= (|p_wlen[14:7]) ? desc_len(p_sel)
                                 : ((p_wlen[6:0] < desc_len(p_sel))
                                    ? p_wlen[6:0] : desc_len(p_sel));
                    p_action  <= (!p_valid) ? 3'd0
                               : (p_is_set_addr || p_is_set_cfg || p_is_cls
                                  || ((p_is_set_lc || p_is_get_lc)
                                      && (w_length == 16'd0))) ? 3'd1
                               : p_is_get_status ? 3'd2
                               : p_is_set_lc ? 3'd3
                               : p_is_get_lc ? 3'd4
                               : 3'd5;
                    st <= EP_ARM;
                end
                EP_ARM: begin
                    // apply stage: one-hot action decode -> datapath +
                    // next state (shallow: 3-bit case + small compares
                    // on registered p_wlen/p_sel)
                    case (p_action)
                    3'd1: st <= EP_STATUS_I;      // no-data control write
                    3'd2: begin                   // GET_STATUS (device)
                        sel <= 3'd6;
                        tog <= 1'b1;
                        if (p_wlen0) begin
                            st <= EP_STATUS_O;    // read -> OUT
                        end else begin
                            st  <= EP_SEND;
                            base <= 10'd0;
                            remain <= (p_wlen[6:0] > 7'd2) ? 7'd2
                                                               : p_wlen[6:0];
                        end
                    end
                    3'd3: begin                   // SET_LINE_CODING
                        tog <= 1'b1;              // host data DATA1
                        st  <= EP_RCV_DATA;
                    end
                    3'd4: begin                   // GET_LINE_CODING
                        sel <= 3'd5;
                        tog <= 1'b1;
                        st  <= EP_SEND;
                        base <= 10'd0;
                        remain <= (p_wlen[6:0] > 7'd7) ? 7'd7
                                                       : p_wlen[6:0];
                    end
                    3'd5: begin                   // GET_DESCRIPTOR
                        sel <= p_sel;
                        tog <= 1'b1;
                        if (p_wlen0) begin
                            st <= p_desc_in ? EP_STATUS_O : EP_STATUS_I;
                        end else begin
                            st  <= EP_SEND;
                            base <= 10'd0;
                            remain <= p_dremain_r;   // precomputed in EP_SETUP
                        end
                    end
                    default: st <= EP_STALL;    // 3'd0: unsupported
                    endcase
                end
                EP_SEND: begin
                    // device->host data stage: answer IN tokens with data.
                    // The packet-endpoint decisions (ZLP? last packet?)
                    // are precomputed HERE from `remain` and consumed as
                    // registers in EP_WAIT_ACK (no compares there).
                    if (tk_in) begin
                        resp_req  <= 1'b1;
                        resp_pid  <= tog ? PID_DATA1 : PID_DATA0;
                        resp_sel  <= sel;
                        resp_base <= base;
                        send_len   <= (remain > 7'd64) ? 7'd64 : remain;
                        resp_len   <= (remain > 7'd64) ? 7'd64 : remain;
                        send_zlp_r <= (remain == 7'd0);
                        last_r     <= (remain <= 7'd64);
                        st <= EP_WAIT_ACK;
                    end
                end
                EP_WAIT_ACK: begin
                    if (hs_ack) begin
                        if (send_zlp_r) begin
                            // status DATA1 ZLP accepted: transfer complete
                            if (pend_addr)
                                dev_addr  <= addr_pending;
                            if (pend_cfg)
                                configured <= cfg_pending;
                            st <= EP_IDLE;
                        end else begin
                            // data packet accepted: advance / complete
                            tog   <= ~tog;
                            base  <= base + {3'd0, send_len};
                            remain <= remain - send_len;
                            st <= last_r ? EP_STATUS_O : EP_SEND;
                        end
                    end else if (tk_in) begin
                        // host retried the IN token (no ACK seen):
                        // re-arm the same packet (base/remain/tog unchanged)
                        st <= send_zlp_r ? EP_STATUS_I : EP_SEND;
                    end
                end
                EP_RCV_DATA: begin
                    // host->device data stage: DATA packets after OUT.
                    // The line_coding byte store lives in its own
                    // always block above (lc_cap_r).  Reacts to the
                    // REGISTERED accept one clk after arrival.
                    if (rcd_acc_r) begin
                        resp_req <= 1'b1;             // ACK the data
                        resp_pid <= PID_ACK;
                        resp_len <= 7'd0;
                        tog      <= ~tog;
                        recv_cnt <= recv_cnt + dt_len;
                        if (recv_cnt + dt_len >= expect_len)
                            st <= EP_STATUS_I;        // write: status is IN
                    end
                    // bad crc / toggle mismatch: silence; host retries
                end
                EP_STATUS_I: begin
                    // status stage (control write / none): device DATA1 ZLP
                    if (tk_in) begin
                        resp_req  <= 1'b1;
                        resp_pid  <= PID_DATA1;
                        resp_len  <= 7'd0;
                        resp_sel  <= 3'd0;
                        resp_base <= 10'd0;
                        send_len  <= 7'd0;
                        send_zlp_r <= 1'b1;
                        last_r     <= 1'b0;
                        st <= EP_WAIT_ACK;
                    end
                end
                EP_STATUS_O: begin
                    // status stage (control read): host OUT + DATA1 ZLP;
                    // ACK it and complete.  Lenient on the data pid
                    // (hosts sending DATA0 here are accepted).
                    // Reacts to the REGISTERED accept one clk after
                    // arrival.
                    if (sto_acc_r) begin
                        resp_req <= 1'b1;
                        resp_pid <= PID_ACK;
                        resp_len <= 7'd0;
                        if (pend_addr)
                            dev_addr  <= addr_pending;
                        if (pend_cfg)
                            configured <= cfg_pending;
                        st <= EP_IDLE;
                    end
                end
                EP_STALL: begin
                    // reacts to the REGISTERED token decode (+1 clk)
                    if (stall_resp_r) begin
                        resp_req <= 1'b1;
                        resp_pid <= PID_STALL;
                        resp_len <= 7'd0;
                    end
                end
                endcase
            end
        end
    end
endmodule

// ===========================================================================
// usb_serial — EP1 bulk endpoint protocol: data toggles + ACK/NAK/ARM
// decisions.  Data movement lives in usb_uart (FIFOs, snapshot regs, tx
// PHY mux).  IN packets are snapshotted from the tx FIFO into usb_uart's
// 64-byte snapshot register at arm time, so an unacknowledged packet is
// retransmitted byte-identically (same data pid) until the host ACKs.
// Data toggles reset on bus reset and SET_CONFIGURATION (USB 2.0 §9.1.1.5).
// ===========================================================================
module usb_serial(
    input  wire        clk,
    input  wire        rst,
    input  wire        bus_reset,
    input  wire        set_cfg,       // pulse: SET_CONFIGURATION completed
    // token strobes (router-gated: address + endpoint 1 matched)
    input  wire        tk_in,
    input  wire        tk_out,
    // data packet strobe (router-gated: an armed OUT to EP1)
    input  wire        dt_valid,
    input  wire  [3:0] dt_pid,
    input  wire  [6:0] dt_len,
    input  wire        dt_crc_ok,
    // handshake strobe (router-gated: our in-flight packet was ACKed)
    input  wire        hs_ack,
    // FIFO status (usb_uart)
    input  wire        txfifo_empty,
    input  wire  [6:0] txfifo_count,
    input  wire        rx_room,       // 1: the whole packet fits the rx FIFO
    input  wire  [6:0] tx_room,        // free slots in the tx FIFO
    // response request (1-clk pulse)
    output reg         resp_req,
    output reg  [3:0] resp_pid,
    output reg  [6:0] resp_len,
    output reg         resp_fresh,     // 1: snapshot len bytes from tx FIFO
    // router hints
    output wire        ser_busy,       // OUT data packet expected
    output wire        ser_ack_wait    // our data packet is in flight
);
    localparam [3:0] PID_DATA0 = 4'h3, PID_DATA1 = 4'hB,
                     PID_ACK = 4'h2, PID_NAK = 4'hA;

    reg       tog_tx, tog_rx;    // 0 => next data pid DATA0
    reg       pkt_pending;       // snapshot holds an unacknowledged packet
    reg [6:0] pkt_len;
    reg       armed_out;
    reg       dt_acc, dt_rej;    // registered accept/reject decision

    assign ser_busy    = armed_out;
    assign ser_ack_wait = pkt_pending;

    always @(posedge clk) begin
        if (rst || bus_reset || set_cfg) begin
            tog_tx     <= 1'b0;
            tog_rx     <= 1'b0;
            pkt_pending <= 1'b0;
            pkt_len    <= 7'd0;
            armed_out  <= 1'b0;
            dt_acc     <= 1'b0;
            dt_rej     <= 1'b0;
            resp_req   <= 1'b0;
        end else begin
            resp_req   <= 1'b0;
            resp_fresh <= 1'b0;

            if (tk_in) begin
                if (pkt_pending) begin
                    // retransmit the unacknowledged packet, same pid
                    resp_req <= 1'b1;
                    resp_pid <= tog_tx ? PID_DATA1 : PID_DATA0;
                    resp_len <= pkt_len;
                end else if (!txfifo_empty) begin
                    // arm a new packet: snapshot min(count, 64) bytes
                    pkt_len     <= (txfifo_count > 7'd64)
                                 ? 7'd64 : {2'b0, txfifo_count};
                    pkt_pending <= 1'b1;
                    resp_req  <= 1'b1;
                    resp_pid  <= tog_tx ? PID_DATA1 : PID_DATA0;
                    resp_len  <= (txfifo_count > 7'd64)
                               ? 7'd64 : {2'b0, txfifo_count};
                    resp_fresh <= 1'b1;
                end else begin
                    resp_req <= 1'b1;      // nothing to send: NAK
                    resp_pid <= PID_NAK;
                    resp_len <= 7'd0;
                end
            end

            if (tk_out)
                armed_out <= 1'b1;          // data packet expected next

            // OUT data decision, REGISTERED one clk after dt_valid: the
            // room check (an add+compare crossing the rx FIFO count) and
            // the response/toggle registers then land in separate
            // cycles (+1 clk response latency is well inside the device
            // turnaround budget).  usb_uart streams the payload on the
            // serial ACK pulse.
            dt_acc <= armed_out && dt_valid && dt_crc_ok
                      && (dt_pid == (tog_rx ? PID_DATA1 : PID_DATA0))
                      && rx_room;
            dt_rej <= armed_out && dt_valid
                      && !(dt_crc_ok
                           && (dt_pid == (tog_rx ? PID_DATA1 : PID_DATA0))
                           && rx_room);
            if (armed_out && dt_valid)
                armed_out <= 1'b0;
            if (dt_acc) begin
                resp_req <= 1'b1;      // accept: usb_uart pushes + we ACK
                resp_pid <= PID_ACK;
                resp_len <= 7'd0;
                tog_rx  <= ~tog_rx;
            end else if (dt_rej) begin
                resp_req <= 1'b1;      // full/bad: NAK, no toggle
                resp_pid <= PID_NAK;
                resp_len <= 7'd0;
            end

            if (hs_ack && pkt_pending) begin
                // our in-flight data packet was accepted: forget it
                pkt_pending <= 1'b0;
                tog_tx      <= ~tog_tx;
            end
        end
    end
endmodule

// ===========================================================================
// usb_uart — core top: descriptor ROM, token router, response arbiter,
// bulk snapshot loader, rx push, FIFOs + byte-stream loopback interface.
// ===========================================================================
module usb_uart(
    input  wire        clk,
    input  wire        rst,           // sync, active high (POR in Top)
    input  wire        dp_rx,         // pad readback (SB_IO D_IN_0)
    input  wire        dn_rx,
    output wire        dp_tx,         // to pads (SB_IO D_OUT_0)
    output wire        dn_tx,
    output wire        oe,            // pad drive enable (SB_IO OUTPUT_ENABLE)
    // byte-stream FIFO interface (plan §2; loopback wired in Top)
    output wire [7:0]  rx_data,
    output wire        rx_valid,      // 1-clk strobe per byte (= FIFO pop)
    input  wire [7:0]  tx_data,
    input  wire        tx_strobe,     // FIFO push strobe
    output wire        tx_ready,      // FIFO has room
    // status (TB visibility)
    output wire        bus_reset_o,
    output wire        connected_o
);
    localparam [3:0] PID_OUT = 4'h1, PID_IN = 4'h9, PID_SETUP = 4'hD,
                     PID_SOF = 4'h5, PID_ACK = 4'h2, PID_NAK = 4'hA;

    // ------------------------------------------------------------------
    // RX chain
    // ------------------------------------------------------------------
    wire rx_bit, rx_bit_v, rx_sop, rx_eop, rx_active, rx_perr, bus_reset;
    usb_rx_phy u_rxphy(
        .clk(clk), .rst(rst),
        .dp(dp_rx), .dn(dn_rx),
        .bit_o(rx_bit), .bit_valid(rx_bit_v),
        .sop(rx_sop), .eop(rx_eop), .active(rx_active),
        .pkt_err(rx_perr), .bus_reset(bus_reset));

    wire tk_valid, tk_crc_ok, dt_valid, dt_crc_ok, hs_valid;
    wire [3:0]  tk_pid, dt_pid, hs_pid;
    wire [6:0]  tk_addr, dt_len;
    wire [3:0]  tk_endp;
    wire [10:0] tk_frame;
    // rx packet payload capture (off-fabric, see usb_rx_packet): the
    // flopped head window + the payload RAM's registered read.  The RAM
    // read address is the rx-push byte index push_i (declared with the
    // rx-push block below).
    wire [63:0] dt_head;
    wire [7:0]  dt_rd_data;
    usb_rx_packet u_rxpkt(
        .clk(clk), .rst(rst), .bus_reset(bus_reset),
        .bit_i(rx_bit), .bit_valid(rx_bit_v),
        .sop(rx_sop), .eop(rx_eop), .pkt_err(rx_perr),
        .tk_valid(tk_valid), .tk_pid(tk_pid), .tk_addr(tk_addr),
        .tk_endp(tk_endp), .tk_frame(tk_frame), .tk_crc_ok(tk_crc_ok),
        .dt_valid(dt_valid), .dt_pid(dt_pid), .dt_len(dt_len),
        .dt_head(dt_head), .dt_rd_data(dt_rd_data),
        .dt_rd_addr(push_i[5:0]), .dt_crc_ok(dt_crc_ok),
        .hs_valid(hs_valid), .hs_pid(hs_pid));

    assign bus_reset_o = bus_reset;

    // ------------------------------------------------------------------
    // FIFOs + byte-stream loopback interface
    // ------------------------------------------------------------------
    wire [7:0] rxfifo_rd, txfifo_rd;
    wire [6:0] rxfifo_cnt, txfifo_cnt;
    wire rxfifo_full, rxfifo_empty, txfifo_full, txfifo_empty;

    // registered loopback strobe pair (now three stages on the rx side:
    // the BRAM read is registered): the pop decision, then the
    // (rx_valid pulse, rx_data byte) pair the consumer latches one clk
    // after the BRAM read port has produced the popped byte.
    // (the PoC Top wires the loopback: tx_data = rx_data, tx_strobe =
    // rx_valid; the pipelined pair keeps the plan §2 contract and takes
    // the empty/full decode off the single-cycle paths)
    reg       rx_pop_d;
    reg       rx_valid_r, rx_valid_rr;
    reg [7:0] rx_data_d;
    always @(posedge clk) begin
        if (rst || bus_reset) begin
            rx_pop_d   <= 1'b0;
            rx_valid_r <= 1'b0;
            rx_valid_rr<= 1'b0;
        end else begin
            // tx headroom: the 3-stage rx pipe (pop decision -> valid
            // -> BRAM read -> Top strobe -> push) keeps up to 3 pushes
            // in flight when a decided pop's byte lands, so the decision
            // requires cnt <= 12 (16 - 3 in flight - 1 margin): a landing
            // push then never meets a full txfifo and the comb write
            // gate at the txfifo never drops a popped byte.  A popped
            // byte MUST land in the txfifo — nothing else can re-read
            // it from the rxfifo.
            rx_pop_d    <= !rxfifo_empty && (txfifo_cnt < 7'd13);
            rx_valid_r  <= rx_pop_d;
            rx_valid_rr <= rx_valid_r;
            // rxfifo_rd (BRAM, registered read) holds the popped byte
            // from one clk after rx_pop_d — pair it with rx_valid_rr
            rx_data_d   <= rxfifo_rd;
        end
    end

    usb_fifo16x8 #(.DEPTH(64), .AW(6), .RAM(1)) u_rxfifo(
        .clk(clk), .rst(rst || bus_reset),
        .wr_en(rx_push_run_d), .wr_data(dt_rd_data),
        .rd_en(rx_pop_d), .rd_data(rxfifo_rd),
        .count(rxfifo_cnt), .full(rxfifo_full), .empty(rxfifo_empty));
    assign rx_data  = rx_data_d;
    assign rx_valid = rx_valid_rr;
    // does the whole packet fit the rx FIFO?  adder+compare (carry
    // chain), combinational — the serial registers the DECISION one
    // clk later, so this stays out of its response cycle
    wire [7:0] rx_sum = {1'b0, rxfifo_cnt} + {1'b0, dt_len};
    wire rx_room_ok = (rx_sum <= 8'd64);

    usb_fifo16x8 u_txfifo(
        .clk(clk), .rst(rst || bus_reset),
        // comb write gate AT THE WRITE EDGE (a registered gate would be
        // stale by the ~1 clk it takes a gated push to land, and a full
        // check at the arm edge lets one in-flight push overflow the
        // count — bytes armed at 15 land at 16; the loopback admission
        // control is the pop decision's headroom, which is sized for
        // THIS 2-clk pipe).  The txfifo is 16 deep: the gate riding the
        // byte clock enables is a short cone, unlike the 64-deep rxfifo
        // (whose full check lives in rx_push_run's D instead).
        .wr_en(tx_strobe && !txfifo_full), .wr_data(tx_data),
        .rd_en(snap_pop), .rd_data(txfifo_rd),
        .count(txfifo_cnt), .full(txfifo_full), .empty(txfifo_empty));
    wire [6:0] tx_room = 7'd16 - txfifo_cnt;
    assign tx_ready = !txfifo_full;

    // ------------------------------------------------------------------
    // EP0 + serial protocol modules
    // ------------------------------------------------------------------
    wire        ep0_resp_req;
    wire [3:0]  ep0_resp_pid;
    wire [6:0]  ep0_resp_len;
    wire [2:0]  ep0_resp_sel;
    wire [9:0]  ep0_resp_base;
    wire [55:0] line_coding;
    wire [6:0]  dev_addr;
    wire        configured, ep0_busy, ep0_wait_ack;

    wire        ser_resp_req, ser_resp_fresh, ser_busy, ser_ack_wait;
    wire [3:0]  ser_resp_pid;
    wire [6:0]  ser_resp_len;

    usb_ep0 u_ep0(
        .clk(clk), .rst(rst), .bus_reset(bus_reset),
        .tk_setup(ep0_tk_setup), .tk_in(ep0_tk_in), .tk_out(ep0_tk_out),
        .dt_valid(ep0_dt_valid_r), .dt_pid(dt_pid), .dt_len(dt_len),
        .dt_head(dt_head), .dt_crc_ok(dt_crc_ok),
        .hs_ack(ep0_hs_ack),
        .resp_req(ep0_resp_req), .resp_pid(ep0_resp_pid),
        .resp_len(ep0_resp_len), .resp_sel(ep0_resp_sel),
        .resp_base(ep0_resp_base),
        .dev_addr(dev_addr), .configured(configured),
        .line_coding(line_coding),
        .ep0_busy(ep0_busy), .ep0_wait_ack(ep0_wait_ack));

    usb_serial u_serial(
        .clk(clk), .rst(rst), .bus_reset(bus_reset), .set_cfg(cfg_strobe),
        .tk_in(ser_tk_in), .tk_out(ser_tk_out),
        .dt_valid(ser_dt_valid), .dt_pid(dt_pid), .dt_len(dt_len),
        .dt_crc_ok(dt_crc_ok),
        .hs_ack(ser_hs_ack),
        .txfifo_empty(txfifo_empty), .txfifo_count(txfifo_cnt),
        .rx_room(rx_room_ok), .tx_room(tx_room),
        .resp_req(ser_resp_req), .resp_pid(ser_resp_pid),
        .resp_len(ser_resp_len), .resp_fresh(ser_resp_fresh),
        .ser_busy(ser_busy), .ser_ack_wait(ser_ack_wait));

    // ------------------------------------------------------------------
    // Token router: address filter + endpoint decode.  The device answers
    // only its own address (starts 0; SET_ADDRESS latches at status
    // completion), SOF tokens decode but are ignored.
    // The address compare + pid/endp decode is REGISTERED (tok_v_r, +1
    // clk on token strobes — well inside the device turnaround budget):
    // the dev_addr compare then sits in ONE register's D-cone instead of
    // fanning into every consumer's clock-enable logic (that fanout was
    // the critical path).  tk_pid/tk_endp hold their values between
    // packets, so decoding them one clk late is exact.
    // ------------------------------------------------------------------
    reg tok_v_r;
    always @(posedge clk) begin
        if (rst || bus_reset)
            tok_v_r <= 1'b0;
        else
            tok_v_r <= tk_valid && tk_crc_ok && (tk_addr == dev_addr);
    end
    // The FULL token strobes are REGISTERED (+1 clk on top of tok_v_r):
    // the pid/endp decode otherwise stays combinational off tok_v_r and
    // every consumer's register-enable cone ingests it (tk_pid -> decode
    // -> resp_*/pkt_len CEN was the last long path family).  tk_pid/
    // tk_endp hold their values between packets, so decoding them one
    // clk later is exact; the +1 clk on token strobes is free (turnaround
    // budget ~30-50 clks; inter-packet gap >= 2 bit times = 8 clks, so
    // no subsequent packet can outrun the delayed strobe).
    reg ep0_tk_setup_r, ep0_tk_in_r, ep0_tk_out_r;
    reg ser_tk_in_r,  ser_tk_out_r;
    reg ep2_tk_in_r,  ep2_tk_out_r;
    always @(posedge clk) begin
        if (rst || bus_reset) begin
            ep0_tk_setup_r <= 1'b0;
            ep0_tk_in_r    <= 1'b0;
            ep0_tk_out_r   <= 1'b0;
            ser_tk_in_r    <= 1'b0;
            ser_tk_out_r   <= 1'b0;
            ep2_tk_in_r    <= 1'b0;
            ep2_tk_out_r   <= 1'b0;
        end else begin
            ep0_tk_setup_r <= tok_v_r && (tk_pid == PID_SETUP) && (tk_endp == 4'd0);
            ep0_tk_in_r    <= tok_v_r && (tk_pid == PID_IN)    && (tk_endp == 4'd0);
            ep0_tk_out_r   <= tok_v_r && (tk_pid == PID_OUT)   && (tk_endp == 4'd0);
            ser_tk_in_r    <= tok_v_r && (tk_pid == PID_IN)    && (tk_endp == 4'd1);
            ser_tk_out_r   <= tok_v_r && (tk_pid == PID_OUT)   && (tk_endp == 4'd1);
            ep2_tk_in_r    <= tok_v_r && (tk_pid == PID_IN)    && (tk_endp == 4'd2);
            ep2_tk_out_r   <= tok_v_r && (tk_pid == PID_OUT)   && (tk_endp == 4'd2);
        end
    end
    wire ep0_tk_setup = ep0_tk_setup_r;
    wire ep0_tk_in    = ep0_tk_in_r;
    wire ep0_tk_out   = ep0_tk_out_r;
    wire ser_tk_in    = ser_tk_in_r;
    wire ser_tk_out   = ser_tk_out_r;
    wire ep2_tk_in    = ep2_tk_in_r;
    wire ep2_tk_out   = ep2_tk_out_r;

    // data packet router: the module expecting a data packet owns it.
    // EP0's strobe is REGISTERED one clk after arrival: the rxpkt EOP
    // outputs (dt_len = bc-2, dt_crc_ok = 16-bit CRC-residual compare)
    // carry wide cones in their D-terms, and fanning them into EP0's
    // accept enables in the ARRIVAL cycle was the critical path.  Every
    // field EP0 reads (dt_head / dt_pid / dt_len / dt_crc_ok) is a
    // register that HOLDS until the next packet's EOP, so the delayed
    // read is bit-identical; the +1 clk is free (device turnaround
    // budget ~30-50 clks, the core uses ~20).  usb_serial keeps the
    // arrival-cycle strobe: its accept decision is already registered
    // one clk later (dt_acc/dt_rej), so its cone was never critical.
    // No token can land in the 1-clk strobe delay: the inter-packet
    // gap is >= 2 bit times = 8 clks.
    wire ep0_dt_gate = dt_valid && ep0_busy;
    reg  ep0_dt_valid_r;
    always @(posedge clk)
        if (rst || bus_reset) ep0_dt_valid_r <= 1'b0;
        else                  ep0_dt_valid_r <= ep0_dt_gate;
    wire ser_dt_valid = dt_valid && !ep0_busy && ser_busy;

    // handshake router: the module with a response in flight owns it
    wire ep0_hs_ack = hs_valid && (hs_pid == PID_ACK) && ep0_wait_ack;
    wire ser_hs_ack = hs_valid && (hs_pid == PID_ACK)
                      && !ep0_wait_ack && ser_ack_wait;

    // ------------------------------------------------------------------
    // Response arbiter: requests never overlap in protocol order (one
    // response per token/data packet), so a simple priority mux with an
    // in-flight owner register suffices.
    // ------------------------------------------------------------------
    // notif EP (EP2): the host polls it, the core never arms it — every
    // token gets a NAK.  REGISTERED: a combinational NAK off the token
    // router would drag the whole token-decode cone into the tx engine's
    // state logic (the response comes 1 clk later, still far inside the
    // device turnaround budget).
    reg ep2_nak_req;
    always @(posedge clk)
        if (rst)        ep2_nak_req <= 1'b0;
        else            ep2_nak_req <= ep2_tk_in_r || ep2_tk_out_r;
    wire tx_start = ep0_resp_req || ser_resp_req || ep2_nak_req;

    reg [1:0] tx_owner;   // 0 none, 1 ep0, 2 serial, 3 ep2
    wire tx_busy;
    wire tx_done;
    always @(posedge clk) begin
        if (rst)                 tx_owner <= 2'd0;
        else if (tx_start)       tx_owner <= ep0_resp_req ? 2'd1
                                       : ser_resp_req ? 2'd2 : 2'd3;
        else if (tx_done)        tx_owner <= 2'd0;
    end

    wire [3:0] tx_pid_w = ep0_resp_req ? ep0_resp_pid
                       : ser_resp_req ? ser_resp_pid
                       : PID_NAK;   // EP2 notif endpoint
    wire [6:0] tx_len_w = ep0_resp_req ? ep0_resp_len
                       : ser_resp_req ? ser_resp_len
                       : 7'd0;

    // ------------------------------------------------------------------
    // Snapshot loader: a fresh bulk-IN arm pops min(count,64) bytes from
    // the tx FIFO into snap[] one byte per clk (byte 0 is first needed
    // ~72+ clks after the arm, so the load always wins the race).
    // ------------------------------------------------------------------
    (* ram_style = "block" *) reg [7:0] snap [0:63];
    reg       load_run;
    reg [6:0] li, snap_len;
    wire      snap_pop = load_run;
    // sync-read port (BRAM): address held one byte ahead (see producer)
    reg [7:0] snap_q;
    always @(posedge clk) snap_q <= snap[byte_rd_addr];

    always @(posedge clk) begin
        if (rst || bus_reset) begin
            load_run <= 1'b0;
            li <= 7'd0;
        end else begin
            if (tx_start && ser_resp_req && ser_resp_fresh) begin
                li <= 7'd0;
                snap_len <= tx_len_w;
                load_run <= (tx_len_w != 7'd0);
            end else if (load_run) begin
                snap[li] <= txfifo_rd;
                if (li + 7'd1 == snap_len)
                    load_run <= 1'b0;
                else
                    li <= li + 7'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // rx push: accepted bulk-OUT payloads stream from the rx packet's
    // payload RAM into the rx FIFO, one byte per clk.  The 64:1 payload
    // mux the old rx_push_byte register closed over is GONE (the payload
    // store is off-fabric, see usb_rx_packet): usb_rx_packet registers
    // dt_ram[push_i] into dt_rd_data one clk after the index holds it,
    // so the 1-stage rx_push_run_d strobe pairs index and byte exactly
    // (byte i written at the C(2+i)->C(3+i) posedge, once, in order):
    //     ser_accept during C0: push_i<-0, rx_push_run<-1
    //     C1: dt_rd_addr=0 -> posedge: dt_rd_data=byte0 during C2,
    //                            rx_push_run_d <- run(C1)=1 during C2
    //     posedge C2: FIFO writes byte0 (wr_en=run_d, wr_data=rd_data)
    // wr_en is pre-gated with !full inside rx_push_run's own D (the
    // serial accept already required cnt+dt_len <= 64), so the rxfifo
    // write port sees two clean registers.
    // ------------------------------------------------------------------
    reg       rx_push_run;
    reg [6:0] push_i;
    reg [6:0] dt_len_hold;
    reg       rx_push_run_d;
    wire ser_accept = ser_resp_req && (ser_resp_pid == 4'h2); // ACK

    always @(posedge clk) begin
        if (rst || bus_reset) begin
            rx_push_run   <= 1'b0;
            rx_push_run_d <= 1'b0;
            push_i <= 7'd0;
            dt_len_hold <= 7'd0;
        end else begin
            rx_push_run_d <= rx_push_run;
            if (ser_accept) begin
                push_i      <= 7'd0;
                dt_len_hold <= dt_len;
                rx_push_run <= (dt_len != 7'd0) && !rxfifo_full;
            end else if (rx_push_run) begin
                if (push_i + 7'd1 >= dt_len_hold)
                    rx_push_run <= 1'b0;
                else
                    push_i <= push_i + 7'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Descriptor ROM: 256 x 8 initialized BRAM (sync read).  Contiguous
    // pages: sel0 device@0 (18 B), sel1 config@18 (75 B), sel2 str0@93,
    // sel3 str1@97 (10 B), sel4 str2@107 (28 B).  sel5 (line coding)
    // and sel6 (status) are dynamic/trivial and bypass the ROM.  This
    // replaces the desc_byte() case cascade: a 128-deep case is ~20 ns
    // of LUT chain; a BRAM read is a register.
    // ------------------------------------------------------------------
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

    // ------------------------------------------------------------------
    // Byte fetch pipeline (per byte of the in-flight packet):
    //   tx_addr_next (comb from s) -> byte_rd_addr (reg)
    //      -> snap_q / desc_q (BRAM sync-read output register)
    // The producer mux below therefore feeds the PHY from registered
    // sources only; the PHY's tx_byte register is a further stage.
    // ------------------------------------------------------------------
    reg [6:0] byte_rd_addr;
    always @(posedge clk) byte_rd_addr <= tx_addr_next_w;

    // EP0 ROM address: page start + transfer offset, latched at arm
    function [7:0] rom_page;
        input [2:0] sd;
        begin
            case (sd)
            3'd0: rom_page = 8'd0;    // device
            3'd1: rom_page = 8'd18;   // config
            3'd2: rom_page = 8'd93;   // string 0
            3'd3: rom_page = 8'd97;   // string 1
            3'd4: rom_page = 8'd107;  // string 2
            default: rom_page = 8'd0;
            endcase
        end
    endfunction
    reg [7:0] rom_base_r;
    always @(posedge clk)
        if (rst) rom_base_r <= 8'd0;
        else if (ep0_resp_req)
            rom_base_r <= rom_page(ep0_resp_sel) + ep0_resp_base[7:0];

    reg [7:0] rom_addr_r;
    always @(posedge clk) rom_addr_r <= rom_base_r + {1'b0, byte_rd_addr};
    reg [7:0] desc_q;
    always @(posedge clk) desc_q <= desc_rom[rom_addr_r];

    // dynamic EP0 responses: line coding (7 B, registers) and
    // GET_STATUS (2 zero bytes)
    reg [7:0] ep0_small_q;
    always @(posedge clk)
        ep0_small_q <= (ep0_resp_sel == 3'd5)
            ? ((byte_rd_addr < 7'd7)
               ? line_coding[8*byte_rd_addr +: 8] : 8'h00)
            : 8'h00;

    // tx PHY producer mux: registered sources only
    wire [7:0] tx_byte = (tx_owner == 2'd2) ? snap_q
                      : (ep0_resp_sel >= 3'd5) ? ep0_small_q
                      : desc_q;

    wire [6:0] tx_addr_next_w;
    usb_tx_phy u_txphy(
        .clk(clk), .rst(rst),
        .tx_start(tx_start), .tx_pid(tx_pid_w), .tx_len(tx_len_w),
        .tx_byte(tx_byte), .tx_addr(), .tx_addr_next(tx_addr_next_w),
        .tx_busy(tx_busy), .tx_done(tx_done),
        .dp(dp_tx), .dn(dn_tx), .oe(oe));

    // SET_CONFIGURATION completed -> reset bulk data toggles (§9.1.1.5)
    reg configured_d;
    wire cfg_strobe = configured ^ configured_d;
    always @(posedge clk)
        if (rst) configured_d <= 1'b0;
        else     configured_d <= configured;

    assign connected_o = configured;
endmodule
