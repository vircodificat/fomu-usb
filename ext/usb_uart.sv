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
//   usb_rx_packet         bit stream -> packet records
//   (usb_tx_phy, the TX PHY, migrated to src/usb.vir as usb::TxPhy)
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
    \usb::Crc5 u_crc5 (
        .clk(clk),
        .rst(sop | rst | bus_reset),
        .adv(bit_valid && (st == ST_BODY) && !sec
             && (nb >= 5'd8) && (nb <= 5'd23)),
        .din(bit_i),
        .crc(c5));
    \usb::Crc16 u_crc16 (
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
    // are regs; bit_valid/pkt_err are registered upstream in usb::RxPhy)
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
                            // check via usb::Crc5 and are not captured.
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
// OUT token (control read).  Descriptor bytes are pulled by usb::TxPhy
// from the usb_uart ROM via (resp_sel, resp_base + tx_addr).
//
// Request pipeline (48 MHz timing; each stage is one clk):
//   EP_RCV_REQ latch request  ->  EP_PARSE decode flags (rq* are
//   registered)  ->  EP_SETUP route flags into the one-hot p_action
//   ->  EP_ARM apply p_action (datapath + state).  The request case
//   tree never reaches a datapath register enable combinationally.
// Data-stage accepts are REGISTERED (req_acc_r / sto_acc_r, one clk
// after the router's data strobe): the accept condition trees
// otherwise all feed st's next-state mux
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
    // router hints
    output wire        ep0_busy,     // EP0 expects a data packet
    output wire        ep0_stalling, // EP0 is STALLing (its data packets
                                  // still route here until the next SETUP)
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
    reg        p_is_cls;
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
    // registered data-stage accepts (see the header comment): each is
    // an independent shallow cone (state decode + packet-field
    // compares) that lands on its OWN register — st's next-state then
    // keys off one clean registered bit instead of three condition
    // trees
    reg        req_acc_r;   // SETUP request packet accepted
    reg        sto_acc_r;   // status-stage ZLP accepted

    reg [3:0] st;
    reg       tog;                   // 1 => next device data pid = DATA1
    reg [6:0] remain;               // data stage bytes left to send
    reg [6:0] send_len;             // bytes in the in-flight packet
    reg [9:0] base;                 // current send offset
    reg [2:0] sel;                   // current descriptor select
    // request bytes as seen THIS cycle (dt_head is stable during the
    // request packet's dt_valid strobe); rq* registers hold them for
    // later stages
    // wLength from the REGISTERED request bytes (the parse runs one
    // clk after the SETUP data, so the registered view is current)
    wire [15:0] w_length = {rq7, rq6};

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

    assign ep0_busy = (st == EP_RCV_REQ)
                   || (st == EP_STATUS_O);
    assign ep0_stalling = (st == EP_STALL);
    assign ep0_wait_ack = (st == EP_WAIT_ACK);

    // descriptor lengths — must match usb_uart's desc_byte ROM
    // sel: 0=device 1=config 2=str0 3=str1 4=str2 6=status
    function [6:0] desc_len;
        input [2:0] s;
        begin
            case (s)
            3'd0: desc_len = 7'd18;
            3'd1: desc_len = 7'd75;
            3'd2: desc_len = 7'd4;
            3'd3: desc_len = 7'd10;
            3'd4: desc_len = 7'd32;
            default: desc_len = 7'd2;   // status
            endcase
        end
    endfunction

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
            sto_acc_r <= 1'b0;
        end else begin
            req_acc_r <= (st == EP_RCV_REQ) && dt_valid
                         && (dt_pid == PID_DATA0) && dt_crc_ok
                         && (dt_len == 7'd8);
            sto_acc_r <= (st == EP_STATUS_O) && dt_valid
                         && (dt_len == 7'd0) && dt_crc_ok;
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
        end else begin
            resp_req <= 1'b0;

            // SETUP aborts any transfer in progress (USB 2.0 §8.5.2.2)
            if (tk_setup) begin
                st       <= EP_RCV_REQ;
                tog      <= 1'b0;        // SETUP data stage is DATA0
                base     <= 10'd0;
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
                    p_is_cls       <= (rq1 == 8'h22) && (rq0 == 8'h21);
                    p_valid        <= ((rq1 == 8'h06) && (new_sel != 3'd7))
                                   || ((rq1 == 8'h00) && (rq0 == 8'h80))
                                   || (rq1 == 8'h05)
                                   || ((rq1 == 8'h09)
                                       && ((rq2 == 8'h01) || (rq2 == 8'h00)))
                                   || ((rq1 == 8'h22) && (rq0 == 8'h21));
                    // side effects latched here, tagged per request so
                    // only the owning transfer's status stage applies them
                    addr_pending <= rq2[6:0];           // SET_ADDRESS value
                    cfg_pending  <= (rq2 == 8'h01);    // SET_CONFIGURATION
                    pend_addr    <= (rq1 == 8'h05);
                    pend_cfg     <= (rq1 == 8'h09)
                                    && ((rq2 == 8'h01) || (rq2 == 8'h00));
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
                               : (p_is_set_addr || p_is_set_cfg || p_is_cls) ? 3'd1
                               : p_is_get_status ? 3'd2
                               : 3'd3;
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
                    3'd3: begin                   // GET_DESCRIPTOR
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
                    // STALL the transfer's remaining stages.  The data
                    // packet (control write) is answered at its EOP
                    // strobe: firing on the token instead would emit
                    // the STALL while the host is still driving the
                    // data packet (bus collision).  The IN token
                    // (control read) has no data stage, so it is
                    // answered immediately.  Both inputs are registered
                    // strobes — shallow cones.
                    if (dt_valid || tk_in) begin
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
module \usb::UsbUart (
    input  wire        clk,
    input  wire        rst,           // sync, active high (registered por_done,
                                      // see src/usb.vir UsbController)
    input  wire        dp_rx,         // pad readback (SB_IO D_IN_0)
    input  wire        dn_rx,
    output wire        dp_tx,         // to pads (SB_IO D_OUT_0)
    output wire        dn_tx,
    output wire        output_enable, // pad drive enable (SB_IO OUTPUT_ENABLE)
    // byte-stream FIFO interface (plan §2; loopback wired in Top)
    output wire [7:0]  rx_data,
    output wire        rx_valid,      // 1-clk strobe per byte (= FIFO pop)
    input  wire [7:0]  tx_data,
    input  wire        tx_strobe,     // FIFO push strobe
    input  wire        rx_ready,      // low = the rx consumer is full:
                                      // stop popping (it NAKs instead)
    output wire        tx_ready,      // FIFO has room
    // USB activity indicators (LED-visible): rxing/txing stay high for
    // ~175 ms after the last byte of the bulk byte stream in each
    // direction (the 1-clk rx_valid/tx_strobe strobes are far too fast
    // to see, so each is stretched).  rxing = the core is receiving
    // data (host -> device), txing = sending (device -> host).  EP0
    // control traffic is not covered (it is not in rx_valid/tx_strobe).
    output wire        rxing,
    output wire        txing,
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
    \usb::RxPhy u_rxphy (
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
            // Pop while the rx FIFO has a byte, the tx FIFO keeps the
            // pipe's headroom, AND the byte-stream consumer signals
            // room (rx_ready).  With the Virdant echo server as the
            // consumer, rx_ready throttles the pop rate to the UART's
            // 115200 baud drain rate: when it goes low the rx FIFO
            // fills, the serial endpoint NAKs bulk-OUT, and the host
            // retries -- end-to-end flow control with no byte loss.
            // Off-by-one fix (stress S4c, cnt wrap 0->127): the gate
            // samples rxfifo_cnt's PRE-pop value.  When a pop is already
            // in flight (rx_pop_d==1) and cnt==1, that pop consumes the
            // last byte at this same edge, so the scheduled next pop
            // would find an empty FIFO -- and the 7-bit cnt subtract
            // wraps to 127.  From then on the bogus count reads "not
            // empty" and the popper drains stale BRAM bytes into the
            // echo chain every clk while rx_ready pulses.  Block a pop
            // when the in-flight pop is consuming the last byte:
            rx_pop_d    <= !rxfifo_empty && !(rxfifo_cnt == 7'd1 && rx_pop_d)
                        && (txfifo_cnt < 7'd13) && rx_ready;
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
        // The write gate is REGISTERED (txfifo_wr_d): the old comb gate
        // at the write edge put the cnt full-compare into every byte's
        // write-enable cone (via the echo's do_push chain) -- the
        // vir-flow critical path, 21.8 ns.  A registered gate is safe
        // here: the overflow scenario the old comment feared (a push
        // landing inside the gate-to-write window) needs TWO pushes
        // within 1 clk, and the echo pushes at most one byte per ~417
        // clks (single hold slot) and is the only writer of this FIFO.
        // tx_data (the echo's hold_data) is stable across the +1 clk:
        // it only changes on the next receiver byte, >=417 clks later.
        .wr_en(txfifo_wr_d), .wr_data(tx_data),
        .rd_en(snap_pop), .rd_data(txfifo_rd),
        .count(txfifo_cnt), .full(txfifo_full), .empty(txfifo_empty));
    reg txfifo_wr_d;
    always @(posedge clk)
        txfifo_wr_d <= tx_strobe && !txfifo_full;
    wire [6:0] tx_room = 7'd16 - txfifo_cnt;
    // tx_ready is REGISTERED (+1 clk stale): the echo's do_push (its
    // hold_valid clear) then reads only registered terms, taking the
    // cnt full-compare out of hold_valid's enable cone.  Stale room is
    // safe: the echo pushes at most one byte per ~417 clks (single
    // hold slot) and is the only writer of the txfifo, so no push can
    // land inside the 1-clk staleness window.  The txfifo_wr_d gate
    // re-checks !full at the registered edge as belt-and-braces.
    reg tx_ready_r;
    always @(posedge clk)
        tx_ready_r <= !txfifo_full;
    assign tx_ready = tx_ready_r;

    // ------------------------------------------------------------------
    // EP0 + serial protocol modules
    // ------------------------------------------------------------------
    wire        ep0_resp_req;
    wire [3:0]  ep0_resp_pid;
    wire [6:0]  ep0_resp_len;
    wire [2:0]  ep0_resp_sel;
    wire [9:0]  ep0_resp_base;
    wire [6:0]  dev_addr;
    wire        configured, ep0_busy, ep0_stalling, ep0_wait_ack;

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
        .ep0_busy(ep0_busy), .ep0_stalling(ep0_stalling),
        .ep0_wait_ack(ep0_wait_ack));

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
    // "last valid token was OUT-to-EP0": qualifies EP0's data-stage
    // STALL.  During EP_STALL the only data packet EP0 may answer is
    // the stalled control transfer's OWN data stage (its token was
    // OUT-to-EP0).  Without this qualifier the router handed EVERY
    // data packet to a stalled EP0, STALLing the host's bulk-OUT
    // writes and killing the echo until the next SETUP (found on
    // hardware: the port enumerated but no byte ever flowed after the
    // connect script's SET_LINE_CODING stalled).  The token precedes
    // its data packet back-to-back, so this register is stable across
    // the data packet; the next token re-evaluates it.
    reg last_out_ep0;
    always @(posedge clk) begin
        if (rst || bus_reset)
            last_out_ep0 <= 1'b0;
        else if (tok_v_r)
            last_out_ep0 <= (tk_pid == PID_OUT) && (tk_endp == 4'd0);
    end
    wire ep0_data_take = ep0_busy || (ep0_stalling && last_out_ep0);
    wire ep0_dt_gate = dt_valid && ep0_data_take;
    reg  ep0_dt_valid_r;
    always @(posedge clk)
        if (rst || bus_reset) ep0_dt_valid_r <= 1'b0;
        else                  ep0_dt_valid_r <= ep0_dt_gate;
    wire ser_dt_valid = dt_valid && ser_busy
                        && !ep0_data_take;

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
    reg [6:0] li;          // write address: counts 0..len-1 while loading
    // Termination is a COUNTDOWN, not an address compare: rem is loaded
    // with the packet length at arm time and decrements once per popped
    // byte; the last byte is being written when rem == 1, i.e.
    // !(|rem[6:1]) — a 6-input OR (1-2 LUTs) into load_run's D.  The old
    // test compared li against a precomputed snap_len_m1 (7-bit
    // equality, 4-LUT chain) landing on BOTH load_run's D and li's clock
    // enable — the vir-flow critical path (21.2 ns).  li's enable is now
    // the plain registered load_run: no compare cone at all.  Byte
    // schedule is identical: one pop per clk, snap[0..len-1] in order.
    reg [6:0] rem;
    wire      snap_pop = load_run;
    // sync-read port (BRAM): address held one byte ahead (see producer)
    reg [7:0] snap_q;
    always @(posedge clk) snap_q <= snap[byte_rd_addr];

    always @(posedge clk) begin
        if (rst || bus_reset) begin
            load_run <= 1'b0;
            li <= 7'd0;
            rem <= 7'd0;
        end else begin
            if (tx_start && ser_resp_req && ser_resp_fresh) begin
                li <= 7'd0;
                rem <= tx_len_w;
                load_run <= (tx_len_w != 7'd0);
            end else if (load_run) begin
                snap[li] <= txfifo_rd;
                li <= li + 7'd1;
                rem <= rem - 7'd1;
                load_run <= (|rem[6:1]);   // rem != 1: more bytes follow
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
    reg       rx_push_run_d;
    // Termination is a COUNTDOWN (push_rem): the old `push_i + 7'd1 >=
    // dt_len_hold` compare rode the 7-bit increment carry chain into
    // push_i's own clock-enable cone -- the vir-flow critical path
    // (22.0 ns).  push_rem counts the bytes still to push; the stop
    // test is a 6-input NOR, and push_i's increment feeds only its own
    // D.  Byte-exact: push_rem==1 in cycle N means cycle N pushes the
    // last byte, so run drops at N's edge exactly where the old
    // compare dropped it (push_i visits one extra index in the
    // trailing cycle; that read is discarded -- run_d is 0 by then).
    reg [6:0] push_rem;
    wire ser_accept = ser_resp_req && (ser_resp_pid == 4'h2); // ACK

    always @(posedge clk) begin
        if (rst || bus_reset) begin
            rx_push_run   <= 1'b0;
            rx_push_run_d <= 1'b0;
            push_i <= 7'd0;
            push_rem <= 7'd0;
        end else begin
            rx_push_run_d <= rx_push_run;
            if (ser_accept) begin
                push_i      <= 7'd0;
                push_rem    <= dt_len;
                rx_push_run <= (dt_len != 7'd0) && !rxfifo_full;
            end else if (rx_push_run) begin
                push_i <= push_i + 7'd1;
                push_rem <= push_rem - 7'd1;
                rx_push_run <= |(push_rem[6:1]);   // push_rem==1: last byte
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

    // dynamic EP0 responses: GET_STATUS (2 zero bytes)
    // tx PHY producer mux: registered sources only
    wire [7:0] tx_byte = (tx_owner == 2'd2) ? snap_q
                      : (ep0_resp_sel == 3'd6) ? 8'h00
                      : desc_q;

    wire [6:0] tx_addr_next_w;
    \usb::TxPhy u_txphy (
        .clk(clk), .rst(rst),
        .tx_start(tx_start), .tx_pid(tx_pid_w), .tx_len(tx_len_w),
        .tx_byte(tx_byte), .tx_addr(), .tx_addr_next(tx_addr_next_w),
        .tx_busy(tx_busy), .tx_done(tx_done),
        .dp(dp_tx), .dn(dn_tx), .output_enable(output_enable));

    // SET_CONFIGURATION completed -> reset bulk data toggles (§9.1.1.5)
    reg configured_d;
    wire cfg_strobe = configured ^ configured_d;
    always @(posedge clk)
        if (rst) configured_d <= 1'b0;
        else     configured_d <= configured;

    // USB activity indicators: the rx/tx byte-stream strobes stretched
    // to ~175 ms (see the port comment).  Self-contained counters --
    // deliberately outside every USB engine path.
    reg [22:0] rx_led_cnt = 23'd0;
    reg [22:0] tx_led_cnt = 23'd0;
    always @(posedge clk) begin
        if (rx_valid)
            rx_led_cnt <= 23'h7F_FFFF;
        else if (rx_led_cnt != 23'd0)
            rx_led_cnt <= rx_led_cnt - 23'd1;
    end
    always @(posedge clk) begin
        if (tx_strobe)
            tx_led_cnt <= 23'h7F_FFFF;
        else if (tx_led_cnt != 23'd0)
            tx_led_cnt <= tx_led_cnt - 23'd1;
    end
    assign rxing = (rx_led_cnt != 23'd0);
    assign txing = (tx_led_cnt != 23'd0);

    assign connected_o = configured;
endmodule
