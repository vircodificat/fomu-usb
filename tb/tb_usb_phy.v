// Fomu UART-over-USB PoC — PHY-level testbench.
// Part of ~/projects/fomu-usb (see FOMU_UART_OVER_USB_PLANv2.md §7).
// Hand-written Verilog-2005 for iverilog.
//
// Wires the BFM's pad driver straight into \phy::RxPhy + \usb::RxPacket,
// and loops the core's usb_tx_phy output back onto the same nets (via a
// tri-state driver, like the SB_IO would): BFM-sent packets must decode
// with correct PID/addr/EP/bytes/CRC, and core-sent packets must be
// captured byte-identically by the BFM's independent receiver (and by
// the core's own RX chain).  Covers stuffing edge cases (64 x 0xFF),
// max-length (64 B) packets, ZLPs, handshakes, SOF and a bad-CRC packet.
`timescale 1ns/1ps

module tb_usb_phy;
    reg clk = 0;
    always #10.416 clk = ~clk;      // 48 MHz
    reg rst = 1'b1;

    wire usb_dp, usb_dn;
    pullup(usb_dp);                  // D+ pull-up: idle J when undriven
    pulldown(usb_dn);

    integer errors = 0;

    // ------------------------------------------------------------------
    // core RX chain
    // ------------------------------------------------------------------
    wire rx_bit, rx_bitv, rx_sop, rx_eop, rx_act, rx_perr, rx_breset;
    \phy::RxPhy rxphy(
        .clock(clk), .reset(rst),
        .dp(usb_dp), .dn(usb_dn),
        .bit_o(rx_bit), .bit_valid(rx_bitv),
        .sop(rx_sop), .eop(rx_eop), .active(rx_act),
        .error(rx_perr), .bus_reset(rx_breset));

    wire [28:0] tk;           // pulsed Valid[TokenRec] (packed)
    wire tk_valid   = (tk[1:0] == 2'b10);     // @Valid tag
    wire [3:0] tk_pid    = tk[5:2];
    wire [6:0] tk_addr   = tk[13:7];
    wire [3:0] tk_endp   = tk[17:14];
    wire [10:0] tk_frame = tk[28:18];
    wire       tk_crc_ok = tk[6];
    wire [13:0] dt;           // pulsed Valid[DataRec] (packed)
    wire dt_valid   = (dt[1:0] == 2'b10);     // @Valid tag
    wire [3:0] dt_pid    = dt[5:2];
    wire [6:0] dt_len    = dt[12:6];
    wire       dt_crc_ok = dt[13];
    wire hs_valid;
    wire [3:0] hs_pid;
    // rx packet payload capture (off-fabric interface): flopped head
    // window + payload RAM read port; the TB drives the read address
    // (reg) and streams the RAM out into s_dt_data for the payload
    // checks (snap_dt task)
    wire [63:0] dt_head;
    wire [7:0]  dt_rd_data;
    reg  [5:0]  dt_rd_addr = 6'd0;
    \usb::RxPacket rxpkt(
        .clock(clk), .reset(rst), .bus_reset(rx_breset),
        .bit_i(rx_bit), .bit_valid(rx_bitv),
        .sop(rx_sop), .eop(rx_eop), .pkt_err(rx_perr),
        .tk(tk),
        .dt(dt),
        .dt_head(dt_head), .dt_rd_data(dt_rd_data),
        .dt_rd_addr(dt_rd_addr),
        .hs_valid(hs_valid), .hs_pid(hs_pid));

    // ------------------------------------------------------------------
    // core TX, tri-stated onto the same nets (as SB_IO would)
    // ------------------------------------------------------------------
    reg tx_start = 1'b0;
    reg [3:0] tx_pid = 4'h0;
    reg [6:0] tx_len = 7'd0;
    reg [7:0] tx_bytes [0:63];
    wire [6:0] tx_addr;
    wire [7:0] tx_byte = tx_bytes[tx_addr];
    wire tx_busy, tx_done, dut_dp, dut_dn, dut_oe;
    // The arm record travels as pulsed Valid[TxRec], packed like the
    // tk/dt records above: tag[1:0] (@Valid = 2'b10), pid[5:2],
    // len[12:6].  @Valid is a 1-clock pulse (same contract as tk/dt):
    // held high, the PHY would re-arm a packet every time its FSM
    // returns to Idle.
    wire [12:0] tx_arm = tx_start ? {tx_len, tx_pid, 2'b10} : 13'd1;
    \phy::TxPhy txphy(
        .clock(clk), .reset(rst),
        .tx_arm(tx_arm),
        .tx_byte(tx_byte), .tx_addr(tx_addr),
        .tx_busy(tx_busy), .tx_done(tx_done),
        .dp(dut_dp), .dn(dut_dn), .output_enable(dut_oe));
    assign usb_dp = dut_oe ? dut_dp : 1'bz;
    assign usb_dn = dut_oe ? dut_dn : 1'bz;

    usb_host_bfm host(.clk(clk), .usb_dp(usb_dp), .usb_dn(usb_dn));

    // ------------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------------
    task check(input cond, input [1023:0] msg);
    begin
        if (cond !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL: %0s", msg);
        end
    end
    endtask

    // sticky capture of 1-clk pulses: pulses may land inside any wait
    // window, so every pulse is latched into a flag + shadow fields
    reg saw_tk, saw_dt, saw_cap;
    reg [3:0]  s_tk_pid, s_dt_pid;
    reg [6:0]  s_tk_addr, s_dt_len;
    reg [3:0]  s_tk_endp;
    reg [10:0] s_tk_frame;
    reg        s_tk_crc, s_dt_crc;
    reg [511:0] s_dt_data;      // streamed from the payload RAM (snap_dt)
    reg [63:0]  s_dt_head;      // flopped head window, shadowed at dt_valid
    reg [3:0]  s_cap_pid;
    reg        s_cap_is_data, s_cap_crc;
    reg [6:0]  s_cap_len;
    reg [7:0]  s_cap_bytes [0:63];

    always @(posedge clk) if (tk_valid) begin
        saw_tk <= 1'b1;
        s_tk_pid   <= tk_pid;
        s_tk_addr  <= tk_addr;
        s_tk_endp  <= tk_endp;
        s_tk_frame <= tk_frame;
        s_tk_crc   <= tk_crc_ok;
    end
    always @(posedge clk) if (dt_valid) begin
        saw_dt <= 1'b1;
        s_dt_pid  <= dt_pid;
        s_dt_len  <= dt_len;
        s_dt_crc  <= dt_crc_ok;
        s_dt_head <= dt_head;
    end
    integer cb;
    always @(posedge clk) if (host.cap_valid) begin
        saw_cap <= 1'b1;
        s_cap_pid      <= host.cap_pid;
        s_cap_is_data  <= host.cap_is_data;
        s_cap_crc      <= host.cap_crc_ok;
        s_cap_len      <= host.cap_len;
        for (cb = 0; cb < 64; cb = cb + 1)
            s_cap_bytes[cb] <= host.cap_bytes[cb];
    end

    task clear_flags;
    begin
        @(posedge clk); #1;
        saw_tk = 1'b0; saw_dt = 1'b0; saw_cap = 1'b0;
        got_dt_crc_ok = 1'b0;
    end
    endtask

    task wait_tk(input integer timeout, output ok);
        integer t;
    begin
        ok = 0;
        for (t = 0; t < timeout; t = t + 1) begin
            @(posedge clk); #1;
            if (saw_tk === 1'b1) begin ok = 1; disable wait_tk; end
        end
    end
    endtask
    task wait_dt(input integer timeout, output ok);
        integer t;
    begin
        ok = 0;
        for (t = 0; t < timeout; t = t + 1) begin
            @(posedge clk); #1;
            if (saw_dt === 1'b1) begin ok = 1; disable wait_dt; end
        end
    end
    endtask
    task wait_cap(input integer timeout, output ok);
        integer t;
    begin
        ok = 0;
        for (t = 0; t < timeout; t = t + 1) begin
            @(posedge clk); #1;
            if (saw_cap === 1'b1) begin ok = 1; disable wait_cap; end
        end
    end
    endtask

    // stream the rx packet payload RAM out into s_dt_data (the payload
    // capture is off-fabric now): present dt_rd_addr = i, sample the
    // registered read dt_rd_data one clk later
    task snap_dt;
        integer i;
    begin
        for (i = 0; i < 64; i = i + 1) begin
            @(posedge clk); #1;
            dt_rd_addr = i;
            @(posedge clk); #1;
            s_dt_data[8*i +: 8] = dt_rd_data;
        end
    end
    endtask

    reg ok;
    reg got_dt_crc_ok;

    // track any dt_valid with crc_ok (for the negative test)
    always @(posedge clk)
        if (dt_valid && dt_crc_ok) got_dt_crc_ok <= 1'b1;

    // fill tx_bytes with a payload pattern
    task fill_tx(input integer n, input [7:0] base);
        integer i;
    begin
        for (i = 0; i < 64; i = i + 1)
            if (i < n) tx_bytes[i] = base + i;
            else       tx_bytes[i] = 8'h00;
    end
    endtask

    integer i;

    initial begin
        got_dt_crc_ok = 1'b0;
        host.host_fail = 1'b0;
        repeat (8) @(posedge clk);
        rst <= 1'b0;
        host.host_idle(64);

        // ---------------------------------------------------------------
        clear_flags;
        $display("--- T1: token decode (SETUP addr=3 ep=0)");
        host.host_send_token(4'hD, 7'd3, 4'd0);
        wait_tk(4096, ok);
        check(ok, "T1: no tk_valid for SETUP token");
        if (ok) begin
            check(s_tk_pid === 4'hD, "T1: tk_pid");
            check(s_tk_addr === 7'd3, "T1: tk_addr");
            check(s_tk_endp === 4'd0, "T1: tk_endp");
            check(s_tk_crc === 1'b1, "T1: tk_crc_ok");
        end
        host.host_idle(32);

        // ---------------------------------------------------------------
        clear_flags;
        $display("--- T2: SOF decode (frame=1234)");
        host.host_sof(11'd1234);
        wait_tk(4096, ok);
        check(ok, "T2: no tk_valid for SOF");
        if (ok) begin
            check(s_tk_pid === 4'h5, "T2: SOF pid");
            check(s_tk_frame === 11'd1234, "T2: SOF frame");
            check(s_tk_crc === 1'b1, "T2: SOF crc");
        end
        host.host_idle(32);

        // ---------------------------------------------------------------
        clear_flags;
        $display("--- T3: data packet decode (8-byte request shape)");
        host.host_data_bytes[0] = 8'h80; host.host_data_bytes[1] = 8'h06;
        host.host_data_bytes[2] = 8'h00; host.host_data_bytes[3] = 8'h01;
        host.host_data_bytes[4] = 8'h00; host.host_data_bytes[5] = 8'h00;
        host.host_data_bytes[6] = 8'h40; host.host_data_bytes[7] = 8'h00;
        host.host_data_len = 7'd8;
        host.host_send_data0;
        wait_dt(4096, ok);
        check(ok, "T3: no dt_valid for DATA0");
        if (ok) begin
            check(s_dt_pid === 4'h3, "T3: dt_pid DATA0");
            check(s_dt_len === 7'd8, "T3: dt_len");
            check(s_dt_crc === 1'b1, "T3: dt_crc_ok");
            // head window (flopped bytes 0..7 — what endpoint::Ep0 latches)
            for (i = 0; i < 8; i = i + 1)
                check(s_dt_head[8*i +: 8] === host.host_data_bytes[i],
                      "T3: payload head byte");
            // payload RAM (streamed out via the read port)
            snap_dt;
            for (i = 0; i < 8; i = i + 1)
                check(s_dt_data[8*i +: 8] === host.host_data_bytes[i],
                      "T3: payload byte");
        end
        host.host_idle(32);

        // ---------------------------------------------------------------
        clear_flags;
        $display("--- T4: bad-CRC data packet must not pass");
        got_dt_crc_ok = 1'b0;
        host.host_data_len = 7'd8;
        host.host_send_data_corrupt(4'h3);
        host.host_idle(256);
        check(got_dt_crc_ok === 1'b0, "T4: bad-CRC packet decoded as good");
        host.host_idle(32);

        // ---------------------------------------------------------------
        clear_flags;
        $display("--- T5: tx round-trip: DATA1, 16 B");
        fill_tx(16, 8'h41);
        tx_pid <= 4'hB; tx_len <= 7'd16;
        @(posedge clk); tx_start <= 1'b1;
        @(posedge clk); tx_start <= 1'b0;
        wait_cap(8192, ok);
        check(ok, "T5: BFM captured nothing");
        if (ok) begin
            check(s_cap_is_data === 1'b1, "T5: cap_is_data");
            check(s_cap_pid === 4'hB, "T5: cap_pid DATA1");
            check(s_cap_len === 7'd16, "T5: cap_len");
            check(s_cap_crc === 1'b1, "T5: cap_crc_ok");
            for (i = 0; i < 16; i = i + 1)
                check(s_cap_bytes[i] === (8'h41 + i),
                      "T5: cap byte value");
        end
        // the core's own RX must have decoded it identically
        wait_dt(256, ok);
        check(ok, "T5: core RX saw its own TX");
        if (ok) begin
            check(s_dt_len === 7'd16, "T5: core RX len");
            check(s_dt_crc === 1'b1, "T5: core RX crc");
            check(s_dt_pid === 4'hB, "T5: core RX pid");
            snap_dt;
            for (i = 0; i < 16; i = i + 1)
                check(s_dt_data[8*i +: 8] === (8'h41 + i),
                      "T5: core RX byte");
        end
        host.host_idle(64);

        // ---------------------------------------------------------------
        clear_flags;
        $display("--- T6: tx zero-length data packet (DATA0 ZLP)");
        tx_pid <= 4'h3; tx_len <= 7'd0;
        @(posedge clk); tx_start <= 1'b1;
        @(posedge clk); tx_start <= 1'b0;
        wait_cap(8192, ok);
        check(ok, "T6: BFM captured nothing");
        if (ok) begin
            check(s_cap_is_data === 1'b1, "T6: cap_is_data");
            check(s_cap_pid === 4'h3, "T6: cap_pid DATA0");
            check(s_cap_len === 7'd0, "T6: cap_len 0");
            check(s_cap_crc === 1'b1, "T6: cap_crc_ok (empty crc)");
        end
        host.host_idle(64);

        // ---------------------------------------------------------------
        clear_flags;
        $display("--- T7: tx handshake (STALL)");
        tx_pid <= 4'hE; tx_len <= 7'd0;
        @(posedge clk); tx_start <= 1'b1;
        @(posedge clk); tx_start <= 1'b0;
        wait_cap(8192, ok);
        check(ok, "T7: BFM captured nothing");
        if (ok) begin
            check(host.cap_is_data === 1'b0, "T7: cap_is_handshake");
            check(s_cap_pid === 4'hE, "T7: cap_pid STALL");
        end
        host.host_idle(64);

        // ---------------------------------------------------------------
        clear_flags;
        $display("--- T8: tx stuffing edge: 64 x 0xFF (512 consecutive 1s)");
        for (i = 0; i < 64; i = i + 1) tx_bytes[i] = 8'hFF;
        tx_pid <= 4'h3; tx_len <= 7'd64;
        @(posedge clk); tx_start <= 1'b1;
        @(posedge clk); tx_start <= 1'b0;
        wait_cap(16384, ok);
        check(ok, "T8: BFM captured nothing");
        if (ok) begin
            check(s_cap_len === 7'd64, "T8: cap_len 64");
            check(s_cap_crc === 1'b1, "T8: cap_crc_ok");
            for (i = 0; i < 64; i = i + 1)
                check(s_cap_bytes[i] === 8'hFF, "T8: 0xFF byte");
        end
        host.host_idle(64);

        // ---------------------------------------------------------------
        clear_flags;
        $display("--- T9: tx max-length mixed pattern (64 B)");
        fill_tx(64, 8'h00);
        tx_bytes[0] = 8'h00; tx_bytes[1] = 8'h01; tx_bytes[2] = 8'h02;
        tx_bytes[3] = 8'h03; tx_bytes[4] = 8'h7F; tx_bytes[5] = 8'h80;
        tx_bytes[6] = 8'hAA; tx_bytes[7] = 8'h55;
        tx_bytes[8] = 8'hFF; tx_bytes[9] = 8'h00;
        tx_pid <= 4'hB; tx_len <= 7'd64;
        @(posedge clk); tx_start <= 1'b1;
        @(posedge clk); tx_start <= 1'b0;
        wait_cap(16384, ok);
        check(ok, "T9: BFM captured nothing");
        if (ok) begin
            check(s_cap_crc === 1'b1, "T9: cap_crc_ok");
            check(s_cap_len === 7'd64, "T9: cap_len 64");
            check(s_cap_bytes[3] === 8'h03, "T9: byte 3");
            check(s_cap_bytes[9] === 8'h00, "T9: byte 9");
            for (i = 10; i < 64; i = i + 1)
                check(s_cap_bytes[i] === (8'h00 + i), "T9: pattern byte");
        end
        host.host_idle(64);

        // ---------------------------------------------------------------
        // summary
        // ---------------------------------------------------------------
        if (errors == 0 && host.host_fail == 1'b0)
            $display("TB_USB_PHY: ALL TESTS PASSED");
        else
            $display("TB_USB_PHY: %0d FAILURES (bfm_fail=%0d)",
                      errors, host.host_fail);
        $finish;
    end
endmodule
