// Fomu UART-over-USB PoC — pin-level USB host model (BFM) for testbenches.
// Part of ~/projects/fomu-usb (see FOMU_UART_OVER_USB_PLANv2.md §7).
// Hand-written Verilog-2005 for iverilog only (never synthesized).
//
// A TB instantiates this module on the dp/dn nets (the TB also declares
// pullup(usb_dp)/pulldown(usb_dn), modelling the device's D+ pull-up so
// undriven time is idle J) and calls its tasks hierarchically.
// TX: builds tokens/data/handshakes with its own behavioural bit-stuffing,
// NRZI and CRC implementations — independent of the RTL, so core-vs-BFM
// agreement is a real cross-check (the primitives themselves are
// known-answer-verified in tb_crc.v against valentyusb).
// RX: passive, with its own 4x oversampling clock recovery — re-aligns a
// 2-bit phase counter on any dp/dn edge, samples mid-bit — so it decodes
// any transmitter with a sane bit period, not just this core.
//
// Bus states: J=(dp=1,dn=0) idle; K=(dp=0,dn=1); SE0=(0,0).
// Packet shapes (valentyusb utils/packet.py, cross-checked against its
// wrap_packet doctests): sync = NRZI(00000001) from idle = "kjkjkjkk";
// EOP = SE0 for 2 bit times then J for 1 bit time, then release.
// PID byte = {~pid[3:0], pid[3:0]}, every byte LSB first.
// token = sync+PID+addr[6:0]+endp[3:0]+crc5[4:0], all LSB first,
//         crc5 = x^5+x^2+1, seed 0x1F, wire = complemented bit-reverse
//         (= valentyusb crc5_token).
// data   = sync+PID+payload+crc16 (low byte then high byte, LSB first),
//         crc16 = x^16+x^15+x^2+1, seed 0xFFFF.
// handshake = sync+PID+EOP.
// SOF = token-shaped: 11-bit frame number + crc5 over those 11 bits.
// A 0 bit is stuffed after any run of six consecutive raw 1s across the
// whole post-sync stream (PID+payload+CRC); the counter resets per packet
// (the sync pattern's trailing 1 never survives the PID byte, since every
// used PID byte contains 0s).
`timescale 1ns/1ps

module usb_host_bfm(
    input  wire clk,
    inout  wire usb_dp,
    inout  wire usb_dn
);
    // ------------------------------------------------------------------
    // pad drivers: strong while transmitting, z otherwise
    // ------------------------------------------------------------------
    reg        host_oe = 1'b0;
    reg        h_dp = 1'b1, h_dn = 1'b0;
    assign usb_dp = host_oe ? h_dp : 1'bz;
    assign usb_dn = host_oe ? h_dn : 1'bz;

    // ------------------------------------------------------------------
    // public TX payload globals (TB sets before host_send_data)
    // ------------------------------------------------------------------
    reg [7:0] host_data_bytes [0:63];
    reg [6:0] host_data_len = 7'd0;

    // ------------------------------------------------------------------
    // public capture regs (RX side)
    // ------------------------------------------------------------------
    reg         cap_valid = 1'b0;   // 1-clk pulse: packet captured
    reg  [3:0]  cap_pid;
    reg         cap_is_data;         // 1: data packet, 0: handshake
    reg  [6:0]  cap_len;            // payload bytes (CRC excluded)
    reg         cap_crc_ok;
    reg  [7:0]  cap_bytes [0:63];

    reg         host_fail = 1'b0;
    reg [1023:0] host_msg;

    localparam [3:0] PID_OUT=4'h1, PID_IN=4'h9, PID_SOF=4'h5, PID_SETUP=4'hD,
                     PID_DATA0=4'h3, PID_DATA1=4'hB,
                     PID_ACK=4'h2, PID_NAK=4'hA, PID_STALL=4'hE;

    // ------------------------------------------------------------------
    // behavioural CRC functions (independent of the RTL; verified vs
    // valentyusb in tb_crc.v)
    // ------------------------------------------------------------------
    function [4:0] crc5_step;
        input [4:0] r;
        input       b;
        reg inv;
        begin
            // x^5+x^2+1, shift-up: r'[0]=inv, r'[1]=r[0], r'[2]=r[1]^inv,
            // r'[3]=r[2], r'[4]=r[3]  (matches usb_crc5 RTL exactly)
            inv = b ^ r[4];
            crc5_step = {r[3], r[2], r[1] ^ inv, r[0], inv};
        end
    endfunction

    function [4:0] crc5_token;
        input [6:0] addr;
        input [3:0] ep;
        reg [4:0] r;
        integer i;
        begin
            r = 5'h1F;
            for (i = 0; i < 7; i = i + 1)
                r = crc5_step(r, (addr >> i) & 1'b1);
            for (i = 0; i < 4; i = i + 1)
                r = crc5_step(r, (ep >> i) & 1'b1);
            crc5_token = {~r[0], ~r[1], ~r[2], ~r[3], ~r[4]};
        end
    endfunction

    function [15:0] crc16_step;
        input [15:0] r;
        input        b;
        reg inv;
        begin
            inv = b ^ r[15];
            crc16_step = {r[14] ^ inv, r[13:2], r[1] ^ inv, r[0], inv};
        end
    endfunction

    reg [15:0] crc16_state;   // running crc16 during TX/RX data builds


    // ------------------------------------------------------------------
    // low-level driving: one bit time = 4 clks
    // ------------------------------------------------------------------
    reg tx_lvl = 1'b0;   // NRZI drive level: 0 = J, 1 = K

    task drive_sym_4(input a, input b);
        integer i;
    begin
        h_dp = a; h_dn = b; host_oe = 1'b1;
        for (i = 0; i < 4; i = i + 1) @(posedge clk);
    end
    endtask

    task nrzi_send(input raw_bit);
    begin
        if (raw_bit == 1'b0) tx_lvl = ~tx_lvl;   // 0: toggle
        if (tx_lvl) drive_sym_4(1'b0, 1'b1);     // K
        else        drive_sym_4(1'b1, 1'b0);     // J
    end
    endtask

    reg [2:0] stuff_cnt = 3'd0;   // consecutive raw 1s (0..6)

    task stuffed_send(input raw_bit);
    begin
        if (stuff_cnt == 3'd6) begin
            nrzi_send(1'b0);       // insert the stuffed 0 after six 1s
            stuff_cnt = 3'd0;
        end
        nrzi_send(raw_bit);
        if (raw_bit) stuff_cnt = stuff_cnt + 3'd1;
        else         stuff_cnt = 3'd0;
    end
    endtask

    // wait for the bus to settle at idle J (device released its EOP
    // tail): reacting to a capture at mid-SE0 and driving immediately
    // would collide with the device's EOP J bit time
    task wait_idle;
        integer n;
        integer guard;
    begin
        n = 0;
        guard = 0;
        while (n < 8 && guard < 4096) begin
            @(posedge clk);
            guard = guard + 1;
            if ((usb_dp === 1'b1) && (usb_dn === 1'b0)) n = n + 1;
            else                                          n = 0;
        end
        if (guard >= 4096) begin
            host_fail = 1'b1;
            $display("BFM: wait_idle TIMEOUT (bus never settled at J)");
        end
    end
    endtask

    task send_sync;
        integer i;
    begin
        // NRZI of 00000001 from idle J => k j k j k j k k
        for (i = 0; i < 7; i = i + 1) nrzi_send(1'b0);
        nrzi_send(1'b1);
        stuff_cnt = 3'd0;          // run counter fresh at the PID
    end
    endtask

    task send_pid(input [3:0] pid);
        reg [7:0] b;
        integer i;
    begin
        b = {~pid[3:0], pid};
        for (i = 0; i < 8; i = i + 1)
            stuffed_send(b[i]);
    end
    endtask

    task send_eop;
    begin
        drive_sym_4(1'b0, 1'b0);   // SE0 bit time 1
        drive_sym_4(1'b0, 1'b0);   // SE0 bit time 2
        drive_sym_4(1'b1, 1'b0);   // J bit time
        host_oe = 1'b0;            // release (pull-up holds J)
        tx_lvl = 1'b0;
    end
    endtask

    // ------------------------------------------------------------------
    // public TX tasks
    // ------------------------------------------------------------------
    task host_idle(input integer cycles);
        integer i;
    begin
        host_oe = 1'b0;
        for (i = 0; i < cycles; i = i + 1) @(posedge clk);
    end
    endtask

    // bus reset: SE0 for 10 ms of sim time (480k clks at 48 MHz)
    task host_reset;
        integer i;
    begin
        wait_idle;
        h_dp = 1'b0; h_dn = 1'b0; host_oe = 1'b1;
        for (i = 0; i < 480000; i = i + 1) @(posedge clk);
        host_oe = 1'b0;
        for (i = 0; i < 64; i = i + 1) @(posedge clk);
    end
    endtask

    task host_send_token(input [3:0] pid, input [6:0] addr,
                         input [3:0] endp);
        reg [4:0] c5;
        reg [10:0] bits;
        integer i;
    begin
        wait_idle;
        send_sync;
        send_pid(pid);
        c5 = crc5_token(addr, endp);
        bits = {endp, addr};        // addr bits 0..6, ep bits 7..10
        for (i = 0; i < 11; i = i + 1)
            stuffed_send(bits[i]);
        for (i = 0; i < 5; i = i + 1)
            stuffed_send(c5[i]);
        send_eop;
    end
    endtask

    task host_sof(input [10:0] frame);
        reg [4:0] c5;
        reg [15:0] r;
        integer i;
    begin
        wait_idle;
        send_sync;
        send_pid(PID_SOF);
        r = 5'h1F;
        for (i = 0; i < 11; i = i + 1)
            r = crc5_step(r, (frame >> i) & 1'b1);
        c5 = {~r[0], ~r[1], ~r[2], ~r[3], ~r[4]};
        for (i = 0; i < 11; i = i + 1)
            stuffed_send((frame >> i) & 1'b1);
        for (i = 0; i < 5; i = i + 1)
            stuffed_send(c5[i]);
        send_eop;
    end
    endtask

    task host_send_data(input [3:0] pid);
        integer i, j;
        reg [7:0] b;
    begin
        wait_idle;
        send_sync;
        send_pid(pid);
        crc16_state = 16'hFFFF;
        for (i = 0; i < host_data_len; i = i + 1) begin
            b = host_data_bytes[i];
            for (j = 0; j < 8; j = j + 1) begin
                stuffed_send((b >> j) & 1'b1);
                crc16_state = crc16_step(crc16_state, (b >> j) & 1'b1);
            end
        end
        b = {~crc16_state[8],  ~crc16_state[9],
             ~crc16_state[10], ~crc16_state[11],
             ~crc16_state[12], ~crc16_state[13],
             ~crc16_state[14], ~crc16_state[15]};   // wire lo byte
        for (j = 0; j < 8; j = j + 1)
            stuffed_send((b >> j) & 1'b1);
        b = {~crc16_state[0],  ~crc16_state[1],
             ~crc16_state[2],  ~crc16_state[3],
             ~crc16_state[4],  ~crc16_state[5],
             ~crc16_state[6],  ~crc16_state[7]};    // wire hi byte
        for (j = 0; j < 8; j = j + 1)
            stuffed_send((b >> j) & 1'b1);
        send_eop;
    end
    endtask

    task host_send_data0; begin host_send_data(PID_DATA0); end endtask
    task host_send_data1; begin host_send_data(PID_DATA1); end endtask

    // like host_send_data, but inverts the final CRC wire bit: the packet
    // must arrive with a bad CRC (negative-test stimulus)
    task host_send_data_corrupt(input [3:0] pid);
        integer i, j;
        reg [7:0] b;
        reg [7:0] lo, hi;
    begin
        wait_idle;
        send_sync;
        send_pid(pid);
        crc16_state = 16'hFFFF;
        for (i = 0; i < host_data_len; i = i + 1) begin
            b = host_data_bytes[i];
            for (j = 0; j < 8; j = j + 1) begin
                stuffed_send((b >> j) & 1'b1);
                crc16_state = crc16_step(crc16_state, (b >> j) & 1'b1);
            end
        end
        lo = {~crc16_state[8],  ~crc16_state[9],
              ~crc16_state[10], ~crc16_state[11],
              ~crc16_state[12], ~crc16_state[13],
              ~crc16_state[14], ~crc16_state[15]};
        hi = {~crc16_state[0],  ~crc16_state[1],
              ~crc16_state[2],  ~crc16_state[3],
              ~crc16_state[4],  ~crc16_state[5],
              ~crc16_state[6],  ~crc16_state[7]} ^ 8'h01; // corrupt
        for (j = 0; j < 8; j = j + 1)
            stuffed_send((lo >> j) & 1'b1);
        for (j = 0; j < 8; j = j + 1)
            stuffed_send((hi >> j) & 1'b1);
        send_eop;
    end
    endtask

    task host_send_handshake(input [3:0] pid);
    begin
        wait_idle;
        send_sync;
        send_pid(pid);
        send_eop;
    end
    endtask

    task host_send_ack;   begin host_send_handshake(PID_ACK);   end endtask
    task host_send_nak;   begin host_send_handshake(PID_NAK);   end endtask
    task host_send_stall; begin host_send_handshake(PID_STALL); end endtask

    // ------------------------------------------------------------------
    // RX: passive clock-recovering decoder (runs while host_oe == 0).
    // Decoded bit n (from SOP): n=0..7 sync, n=8..15 PID, n>=16 payload
    // + crc16 wire bits.  CRC16 residual check: 0x800D over everything
    // post-PID; cap_len excludes the 2 CRC bytes.
    // ------------------------------------------------------------------
    reg [1:0]  rxs = 2'b10, rxs_prev = 2'b10;
    reg [1:0]  rx_phase = 2'd0;
    reg [1:0]  rx_lev = 2'b10;      // previous bit sample (NRZI)
    reg        rx_act = 1'b0;
    reg [2:0]  rx_ones = 3'd0;
    reg        rx_drop = 1'b0;
    reg [7:0]  rx_pid = 8'd0;
    reg [9:0]  rx_n = 10'd0;        // decoded bit counter
    reg [7:0]  rx_byte = 8'd0;      // byte assembly (first bit -> LSB)
    reg [6:0]  rx_bycnt = 7'd0;
    reg [15:0] rx_crc = 16'hFFFF;
    reg [2:0]  rx_se0 = 3'd0;
    reg        rx_bad = 1'b0;

    wire decoded = (rxs === rx_lev);   // same level -> 1 (NRZI)

    always @(posedge clk) begin
        rxs_prev <= rxs;
        rxs      <= {usb_dp, usb_dn};
        cap_valid <= 1'b0;

        if (host_oe) begin
            // transmitting: receiver off, nets are ours
            rx_act <= 1'b0;
            rx_se0 <= 3'd0;
        end else begin
            // SE0 timing (EOP)
            if (rxs === 2'b00) rx_se0 <= rx_se0 + 3'd1;
            else               rx_se0 <= 3'd0;

            if (!rx_act) begin
                // idle: wait for the J->K sync edge
                rx_ones <= 3'd0;
                rx_drop <= 1'b0;
                rx_bad  <= 1'b0;
                rx_n    <= 10'd0;
                rx_bycnt <= 7'd0;
                rx_pid  <= 8'd0;
                rx_byte <= 8'd0;
                if ((rxs_prev === 2'b10) && (rxs === 2'b01)) begin
                    rx_act   <= 1'b1;
                    rx_phase <= 2'd0;
                    rx_lev   <= 2'b10;    // previous bit was idle J
                end
            end else if (rxs === 2'b00) begin
                // SE0: EOP after >= 2 bit times (fire once, at 7 clks)
                if (rx_se0 == 3'd7) begin
                    rx_act <= 1'b0;
                    if (!rx_bad && (rx_pid[3:0] == ~rx_pid[7:4])) begin
                        cap_valid <= 1'b1;
                        cap_pid   <= rx_pid[3:0];
                        if (rx_pid[1:0] == 2'b11) begin
                            cap_is_data <= 1'b1;
                            cap_len     <= (rx_bycnt >= 7'd2)
                                          ? (rx_bycnt - 7'd2) : 7'd0;
                            cap_crc_ok  <= (rx_crc == 16'h800D);
                        end else begin
                            cap_is_data <= 1'b0;
                            cap_len     <= 7'd0;
                            cap_crc_ok  <= 1'b1;
                        end
                    end
                end
            end else begin
                // J or K level: clock recovery
                if (rxs !== rxs_prev) rx_phase <= 2'd0;
                else                  rx_phase <= rx_phase + 2'd1;
                if (rx_phase == 2'd2) begin
                    // mid-bit sample: NRZI decode + unstuff + capture
                    rx_lev <= rxs;
                    if (rx_drop) begin
                        // this decoded bit is the stuffed 0: drop it
                        rx_drop <= 1'b0;
                        rx_ones <= 3'd0;
                        if (decoded) rx_bad <= 1'b1;  // 6 ones then a 1
                    end else begin
                        rx_n <= rx_n + 10'd1;
                        if (decoded) begin
                            rx_ones <= rx_ones + 3'd1;
                            if (rx_ones == 3'd5)
                                rx_drop <= 1'b1;      // after six 1s
                        end else
                            rx_ones <= 3'd0;
                        // capture
                        if (rx_n < 10'd8) begin
                            // sync bit: nothing (counter fresh at PID)
                        end else if (rx_n < 10'd16) begin
                            rx_pid <= {decoded, rx_pid[7:1]};
                        end else begin
                            // post-PID bit: crc16 + byte assembly
                            rx_crc  <= crc16_step(rx_crc, decoded);
                            rx_byte <= {decoded, rx_byte[7:1]};
                            if (rx_n[2:0] == 3'd7) begin
                                cap_bytes[rx_bycnt]
                                    <= {decoded, rx_byte[7:1]};
                                rx_bycnt <= rx_bycnt + 7'd1;
                                // payload(64) + crc16(2) = 66 legal bytes;
                                // a 67th byte is an overflow
                                if (rx_bycnt == 7'd66)
                                    rx_bad <= 1'b1;
                            end
                        end
                        if (rx_n == 10'd15)
                            rx_crc <= 16'hFFFF;  // seed after the PID
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // expect tasks: wait for a captured packet with a timeout
    // ------------------------------------------------------------------
    task host_expect_handshake(input [3:0] pid, input integer timeout);
        integer t;
    begin
        for (t = 0; t < timeout; t = t + 1) begin
            @(posedge clk);
            if (cap_valid) begin
                if (cap_is_data) begin
                    host_fail = 1'b1;
                    host_msg = "expected handshake, got data";
                    $display("BFM: expected handshake pid=%h, got data", pid);
                end else if (cap_pid !== pid) begin
                    host_fail = 1'b1;
                    $display("BFM: expected handshake pid=%h, got %h",
                             pid, cap_pid);
                end else begin
                    $display("BFM: handshake %h ok", pid);
                end
                disable host_expect_handshake;
            end
        end
        host_fail = 1'b1;
        host_msg = "timeout waiting for handshake";
        $display("BFM: timeout waiting for handshake %h", pid);
    end
    endtask

    task host_expect_data(input [3:0] pid, input integer timeout);
        integer t;
    begin
        for (t = 0; t < timeout; t = t + 1) begin
            @(posedge clk);
            if (cap_valid) begin
                if (!cap_is_data) begin
                    host_fail = 1'b1;
                    $display("BFM: expected data pid=%h, got handshake %h",
                             pid, cap_pid);
                end else if (cap_pid !== pid) begin
                    host_fail = 1'b1;
                    $display("BFM: expected data pid=%h, got %h",
                             pid, cap_pid);
                end else if (!cap_crc_ok) begin
                    host_fail = 1'b1;
                    $display("BFM: data pid=%h CRC FAILED", pid);
                end else begin
                    $display("BFM: data %h len=%0d crc ok", pid, cap_len);
                end
                disable host_expect_data;
            end
        end
        host_fail = 1'b1;
        host_msg = "timeout waiting for data";
        $display("BFM: timeout waiting for data pid=%h", pid);
    end
    endtask

endmodule