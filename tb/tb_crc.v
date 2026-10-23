// Fomu UART-over-USB PoC — known-answer TB for usb_crc5 / usb_crc16.
// Part of ~/projects/fomu-usb (see FOMU_UART_OVER_USB_PLANv2.md §7).
// Hand-written Verilog-2005.  Vectors cross-checked against valentyusb
// (BSD-3-Clause, ~/repos/valentyusb):
//   - crc5_token(addr,ep) values from usbcore/utils/packet.py doctests;
//   - crc16(payload) values computed with valentyusb's CrcMoose3
//     (CRC-16/USB: poly 0x8005, init 0xFFFF, refin/refout true, xorout
//     0xFFFF, low byte first);
//   - checker residuals: token stream (addr+ep+crc5 wire bits, 16 bits,
//     NO PID) -> 5'h0C, matching valentyusb rx/crc_test.py residual
//     0b01100; data stream (payload+crc16 wire bits) -> 16'h800D,
//     matching valentyusb rx/crc_test.py data vectors.
`timescale 1ns/1ps

module tb_crc;
    reg clk = 0;
    always #10.416 clk = ~clk;    // 48 MHz

    reg        rst;
    reg        adv;
    reg        din;
    wire [4:0]  crc5;
    wire [15:0] crc16;

    usb_crc5 u_crc5(.clk(clk), .rst(rst), .adv(adv), .din(din), .crc(crc5));
    usb_crc16 u_crc16(.clk(clk), .rst(rst), .adv(adv), .din(din), .crc(crc16));

    integer errors = 0;
    reg [7:0] tmp [0:63];
    integer k;

    // drive one serial bit (wire order) and return the LFSR output
    task send_bit(input b);
    begin
        @(negedge clk);
        adv <= 1'b1;
        din <= b;
        @(negedge clk);
        adv <= 1'b0;
    end
    endtask

    task seed;
    begin
        @(negedge clk);
        rst <= 1'b1;
        adv <= 1'b0;
        din <= 1'b0;
        @(negedge clk);
        rst <= 1'b0;
    end
    endtask

    // LSB-first bit feed of the global `tmp` array (n bytes)
    task send_bytes(input integer n);
        integer i, j;
    begin
        for (i = 0; i < n; i = i + 1)
            for (j = 0; j < 8; j = j + 1)
                send_bit((tmp[i] >> j) & 1'b1);
    end
    endtask

    // ------------------------------------------------------------------
    // CRC5 token generator: feed addr(7)+ep(4) LSB-first; wire bits
    // crc_wire[i] = ~crc5[4-i] must equal valentyusb's crc5_token value.
    // ------------------------------------------------------------------
    task check_crc5_token(input [6:0] addr, input [3:0] ep,
                          input [4:0] expect_crc);
        integer i;
    begin
        seed;
        for (i = 0; i < 7; i = i + 1)
            send_bit((addr >> i) & 1'b1);
        for (i = 0; i < 4; i = i + 1)
            send_bit((ep >> i) & 1'b1);
        // wire emission: complement + bit-reverse
        if ({~crc5[0], ~crc5[1], ~crc5[2], ~crc5[3], ~crc5[4]} !== expect_crc)
        begin
            errors = errors + 1;
            $display("FAIL crc5_token addr=%0d ep=%0d: got %h expect %h",
                      addr, ep,
                      {~crc5[0], ~crc5[1], ~crc5[2], ~crc5[3], ~crc5[4]},
                      expect_crc);
        end
        else
            $display("PASS crc5_token addr=%0d ep=%0d -> %h",
                      addr, ep, expect_crc);
    end
    endtask

    // ------------------------------------------------------------------
    // CRC5 checker residual: feed addr+ep+the 5 wire CRC bits (16 bits,
    // no PID) -> final state must be 5'h0C (valentyusb rx convention).
    // ------------------------------------------------------------------
    task check_crc5_residual(input [6:0] addr, input [3:0] ep);
        integer i;
        reg [4:0] wire_crc;
    begin
        seed;
        for (i = 0; i < 7; i = i + 1)
            send_bit((addr >> i) & 1'b1);
        for (i = 0; i < 4; i = i + 1)
            send_bit((ep >> i) & 1'b1);
        // snapshot the wire bits from the generator state after addr+ep
        for (i = 0; i < 5; i = i + 1)
            wire_crc[i] = ~crc5[4 - i];
        for (i = 0; i < 5; i = i + 1)
            send_bit(wire_crc[i]);
        if (crc5 !== 5'h0C)
        begin
            errors = errors + 1;
            $display("FAIL crc5 residual addr=%0d ep=%0d: got %h expect 0c",
                      addr, ep, crc5);
        end
        else
            $display("PASS crc5 residual addr=%0d ep=%0d -> 0c", addr, ep);
    end
    endtask

    // ------------------------------------------------------------------
    // CRC16 generator: feed payload bytes LSB-first; wire bytes must equal
    // valentyusb crc16(payload) = [low, high], each LSB-first.
    // ------------------------------------------------------------------
    task check_crc16(input integer n, input [7:0] exp_lo, input [7:0] exp_hi);
    begin
        seed;
        send_bytes(n);
        // wire byte low = bits 0..7 = ~crc16[15..8], MSB first in the
        // concatenation: {bit7..bit0} = {~crc16[8], ..., ~crc16[15]}
        if ({~crc16[8],  ~crc16[9],  ~crc16[10], ~crc16[11],
             ~crc16[12], ~crc16[13], ~crc16[14], ~crc16[15]} !== exp_lo
            ||
            {~crc16[0],  ~crc16[1],  ~crc16[2],  ~crc16[3],
             ~crc16[4],  ~crc16[5],  ~crc16[6],  ~crc16[7]} !== exp_hi)
        begin
            errors = errors + 1;
            $display("FAIL crc16 len=%0d: got lo=%h hi=%h expect lo=%h hi=%h",
                      n,
                      {~crc16[8], ~crc16[9], ~crc16[10], ~crc16[11],
                       ~crc16[12], ~crc16[13], ~crc16[14], ~crc16[15]},
                      {~crc16[0], ~crc16[1], ~crc16[2], ~crc16[3],
                       ~crc16[4], ~crc16[5], ~crc16[6], ~crc16[7]},
                      exp_lo, exp_hi);
        end
        else
            $display("PASS crc16 len=%0d -> %h %h", n, exp_lo, exp_hi);
    end
    endtask

    // ------------------------------------------------------------------
    // CRC16 checker residual: payload + the 16 wire bits -> 16'h800D.
    // ------------------------------------------------------------------
    task check_crc16_residual(input integer n);
        integer i, j;
        reg [15:0] wire_crc;
    begin
        // generator pass: wire bits = ~crc[15-i]
        seed;
        send_bytes(n);
        for (i = 0; i < 16; i = i + 1)
            wire_crc[i] = ~crc16[15 - i];
        // checker pass: payload + wire bits -> 800D
        seed;
        send_bytes(n);
        for (i = 0; i < 16; i = i + 1)
            send_bit(wire_crc[i]);
        if (crc16 !== 16'h800D)
        begin
            errors = errors + 1;
            $display("FAIL crc16 residual len=%0d: got %h expect 800d",
                      n, crc16);
        end
        else
            $display("PASS crc16 residual len=%0d -> 800d", n);
    end
    endtask


    initial begin
        $dumpoff;
        // ---- CRC5 token generator vectors (valentyusb crc5_token doctests)
        check_crc5_token(7'd0,  4'd0,  5'h02);
        check_crc5_token(7'd92, 4'd0,  5'h1C);
        check_crc5_token(7'd3,  4'd0,  5'h0A);
        check_crc5_token(7'd56, 4'd4,  5'h0B);
        check_crc5_token(7'd42, 4'd1,  5'h1C);
        check_crc5_token(7'd7,  4'd15, 5'h00);

        // ---- CRC5 checker residual (16-bit stream, no PID) -> 0x0C
        check_crc5_residual(7'd0,  4'd0);
        check_crc5_residual(7'd92, 4'd0);
        check_crc5_residual(7'd56, 4'd4);
        check_crc5_residual(7'd42, 4'd1);

        // ---- CRC16 generator vectors (valentyusb CrcMoose3 CRC-16/USB)
        tmp[0] = 8'd0;
        check_crc16(0, 8'h00, 8'h00);          // empty payload
        tmp[0] = 8'h01;
        check_crc16(1, 8'h81, 8'h7F);
        tmp[0] = "F"; tmp[1] = "o"; tmp[2] = "m"; tmp[3] = "u";
        check_crc16(4, 8'h37, 8'hF9);
        tmp[0] = "1"; tmp[1] = "2"; tmp[2] = "3"; tmp[3] = "4";
        tmp[4] = "5"; tmp[5] = "6"; tmp[6] = "7"; tmp[7] = "8";
        tmp[8] = "9";
        check_crc16(9, 8'hC8, 8'hB4);
        for (k = 0; k < 64; k = k + 1)
            tmp[k] = k[7:0];
        check_crc16(64, 8'h26, 8'hF7);         // max bulk packet
        tmp[0] = 8'h05; tmp[1] = 8'h06;
        check_crc16(2, 8'h7D, 8'h1D);

        // ---- CRC16 checker residual -> 0x800D
        check_crc16_residual(0);
        tmp[0] = 8'h01;
        check_crc16_residual(1);
        tmp[0] = "F"; tmp[1] = "o"; tmp[2] = "m"; tmp[3] = "u";
        check_crc16_residual(4);
        for (k = 0; k < 64; k = k + 1)
            tmp[k] = k[7:0];
        check_crc16_residual(64);

        // ---- summary
        if (errors == 0)
            $display("TB_CRC: ALL TESTS PASSED");
        else
            $display("TB_CRC: %0d FAILURES", errors);
        $finish;
    end
endmodule