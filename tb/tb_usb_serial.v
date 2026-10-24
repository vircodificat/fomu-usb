// Fomu UART-over-USB PoC — full-device acceptance testbench.
// Part of ~/projects/fomu-usb (see FOMU_UART_OVER_USB_PLANv2.md §7).
// Hand-written Verilog-2005 for iverilog.
//
// Instantiates the ext core `\top::FomuTop` directly (ext/fomu_top.sv,
// with sb_io_sim for the pads): the Virdant platform wrapper (the
// generated build/top.sv `module Top`, from src/top.vir's
// `export mod Top for ice40::Fomu`) ties usb_dp_pu/led_* constant and
// has no POR_CYCLES parameter, so the full-device acceptance runs
// against the ext core, not the wrapper — on a dp/dn net pair with
// the BFM, exercising the plan's acceptance sequence:
//   1. reset; wait for usb_dp_pu (connect)
//   2. enumerate (device descriptor, SET_ADDRESS(42), config descriptor,
//      strings, SET_CONFIGURATION(1))
//   3. negative: tokens to the old address (0) get no response
//   4. bulk loopback: 512 bytes written in 64-byte packets (both toggle
//      phases, NAK-retry + IN-drain interleaving) echo back byte-identical
//      and in order, with DATA0/DATA1 toggling checked both directions
//   5. CDC control traffic: SET_LINE_CODING / GET_LINE_CODING are
//      stripped from the core and must STALL; SET_CONTROL_LINE_STATE
//   6. SOF tokens interleaved between every transaction
//   7. bus reset mid-stream -> device returns to address 0 ->
//      re-enumerate (addr 7) -> echo still works
`timescale 1ns/1ps

module tb_usb_serial;
    reg clk = 0;
    always #10.416 clk = ~clk;      // 48 MHz

    wire usb_dp, usb_dn;
    pullup(usb_dp);                 // D+ pull-up (idle J when undriven)
    pulldown(usb_dn);

    integer errors = 0;
    integer sof_frame = 0;

    // ------------------------------------------------------------------
    // DUT (short POR for sim; hardware default is ~175 ms)
    // ------------------------------------------------------------------
    wire usb_dp_pu, rxing, txing;
    \top::FomuTop #(.POR_CYCLES(24'd200)) dut(
        .clock(clk),
        .usb_dp(usb_dp), .usb_dn(usb_dn),
        .usb_dp_pu(usb_dp_pu),
        .rxing(rxing), .txing(txing),
        // byte-stream interface (the Virdant echo server consumes it,
        // exactly as the platform top in build/top.sv wires it)
        .rx_data(echo_rx_data),
        .rx_valid(echo_rx_valid),
        .tx_ready(core_tx_ready),
        .tx_data(echo_tx_data),
        .tx_strobe(echo_tx_strobe),
        .rx_ready(echo_room));

    // The Virdant echo server (generated Verilog from src/echo.vir) and
    // the raw UART loop wires -- mirrors export mod Top in src/top.vir:
    // host bytes are rendered onto uart_txd at 115200 8N1 and looped
    // into uart_rxd, and whatever the echo server reads there goes back
    // to the host.
    wire [7:0] echo_rx_data;
    wire       echo_rx_valid;
    wire       core_tx_ready;
    wire [7:0] echo_tx_data;
    wire       echo_tx_strobe;
    wire       echo_room;
    wire       uart_txd, uart_rxd;

    \echo::EchoServer echo(
        .clock(clk),
        .usb_rx_data (echo_rx_data),
        .usb_rx_valid(echo_rx_valid),
        .usb_tx_ready(core_tx_ready),
        .tx_data     (echo_tx_data),
        .tx_strobe   (echo_tx_strobe),
        .room        (echo_room),
        .txd         (uart_txd),
        .rxd         (uart_rxd)
    );
    assign uart_rxd = uart_txd;   // the loopback jumper (uart_rx_line)

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

    reg [7:0] got [0:1023];        // collected device bytes
    integer   got_n;

    task collect_cap;              // append the BFM capture to got[]
        integer i;
    begin
        for (i = 0; i < host.cap_len; i = i + 1)
            got[got_n + i] = host.cap_bytes[i];
        got_n = got_n + host.cap_len;
    end
    endtask

    // SOF between transactions (plan step 6: exercises the address filter)
    task send_sof;
    begin
        sof_frame = sof_frame + 1;
        host.host_sof(sof_frame[10:0]);
    end
    endtask

    // wait for a BFM capture with a long timeout, capturing into got[]
    task wait_and_collect(input integer timeout, output ok);
        integer t;
    begin
        ok = 0;
        for (t = 0; t < timeout; t = t + 1) begin
            @(posedge clk);
            if (host.cap_valid === 1'b1) begin
                collect_cap;
                ok = 1;
                disable wait_and_collect;
            end
        end
    end
    endtask

    // expect a specific handshake pid (or fail on a different one)
    task wait_hs(input [3:0] pid, input integer timeout, output ok);
        integer t;
    begin
        ok = 0;
        for (t = 0; t < timeout; t = t + 1) begin
            @(posedge clk);
            if (host.cap_valid === 1'b1) begin
                if (host.cap_pid !== pid) begin
                    errors = errors + 1;
                    $display("FAIL: expected handshake %h, got %h",
                             pid, host.cap_pid);
                end
                ok = 1;
                disable wait_hs;
            end
        end
    end
    endtask

    // ------------------------------------------------------------------
    // control-transfer helpers (host side)
    // ------------------------------------------------------------------
    localparam [3:0] PID_OUT = 4'h1, PID_IN = 4'h9, PID_SETUP = 4'hD;

    // SETUP stage: token + DATA0(request) -> device ACK
    task ctrl_setup(input [6:0] addr);
        integer i;
    begin
        send_sof;
        for (i = 0; i < 8; i = i + 1)
            host.host_data_bytes[i] = req[i];
        host.host_data_len = 7'd8;
        host.host_send_token(PID_SETUP, addr, 4'd0);
        host.host_send_data0;
        wait_hs(4'h2, 8192, ok);      // ACK
        if (!ok) begin
            errors = errors + 1;
            $display("FAIL: SETUP not ACKed (addr=%0d bRequest=%h)",
                     addr, req[1]);
        end
    end
    endtask

    // control read: SETUP + IN polls (DATA1 first, alternate) +
    // OUT status (DATA1 ZLP -> device ACK).  Collects n bytes into got[].
    task ctrl_read(input [6:0] addr, input integer n);
        integer collected;
        reg tog;
        reg [7:0] lpid;
    begin
        got_n = 0;
        ctrl_setup(addr);
        collected = 0;
        tog = 1'b1;                   // device data stage starts DATA1
        while (collected < n) begin
            send_sof;
            host.host_send_token(PID_IN, addr, 4'd0);
            lpid = tog ? 4'hB : 4'h3;
            // wait for the data packet
            wait_and_collect(16384, ok);
            if (!ok) begin
                errors = errors + 1;
                $display("FAIL: ctrl_read: no data packet");
                collected = n;      // abort
            end else begin
                if (host.cap_pid !== lpid) begin
                    errors = errors + 1;
                    $display("FAIL: ctrl_read: pid %h expected %h",
                             host.cap_pid, lpid);
                end
                if (!host.cap_crc_ok) begin
                    errors = errors + 1;
                    $display("FAIL: ctrl_read: bad CRC");
                end
                collected = collected + host.cap_len;
                host.host_send_ack;
                tog = ~tog;
            end
        end
        // status: OUT token + DATA1 ZLP -> device ACK
        send_sof;
        host.host_send_token(PID_OUT, addr, 4'd0);
        host.host_data_len = 7'd0;
        host.host_send_data1;
        wait_hs(4'h2, 8192, ok);
        if (!ok) begin
            errors = errors + 1;
            $display("FAIL: ctrl_read: status not ACKed");
        end
    end
    endtask

    // control write (with data stage): SETUP + OUT DATAx(payload) +
    // IN status (device DATA1 ZLP -> host ACK)
    task ctrl_write(input [6:0] addr, input integer n);
        integer i;
        reg tog;
        reg [7:0] lpid;
    begin
        ctrl_setup(addr);
        tog = 1'b1;                   // host data stage starts DATA1
        for (i = 0; i < n; i = i + 1)
            host.host_data_bytes[i] = payload[i];
        host.host_data_len = n[6:0];
        send_sof;
        host.host_send_token(PID_OUT, addr, 4'd0);
        lpid = tog ? 4'hB : 4'h3;
        if (lpid == 4'hB) host.host_send_data1;
        else              host.host_send_data0;
        wait_hs(4'h2, 8192, ok);
        if (!ok) begin
            errors = errors + 1;
            $display("FAIL: ctrl_write: data not ACKed");
        end
        // status IN: device sends a DATA1 ZLP; host ACKs it
        send_sof;
        host.host_send_token(PID_IN, addr, 4'd0);
        wait_and_collect(16384, ok);
        if (!ok) begin
            errors = errors + 1;
            $display("FAIL: ctrl_write: no status ZLP");
        end else begin
            if ((host.cap_pid !== 4'hB) || (host.cap_len !== 7'd0)) begin
                errors = errors + 1;
                $display("FAIL: ctrl_write: status not a DATA1 ZLP (pid=%h len=%0d)",
                         host.cap_pid, host.cap_len);
            end
            host.host_send_ack;
        end
    end
    endtask

    // control no-data (SET_ADDRESS / SET_CONFIGURATION / CLS): status IN
    task ctrl_no_data(input [6:0] addr);
    begin
        ctrl_setup(addr);
        send_sof;
        host.host_send_token(PID_IN, addr, 4'd0);
        wait_and_collect(16384, ok);
        if (!ok) begin
            errors = errors + 1;
            $display("FAIL: ctrl_no_data: no status ZLP");
        end else begin
            if ((host.cap_pid !== 4'hB) || (host.cap_len !== 7'd0)) begin
                errors = errors + 1;
                $display("FAIL: ctrl_no_data: status not a DATA1 ZLP");
            end
            host.host_send_ack;
            // settle: let the device decode the ACK and latch
            // dev_addr / configured before the next transaction
            repeat (16) @(posedge clk);
        end
    end
    endtask

    // ------------------------------------------------------------------
    // bulk helpers
    // ------------------------------------------------------------------
    reg out_tog;   // 0 => next host bulk-OUT data pid DATA0
    reg in_tog;    // 0 => next device bulk-IN data pid DATA0

    // try a bulk OUT: returns 1 if ACKed, 0 if NAKed (retry later)
    task bulk_out_try(input [6:0] addr, input integer n, output acked);
        integer i;
        reg [7:0] lpid;
    begin
        acked = 0;
        for (i = 0; i < n; i = i + 1)
            host.host_data_bytes[i] = out_buf[i];
        host.host_data_len = n[6:0];
        send_sof;
        host.host_send_token(PID_OUT, addr, 4'd1);
        lpid = out_tog ? 4'hB : 4'h3;
        if (lpid == 4'hB) host.host_send_data1;
        else              host.host_send_data0;
        // ACK accepts (toggle advances), NAK just means retry;
        // capture any handshake (cap_len is 0 for handshakes, so
        // wait_and_collect appends nothing to got[])
        wait_and_collect(16384, ok);
        if (host.cap_pid === 4'h2) begin
            acked = 1;
            out_tog = ~out_tog;
        end else if (host.cap_pid === 4'hA) begin
            acked = 0;              // device FIFO full: retry later
        end else begin
            errors = errors + 1;
            $display("FAIL: bulk_out: unexpected handshake %h",
                     host.cap_pid);
        end
    end
    endtask

    // poll IN once: 1 = data captured (host ACKs it), 0 = NAKed (empty)
    task bulk_in_try(input [6:0] addr, output got_data);
        reg [7:0] lpid;
    begin
        got_data = 0;
        send_sof;
        host.host_send_token(PID_IN, addr, 4'd1);
        // data packet or NAK — both are captures
        wait_and_collect(16384, ok);
        if (!ok) begin
            errors = errors + 1;
            $display("FAIL: bulk_in: no response");
        end else if (host.cap_is_data === 1'b1) begin
            lpid = in_tog ? 4'hB : 4'h3;
            if (host.cap_pid !== lpid) begin
                errors = errors + 1;
                $display("FAIL: bulk_in: pid %h expected %h",
                         host.cap_pid, lpid);
            end
            if (!host.cap_crc_ok) begin
                errors = errors + 1;
                $display("FAIL: bulk_in: bad CRC");
            end
            host.host_send_ack;
            in_tog = ~in_tog;
            got_data = 1;
        end else if (host.cap_pid === 4'hA) begin
            got_data = 0;             // NAK: nothing to send
        end else begin
            errors = errors + 1;
            $display("FAIL: bulk_in: unexpected pid %h", host.cap_pid);
        end
    end
    endtask

    // drain the echo: poll IN until `want` MORE bytes are appended to
    // got[] (append-based: the S4 retry loop drains the echo along the
    // way, so the caller resets got_n once per phase and compares totals)
    task drain_in(input [6:0] addr, input integer want);
        integer guard;
        integer start;
        reg d;
    begin
        start = got_n;
        guard = 0;
        while (((got_n - start) < want) && (guard < 4096)) begin
            bulk_in_try(addr, d);
            guard = guard + 1;
        end
        if ((got_n - start) < want) begin
            errors = errors + 1;
            $display("FAIL: drain_in: got %0d more, wanted %0d (total %0d)",
                     got_n - start, want, got_n);
        end
    end
    endtask

    // ------------------------------------------------------------------
    // shared state
    // ------------------------------------------------------------------
    reg [7:0] req [0:7];      // control request bytes
    reg [7:0] payload [0:63]; // control write payload
    reg [7:0] out_buf [0:63]; // bulk OUT payload
    reg ok;

    integer i, j, pkt;

    initial begin
        got_n = 0;
        host.host_fail = 1'b0;
        out_tog = 1'b0;
        in_tog = 1'b0;

        // ---------------------------------------------------------------
        // 1. reset; wait for connect
        // ---------------------------------------------------------------
        $display("--- S1: wait for usb_dp_pu (connect)");
        wait (usb_dp_pu === 1'b1);
        $display("connected at %0t", $time);
        host.host_idle(256);

        // ---------------------------------------------------------------
        // 2. enumeration
        // ---------------------------------------------------------------
        $display("--- S2: GET_DESCRIPTOR(device) at address 0");
        req[0] = 8'h80; req[1] = 8'h06; req[2] = 8'h00; req[3] = 8'h01;
        req[4] = 8'h00; req[5] = 8'h00; req[6] = 8'h12; req[7] = 8'h00;
        ctrl_read(7'd0, 18);
        check(got_n === 18, "S2: device descriptor length");
        check(got[0]  === 8'h12,              "S2: bLength");
        check(got[1]  === 8'h01,              "S2: bDescriptorType");
        check(got[4]  === 8'hEF,              "S2: bDeviceClass (IAD)");
        check(got[5]  === 8'h02,              "S2: bDeviceSubClass");
        check(got[7]  === 8'h40,              "S2: bMaxPacketSize0");
        check(got[8]  === 8'h09 && got[9] === 8'h12, "S2: idVendor 0x1209");
        check(got[10] === 8'h10 && got[11] === 8'h7A, "S2: idProduct 0x7A10");
        check(got[17] === 8'h01,              "S2: bNumConfigurations");

        $display("--- S2: SET_ADDRESS(42)");
        req[0] = 8'h00; req[1] = 8'h05; req[2] = 8'h2A; req[3] = 8'h00;
        req[4] = 8'h00; req[5] = 8'h00; req[6] = 8'h00; req[7] = 8'h00;
        ctrl_no_data(7'd0);
        host.host_idle(64);

        $display("--- S2: GET_DESCRIPTOR(config) at address 42");
        req[0] = 8'h80; req[1] = 8'h06; req[2] = 8'h00; req[3] = 8'h02;
        req[4] = 8'h00; req[5] = 8'h00; req[6] = 8'h4B; req[7] = 8'h00;
        ctrl_read(7'd42, 75);
        check(got_n === 75, "S2: config descriptor length");
        check(got[0]  === 8'h09,              "S2: cfg bLength");
        check(got[2]  === 8'h4B,              "S2: wTotalLength lo");
        check(got[3]  === 8'h00,              "S2: wTotalLength hi");
        check(got[4]  === 8'h02,              "S2: bNumInterfaces");
        check(got[13] === 8'h02 && got[14] === 8'h02,
              "S2: IAD CDC/ACM");            // IAD function class/subclass
        check(got[22] === 8'h02 && got[23] === 8'h02,
              "S2: comm interface CDC/ACM"); // comm if class/subclass
        check(got[48] === 8'h03,             "S2: notification EP interrupt");
        check(got[57] === 8'h0A,             "S2: data interface class");
        check(got[63] === 8'h81 && got[64] === 8'h02,
              "S2: EP1 IN bulk");
        check(got[70] === 8'h01 && got[71] === 8'h02,
              "S2: EP1 OUT bulk");

        $display("--- S2: GET_DESCRIPTOR(string 0..2)");
        req[0] = 8'h80; req[1] = 8'h06; req[2] = 8'h00; req[3] = 8'h03;
        req[4] = 8'h00; req[5] = 8'h00; req[6] = 8'h04; req[7] = 8'h00;
        ctrl_read(7'd42, 4);
        check(got[0] === 8'h04 && got[1] === 8'h03, "S2: string0 header");
        check(got[2] === 8'h09 && got[3] === 8'h04, "S2: string0 LANGID");

        req[2] = 8'h01; req[6] = 8'h0A;
        ctrl_read(7'd42, 10);
        check(got[0] === 8'h0A && got[1] === 8'h03, "S2: string1 header");
        check(got[2] === 8'h46, "S2: string1 'F'");

        req[2] = 8'h02; req[6] = 8'h20;
        ctrl_read(7'd42, 32);
        check(got[0] === 8'h20 && got[1] === 8'h03, "S2: string2 header");
        check(got[2] === 8'h46, "S2: string2 'F'");

        $display("--- S2: SET_CONFIGURATION(1)");
        req[0] = 8'h00; req[1] = 8'h09; req[2] = 8'h01; req[3] = 8'h00;
        req[4] = 8'h00; req[5] = 8'h00; req[6] = 8'h00; req[7] = 8'h00;
        ctrl_no_data(7'd42);
        check(dut.u_core.u_ep0.configured === 1'b1, "S2: configured");

        // ---------------------------------------------------------------
        // 3. negative: token to the OLD address (0) must get NO response
        // ---------------------------------------------------------------
        $display("--- S3: negative: OUT to old address 0 -> silence");
        begin
            integer t;
            reg heard;
            heard = 1'b0;
            host.host_send_token(PID_OUT, 7'd0, 4'd1);
            for (t = 0; t < 2048; t = t + 1) begin
                @(posedge clk);
                if (host.cap_valid === 1'b1)
                    heard = 1'b1;
            end
            check(heard === 1'b0, "S3: device must not answer old address");
        end

        // ---------------------------------------------------------------
        // 4. bulk loopback: 512 bytes in 64-byte packets
        // ---------------------------------------------------------------
        $display("--- S4: bulk loopback 512 bytes");
        out_tog = 1'b0;
        in_tog  = 1'b0;
        got_n = 0;                    // one accumulator for the whole phase
        for (pkt = 0; pkt < 8; pkt = pkt + 1) begin
            // build this packet's bytes
            for (i = 0; i < 64; i = i + 1)
                out_buf[i] = (pkt * 64 + i) & 8'hFF;
            // send with NAK-retry: if the device NAKs, drain the echo
            // (like a real host driver) and retry the same packet
            begin
                integer tries;
                reg acked;
                acked = 0;
                tries = 0;
                while (!acked && tries < 512) begin
                    bulk_out_try(7'd42, 64, acked);
                    if (!acked) begin
                        drain_in(7'd42, 16);   // pull the echo along
                    end
                    tries = tries + 1;
                end
                check(acked, "S4: bulk OUT packet accepted");
            end
        end
        // drain whatever is still backlogged
        drain_in(7'd42, 512 - got_n);
        check(got_n === 512, "S4: echoed byte count");
        check(rxing === 1'b1, "S4: usb rxing indicator high");
        check(txing === 1'b1, "S4: usb txing indicator high");
        for (i = 0; i < 512; i = i + 1)
            if (got[i] !== (i & 8'hFF)) begin
                errors = errors + 1;
                $display("FAIL: S4: echo[%0d] = %h expected %h",
                         i, got[i], i & 8'hFF);
                i = 512;   // stop reporting
            end

        // ---------------------------------------------------------------
        // 5. CDC control traffic.  SET_LINE_CODING / GET_LINE_CODING are
        // stripped (the echo server never reads line coding): both must
        // STALL.  Linux cdc_acm tolerates the SET side failing.
        // ---------------------------------------------------------------
        $display("--- S5: SET_LINE_CODING (stripped: expect STALL)");
        req[0] = 8'h21; req[1] = 8'h20; req[2] = 8'h00; req[3] = 8'h00;
        req[4] = 8'h01; req[5] = 8'h00; req[6] = 8'h07; req[7] = 8'h00;
        payload[0] = 8'h00; payload[1] = 8'hC2; payload[2] = 8'h01;
        payload[3] = 8'h00; payload[4] = 8'h00; payload[5] = 8'h00;
        payload[6] = 8'h08;
        ctrl_setup(7'd42);
        send_sof;
        for (i = 0; i < 7; i = i + 1)
            host.host_data_bytes[i] = payload[i];
        host.host_data_len = 7'd7;
        host.host_send_token(PID_OUT, 7'd42, 4'd0);
        host.host_send_data1;
        wait_hs(4'hE, 8192, ok);              // STALL, not ACK
        if (!ok) begin
            errors = errors + 1;
            $display("FAIL: S5: SET_LINE_CODING not STALLed");
        end

        $display("--- S5: GET_LINE_CODING (stripped: expect STALL)");
        req[0] = 8'hA1; req[1] = 8'h21; req[2] = 8'h00; req[3] = 8'h00;
        req[4] = 8'h01; req[5] = 8'h00; req[6] = 8'h07; req[7] = 8'h00;
        ctrl_setup(7'd42);
        send_sof;
        host.host_send_token(PID_IN, 7'd42, 4'd0);
        wait_hs(4'hE, 8192, ok);              // STALL, not data
        if (!ok) begin
            errors = errors + 1;
            $display("FAIL: S5: GET_LINE_CODING not STALLed");
        end

        // ---------------------------------------------------------------
        // 5b. bulk traffic while EP0 is STALLed (regression: the data
        // router once handed EVERY data packet to a stalled EP0,
        // STALLing the host's bulk-OUT writes and killing the echo
        // after the connect script's SET_LINE_CODING stalled).  No
        // SETUP intervenes here, so the device stays in EP_STALL for
        // the whole exchange; the SET_CONTROL_LINE_STATE below (a
        // SETUP) is what clears it.
        // ---------------------------------------------------------------
        $display("--- S5b: bulk loopback while EP0 is STALLed");
        begin : s5b_bulk_during_stall
            reg acked;
            reg [7:0] expb;
            integer k;
            got_n = 0;
            for (k = 0; k < 8; k = k + 1)
                out_buf[k] = 8'h40 + k;
            bulk_out_try(7'd42, 8, acked);
            check(acked, "S5b: bulk OUT ACKed while EP0 stalled");
            drain_in(7'd42, 8);
            check(got_n === 8, "S5b: echoed 8 bytes while EP0 stalled");
            for (k = 0; k < 8; k = k + 1) begin
                expb = 8'h40 + k;
                if (got[k] !== expb) begin
                    errors = errors + 1;
                    $display("FAIL: S5b: echo[%0d] = %h expected %h",
                             k, got[k], expb);
                    k = 8;
                end
            end
        end

        $display("--- S5: SET_CONTROL_LINE_STATE (DTR|RTS)");
        req[0] = 8'h21; req[1] = 8'h22; req[2] = 8'h03; req[3] = 8'h00;
        req[4] = 8'h01; req[5] = 8'h00; req[6] = 8'h00; req[7] = 8'h00;
        ctrl_no_data(7'd42);

        // ---------------------------------------------------------------
        // 7. bus reset mid-stream -> re-enumerate -> echo still works
        // ---------------------------------------------------------------
        $display("--- S7: bus reset (10 ms SE0)");
        host.host_reset;
        host.host_idle(256);
        check(dut.u_core.u_ep0.dev_addr === 7'd0, "S7: address back to 0");
        check(dut.u_core.u_ep0.configured === 1'b0, "S7: unconfigured");

        $display("--- S7: re-enumerate at address 7");
        req[0] = 8'h80; req[1] = 8'h06; req[2] = 8'h00; req[3] = 8'h01;
        req[4] = 8'h00; req[5] = 8'h00; req[6] = 8'h12; req[7] = 8'h00;
        ctrl_read(7'd0, 18);
        check(got_n === 18, "S7: device descriptor after reset");

        req[0] = 8'h00; req[1] = 8'h05; req[2] = 8'h07; req[3] = 8'h00;
        req[4] = 8'h00; req[5] = 8'h00; req[6] = 8'h00; req[7] = 8'h00;
        ctrl_no_data(7'd0);
        req[0] = 8'h00; req[1] = 8'h09; req[2] = 8'h01; req[3] = 8'h00;
        req[4] = 8'h00; req[5] = 8'h00; req[6] = 8'h00; req[7] = 8'h00;
        ctrl_no_data(7'd7);

        $display("--- S7: echo after re-enumeration");
        out_tog = 1'b0;
        in_tog  = 1'b0;
        got_n = 0;
        for (i = 0; i < 32; i = i + 1)
            out_buf[i] = 8'hA0 + i;
        begin
            integer tries;
            reg acked;
            acked = 0;
            tries = 0;
            while (!acked && tries < 512) begin
                bulk_out_try(7'd7, 32, acked);
                if (!acked) drain_in(7'd7, 16);
                tries = tries + 1;
            end
            check(acked, "S7: bulk OUT after reset");
        end
        drain_in(7'd7, 32 - got_n);
        check(got_n === 32, "S7: echo count after reset");
        for (i = 0; i < 32; i = i + 1)
            check(got[i] === (8'hA0 + i), "S7: echo byte");

        // ---------------------------------------------------------------
        // summary
        // ---------------------------------------------------------------
        if (errors == 0 && host.host_fail == 1'b0)
            $display("TB_USB_SERIAL: ALL TESTS PASSED");
        else
            $display("TB_USB_SERIAL: %0d FAILURES (bfm_fail=%0d)",
                     errors, host.host_fail);
        $finish;
    end
    // --------------------------------------------------------------
    // Instrumentation: monitor the core's RX packet CRC and length
    // --------------------------------------------------------------
    always @(posedge clk) begin
        if (dut.u_core.u_rxpkt.dt_valid) begin
            $display("[%0t] RX pkt: dt_len=%0d dt_crc_ok=%b",
                     $time, dut.u_core.u_rxpkt.dt_len,
                     dut.u_core.u_rxpkt.dt_crc_ok);
        end
    end

endmodule