//=============================================================================
// tb_gnss_gpsdo_top.v -- self-checking testbench for the AX7010 GPSDO top level
//-----------------------------------------------------------------------------
// This is a CHECKPOINT testbench, not a waveform dump. Every claim the design
// makes is turned into an assertion, and a failure prints [FAIL] with the numbers
// that disagreed and then stops. [PASS] means the checked property genuinely held.
//
//==============================================================================
// ARCHITECTURE UNDER TEST
//==============================================================================
// The reference clock (clk, standing in for FCLK_CLK0) is INDEPENDENT of the
// OCXO. The OCXO is the device being measured:
//
//     clk      = the ruler      -> counts the gate interval AND the OCXO edges
//     ocxo_in  = the measurand  -> counted over one GNSS 1PPS interval
//     pps_in   = the gate       -> ideally one second
//
// phase_error is positive when the OCXO is SLOW, because a slow oscillator
// produces fewer edges than nominal.
//
//==============================================================================
// THE TIME SCALE, AND THE TRAP IT SETS
//==============================================================================
// The scale is compressed by 100 so a whole 1PPS period fits in a millisecond of
// simulated time, but the RATIOS that the logic depends on are preserved exactly:
//
//     reference : OCXO        = 1 MHz : 100 kHz  = 10 : 1   (real: 100 MHz : 10 MHz)
//     reference : baud rate   = 1 MHz : 960      = 10416 clocks/bit
//                                    (real: 100 MHz : 9600 = 10416 clocks/bit)
//
// THE TRAP: it is tempting to compress the clock alone while leaving the baud rate
// at 9600. That leaves 0.1 clocks per bit, and the UART -- whose whole design
// depends on oversampling margin -- fails deep inside its state machine. An
// earlier version of this file made exactly that mistake. Keep the ratios.
//
//==============================================================================
// WHAT IS VERIFIED
//==============================================================================
//   T1  Phase error SIGN and MAGNITUDE for a known OCXO offset. A slow OCXO must
//       give a POSITIVE error. This is the assertion that protects the PI loop
//       from a polarity inversion, which turns convergence into divergence.
//   T2  The raw OCXO count is right: nominally 100 edges per gate.
//   T3  Mailbox hold: a byte arriving while NEW_DATA is set is DISCARDED and
//       counted in drop_count, and never overwrites the pending byte.
//   T4  Mailbox write-1-to-clear: only a write with bit 8 set clears the flag.
//   T5  SPI frame: writing 0x08 emits exactly one 16-bit frame, header 0b0011,
//       low 12 bits equal to the written value.
//
//==============================================================================
// WHAT IS NOT VERIFIED HERE
//==============================================================================
//   * The TDC's tap delay and hence its absolute accuracy. That is a physical
//     property of the device; tdc_carry.v says so explicitly. The TDC's ability
//     to resolve within one clock period is asserted in tb_tdc_carry.v, which can
//     inject known sub-cycle offsets.
//   * Timing closure. Only implementation can establish that.
//
// NOTE: every comment and string in this file is ASCII on purpose.
//=============================================================================

`timescale 1ns / 1ps

module tb_gnss_gpsdo_top;

    //--------------------------------------------------------------------------
    // Time scale -- see the header for why these particular numbers
    //--------------------------------------------------------------------------
    localparam integer CLK_HALF   = 500;          // 1 MHz reference, 1000 ns period
    localparam integer CLK_PER    = 1000;         // reference period, ns
    localparam integer CLK_FREQ   = 1_000_000;    // declared to the UART
    localparam integer BAUD       = 960;          // same 10416 clk/bit as 9600@100MHz
    localparam integer BIT_NS     = 1041666;      // 1041.666 us per bit

    localparam integer GATE_NS    = 1_000_000;    // 1PPS period: 1 ms simulated

    // OCXO: nominal 100 kHz, i.e. 10.0 us period = 100 edges per 1 ms gate.
    localparam integer OCXO_NOM_PS = 10_000;      // 10.000 us
    localparam integer OCXO_FAST_PS = 9_900;      // 9.900 us -> 101 edges (FAST)
    localparam integer OCXO_SLOW_PS = 10_100;     // 10.100 us ->  99 edges (SLOW)

    localparam integer NOMINAL_OCXO = 100;        // nominal edges per gate
    localparam integer NOMINAL_CYC  = 1000;       // nominal reference clocks per gate

    localparam integer TAP_PER   = 16'd2000;
    localparam integer TAP_OFF   = 16'd0;
    localparam integer SCALE     = 256;           // phase_error units per ref period

    // One count error = 256 units. The fine term is nominally zero here (the
    // gate is a whole number of reference periods and the OCXO phase repeats),
    // but the TDC contributes a bounded residue, so the tolerance admits it while
    // staying far below the 256-unit signal.
    localparam integer TOL       = 400;

    integer errors = 0;

    //--------------------------------------------------------------------------
    // Clock and reset
    //--------------------------------------------------------------------------
    reg clk = 1'b0;
    always #(CLK_HALF) clk = ~clk;

    reg rst_n = 1'b0;

    //--------------------------------------------------------------------------
    // DUT signals
    //--------------------------------------------------------------------------
    reg  pps_in  = 1'b0;
    reg  ocxo_in = 1'b0;
    reg  uart_rx = 1'b1;         // idle high

    wire        spi_sck, spi_mosi, spi_cs_n;
    wire [3:0]  led;
    wire [31:0] status_dbg;

    reg  [11:0] awaddr  = 12'd0;
    reg  [2:0]  awprot  = 3'd0;
    reg         awvalid = 1'b0;
    wire        awready;
    reg  [31:0] wdata   = 32'd0;
    reg  [3:0]  wstrb   = 4'hF;
    reg         wvalid  = 1'b0;
    wire        wready;
    wire [1:0]  bresp;
    wire        bvalid;
    reg         bready  = 1'b1;
    reg  [11:0] araddr  = 12'd0;
    reg  [2:0]  arprot  = 3'd0;
    reg         arvalid = 1'b0;
    wire        arready;
    wire [31:0] rdata;
    wire [1:0]  rresp;
    wire        rvalid;
    reg         rready  = 1'b1;

    //--------------------------------------------------------------------------
    // DUT
    //--------------------------------------------------------------------------
    gnss_gpsdo_top #(
        .NOMINAL_CYCLES (NOMINAL_CYC),
        .NOMINAL_OCXO   (NOMINAL_OCXO),
        .CLK_FREQ_HZ    (CLK_FREQ),
        .CNT_WIDTH      (48),
        .NUM_CARRY4     (8),          // small line: simulation speed only
        .FRAC_BITS      (16),
        .SCALE_BITS     (8),
        .BAUD_RATE      (BAUD),
        .SPI_DIV        (4),
        .SIM_DELAY_MODEL(1),          // behavioural delay line
        .SIM_TAP_PS     (150)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .pps_in     (pps_in),
        .ocxo_in    (ocxo_in),
        .uart_rx    (uart_rx),
        .spi_sck    (spi_sck),
        .spi_mosi   (spi_mosi),
        .spi_cs_n   (spi_cs_n),
        .led        (led),
        .status_dbg (status_dbg),
        .S_AXI_AWADDR  (awaddr),
        .S_AXI_AWPROT  (awprot),
        .S_AXI_AWVALID (awvalid),
        .S_AXI_AWREADY (awready),
        .S_AXI_WDATA   (wdata),
        .S_AXI_WSTRB   (wstrb),
        .S_AXI_WVALID  (wvalid),
        .S_AXI_WREADY  (wready),
        .S_AXI_BRESP   (bresp),
        .S_AXI_BVALID  (bvalid),
        .S_AXI_BREADY  (bready),
        .S_AXI_ARADDR  (araddr),
        .S_AXI_ARPROT  (arprot),
        .S_AXI_ARVALID (arvalid),
        .S_AXI_ARREADY (arready),
        .S_AXI_RDATA   (rdata),
        .S_AXI_RRESP   (rresp),
        .S_AXI_RVALID  (rvalid),
        .S_AXI_RREADY  (rready)
    );

    //==========================================================================
    // Free-running OCXO model
    //
    // It MUST free-run and its period must NOT be reset at each gate edge: the
    // fractional part of the count is part of what the design measures, and a
    // model that restarts on the gate would hide exactly the behaviour under test.
    //
    // A separate always block keyed on a parameter change is avoided on purpose --
    // the period is switched by restarting the generator between test phases,
    // which is what a real oscillator change would look like.
    //==========================================================================
    integer ocxo_half_ps = OCXO_NOM_PS / 2;
    reg     ocxo_run     = 1'b1;

    initial begin
        ocxo_in = 1'b0;
        forever begin
            if (ocxo_run) begin
                #(ocxo_half_ps);
                ocxo_in = ~ocxo_in;
            end else begin
                #1000;
            end
        end
    end

    //==========================================================================
    // SPI frame capture
    //   Samples MOSI on every SCK rising edge: the MCP4921 latches on that edge,
    //   so this reproduces exactly what the part would receive.
    //==========================================================================
    reg [15:0] spi_frame    = 16'd0;
    integer    spi_bits     = 0;
    integer    spi_frames   = 0;
    reg [15:0] spi_captured = 16'd0;
    integer    n_sck_rise   = 0;

    always @(posedge spi_sck) begin
        n_sck_rise = n_sck_rise + 1;
        if (!spi_cs_n) begin
            spi_frame = {spi_frame[14:0], spi_mosi};
            spi_bits  = spi_bits + 1;
            if (spi_bits == 16) begin
                spi_captured = spi_frame;
                spi_frames   = spi_frames + 1;
                spi_bits     = 0;
            end
        end
    end

    //==========================================================================
    // AXI4-Lite helpers
    //==========================================================================
    task axi_write;
        input [11:0] addr;
        input [31:0] data;
        begin
            @(posedge clk);
            awaddr  <= addr;
            wdata   <= data;
            awvalid <= 1'b1;
            wvalid  <= 1'b1;
            while (!(awready && wready)) @(posedge clk);
            @(posedge clk);
            awvalid <= 1'b0;
            wvalid  <= 1'b0;
            while (!bvalid) @(posedge clk);
            @(posedge clk);
        end
    endtask

    task axi_read;
        input  [11:0] addr;
        output [31:0] data;
        begin
            @(posedge clk);
            araddr  <= addr;
            arvalid <= 1'b1;
            while (!arready) @(posedge clk);
            @(posedge clk);
            arvalid <= 1'b0;
            while (!rvalid) @(posedge clk);
            data = rdata;
            @(posedge clk);
        end
    endtask

    //==========================================================================
    // UART byte transmitter (8N1, LSB first)
    //
    // THE IDLE GUARD IS NOT OPTIONAL. uart_rx refuses a start bit until the line
    // has been continuously idle for a FULL bit period -- that guard is what stops
    // a framing error's leftover low level being read as a phantom frame. A
    // testbench that transmits shortly after reset has its first byte silently
    // dropped, which looks exactly like a broken receiver.
    //==========================================================================
    //==========================================================================
    // UART stimulus -- GAPLESS, exactly like a real NMEA stream
    //
    // The previous version inserted
    //     #(BIT_NS * 2);   // satisfy the re-arm guard
    // before every start bit.  That hid a real hardware failure: the guard used
    // to demand a FULL bit period of contiguous idle, while a gapless stream
    // offers exactly one stop bit.  Because NMEA is ASCII (bit 7 always 0) the
    // line is high for the stop bit and nothing else, leaving 0.67 clocks of
    // margin -- so the receiver went deaf between sentences on real hardware
    // while this testbench still passed.
    //
    // uart_send() now emits a byte with NO idle in front of it.  Call uart_idle()
    // explicitly where a gap is genuinely wanted (start of test, between bursts).
    //==========================================================================
    task uart_idle;
        input integer bits;
        begin
            uart_rx = 1'b1;
            #(BIT_NS * bits);
        end
    endtask

    task uart_send;
        input [7:0] b;
        integer i;
        begin
            uart_rx = 1'b0;                 // start bit: begins the instant the
            #(BIT_NS);                      // previous byte's stop bit ends
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx = b[i];
                #(BIT_NS);
            end
            uart_rx = 1'b1;                 // stop bit
            #(BIT_NS);
        end
    endtask

    //==========================================================================
    // 1PPS driver: one pulse per gate, ~100 us wide like a real receiver
    //==========================================================================
    task drive_pps;
        begin
            pps_in = 1'b1;
            #(100_000);
            pps_in = 1'b0;
            #(GATE_NS - 100_000);
        end
    endtask

    //==========================================================================
    // Assertions
    //==========================================================================
    reg signed [31:0] pe;
    reg        [31:0] ocnt;

    task check_phase;
        input signed [31:0] expect;
        input               want_positive;   // 1 = must be > 0, 0 = must be < 0
        input [255:0]       label;
        begin
            axi_read(12'h000, pe);
            if (pe > (expect + TOL) || pe < (expect - TOL)) begin
                errors = errors + 1;
                $display("[FAIL] %0s: phase_error = %0d, expected %0d +/- %0d",
                         label, pe, expect, TOL);
            end else if (want_positive && (pe <= 0)) begin
                errors = errors + 1;
                $display("[FAIL] %0s: phase_error = %0d has the WRONG SIGN (must be > 0)",
                         label, pe);
            end else if (!want_positive && (pe >= 0)) begin
                errors = errors + 1;
                $display("[FAIL] %0s: phase_error = %0d has the WRONG SIGN (must be < 0)",
                         label, pe);
            end else begin
                $display("[PASS] %0s: phase_error = %0d (expected %0d +/- %0d)",
                         label, pe, expect, TOL);
            end
        end
    endtask

    task do_reset_and_cal;
        begin
            rst_n = 1'b0;
            repeat (10) @(posedge clk);
            rst_n = 1'b1;
            repeat (20) @(posedge clk);
            axi_write(12'h00C, TAP_PER);   // 0x0C TDC_TAP_PERIOD
            axi_write(12'h010, TAP_OFF);   // 0x10 TDC_TAP_OFFSET
        end
    endtask

    //==========================================================================
    // Test sequence
    //==========================================================================
    integer    i;
    reg [31:0] rd;
    reg [31:0] drops_before, drops_after;

    initial begin
        $display("================================================================");
        $display(" tb_gnss_gpsdo_top -- AX7010 GPSDO PL top level");
        $display(" reference = 1 MHz (FCLK_CLK0 stand-in), BAUD = %0d (%0d clk/bit)",
                 BAUD, CLK_FREQ / BAUD);
        $display(" gate = %0d ns, nominal OCXO edges/gate = %0d", GATE_NS, NOMINAL_OCXO);
        $display(" real-hardware scaling: x100 (100 MHz ref, 10 MHz OCXO, 1 s gate)");
        $display("================================================================");

        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);

        //----------------------------------------------------------------------
        // T1a: OCXO FAST -> NEGATIVE phase error
        //
        // It is worth stating why fast is negative here, because the sign of this
        // design has already been got wrong once. A fast oscillator fits MORE
        // edges into the fixed gate, so ocxo_count is ABOVE nominal; the formula
        // is (NOMINAL - count), so the result is negative. The loop's contract is
        // that a positive error means "slow, speed it up".
        //----------------------------------------------------------------------
        $display("\n--- T1a: OCXO fast (period %0d ps -> %0d edges/gate) ---",
                 OCXO_FAST_PS, NOMINAL_OCXO + 1);
        ocxo_half_ps = OCXO_FAST_PS / 2;
        ocxo_run     = 1'b0; #2000; ocxo_run = 1'b1;
        do_reset_and_cal;
        for (i = 0; i < 6; i = i + 1) drive_pps;
        repeat (50) @(posedge clk);

        axi_read(12'h014, ocnt);            // 0x14 OCXO_COUNT
        if (ocnt !== (NOMINAL_OCXO + 1)) begin
            errors = errors + 1;
            $display("[FAIL] T2: ocxo_count = %0d, expected %0d", ocnt, NOMINAL_OCXO + 1);
        end else begin
            $display("[PASS] T2: ocxo_count = %0d as expected for a fast OCXO", ocnt);
        end

        check_phase(-1 * SCALE, 1'b0, "T1a OCXO fast -> NEGATIVE");

        //----------------------------------------------------------------------
        // T1b: OCXO SLOW -> POSITIVE phase error
        //----------------------------------------------------------------------
        $display("\n--- T1b: OCXO slow (period %0d ps -> %0d edges/gate) ---",
                 OCXO_SLOW_PS, NOMINAL_OCXO - 1);
        ocxo_half_ps = OCXO_SLOW_PS / 2;
        ocxo_run     = 1'b0; #2000; ocxo_run = 1'b1;
        do_reset_and_cal;
        for (i = 0; i < 6; i = i + 1) drive_pps;
        repeat (50) @(posedge clk);

        axi_read(12'h014, ocnt);
        if (ocnt !== (NOMINAL_OCXO - 1)) begin
            errors = errors + 1;
            $display("[FAIL] T2: ocxo_count = %0d, expected %0d", ocnt, NOMINAL_OCXO - 1);
        end else begin
            $display("[PASS] T2: ocxo_count = %0d as expected for a slow OCXO", ocnt);
        end

        check_phase(1 * SCALE, 1'b1, "T1b OCXO slow -> POSITIVE");

        //----------------------------------------------------------------------
        // T1c: OCXO on frequency -> near zero
        //
        // No sign can be asserted at exactly nominal: the true error is zero and
        // requiring a sign would be asserting that the design is wrong. Only the
        // magnitude is checked.
        //----------------------------------------------------------------------
        $display("\n--- T1c: OCXO on frequency (period %0d ps) ---", OCXO_NOM_PS);
        ocxo_half_ps = OCXO_NOM_PS / 2;
        ocxo_run     = 1'b0; #2000; ocxo_run = 1'b1;
        do_reset_and_cal;
        for (i = 0; i < 6; i = i + 1) drive_pps;
        repeat (50) @(posedge clk);

        axi_read(12'h014, ocnt);
        if (ocnt !== NOMINAL_OCXO) begin
            errors = errors + 1;
            $display("[FAIL] T2: ocxo_count = %0d, expected %0d", ocnt, NOMINAL_OCXO);
        end else begin
            $display("[PASS] T2: ocxo_count = %0d as expected on frequency", ocnt);
        end

        axi_read(12'h000, pe);
        if (pe > TOL || pe < -TOL) begin
            errors = errors + 1;
            $display("[FAIL] T1c on-frequency: phase_error = %0d exceeds +/-%0d", pe, TOL);
        end else begin
            $display("[PASS] T1c on-frequency: phase_error = %0d (within +/-%0d)", pe, TOL);
        end

        //----------------------------------------------------------------------
        // T3 + T4: mailbox hold and write-1-to-clear
        //----------------------------------------------------------------------
        $display("\n--- T3/T4: NMEA mailbox hold + W1C ---");
        do_reset_and_cal;

        axi_read(12'h004, rd);
        if (rd[8]) axi_write(12'h004, 32'h0000_0100);

        axi_read(12'h024, drops_before);   // 0x24 DROP_COUNT

        // Three bytes back to back with none acknowledged, so the mailbox must
        // hold the FIRST and discard the other two.
        //
        // NOTE ON WHY THIS TEST EARNED ITS KEEP: the first version of the receiver
        // in this project sampled data bit 0 half a bit period early, which shifted
        // every assembled byte by one bit position -- 'A' (0x41) arrived as 0x42.
        // The signature was unmistakable once traced per byte (0x42 >> 1 == 0x41),
        // and it is what pointed at the sampling constant rather than at the hold
        // logic. Asserting the exact expected character is what makes that
        // detectable; asserting only "bit8 is set" would have passed.
    uart_idle(2);   // 先让接收机重新武装, 再打成串的字节
        uart_send(8'h41);   // 'A'
        uart_send(8'h42);   // 'B'
        uart_send(8'h43);   // 'C'
        repeat (400) @(posedge clk);

        axi_read(12'h004, rd);
        if (!rd[8]) begin
            errors = errors + 1;
            $display("[FAIL] T3: NEW_DATA not set after three bytes (mailbox=0x%08x)", rd);
        end else if (rd[7:0] !== 8'h41) begin
            errors = errors + 1;
            $display("[FAIL] T3: mailbox holds 0x%02x, expected 0x41 -- byte was overwritten",
                     rd[7:0]);
        end else begin
            $display("[PASS] T3: mailbox held the FIRST byte (0x%02x); later bytes discarded",
                     rd[7:0]);
        end

        axi_read(12'h024, drops_after);
        if (drops_after !== (drops_before + 2)) begin
            errors = errors + 1;
            $display("[FAIL] T3: drop_count went %0d -> %0d, expected +2",
                     drops_before, drops_after);
        end else begin
            $display("[PASS] T3: drop_count rose by exactly 2 (%0d -> %0d)",
                     drops_before, drops_after);
        end

        axi_write(12'h004, 32'h0000_0001);
        repeat (20) @(posedge clk);
        axi_read(12'h004, rd);
        if (!rd[8]) begin
            errors = errors + 1;
            $display("[FAIL] T4: a write of 0x1 cleared NEW_DATA (it must not)");
        end else begin
            $display("[PASS] T4: write of 0x1 left NEW_DATA set");
        end

        axi_write(12'h004, 32'h0000_0100);
        repeat (20) @(posedge clk);
        axi_read(12'h004, rd);
        if (rd[8]) begin
            errors = errors + 1;
            $display("[FAIL] T4: write of 0x100 did NOT clear NEW_DATA");
        end else begin
            $display("[PASS] T4: write of 0x100 cleared NEW_DATA");
        end

    uart_idle(2);   // 先让接收机重新武装, 再打成串的字节
        uart_send(8'h44);   // 'D'
        repeat (300) @(posedge clk);
        axi_read(12'h004, rd);
        if (!rd[8] || rd[7:0] !== 8'h44) begin
            errors = errors + 1;
            $display("[FAIL] T4: after ack the next byte was not accepted (mailbox=0x%08x)", rd);
        end else begin
            $display("[PASS] T4: after ack the next byte 0x%02x was accepted", rd[7:0]);
        end

        //----------------------------------------------------------------------
        // T5: SPI frame
        //----------------------------------------------------------------------
        $display("\n--- T5: MCP4921 SPI frame ---");
        spi_frames = 0;
        spi_bits   = 0;
        n_sck_rise = 0;
        axi_write(12'h008, 32'h0000_0ABC);   // 0x08 DAC_VALUE
        repeat (5000) @(posedge clk);
        $display("  [INFO] T5: sck rising edges = %0d (expect 16 for one 16-bit frame)",
                 n_sck_rise);

        if (spi_frames != 1) begin
            errors = errors + 1;
            $display("[FAIL] T5: %0d frames emitted for one write, expected exactly 1", spi_frames);
        end else if (spi_captured[15:12] !== 4'b0011) begin
            errors = errors + 1;
            $display("[FAIL] T5: header = 0b%04b, expected 0b0011", spi_captured[15:12]);
        end else if (spi_captured[11:0] !== 12'hABC) begin
            errors = errors + 1;
            $display("[FAIL] T5: data = 0x%03x, expected 0xABC", spi_captured[11:0]);
        end else begin
            $display("[PASS] T5: frame = 0x%04x (header 0b0011, data 0x%03x), exactly 1 frame",
                     spi_captured, spi_captured[11:0]);
        end

        //----------------------------------------------------------------------
        // Result
        //----------------------------------------------------------------------
        $display("\n================================================================");
        if (errors == 0)
            $display(" RESULT: [PASS] all checkpoint assertions held");
        else
            $display(" RESULT: [FAIL] %0d assertion(s) failed -- do NOT proceed", errors);
        $display("================================================================\n");

        if (errors != 0) $fatal(1, "testbench assertions failed");
        $finish;
    end

    //--------------------------------------------------------------------------
    // Global timeout, so a deadlock fails loudly instead of hanging
    //--------------------------------------------------------------------------
    initial begin
        #120_000_000;   // 120 ms of simulated time
        $display("[FAIL] global timeout -- the design deadlocked");
        $fatal(1, "timeout");
    end

endmodule
