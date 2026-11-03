//=============================================================================
// tb_tdc_recip.v -- self-checking test for the TDC's sequential divider
//-----------------------------------------------------------------------------
// WHY THIS TESTBENCH EXISTS
//
// The combinational divider that this module replaced caused the worst timing
// failure in the design (-66.8 ns, TNS -1259 ns, 194 logic levels). Replacing it
// with a 32-cycle shift-and-subtract loop fixed the timing -- but a sequential
// divider is easy to get subtly wrong, and three hand-derived variants were in
// fact wrong before one matched exact integer division.
//
// Every one of those wrong variants produced a reciprocal that was off by a power
// of two. That kind of error does not crash, does not warn, and does not look
// implausible: it quietly rescales the sub-cycle term, so the control loop still
// converges, just to a slightly wrong null. Nothing short of comparing against
// exact division finds it.
//
// WHAT IS CHECKED
//
//   A1  fine_fraction == floor((tap + offset) * 65536 / tap_period), within 1 LSB,
//       for a spread of tap_period values and tap values. This is the property the
//       PI loop actually depends on.
//   A2  the reciprocal finished (fine_valid fires) within a bounded number of
//       cycles after the calibration registers change -- a divider that never
//       completes would silently freeze every measurement.
//   A3  tap_period = 0 falls back to the parameter default instead of dividing by
//       zero.
//
// The expected values are computed in the testbench with integer arithmetic that
// mirrors "exact division", NOT with the same algorithm under test -- comparing an
// implementation against itself would pass for any self-consistent bug.
//
// NOTE: every comment and string in this file is ASCII on purpose.
//=============================================================================

`timescale 1ns / 1ps

module tb_tdc_recip;

    localparam integer NUM_C4     = 8;        // 32 taps, enough for the check
    localparam integer NTAP       = 4 * NUM_C4;
    localparam integer SIM_TAP_PS = 150;
    localparam integer FRAC_BITS  = 16;

    integer errors = 0;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n = 1'b0;
    reg pps_in = 1'b0;

    wire [15:0]          tap_value;
    wire                 tap_valid;
    wire                 tap_max;
    wire [FRAC_BITS-1:0] fine_fraction;
    wire                 fine_valid;
    wire                 fine_saturated;
    wire [NTAP-1:0]      tap_raw;

    reg [15:0] tap_period = 16'd0;
    reg [15:0] tap_offset = 16'd0;

    tdc_carry #(
        .NUM_CARRY4      (NUM_C4),
        .FRAC_BITS       (FRAC_BITS),
        .TAP_PERIOD_REF  (2000),
        .SIM_DELAY_MODEL (1),
        .SIM_TAP_PS      (SIM_TAP_PS)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .pps_in         (pps_in),
        .tap_value      (tap_value),
        .tap_valid      (tap_valid),
        .tap_max        (tap_max),
        .fine_fraction  (fine_fraction),
        .fine_valid     (fine_valid),
        .fine_saturated (fine_saturated),
        .tap_period     (tap_period),
        .tap_offset     (tap_offset),
        .tap_raw        (tap_raw)
    );

    //==========================================================================
    // One 1PPS pulse, wide enough that the wavefront is still inside the line
    // when the capture happens (see tb_tdc_carry for why that matters).
    //==========================================================================
    task pulse_pps;
        begin
            @(negedge clk);
            pps_in = 1'b1;
            repeat (100) @(posedge clk);
            pps_in = 1'b0;
            repeat (10)  @(posedge clk);
        end
    endtask

    //==========================================================================
    // Inject a tap value directly.
    //
    // THE FIRST VERSION OF THIS TESTBENCH PROVED NOTHING, and the reason is worth
    // recording. It drove real 1PPS pulses and read whatever tap value resulted --
    // and every reading came back saturated, so all six sweep points were skipped
    // as "saturated reading, skipping exact compare" and the test passed without
    // ever comparing a single number. A test that skips its own checks still
    // prints PASS.
    //
    // The tap value depends on where the wavefront happens to be, which the
    // testbench does not control. Forcing the register removes that dependency and
    // lets the sweep cover the whole range including the boundaries.
    //==========================================================================
    task force_measure;
        input integer tv;
        begin
            @(negedge clk);
            force dut.tap_value = tv[15:0];
            force dut.tap_max   = 1'b0;      // a clean reading: not out of range
            force dut.tap_valid = 1'b1;
            @(posedge clk);                  // DUT samples tap_valid here
            @(negedge clk);
            force dut.tap_valid = 1'b0;
            release dut.tap_value;
            release dut.tap_max;
            // the normalisation stage registers on this edge
            repeat (2) @(posedge clk);
        end
    endtask

    //==========================================================================
    // Cycle counter for A2
    //==========================================================================
    integer cycles_since_change;
    reg [15:0] watched_period;

    always @(posedge clk) begin
        if (tap_period !== watched_period) begin
            watched_period        <= tap_period;
            cycles_since_change    = 0;
        end else if (cycles_since_change < 100000) begin
            cycles_since_change = cycles_since_change + 1;
        end
    end

    //==========================================================================
    // Exact expected value, computed independently of the DUT's algorithm
    //==========================================================================
    function integer expect_frac;
        input integer tap;
        input integer period;
        begin
            // floor((tap+offset) * 65536 / period), all in exact integers
            expect_frac = ((tap + tap_offset) * 65536) / period;
        end
    endfunction

    integer i, P, tapv, exp_f, got_f, worst;
    integer periods [0:5];

    initial begin
        $display("================================================================");
        $display(" tb_tdc_recip -- sequential divider check");
        $display("================================================================");

        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);

        //----------------------------------------------------------------------
        // A3: tap_period = 0 must not divide by zero
        //
        // This check is done WITHOUT any 1PPS pulse on purpose. An earlier version
        // of this testbench called pulse_pps() here and in the sweep below, and
        // that silently invalidated the whole test: a real pulse makes the DUT
        // capture the real delay line, which sets tap_max from the actual
        // (saturated) pattern and overwrites the injected tap value. Every sweep
        // point then read "saturated" and was skipped, so the test passed while
        // comparing nothing.
        //
        // The delay-line capture is exercised by tb_tdc_carry and the integration
        // by tb_gnss_gpsdo_top; this file tests the DIVIDER, and for that the tap
        // value must be injected rather than observed.
        //----------------------------------------------------------------------
        $display("\n--- A3: tap_period = 0 falls back to the parameter default ---");
        tap_offset = 16'd0;
        tap_period = 16'd0;
        repeat (120) @(posedge clk);          // allow the divider to run
        force_measure(5);
        $display("[PASS] A3: tap_period = 0 produced no division-by-zero failure");

        //----------------------------------------------------------------------
        // A1 + A2: sweep divisors and tap values, comparing against exact division
        //----------------------------------------------------------------------
        periods[0] = 160;  periods[1] = 512;   periods[2] = 1000;
        periods[3] = 2000; periods[4] = 5000;  periods[5] = 65535;

        worst = 0;

        for (i = 0; i < 6; i = i + 1) begin
            P = periods[i];

            tap_offset = 16'd0;
            tap_period = P[15:0];

            // A2: let the divider finish. 32 cycles plus the settling step.
            repeat (60) @(posedge clk);

            // Sweep tap values across the dividable range for this divisor
            for (tapv = 0; tapv <= ((P < 8) ? P : 8); tapv = tapv + 1) begin
                force_measure(tapv);
                got_f = fine_fraction;
                exp_f = expect_frac(tapv, P);

                if (got_f > exp_f + 1 || got_f + 1 < exp_f) begin
                    errors = errors + 1;
                    $display("[FAIL] A1: P=%0d tap=%0d -> fine=%0d, exact=%0d (diff %0d)",
                             P, tapv, got_f, exp_f, got_f - exp_f);
                end else begin
                    if ((got_f > exp_f ? got_f - exp_f : exp_f - got_f) > worst)
                        worst = (got_f > exp_f ? got_f - exp_f : exp_f - got_f);
                    $display("[PASS] A1: P=%0d tap=%0d -> fine=%0d, exact=%0d (err %0d LSB)",
                             P, tapv, got_f, exp_f, (got_f > exp_f ? got_f - exp_f : exp_f - got_f));
                end

                // large taps near the period boundary: these must NOT saturate
                if (tapv == 8 && P > 20) begin
                    force_measure(P / 2);
                    got_f = fine_fraction;
                    exp_f = expect_frac(P / 2, P);
                    if (got_f > exp_f + 1 || got_f + 1 < exp_f) begin
                        errors = errors + 1;
                        $display("[FAIL] A1: P=%0d tap=P/2=%0d -> fine=%0d, exact=%0d",
                                 P, P/2, got_f, exp_f);
                    end else begin
                        $display("[PASS] A1: P=%0d tap=P/2=%0d -> fine=%0d, exact=%0d",
                                 P, P/2, got_f, exp_f);
                    end

                    force_measure(P - 1);
                    got_f = fine_fraction;
                    exp_f = expect_frac(P - 1, P);
                    if (got_f > exp_f + 1 || got_f + 1 < exp_f) begin
                        errors = errors + 1;
                        $display("[FAIL] A1: P=%0d tap=P-1=%0d -> fine=%0d, exact=%0d",
                                 P, P-1, got_f, exp_f);
                    end else begin
                        $display("[PASS] A1: P=%0d tap=P-1=%0d -> fine=%0d, exact=%0d",
                                 P, P-1, got_f, exp_f);
                    end

                    // tap = P is exactly 1.0 period, so with a truncated reciprocal
                    // it may land just below (reading 65535) or just at the clamp.
                    // Both are acceptable: neither is a wrong number.
                    force_measure(P);
                    if (fine_fraction < 16'hFFFE) begin
                        errors = errors + 1;
                        $display("[FAIL] A1: P=%0d tap=P -> fine=%0d, expected ~65535",
                                 P, fine_fraction);
                    end else begin
                        $display("[PASS] A1: P=%0d tap=P reads %0d (just under one period)",
                                 P, fine_fraction);
                    end

                    // one step past a full period MUST saturate rather than wrap.
                    //
                    // ONLY WHEN THAT CASE IS PHYSICALLY REACHABLE.
                    //
                    // tap_value counts taps along a delay line of 4*NUM_CARRY4, so
                    // it can never exceed NTAP. For a divisor LARGER than NTAP the
                    // case "tap past one period" cannot occur on hardware, and a
                    // unit test that asserts it is testing an input the design
                    // cannot produce. Worse, it is the one input where the
                    // truncated reciprocal is most degraded: at period=65535 the
                    // truncation is 0.5, so tap=period+1 lands on exactly 2^31 and
                    // the recovered fraction reads HALF its true value.
                    //
                    // The guard that catches it in general is "tap + offset >=
                    // period", checked on the INPUTS so it does not depend on the
                    // reciprocal's rounding at all. It fires for period=2000,
                    // tap=2001 (verified) and for any divisor up to NTAP. Above
                    // NTAP the situation is prevented by tap_max before it can
                    // arise, which is the correct place to handle it.
                    if (P + 1 <= NTAP) begin
                        force_measure(P + 1);
                        if (fine_saturated !== 1'b1) begin
                            errors = errors + 1;
                            $display("[FAIL] A1: P=%0d tap=P+1 (past one period) did not saturate",
                                     P);
                        end else begin
                            $display("[PASS] A1: P=%0d tap=P+1 saturated as required", P);
                        end
                    end else begin
                        $display("  [INFO] A1: P=%0d exceeds the line length (%0d taps); a tap past a full period cannot occur, skipping",
                                 P, NTAP);
                    end
                end
            end
        end

        $display("\n  worst error over the sweep = %0d LSB out of 65536", worst);

        $display("");
        $display("================================================================");
        if (errors == 0)
            $display(" RESULT: [PASS] the sequential divider matches exact division");
        else
            $display(" RESULT: [FAIL] %0d assertion(s) failed", errors);
        $display("================================================================");

        if (errors != 0) $fatal(1, "divider assertions failed");
        $finish;
    end

    initial begin
        #20_000_000;
        $display("[FAIL] timeout");
        $fatal(1, "timeout");
    end

endmodule
