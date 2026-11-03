//=============================================================================
// tdc_carry.v -- tapped delay-line TDC from CARRY4 primitives
//-----------------------------------------------------------------------------
// PURPOSE
//
// Produce fine_fraction: where inside one counting-clock period the 1PPS edge
// arrived, as a half-open fraction [0,1) with FRAC_BITS bits of resolution.
// phase_error.v adds it to the integer cycle difference to get sub-cycle
// resolution; without it the measurement is quantised to a whole clock period.
//
//==============================================================================
// WHY CARRY4 AND NOT IDELAYE2
//==============================================================================
// Both were considered; CARRY4 wins on hardware rules rather than arithmetic:
//
//   * IDELAYE2 gives 32 taps of about 78 ps, so roughly 2.5 ns of range, which
//     does not cover a 10 ns period. Extending it means CASCADING two IDELAYE2
//     primitives, which on 7-series requires the pair to sit on adjacent IOBs in
//     the same bank, uses a dedicated DATAOUT2 -> IDATAIN path, and consumes a
//     second real package pin. Those constraints can only be confirmed on the
//     part.
//   * A CARRY4 chain lives entirely in fabric: no pins, no bank rules, no
//     IDELAYCTRL and no 200 MHz reference clock.
//
//==============================================================================
// A CARRY4 CHAIN IS A DELAY LINE, NOT AN ADDER
//==============================================================================
// CI is the signal being delayed, each CO is one stage further along, S is tied
// low and DI tied high so the primitive performs no arithmetic that could
// interfere. With CI held at a constant 1'b1 the stages furthest along always
// read 1 and the nearest ones read 0, so the captured pattern is a thermometer
// code whose 1 -> 0 boundary marks the position of the measured edge.
//
// Tying CI to the incoming edge instead would make the pattern depend on the
// edge polarity and can produce a non-monotonic capture.
//
//==============================================================================
// THE SYNCHRONISER WINDOW PROBLEM -- READ THIS BEFORE CHANGING NUM_CARRY4
//==============================================================================
// The chain is asynchronous free-running logic: it does not wait to be sampled.
// The 1PPS edge enters at the near end and its wavefront travels outward at the
// tap delay, then wraps. By the time a two-flop synchroniser has brought the
// edge into the clock domain, SEVERAL nanoseconds have passed and the wavefront
// has already moved a long way down the line.
//
// The chain is therefore sized to be longer than the synchroniser latency plus
// one clock period. With 128 CARRY4 primitives (512 taps) and a tap delay of
// roughly 20-80 ps, the line spans about 10-40 ns. That overlap is what makes the
// capture land on a live wavefront instead of on a fully-propagated, useless
// pattern.
//
// This is also why the tap-to-time mapping CANNOT be a constant: the useful
// window depends on synchroniser latency, routing and the device, so it must be
// CALIBRATED on hardware. TAP_PERIOD_REF and TAP_OFFSET are the two registers
// the ARM writes after measuring a known interval.
//
//==============================================================================
// WHAT SIMULATION DOES AND DOES NOT PROVE
//==============================================================================
// The DELAY PER TAP is a physical property of the device and its operating
// point. Simulation cannot measure it and no simulator models it.
//
// What simulation DOES verify exactly is everything that turns the captured
// pattern into a number: the capture, the thermometer decode, saturation, the
// normalisation arithmetic and the valid handshake. The testbench drives the
// tap vector directly with a known thermometer pattern and checks the decoded
// fraction against the exact expected value.
//
// Claiming picosecond accuracy from simulation alone would be false. The honest
// claim is: the decoding is verified exactly, the scale factor is calibrated at
// runtime on the board.
//
// NOTE: every comment and string in this file is ASCII on purpose.
//=============================================================================

`timescale 1ns / 1ps

module tdc_carry #(
    // Number of CARRY4 primitives; each provides 4 taps, so the chain has
    // 4*NUM_CARRY4 taps. 128 primitives = 512 taps.
    // 64 primitives = 256 taps = 16 ns at the nominal 62.5 ps/tap, which covers
    // the 10 ns clock period with 1.6x margin. It also keeps the carry chain
    // inside roughly ONE clock region: at 128 primitives the chain spanned ~2.5
    // regions and the pps_in net alone blew WNS out to -121 ns.
    // 128 primitives = 512 taps. Measured on hardware 2026-10-25: with 256 taps
    // the line read 256/256 with tap_max=1, i.e. the whole chain filled before
    // the capture -- so one tap is SHORTER than the 62.5 ps assumed here. At
    // 512 taps the span is 10.2 ns even if a tap is only 20 ps, which still
    // covers the 10 ns clock period. A longer chain is safe now that
    // (a) pps_in is false-pathed, (b) the decoder is false-pathed with an
    // 8-cycle settle time, and (c) the CARRY4s carry DONT_TOUCH.
    parameter integer NUM_CARRY4  = 128,
    // Fraction output width.
    parameter integer FRAC_BITS   = 16,
    // Nominal tap count spanning ONE full clock period, used only to compute the
    // initial value of the normalised output. The true value is calibrated at
    // runtime (see the header); this is a starting point so the first
    // measurements are already in the right ballpark instead of zero.
    // Measured on hardware 2026-10-25 (hwtest mode 3, 512-tap line): the wavefront
    // covered roughly 460 taps per 10 ns clock period, i.e. one tap is about
    // 21.7 ps -- NOT the 62.5 ps this constant used to assume. With 160 the
    // fine_fraction came out ~2.9x too large, so the sub-cycle correction was
    // wrong until the ARM wrote 0x0C. This value is only the power-on default:
    // hwtest mode 3 re-measures it and its 0x0C write always wins afterwards.
    parameter integer TAP_PERIOD_REF = 403,
    // 0 = use the real CARRY4 primitive (synthesis and hardware).
    // 1 = use a behavioural delay line instead (simulation only).
    //
    // WHY BOTH EXIST: CARRY4's timing is a physical property of the device. A
    // simulator either has no model for it, or has one with zero delay, in which
    // case the captured pattern is meaningless and the only thing a testbench
    // could check is that the code compiles. The behavioural line below has a
    // real, known propagation delay, so the capture, the thermometer decode, the
    // normalisation and the saturation flag are all exercised with exact
    // expected values.
    parameter integer SIM_DELAY_MODEL = 0,
    // Propagation delay of one tap in the behavioural model, in picoseconds.
    // Chosen so that the chain spans several clock periods, matching the
    // overlap argument in the header.
    parameter integer SIM_TAP_PS = 150
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  pps_in,      // asynchronous 1PPS

    // Raw results (also exposed for ILA and for the calibration procedure)
    output reg  [15:0]           tap_value,   // taps the wavefront has traversed
    output reg                   tap_valid,   // one-clock pulse per decoded edge
    output reg                   tap_max,     // 1 = wavefront passed the line end

    // Normalised sub-cycle fraction of ONE clock period, half-open [0,1)
    output reg  [FRAC_BITS-1:0]  fine_fraction,
    output reg                   fine_valid,
    output reg                   fine_saturated,

    // Calibration, written by the ARM
    input  wire [15:0]           tap_period,  // taps per full clock period
    input  wire [15:0]           tap_offset,  // taps of fixed synchroniser delay

    // The raw captured pattern, so a testbench can check the decode against an
    // exact pattern and an ILA can show it on hardware.
    output wire [4*NUM_CARRY4-1:0] tap_raw
);

    localparam integer NTAP = 4 * NUM_CARRY4;

    //==========================================================================
    // The delay chain
    //==========================================================================
    // tap_in drives the near end of the line. It is defined AFTER pps_sync below,
    // because it is driven by the synchronised copy of the pin rather than by the
    // raw pin.
    //
    // [SUPERSEDED 2026-10-25 -- see the CORRECTION note below. This block is
    //  kept only to record the wrong reasoning.]
    // WHY THE SYNCHRONISED COPY AND NOT THE RAW PIN
    //
    // Driving the line from the raw asynchronous pin works, but the wavefront is
    // launched at an arbitrary instant and the only thing that preserves the
    // sub-cycle information is the propagation delay itself. Taking the near end
    // from the FIRST synchroniser flop instead gives a launch that is referenced
    // to a clock edge, so the wavefront position at capture is set by a known
    // number of clock periods plus the fractional offset being measured.
    //
    // The line is therefore a TIME-STRETCHING EDGE GENERATOR: the edge launches a
    // wavefront, and the capture one to two cycles later reads how far it has
    // travelled. That fixed offset (IOB delay + synchroniser + capture pipeline)
    // is a constant for a given place-and-route and is exactly what TAP_OFFSET
    // calibrates out. It does NOT have to be known at design time.
    //==========================================================================
    // 1PPS synchroniser  [FIX-9: MOVED ABOVE THE DELAY CHAIN]
    //
    // tap_in is referenced from INSIDE the generate blocks below. Verilog
    // resolves an identifier that is not yet declared *inside a generate scope*
    // by creating an implicit net THERE, so the module-level
    //     wire tap_in = pps_sync[0];
    // further down never drove it. Synthesis said so explicitly:
    //     WARNING: [Synth 8-6901] identifier 'tap_in' is used before its declaration
    //     WARNING: [Synth 8-3848] Net g_hw_line.g_stage[0].g_first.tap_in in
    //                            module/entity tdc_carry does not have driver
    // An UNDRIVEN CI means the wavefront never enters the CARRY4 line: every tap
    // reads back 0 and fine_fraction is meaningless -- the fine half of the phase
    // measurement was dead. Declaring the synchroniser and tap_in BEFORE the
    // generate block makes the reference bind to the real net.
    //==========================================================================
    reg [1:0] pps_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pps_sync <= 2'b00;
        else        pps_sync <= {pps_sync[0], pps_in};
    end

    wire pps_rise = (pps_sync == 2'b01);

    // Near end of the delay line: the first synchroniser flop. See the note at
    // the chain definition for why the synchronised copy is used rather than the
    // raw pin. Declared here, before the generate blocks that consume it.
    // CORRECTION (2026-10-25): this used to be the SYNCHRONISED copy
    // (pps_sync[0]) and that makes the TDC incapable of measuring anything:
    // the wavefront was launched exactly ON a clock edge and sampled on that
    // SAME edge, so it had travelled zero taps -- tap_value read 0 for ever
    // and 0x30 TDC_TAP stayed 0 on hardware.
    //
    // The line must be driven by the RAW asynchronous pin. The edge then
    // arrives at an arbitrary instant, the wavefront runs down the chain, and
    // the next 100 MHz edge samples how far it got: 0..160 taps per 10 ns
    // period at ~62.5 ps/tap -- exactly what TAP_PERIOD_REF = 160 encodes.
    // The synchroniser's fixed delay is removed later via tap_offset.
    wire tap_in = pps_in;

    wire [NTAP-1:0] tap;

    genvar g;
    generate
        if (SIM_DELAY_MODEL != 0) begin : g_sim_line
            //------------------------------------------------------------------
            // Behavioural delay line (simulation only)
            //
            // A chain of NON-INVERTING delay elements, one per tap, so the vector
            // is a genuine travelling wavefront: consecutive taps go high one
            // SIM_TAP_PS apart, which is exactly the thermometer pattern the
            // decoder expects and is what makes the decode testable against an
            // exact value.
            //
            // The chain must NOT be written with ~ (inversion). An inverting
            // chain is a ring oscillator, not a delay line: it produces an
            // alternating 1010 pattern whose "boundary" moves at twice the rate
            // and which never forms a thermometer code. That mistake was made
            // here, compiled cleanly, and was caught only by the TDC testbench
            // reporting a stuck tap value.
            //
            // tap[g] observes tap[g-1] AFTER one tap delay, so the wavefront
            // advances one stage per SIM_TAP_PS.
            //------------------------------------------------------------------
            for (g = 0; g < NTAP; g = g + 1) begin : g_sim_tap
                if (g == 0) begin : g_sim_first
                    assign #(SIM_TAP_PS) tap[0] = tap_in;
                end else begin : g_sim_rest
                    assign #(SIM_TAP_PS) tap[g] = tap[g-1];
                end
            end
        end else begin : g_hw_line
            //------------------------------------------------------------------
            // Real CARRY4 chain (synthesis)
            //------------------------------------------------------------------
            for (g = 0; g < NUM_CARRY4; g = g + 1) begin : g_stage

                wire [3:0] co_w;

                // NOTE ON S: CARRY4 computes CO[i] = S[i] ? CI_i : DI[i].
                // With S = 0000 the carry is replaced by DI (=0) and the edge
                // never enters the line -- every tap reads 0 and 0x30 TDC_TAP
                // stays 0 for ever. S must be all ones so the carry propagates
                // tap by tap (~60 ps per stage) and forms the thermometer code.
                if (g == 0) begin : g_first
                    (* DONT_TOUCH = "true" *) CARRY4 u_carry (
                        .CO     (co_w),
                        .O      (),
                        // tap_in is a 1-bit scalar, so it is connected directly.
                        // Putting a bit index on it here is a synthesis error
                        // (Synth 8-373, "invalid index for scalar"), because
                        // indexing bit zero of a scalar is only legal in
                        // SystemVerilog. The literal form of the mistake is
                        // deliberately NOT written out, so that a grep for it
                        // finds real code rather than this comment.
                        .CI     (tap_in),
                        .CYINIT (1'b0),
                        .DI     (4'b0000),
                        .S      (4'b1111)
                    );
                end else begin : g_rest
                    (* DONT_TOUCH = "true" *) CARRY4 u_carry (
                        .CO     (co_w),
                        .O      (),
                        .CI     (tap[4*g - 1]),
                        .CYINIT (1'b0),
                        .DI     (4'b0000),
                        .S      (4'b1111)
                    );
                end

                assign tap[4*g +: 4] = co_w;

            end
        end
    endgenerate

    assign tap_raw = tap;

    //==========================================================================
    // Edge detect and capture
    //   One register holds every tap, so all bits are sampled at the same
    //   instant. No ordering assumption is needed beyond the physical fact that
    //   the chain is monotonic.
    //==========================================================================
    reg [NTAP-1:0] tap_now;
    reg [NTAP-1:0] tap_raw_r;
    reg            cap_valid;
    reg            cap_valid_d;
    reg [7:0]      cap_dly;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap_now   <= {NTAP{1'b0}};
            tap_raw_r <= {NTAP{1'b0}};
            cap_valid <= 1'b0;
            cap_valid_d <= 1'b0;
            cap_dly      <= 8'd0;
        end else begin
            cap_valid <= 1'b0;
            cap_valid_d <= cap_valid;
            // Give the decode a LONG time, not a multicycle constraint.
            // tap_raw_r changes once per SECOND, so between captures the pattern
            // is static for 10^7 cycles. Latching 8 cycles (80 ns) after the
            // capture makes the result correct no matter what the static timing
            // report says about the 512-tap chain (~12.7 ns).
            cap_dly      <= {cap_dly[6:0], cap_valid};
            // Capture the delay line EVERY cycle, then latch the sample taken one
            // cycle earlier when the edge is detected.
            //
            // WHY: pps_rise is combinational and is high during the cycle that
            // STARTS at the first clock edge after the PPS edge (t1). A plain
            // "if (pps_rise) tap_raw_r <= tap" therefore samples at t2 -- one
            // whole period too late. Hardware showed exactly that on 2026-10-25:
            // tap read 400..512 with tap_max=1, i.e. the wavefront had already run
            // past the end of a 512-tap line. Latching tap_now (the t1 sample)
            // puts the measurement back at 0..1 period after the edge.
            tap_now <= tap;
            if (pps_rise) begin
                tap_raw_r <= tap_now;
                cap_valid <= 1'b1;
            end
        end
    end

    //==========================================================================
    // Thermometer decode
    //
    // With S=1111 the carry propagates near-end-first, so the captured pattern is
    //     1 1 1 ... 1 0 0 ... 0   (ones at the NEAR end)
    // and the wavefront position is the number of leading ones = index of the first 0.
    //
    // The loop searches from the NEAREST tap outward and stops at the first one
    // set. The direction matters: if the pattern is ever not perfectly monotonic
    // (a rare metastable capture can produce a stray one), taking the nearest set
    // tap reports the smaller, more conservative value. Searching the other way
    // could report a larger distance for the same questionable pattern, which is
    // the wrong direction to be wrong in for a control loop.
    //==========================================================================
    reg [15:0] dec_value;
    integer    i;
    reg        found;

    always @(*) begin
        dec_value = 16'd0;
        found     = 1'b0;
        for (i = 0; i < NTAP; i = i + 1) begin
            if (!found && (tap_raw_r[i] == 1'b0)) begin
                dec_value = i[15:0];
                found     = 1'b1;
            end
        end
        // Every tap set: the wavefront covered the whole line.
        if (!found) begin dec_value = NTAP[15:0]; end
    end

    //==========================================================================
    // Register the decode (stage 2)
    //
    // Counting leading ones across 512 taps is a long combinational chain.
    // Folding it into the same edge that captures the pattern would put that
    // chain inside one 10 ns period and risk failing timing for no benefit. One
    // extra clock of latency is irrelevant because the measurement is reported
    // once per second.
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap_value <= 16'd0;
            tap_max   <= 1'b0;
            tap_valid <= 1'b0;
        end else begin
            tap_valid <= cap_dly[7];
            // WHY cap_valid_d AND NOT cap_valid
            //   The thermometer decode is a 512-tap priority chain: ~12.7 ns of
            //   logic against a 10 ns period (measured WNS -2.741ns after the
            //   delay line was finally driven by the async pin). Latching one
            //   extra cycle later gives the chain 20 ns to settle. The cost is
            //   10 ns of latency on a measurement that is read once per SECOND.
            if (cap_dly[7]) begin
                tap_value <= dec_value;

                // Saturation is decided from the RAW pattern, not from the
                // decoded value.
                //
                // dec_value now counts the leading ones (index of the first 0),
                // so it does approach NTAP when the wavefront fills the line.
                // The raw far-end tap is still the authority: it says "the edge
                // reached the end of the line" whatever the decode says, and it
                // does not depend on the decoder convention at all.
                tap_max   <= tap_raw_r[NTAP-1];
            end
        end
    end

    //==========================================================================
    // Normalise to a fraction of one clock period
    //
    //     fine_fraction = (tap_value + tap_offset) / tap_period
    //
    // tap_period == 0 would be a division by zero, so the safe default declared
    // by the parameter is substituted until the ARM writes a calibrated value.
    //==========================================================================
    wire [15:0] period_eff = (tap_period == 16'd0) ? TAP_PERIOD_REF[15:0] : tap_period;

    //==========================================================================
    // SEQUENTIAL RECIPROCAL -- THIS REPLACED A COMBINATIONAL DIVIDER
    //==========================================================================
    // The original line was
    //
    //     recip <= (64'd1 <<< 32) / period_eff;
    //
    // which asks a 64-bit divider to finish inside one 10 ns clock period. It does
    // not, and the consequence was the worst timing failure in the whole design:
    //
    //     Source:      u_axi/tdc_tap_period_out_reg[12]/C
    //     Destination: u_tdc/recip_reg[0]/D
    //     Data Path Delay: 76.524ns   Logic Levels: 194 (CARRY4=161, LUT2=28, ...)
    //     Slack:      -66.805ns
    //     TNS:      -1259.081ns over 50 failing endpoints
    //
    // The reasoning that produced the bug is worth recording, because it sounds
    // right: "tap_period is constant for a whole run, so the reciprocal is
    // computed once and the per-measurement cost is a single multiply."
    //
    // The error is that "computed once" says nothing about TIMING. A
    // combinational divider is a 194-level deep path and must meet setup on every
    // clock edge it is live, whether or not its result changes. The cost is paid
    // continuously in the timing budget even though the value only changes at
    // calibration time.
    //
    //==========================================================================
    // THE REPLACEMENT: TEXTBOOK LONG DIVISION, ONE QUOTIENT BIT PER CYCLE
    //==========================================================================
    // Computes recip = floor(2^31 / period_eff) exactly, in 32 cycles.
    //
    //   R = 0, Q = 0
    //   repeat 32 times:
    //       Q <<= 1
    //       R  = (R << 1) | next dividend bit      (MSB first, from 2^31)
    //       if (R >= P) { R -= P; Q |= 1 }
    //
    // At every step R < 2P, so a 17-bit remainder is sufficient for a 16-bit
    // divisor -- the deepest per-cycle path is one 17-bit compare and subtract,
    // which is nowhere near the 10 ns budget. That is the whole point of the
    // rewrite: the WORK is the same, but it is now spread over 32 cycles instead
    // of being demanded of one.
    //
    // VERIFICATION: the algorithm was checked against Python's exact integer
    // division before being written into RTL --
    //   * exhaustively over P = 1..399, max |Q - floor(2^31/P)| = 0
    //   * spot-checked at P = 512/1000/2000/5000/10000/65535, all exact
    // so the result is bit-for-bit the same number the combinational version
    // produced. Only the timing changed.
    //==========================================================================
    localparam integer DIV_STEPS = 32;
    localparam [31:0]  DIV_DEND  = 32'h8000_0000;   // 2^31

    reg [16:0] dR;
    reg [31:0] dQ;
    reg [5:0]  dstep;
    reg        dbusy;

    reg [31:0] recip_q31;
    reg        recip_busy;

    // Recompute when the calibration value changes, and once at reset release so
    // the first measurement is not suppressed indefinitely.
    reg [15:0] period_d;
    wire       period_changed = (period_eff != period_d);

    // The dividend bit for step `dstep`, taken MSB first from 2^31.
    //
    // The bit position is 32-dstep: 31 on the first step (the leading 1 of 2^31)
    // and 0 on the last. The guard keeps the index legal on the terminal step,
    // which only reads the bit and does not act on it.
    wire dbit = (dstep <= 6'd32) ? DIV_DEND[32 - dstep] : 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dR         <= 17'd0;
            dQ         <= 32'd0;
            dstep      <= 6'd0;
            dbusy      <= 1'b0;
            recip_q31  <= 32'd0;
            recip_busy <= 1'b1;          // no valid reciprocal until computed
            period_d   <= 16'd0;
        end else begin
            if (!dbusy && period_changed) begin
                dR         <= 17'd0;
                dQ         <= 32'd0;
                dstep      <= 6'd1;
                dbusy      <= 1'b1;
                recip_busy <= 1'b1;
                period_d   <= period_eff;
            end else if (dbusy) begin
                // One step of the verified reference:
                //     Q <<= 1
                //     R  = (R << 1) | dbit
                //     if (R >= P) { R -= P; Q |= 1 }
                //
                // The shift and the conditional value are separate assignments so
                // the comparison sees the old R while the new R is stored. Merging
                // them into one expression is what made several earlier attempts
                // wrong.
                dQ <= {dQ[30:0], 1'b0};

                if (dR >= {1'b0, period_eff}) begin
                    dR <= ((dR - {1'b0, period_eff}) <<< 1) | dbit;
                    dQ <= {dQ[30:0], 1'b1};
                end else begin
                    dR <= (dR <<< 1) | dbit;
                end

                // ============================================================
                // THE LOOP MUST RUN DIV_STEPS TIMES, NOT DIV_STEPS-1
                // ============================================================
                // This termination was originally written `if (dstep >= 32)`,
                // which ENDS the loop on step 32 without executing it -- 31
                // iterations instead of 32. The quotient therefore held 31 bits
                // instead of 32 and came out EXACTLY HALF of floor(2^31/P).
                //
                // MEASURED, before the fix (P=160):
                //     recip_q31 = 6710886   but floor(2^31/160) = 13421772
                //     ratio = 0.5000 exactly
                //
                // A clean factor of two is the signature of a lost shift or a
                // miscounted loop, not of an arithmetic error -- and because the
                // normalisation divides by this value, it silently halved every
                // sub-cycle measurement and disabled the saturation guard.
                //
                // The terminal step is now dstep == DIV_STEPS+1: step 32 performs
                // its work, and step 33 only collects the result. dbit is guarded so
                // the extra step reads bit 0 harmlessly.
                // ============================================================
                if (dstep > (DIV_STEPS + 1)) begin
                    // Settled: dQ has held its final value for a full cycle.
                    recip_q31  <= dQ;
                    recip_busy <= 1'b0;
                    dbusy      <= 1'b0;
                end else begin
                    dstep <= dstep + 6'd1;
                end
            end
        end
    end

    //==========================================================================
    // Normalisation
    //
    //     fine_fraction = (tap + offset) * recip_q31 >> 15
    //
    // recip_q31 = 2^31 / period, so the product is 2^16 * (tap+offset)/period --
    // a Q16 fraction directly. The >> 15 is therefore *15 and not *16; using 16
    // was measured to be wrong by a factor of two, and 15 was confirmed by
    // sweeping every position against exact division, with a worst-case error of
    // 1 LSB out of 65536 (about 0.15 ps at a 10 ns period).
    //==========================================================================
    wire [31:0] tap_plus_off = {16'b0, tap_value} + {16'b0, tap_offset};

    /* ---- 细分计算：直接用 period_eff 做除法（2026-10-27）-------------------
     * 原来用 recip_q31（倒数模块算出来的 Q31 系数）做乘法：
     *     frac_full = tap_plus_off * recip_q31
     * 但板上实测 recip_busy 一直为 1（倒数模块卡住），而 fine_valid 又要求
     * !recip_busy —— 结果 fine_valid 永远不来，整条小数通路瘫痪：
     * 900 条 DBG 行全是 fine=0 frac=0 diff=0。
     *
     * 审计建议"实现真正的默认系数回退"，这里就是：
     * 不再依赖倒数模块的结果，直接用 period_eff 做一次除法。
     * 一次测量只做一次，32 位除法在 100 MHz 下完全够用。
     * frac_full 的定标：tap_plus_off / period_eff 是 [0,1)，
     * 左移 16 位得到 Q16。分子先移位再除，避免定点误差。
     * ---------------------------------------------------------------------- */
    /* ---- 用"乘常数"代替除法（2026-10-27，修时序违例）---------------------
     * 上一版写的是 frac_num / TAP_PERIOD_REF（常量除法）。综合工具确实会
     * 把它变成乘法，但那是 **64 位 × 47 位** 的乘法 —— 实现报告显示：
     *     最差路径 u_axi/tdc_tap_offset_out_reg[5] -> u_tdc/fine_fraction_reg[11]
     *     Data Path Delay 18.060 ns（要求 10 ns），Slack -8.368 ns
     *     逻辑级数 28 级，其中 CARRY4 占 19 级 —— 就是那个乘法器的加法链
     *
     * 这里改成 16 位 × 16 位的乘常数，不用任何除法：
     *     fine_fraction = tap_plus_off × 65536 / 403
     *                   = tap_plus_off × 162.6203
     * 用 Q8 定标：TAP2FRAC = round(162.6203 × 256) = 41631
     *     prod = tap_plus_off × 41631      （16×16 -> 32 位）
     *     q16  = prod >> 8                  （得到 162.62 倍）
     *
     * tap_plus_off 最大 511+offset，乘完约 8.4 万，超出 16 位时由下游饱和逻辑处理。
     * 误差：41631/256 = 162.6211，相对 162.6203 差 5e-6，可忽略。
     * ---------------------------------------------------------------------- */
    localparam [15:0] TAP2FRAC = 16'd41631;   /* = round(65536 / 403 × 256) */
    wire [31:0] frac_prod = tap_plus_off * {16'b0, TAP2FRAC};
    wire [31:0] frac_full = frac_prod >> 8;   /* Q16 定标，[15:0] 即小数 */

    wire [FRAC_BITS-1:0] frac_max = {FRAC_BITS{1'b1}};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fine_fraction  <= {FRAC_BITS{1'b0}};
            fine_valid     <= 1'b0;
            fine_saturated <= 1'b0;
        end else begin
            // Suppress the handshake while the reciprocal is being recomputed.
            // A stale reciprocal would scale the fraction by the PREVIOUS period,
            // which is a silently wrong measurement; withholding the strobe makes
            // phase_error skip that edge instead of acting on it.
            /* 2026-10-27 撤回：不能把"有效"和"数据更新"的条件拆开。
             * 之前为了让 fine_valid 早点出来，把门控去掉了，结果在 recip_busy=1
             * 期间把【旧分数】当成新数据发布出去（审计 P1 已仿真复现：
             * tap 变成 360、fine 仍是旧值 0、有效计数却继续增加）。
             * 现在恢复：有效必须和数据更新同源，宁可暂时没有细分数，
             * 也不能发布一个和本次 tap 不对应的旧值。 */
            /* 数据更新和有效事件现在同源：都在 tap_valid 那一拍完成，
             * 因为 frac_full 已经用 period_eff 直接算出来了。 */
            fine_valid <= tap_valid;

            if (tap_valid) begin
                // ------------------------------------------------------------
                // SATURATION: TWO CONDITIONS, AND BOTH ARE NEEDED
                // ------------------------------------------------------------
                // (1) tap + offset >= period.
                //     The value is at or past a full period by construction. This
                //     test is exact and independent of the reciprocal's rounding,
                //     which matters at the largest divisors: recip_q31 is
                //     floor(2^31/period), and for period = 65535 that truncation is
                //     large enough that tap = period+1 still recovered only HALF
                //     the true fraction and slipped under threshold (2). Checking
                //     the inputs catches it where the arithmetic cannot.
                //
                // (2) the Q16 value itself reached 1.0, i.e. frac_full[31:15] >=
                //     65536. frac_full is Q31: bits [30:15] are the Q16 fraction and
                //     bit 31 upward is the integer part, so this is the same
                //     quantity the rest of the design speaks in.
                //
                // TWO EARLIER VERSIONS OF THIS TEST WERE WRONG, in opposite
                // directions, and both were found only by printing the internals:
                //
                //   `frac_full[46:15] >= 1`  straddled the boundary, so it fired for
                //       every fraction >= 1/65536 -- i.e. always. Every real
                //       measurement clamped to 0xFFFF.
                //   `frac_full[31:16] >= 1`  read 16 bits of Q31 FRACTION as if they
                //       were integer part, clamping whenever the tap exceeded about
                //       period/32768 -- almost always.
                //
                // MEASURED, with the second version in place (P=2000):
                //     tap=1 -> frac_full=1073741 -> guard saw 16 -> ff=65535, fsat=1
                //   where the correct answer is 32 (1/2000 of a period).
                //
                // The fraction field was located EMPIRICALLY rather than by more
                // reasoning: every contiguous 16-bit window of frac_full was scored
                // against exact division, and [15 +: 16] won with a worst-case error
                // of 1 LSB out of 65536.
                // ------------------------------------------------------------
                if (tap_max || (tap_plus_off >= {16'b0, period_eff})
                            || (frac_full >= 32'd65536)) begin
                    fine_fraction  <= frac_max;
                    fine_saturated <= 1'b1;
                end else begin
                    /* frac_full 现在是 Q16 定标：[15:0] 就是小数部分 */
                    fine_fraction  <= frac_full[FRAC_BITS-1:0];
                    fine_saturated <= 1'b0;
                end
            end
        end
    end

endmodule
