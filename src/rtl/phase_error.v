//=============================================================================
// phase_error.v -- turn the OCXO count into the loop's error signal
//-----------------------------------------------------------------------------
// THE FORMULA
//
//     phase_error = (ocxo_count - NOMINAL_OCXO) + (fine_prev - fine_now) * OCXO_HZ/PS_CLK_HZ
//
//     ocxo_count  : OCXO 20 MHz edges counted in one GNSS 1PPS interval,
//                   as reported by counter_measure.v. Ideally 20_000_000.
//     fine_now    : sub-cycle phase of THIS 1PPS edge, [0,1) of a reference
//                   clock period, from tdc_carry.v.
//     fine_prev   : the same quantity for the PREVIOUS edge, held here.
//
// SIGN CONVENTION -- SAME SIGN AS THE COUNTING MODULE (统一约定, 2026-10)
//
//     正 = 本地 OCXO 快
//     负 = 本地 OCXO 慢
//
// A fast oscillator produces MORE edges than nominal, so (ocxo_count -
// NOMINAL_OCXO) is POSITIVE for a fast oscillator, and the sub-cycle term has
// the matching polarity (fine_prev - fine_now).
//
// The ARM's integrator is written against this same convention:
//     X[1] > 0  =>  本地 OCXO 快  =>  dac_acc -= X[1]/DAC_SCALE  (调低 DAC)
// which converges iff the OCXO sensitivity
//     g = df/d(dac)  is POSITIVE  (抬高 DAC 让晶振变快)
// with 0 < g/DAC_SCALE < 2, and g == DAC_SCALE giving deadbeat.
//
// 这样 0x00 PHASE_ERR 和 0x18 OCXO_DELTA 符号一致，全系统只有一个约定：
//     正 = 快。模块之间不再有任何取反。
//
// This is the single most dangerous line in the design. An inverted loop does not
// fail to run; it runs away, driving the oscillator to the rail. The testbench
// asserts the sign explicitly for that reason.
//
//==============================================================================
// THE TWO TERMS ARE WILDLY DIFFERENT IN SIZE -- AND THE FINE TERM STILL MATTERS
//==============================================================================
// One OCXO count is 50 ns (20 MHz); one reference clock period is 10 ns. At the 100 MHz
// reference there are only 10 reference ticks per OCXO count, so a whole-count
// change in ocxo_count is a full 50 ns of phase, and the sub-cycle term can
// contribute at most +/-10 ns of that.
//
// It is therefore tempting to drop the fine term. Do not:
//
//   * it removes the quantization floor of +/-1 OCXO count (+/-50 ns), which is
//     the difference between a 1e-7 and a far finer measurement -- the whole point
//     of having a TDC;
//   * it is a DIFFERENCE, not a level. The absolute sub-cycle phase contains the
//     arbitrary phase between the two independent clocks at reset; using the
//     difference cancels it. Using the level would make the loop lock to a wrong
//     frequency and hold it, which is the same trap documented in the project's
//     200 MHz fine counter.
//
// What is honest to say: the fine term refines the measurement to sub-count
// resolution. If TDC_TAP_PERIOD is left uncalibrated the term is mis-scaled --
// but mis-scaled by a bounded amount, so the loop still converges to a slightly
// wrong null instead of diverging. Calibration is what turns it into accuracy.
//
//==============================================================================
// FIXED-POINT REPRESENTATION
//==============================================================================
// phase_error is reported in units of reference period / 2^SCALE_BITS:
//
//     LSB = 10 ns / 256 = 39.0625 ps   (at a 100 MHz reference)
//
// The ARM divides by 256 to recover reference periods, or multiplies by 10/256 ns
// to recover nanoseconds. Reporting whole periods would truncate the entire
// sub-cycle term to zero.
//
//==============================================================================
// SATURATION
//==============================================================================
// A missing 1PPS, a glitch, or an oscillator that is simply not running can
// produce one enormous difference. Saturating keeps the value inside the
// signed-32-bit contract, and sat_flag tells the loop the number is a bound
// rather than a measurement so it can decline to act on it.
//
// The sum is formed in 64 bits first: shifting a 32-bit signed value left by 8
// overflows by construction, and letting that happen in 32 bits would silently
// turn a large negative error into a large positive one -- the worst possible
// failure for a control loop, because it drives the loop the wrong way.
//
// NOTE: every comment and string in this file is ASCII on purpose.
//=============================================================================

`timescale 1ns / 1ps

module phase_error #(
    // Nominal OCXO edges per 1PPS interval. 20_000_000 for a 20 MHz OCXO.
    parameter integer NOMINAL_OCXO = 20_000_000,
    // Counting-clock frequency.  Only used to bring the sub-cycle term into
    // the same LSB as the coarse term (see FINE_K below).
    parameter integer PS_CLK_HZ    = 100_000_000,
    parameter integer FRAC_BITS    = 16,
    parameter integer SCALE_BITS   = 8,
    parameter integer LOCK_EDGES   = 3,
    parameter integer PPS_TIMEOUT_CYCLES = 150_000_000
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // from counter_measure
    input  wire signed [31:0]    ocxo_delta,     // count - NOMINAL_OCXO（诊断用）
input  wire signed [63:0]    ocxo_delta_frac,// Q16 全小数频差（控制用）
    input  wire                  diff_valid,
    input  wire                  pps_seen,

    // from tdc_carry
    input  wire [FRAC_BITS-1:0]  fine_fraction,  // [0,1) of a reference period
    input  wire                  fine_valid,
    // 单拍脉冲：counter_measure 的完整测量已经算好（比 ocxo_delta_frac 晚一拍）。
    // 用这个事件采样，才不会拿到上一轮的粗项。
    input  wire                  meas_done,
    input  wire                  fine_saturated,

    // results
    output reg  signed [31:0]    phase_error,    // units of period / 2^SCALE_BITS
    output reg                   meas_valid,
    output reg                   locked,
    output reg                   pps_lost,
    output reg                   sat_flag
);

    localparam signed [63:0] ERR_MAX =  64'sd2147483647;    //  2^31 - 1
    localparam signed [63:0] ERR_MIN = -64'sd2147483648;    // -2^31

    //--------------------------------------------------------------------------
    // Scale the sub-cycle phase into the output's fixed-point format.
    //
    // The generate covers both parameter orders instead of assuming one, because
    // a right shift written as a left shift compiles and is silently wrong.
    //--------------------------------------------------------------------------
    reg signed [63:0] fine_now_scaled;

    generate
        if (SCALE_BITS >= FRAC_BITS) begin : g_scale_up
            always @(*) fine_now_scaled = $signed({1'b0, fine_fraction}) <<< (SCALE_BITS - FRAC_BITS);
        end else begin : g_scale_down
            always @(*) fine_now_scaled = $signed({1'b0, fine_fraction}) >>> (FRAC_BITS - SCALE_BITS);
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Previous sub-cycle phase, so the fine term can be a DIFFERENCE. See the
    // header for why a level would lock the loop to a wrong frequency.
    //--------------------------------------------------------------------------
    reg signed [63:0] fine_prev_scaled;
    reg               fine_prev_valid;

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    // The coarse term, SAME sign as the counting module's convention.
    //
    //   counter_measure reports  ocxo_delta = count - NOMINAL   (正 = 快)
    //   this module needs        count - NOMINAL               (正 = 快)
    //
    // so NO inversion; the sub-cycle term is flipped to match instead.
    //--------------------------------------------------------------------------
    //----------------------------------------------------------------------
    // 粗项（2026-10-26 改）
    //   以前 = ocxo_delta <<< 8，只有整数，1 格 = 50 ppb；
    //          细项 fine_delta 只能补 10 ns（50 ns 的 1/5），
    //          粗计数一跳就有 40 ns 漏过去（实测 0x00 每秒跳 ±60 ns）。
    //   现在 = ocxo_delta_frac >>> 8，Q16 全小数已经包含细项，
    //          0x00 的 LSB 仍是 1/256 周期 = 0.195 ns，PS 侧不用改。
    //----------------------------------------------------------------------
    // Q16 -> Q8: round to nearest; exact half-LSB ties go toward +infinity.
    // This removes one-sided truncation; it does not guarantee zero sample mean.
    wire signed [63:0] coarse_term = ($signed(ocxo_delta_frac) + 64'sd128) >>> 8;

    //--------------------------------------------------------------------------
    // Sub-cycle scaling -- the two terms carry DIFFERENT LSBs.
    //
    //   coarse_term : 1 LSB = 1/2^SCALE_BITS of an OCXO period
    //   fine_*_scaled : 1 LSB = 1/2^SCALE_BITS of a REFERENCE (PS clock) period
    //
    // They may only be added after being brought to the same unit:
    //
    //   fine_delta = (fine_prev - fine_now) * NOMINAL_OCXO / PS_CLK_HZ
    //
    // Skipping this factor over-corrects the sub-cycle term by
    // PS_CLK_HZ / NOMINAL_OCXO (5x at 20 MHz/100 MHz, 10x at 10 MHz/100 MHz),
    // which is a full OCXO count instead of one reference period.
    //
    // Implemented as a constant multiply-shift FINE_K / 2^SCALE_BITS with
    //   FINE_K = NOMINAL_OCXO / (PS_CLK_HZ / 2^SCALE_BITS)    (= 51 @ 20 MHz)
    // 51/256 = 0.19922 vs the exact 0.2, i.e. -0.4 % -- far below one LSB.
    //--------------------------------------------------------------------------
    localparam integer FINE_K = NOMINAL_OCXO / (PS_CLK_HZ / (1 << SCALE_BITS));

    wire signed [15:0] frac_delta16 = $signed(fine_prev_scaled[15:0])
                                    - $signed(fine_now_scaled[15:0]);
    wire signed [31:0] fine_mul     = frac_delta16 * FINE_K;
    wire signed [15:0] fine_delta16 = $signed(fine_mul[SCALE_BITS +: 16]);
    wire signed [63:0] fine_delta   = {{48{fine_delta16[15]}}, fine_delta16};

    //--------------------------------------------------------------------------
    // Stage 1: form the full-width sum on the edge that carries the fresh
    // sub-cycle reading, at which point counter_measure's value for the SAME edge
    // has also settled. Pairing a fresh integer with a stale fraction is the
    // classic way to build a loop that oscillates.
    //
    // The first edge after reset has no predecessor, so both diff_valid (two
    // edges seen) and fine_prev_valid guard the update.
    //--------------------------------------------------------------------------
    reg signed [63:0] sum_full;
    reg               sum_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_full         <= 64'sd0;
            sum_valid        <= 1'b0;
            fine_prev_scaled <= 64'sd0;
            fine_prev_valid  <= 1'b0;
        end else begin
            /* 2026-10-27：采样时机从 fine_valid 改成 meas_done。
             * fine_valid 比 counter_measure 算完结果【早两拍】，
             * 原来在这里取 coarse_term，拿到的是上一轮的 ocxo_delta_frac ——
             * 也就是 0x00 比 0x18 晚一个 PPS（审计 P1 已仿真确认：
             * 本次应输出 261，实际输出上一轮的 5）。
             * meas_done 是"结果已写入后再延迟一拍"的事件，此刻读到的一定是本秒值。 */
            sum_valid <= 1'b0;              /* 默认清零 -> 单拍事件 */
            if (meas_done && diff_valid) begin
                sum_full  <= coarse_term;
                sum_valid <= 1'b1;
            end
            // Update the history regardless, so one dropped coarse measurement
            // does not stall the fine history too.
            fine_prev_scaled <= fine_now_scaled;
            fine_prev_valid  <= 1'b1;
        end
    end

    //--------------------------------------------------------------------------
    // Stage 2: saturate into the signed-32 contract
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_error <= 32'sd0;
            meas_valid  <= 1'b0;
            sat_flag    <= 1'b0;
        end else if (sum_valid) begin
            meas_valid <= 1'b1;

            if (sum_full > ERR_MAX) begin
                phase_error <= 32'sh7FFF_FFFF;
                sat_flag    <= 1'b1;
            end else if (sum_full < ERR_MIN) begin
                phase_error <= 32'sh8000_0000;
                sat_flag    <= 1'b1;
            end else begin
                phase_error <= sum_full[31:0];
                sat_flag    <= fine_saturated;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Lock acquisition
    //   Advances on every ACCEPTED measurement, not on raw 1PPS edges: the
    //   counting module has its own synchroniser on the same pin, so counting raw
    //   edges here would double-count and the two would not agree on which edge
    //   was which.
    //--------------------------------------------------------------------------
    reg [15:0] good_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            good_cnt <= 16'd0;
            locked   <= 1'b0;
        end else if (pps_lost) begin
            // Losing the reference releases the lock. Holding it while the 1PPS is
            // absent would let the PI loop act on a frozen measurement.
            locked   <= 1'b0;
            good_cnt <= 16'd0;
        end else if (sum_valid) begin
            if (good_cnt < 16'hFFFF) good_cnt <= good_cnt + 16'd1;
            if (good_cnt >= (LOCK_EDGES - 1)) locked <= 1'b1;
        end
    end

    //--------------------------------------------------------------------------
    // 1PPS loss detection (the ARM uses this to enter holdover)
    //--------------------------------------------------------------------------
    reg [31:0] timeout_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timeout_cnt <= 32'd0;
            pps_lost    <= 1'b0;
        end else if (fine_valid) begin
            timeout_cnt <= 32'd0;
            pps_lost    <= 1'b0;
        end else if (timeout_cnt >= PPS_TIMEOUT_CYCLES[31:0]) begin
            pps_lost    <= 1'b1;
        end else begin
            timeout_cnt <= timeout_cnt + 32'd1;
        end
    end

endmodule
