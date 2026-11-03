//=============================================================================
// counter_measure.v -- 1PPS-gated counters for the AX7010 GPSDO
//-----------------------------------------------------------------------------
// TWO counters run here, and they answer different questions:
//
//   Q1  ocxo_count  : how many OCXO 20 MHz edges fit in one GNSS 1PPS interval?
//                     THIS IS THE MEASUREMENT. Ideally 20_000_000.
//   Q2  clk_interval: how many 100 MHz reference clocks fit in the same interval?
//                     DIAGNOSTIC ONLY. Ideally 100_000_000.
//
//==============================================================================
// WHY THE OCXO IS THE THING BEING COUNTED, NOT THE CLOCK
//==============================================================================
// The reference clock (FCLK_CLK0, from the PS PLL) and the OCXO are two
// INDEPENDENT oscillators. The quantity being disciplined is the OCXO, so the
// OCXO is what gets counted; the reference clock is the ruler.
//
// The reverse arrangement -- counting a clock derived FROM the OCXO -- measures
// the oscillator against itself. The reference clock's absolute frequency would
// then be irrelevant (the same OCXO sets it), the 1PPS would act only as a gate,
// and GNSS 1PPS jitter would pass straight into the measurement instead of being
// averaged away by an independent ruler.
//
//==============================================================================
// WHY THE COUNTERS FREE-RUN AND ARE ONLY LATCHED
//==============================================================================
// The frequency error appears in the FRACTIONAL part of the count:
//
//     ocxo_count = 20_000_000 * (1 + delta)
//
//     delta = -6.8e-9  ->  9_999_999.932   -> integer 9_999_999, frac 0.932
//     delta = +6.8e-9  -> 20_000_000.136   -> integer 20_000_000, frac 0.136
//
// Clearing a counter at the gate edge discards that fraction, so the reported
// error collapses to integer counts and the residual resolution is thrown away.
// The counters therefore never reset; only the DIFFERENCE between consecutive
// latches is used.
//
//==============================================================================
// SIGN CONVENTION
//==============================================================================
//     ocxo_delta = ocxo_count - NOMINAL_OCXO
//
//     positive = the oscillator produced MORE edges than nominal = FAST
//     negative = fewer edges = SLOW
//
// Phase_error.v inverts this against its own contract; the inversion lives in
// exactly one place (there) so that a reader of either module finds a single,
// stated convention rather than two half-applied ones.
//
// A slow oscillator counts LOW, so (count - NOMINAL) is negative for a slow
// oscillator. Earlier in this project the analogous subtraction was written the
// other way round and produced an inverted loop; the testbench's sign assertion
// exists to catch exactly that.
//
//==============================================================================
// THE FIRST MEASUREMENT IS SUPPRESSED
//==============================================================================
// At reset the counters hold arbitrary values, so the first 1PPS edge has no
// valid predecessor and its difference is meaningless. Reporting it would make
// the ARM's very first PI iteration act on a full-scale bogus error.
// diff_valid stays low until two edges have been seen.
//
// NOTE: every comment and string in this file is ASCII on purpose. xvlog reads
// the source with the system code page, so UTF-8 Chinese would compile but print
// as garbage in simulation output.
//=============================================================================

`timescale 1ns / 1ps

module counter_measure #(
    // Nominal 100 MHz reference clocks per 1PPS interval (diagnostic counter).
    parameter integer NOMINAL_CYCLES = 100_000_000,
    // Nominal OCXO edges per 1PPS interval. 20_000_000 for a 20 MHz OCXO.
    // A parameter so a testbench can shrink the scale without changing the
    // oscillator, and so a different OCXO needs a parameter change and no logic
    // change.
    parameter integer NOMINAL_OCXO   = 20_000_000,
    // Width of the free-running counters.
    parameter integer CNT_WIDTH      = 48,
    parameter integer SYNC_STAGES    = 2
)(
    input  wire                  clk,            // 100 MHz reference (FCLK_CLK0)
    input  wire                  rst_n,
    input  wire                  pps_in,         // GNSS 1PPS, asynchronous
    input  wire                  ocxo_in,        // OCXO output, asynchronous
        // 小数相位（来自 ocxo_frac）：PPS 边沿在一个 OCXO 周期内的位置 [0,1)
        input  wire [15:0]           frac_in,
        input  wire                  frac_valid,

    // OCXO measurement (the one the loop uses)
    output reg  signed [31:0]    ocxo_delta,     // count - NOMINAL_OCXO, signed
    output reg  [31:0]           ocxo_count,     // last latched count (diagnostic)
        // 带小数的频差：Q16 定点，1.0 = 一个 OCXO 周期。
        // 分辨率 = 1/1280 周期 = 39 ps = 0.039 ppb（整数版本是 50 ppb）
        output reg  signed [63:0]    ocxo_delta_frac,

    // Reference-clock interval (diagnostic)
    output reg  signed [31:0]    clk_delta,      // count - NOMINAL_CYCLES, signed
    output reg  [31:0]           clk_count,

    output reg                   diff_valid,     // high from the 2nd edge onwards
    output reg                   pps_seen,       // at least one edge has arrived
    output reg  [15:0]           pps_interval,   // saturating edge counter

    // 单拍脉冲：本秒的完整测量（整数+分数）已经算好，且 ocxo_delta_frac
    // 已在上一拍写入。下游据此采样，可以保证拿到的是【本秒】的结果。
    output reg                   meas_done
);

    //==========================================================================
    // Input synchronisers and rising-edge detect
    //   Both inputs are asynchronous to clk by design. A two-flop synchroniser
    //   plus edge detection is the correct treatment: there is no setup/hold
    //   relationship for the tool to close.
    //==========================================================================
    reg [SYNC_STAGES-1:0] pps_sync;
    reg [SYNC_STAGES-1:0] ocxo_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pps_sync  <= {SYNC_STAGES{1'b0}};
            ocxo_sync <= {SYNC_STAGES{1'b0}};
        end else begin
            pps_sync  <= {pps_sync[SYNC_STAGES-2:0],  pps_in};
            ocxo_sync <= {ocxo_sync[SYNC_STAGES-2:0], ocxo_in};
        end
    end

    // Previous cycle 0, current cycle 1 => rising edge.
    wire pps_rise  = (pps_sync[SYNC_STAGES-1:SYNC_STAGES-2]   == 2'b01);
    wire ocxo_rise = (ocxo_sync[SYNC_STAGES-1:SYNC_STAGES-2]  == 2'b01);

    //==========================================================================
    // Free-running counters -- deliberately NEVER cleared by pps_rise.
    //==========================================================================
    reg [CNT_WIDTH-1:0] clk_cnt;
    reg [CNT_WIDTH-1:0] ocxo_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt  <= {CNT_WIDTH{1'b0}};
            ocxo_cnt <= {CNT_WIDTH{1'b0}};
        end else begin
            clk_cnt <= clk_cnt + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
            if (ocxo_rise)
                ocxo_cnt <= ocxo_cnt + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
        end
    end

    //==========================================================================
    // Capture at the 1PPS edge
    //==========================================================================
    reg [CNT_WIDTH-1:0] clk_prev;
    reg [CNT_WIDTH-1:0] ocxo_prev;
    reg [15:0]          frac_prev;   // 上一次的全小数相位
    reg [31:0]          ocxo_diff_lat;  // pps_rise 时锁存的整数边沿差
    reg [31:0]          clk_diff_lat;   // 同上（诊断）
    reg                 meas_pend;      // 等 fine_valid 来配对
    reg signed [17:0]   frac_diff;      // 分数部分的差（含翻圈修正，只补一次）
    reg                 frac_valid_d;   // frac_valid 延迟一拍：此时 frac_in 已是新值
    reg                 meas_done_r;    // 内部：结果已写入的那一拍

    // The raw differences have to be named before they can be sliced: Verilog
    // does not allow a part-select of an expression.
    wire [CNT_WIDTH-1:0] clk_diff_raw  = clk_cnt  - clk_prev;
    wire [CNT_WIDTH-1:0] ocxo_diff_raw = ocxo_cnt - ocxo_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_prev     <= {CNT_WIDTH{1'b0}};
            ocxo_prev    <= {CNT_WIDTH{1'b0}};
            ocxo_delta   <= 32'sd0;
            ocxo_count   <= 32'd0;
            clk_delta    <= 32'sd0;
            clk_count    <= 32'd0;
            ocxo_diff_lat <= 32'd0;
            clk_diff_lat  <= 32'd0;
            pps_seen     <= 1'b0;
            pps_interval <= 16'd0;
        end else if (pps_rise) begin
            pps_seen <= 1'b1;

            if (pps_seen) begin
                // Difference since the previous edge, signed against nominal.
                // See the header for why the subtraction is written this way.
                ocxo_delta <= $signed({1'b0, ocxo_diff_raw[31:0]})
                              - $signed(NOMINAL_OCXO);
                clk_delta  <= $signed({1'b0, clk_diff_raw[31:0]})
                              - $signed(NOMINAL_CYCLES);

                /* ---- 整数差在 pps_rise 锁存，小数等 frac_valid 配对 ----------
                 * pps_rise 这一刻 ocxo_frac 的 frac 还是上一秒的值，
                 * 本秒的 fine 要等 TDC 流水线走完才出，所以这里只锁存整数差。
                 * ------------------------------------------------------------ */
                ocxo_diff_lat <= ocxo_diff_raw[31:0];
                clk_diff_lat  <= clk_diff_raw[31:0];

                ocxo_count <= ocxo_diff_raw[31:0];
                clk_count  <= clk_diff_raw[31:0];
            end
            /* else: 第一个边沿，没有前驱，diff_valid 保持低 */

            clk_prev  <= clk_cnt;
            ocxo_prev <= ocxo_cnt;

            if (pps_interval != 16'hFFFF)
                pps_interval <= pps_interval + 16'd1;
        end
    end

    //==========================================================================
    // 小数配对（2026-10-27）
    //   粗计数在 pps_rise 锁存，本秒的 frac 要等 TDC 流水线走完（frac_valid）
    //   才出。两者必须配对，且本模块里 meas_pend / frac_prev / diff_valid /
    //   ocxo_delta_frac 只在这里赋值，避免 multiple drivers。
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frac_valid_d    <= 1'b0;
            meas_pend       <= 1'b0;
            frac_prev       <= 16'd0;
            diff_valid      <= 1'b0;
            ocxo_delta_frac <= 64'sd0;
            meas_done_r     <= 1'b0;
            meas_done       <= 1'b0;
        end else begin
            meas_done_r <= 1'b0;          /* 默认清零 -> 单拍 */
            meas_done   <= meas_done_r;   /* 再延迟一拍，保证下游读到的是新值 */
            /* 1) 本秒粗计数已锁存 -> 标记待配对 */
            if (pps_rise && pps_seen) begin
                meas_pend <= 1'b1;
            end

            /* 2) frac 到齐 -> 与锁存的整数差配对。
             *    用延迟一拍的 frac_valid_d：ocxo_frac 在 fine_valid 那一拍
             *    才把新值写进 frac，同一拍读到的是旧值。 */
            frac_valid_d <= frac_valid;

            if (frac_valid_d) begin
                if (meas_pend) begin
                    /* ---- 跨圈不做任何补偿（2026-10-27，审计 P1）-------------
                     * ocxo_diff_lat 是「两个 PPS 之间过了几个 OCXO 边沿」，
                     * frac 是「PPS 落在第几个周期的哪个位置」，两者边界一致：
                     *     Δ = Δn + frac_now - frac_prev   （单位：OCXO 周期）
                     * 跨圈会被 Δn 与 (frac_now - frac_prev) 自然抵消。
                     * 之前「|差|>32768 就补一圈」会把本来就对的小差值
                     * 改大整整一个周期 —— 审计复现：正确 10 ns 算成 60 ns。
                     * ------------------------------------------------------- */
                    frac_diff = $signed({2'b00, frac_in}) - $signed({2'b00, frac_prev});

                    ocxo_delta_frac <= (($signed({32'd0, ocxo_diff_lat}) <<< 16)
                                        + $signed({{46{frac_diff[17]}}, frac_diff}))
                                       - ($signed(NOMINAL_OCXO) <<< 16);
                    diff_valid  <= 1'b1;
                    meas_done_r <= 1'b1;     /* 本拍写入结果，下一拍发出 done */
                    meas_pend   <= 1'b0;
                end
                frac_prev <= frac_in;
            end
        end
    end

endmodule
