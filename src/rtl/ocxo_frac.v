//=============================================================================
// ocxo_frac.v  --  把"PPS 边沿在一个 OCXO 周期内的位置"算成 16 位小数
//-----------------------------------------------------------------------------
// 为什么需要它
//   现在的 phase_error.v 用  ocxo_delta<<8 + fine_delta  合成 0x00。
//   ocxo_delta 是整数（1 格 = 50 ppb），而 fine_delta 只能覆盖 10 ns，
//   50 ns 的 1/5 —— 粗计数一跳 ±50 ns 时，剩下 40 ns 无法补偿，
//   实测 0x00 每秒在 ±60 ns 之间跳。
//
// 本模块做什么
//   复用【现有 TDC】的 fine_fraction（它就测的是 PPS 边沿在 10 ns 内的位置），
//   再补一个 100 MHz 周期计数器 sub（0..4），就能算出 PPS 边沿在
//   【一个完整 OCXO 周期 50 ns 内】的归一化位置 frac ∈ [0,1)。
//
//       x     = sub * 256 + fine8            0 .. 1279
//       frac  = x * 65536 / 1280 = (x<<8)/5  0 .. 65535
//
//   分辨率 = 50 ns / 1280 = 39 ps = 0.039 ppb
//
// 优点
//   · 不例化第二个 TDC，不增加任何 CARRY4
//   · 没有新的异步路径，时序风险为零
//   · phase_error.v 里 0x00 的 LSB 保持 0.195 ns 不变，PS 侧不用改
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module ocxo_frac (
    input  wire                 clk,           // 100 MHz
    input  wire                 rst_n,
    input  wire                 ocxo_in,       // OCXO 20 MHz, 异步（只用来产生复位脉冲）
    input  wire [15:0]          fine_fraction, // 现有 TDC 的输出（PPS 边沿位置）
    input  wire                 fine_valid,
    input  wire                 tap_valid,     // TDC 每检测到一个 PPS 边沿就出 1 拍脉冲（保留，不再用于锁存）
    input  wire                 pps_in,        // GNSS 1PPS 原始输入

    output reg  [15:0]          frac,          // [0,1) of one OCXO period
    output reg                  frac_valid,
    output reg  [2:0]           sub_dbg        // 0..4, 诊断用
);

    //-------------------------------------------------------------------------
    // 1) OCXO 边沿同步（只为了得到 clk 域的"发生过沿"脉冲）
    //-------------------------------------------------------------------------
    reg [1:0] ocxo_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ocxo_sync <= 2'b00;
        else        ocxo_sync <= {ocxo_sync[0], ocxo_in};
    end
    wire ocxo_rise = (ocxo_sync == 2'b01);

    //-------------------------------------------------------------------------
    // 2) 100 MHz 周期计数：每个 OCXO 边沿清零，最多数到 4
    //    一个 OCXO 周期 50 ns = 5 个 100 MHz 周期，所以取值 0..4
    //-------------------------------------------------------------------------
    reg [2:0] sub;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sub <= 3'd0;
        end else if (ocxo_rise) begin
            sub <= 3'd0;
        end else if (sub != 3'd4) begin
            sub <= sub + 3'd1;
        end
    end

    /* ---- 在 PPS 边沿那一刻锁存 sub（2026-10-27 修正，审计第 7 条）---------
     * 原来是在 tap_valid 时锁存 —— 而 tap_valid 比 PPS 晚约 11 拍：
     *     2 拍同步 + 1 拍 tap_now + 1 拍 tap_raw_r + 8 拍 cap_dly
     * sub 每 5 拍回绕一次，11 拍 mod 5 = 1，
     * 于是锁到的 sub 和 PPS 时刻的 sub 差了整整一个 10 ns，
     * 而且这个偏差是固定的、从寄存器上看不出来。
     *
     * 正确做法：和 TDC 用【同一套】2 级同步、在【同一拍】采样。
     * TDC 是在 pps_rise 那一拍把 tap_now 锁进 tap_raw_r 的，
     * 所以 fine 代表"PPS 相对那一刻时钟沿的位置"；
     * sub 也必须在同一拍采样，两者才描述同一个时间点。
     * -------------------------------------------------------------------- */
    reg [1:0] pps_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pps_sync <= 2'b00;
        else        pps_sync <= {pps_sync[0], pps_in};
    end
    wire pps_edge = (pps_sync == 2'b01);

    reg [2:0] sub_pps;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        sub_pps <= 3'd0;
        else if (pps_edge) sub_pps <= sub;      /* 与 TDC 参考同一拍 */
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sub_dbg <= 3'd0;
        else        sub_dbg <= sub_pps;
    end

    //-------------------------------------------------------------------------
    // 3) x = (sub+1)*256 - fine8   （2026-10-27 修正方向）
    //
    //    fine_fraction 的定义是【PPS 边沿到下一个采样沿的时间】r_P，
    //    不是"PPS 在周期内的位置"。所以：
    //        P - O = (s_P - s_O) - r_P + r_O
    //              = sub*T - r_P + r_O
    //    即 fine 必须以【负号】进入，不能当递增位置相加。
    //
    //    用 (sub+1)*256 - fine8 实现：
    //        等价于 sub*256 + (256 - fine8)，且不会出负数。
    //    x 的范围 0..1280，除以 1280 就是 [0,1)。
    //
    //    数值直觉：采样沿固定在 30 ns，
    //        PPS 在 22 ns -> TDC 读到 8 ns
    //        PPS 在 23 ns -> TDC 读到 7 ns
    //    实际晚了 1 ns，TDC 数值却小了 1 ns —— 所以是减不是加。
    //-------------------------------------------------------------------------
    wire [7:0]  fine8 = fine_fraction[15:8];
    wire [3:0]  sub1  = {1'b0, sub_pps} + 4'd1;         // sub+1，范围 1..5
    wire [11:0] x     = {sub1, 8'd0} - {4'd0, fine8};   // (sub+1)*256 - fine8，范围 1..1280
    wire [18:0] xsh   = {x, 8'd0};             // x << 8，最大 327680
    wire [17:0]quo18  = xsh / 19'd5;           // /5，最大 65536 —— 需要 17 位
    // x = 1279 时 quo18 = 65536，正好超出 16 位；钳到 65535，
    // 否则 frac 会回绕成 0，造成一次满量程跳变。
    wire [15:0] quo   = (quo18 > 18'd65535) ? 16'hFFFF : quo18[15:0];

    /* ---- frac_valid 必须是【单拍脉冲】(2026-10-27 修正) -------------------
     * 原来写成"置 1 后永不清零"，是一个【电平】。
     * 而 counter_measure 的新配对逻辑是把它当【事件】用的：
     *     if (frac_valid) if (meas_pend) -> 配对
     * 电平一旦置起，pps_rise 之后【下一个时钟周期】就满足条件，
     * 此时本秒的 frac 还没写进来，用的是两个旧值，
     * 于是 frac_diff 恒等于 0 —— 这就是"分数项恒为零"的直接原因。
     *
     * 改成：默认清零，只有 fine_valid 那一拍高。
     * ---------------------------------------------------------------------- */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frac       <= 16'd0;
            frac_valid <= 1'b0;
        end else begin
            frac_valid <= 1'b0;              /* 默认清零 -> 单拍脉冲 */
            if (fine_valid) begin
                frac       <= quo;
                frac_valid <= 1'b1;
            end
        end
    end

endmodule
