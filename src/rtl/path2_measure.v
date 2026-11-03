//=============================================================================
// path2_measure.v -- 第二路(慢环)的 T 秒区间相位差累加 / 锁存
//-----------------------------------------------------------------------------
// 为什么单独一个模块
//   X[1] < 10ppb 之后, 每秒一次的相位差里的噪声已经远大于真实漂移, 再每秒调一次
//   DAC 只是在追噪声。第二路把观察窗口拉长到 T 秒: 在 T 内把相位差一直累加起来,
//   到 T 边界一次性交给 PS。长窗口测频的全部意义就在于白噪声按 sqrt(T) 下降。
//
// 为什么是 64 位
//   单位是 PHASE_ERR 的原始计数(PS 侧按 10ns/LSB 换算)。T 可以到上千秒,
//   累加值会超出 32 位, 所以内部用 signed 64 位累加, 输出拆成两个 32 位寄存器
//   (0x48 = 低 32 位, 0x4C = 高 32 位), 由 PS 拼接成 int64_t。
//
// 时序
//   phase_in / ocxo_delta_in 都是在 1PPS 沿上更新的, 而且各自有自己的 2FF
//   同步器。这里把 1PPS 自己同步一次, 再延迟 DM 拍才真正累加, 保证读到的是
//   本次的新值而不是上一次的。DM=4 在 100MHz 下是 40ns, 远小于 1s。
//=============================================================================
`timescale 1ns / 1ps

module path2_measure (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [15:0]        t_in,             // 0x44 P2_T: 计数周期(秒)
    input  wire               pps_in,           // 原始 1PPS 引脚
    input  wire signed [31:0] phase_in,         // phase_error.v 输出
    input  wire signed [31:0] ocxo_delta_in,    // counter_measure.v 输出
    input  wire               clr_new,          // 0x50 写 1: 清 NEW 标志
    output reg  signed [63:0] phase_out,        // T 内的相位差累加 (0x48/0x4C)
    output reg  signed [31:0] ocxo_out,         // T 内 ocxo_delta 累加 (0x58)
    output reg                new_flag,         // 0x50 bit0: 有新数据(粘滞)
    output reg  [15:0]        count_out         // 0x54: 窗口内已过的 1PPS 数
);

    localparam integer DM = 4;      // 数据稳定后再累加的延迟拍数

    //-------------------------------------------------------------------------
    // 1PPS 同步 + 上升沿
    //-------------------------------------------------------------------------
    reg [1:0] pps_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pps_sync <= 2'b00;
        else        pps_sync <= {pps_sync[0], pps_in};
    end
    wire pps_rise = (pps_sync == 2'b01);

    reg [DM-1:0] dm;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) dm <= {DM{1'b0}};
        else        dm <= {dm[DM-2:0], pps_rise};
    end
    wire pps_use = dm[DM-1];

    wire [15:0] t_eff = (t_in == 16'd0) ? 16'd1 : t_in;

    //-------------------------------------------------------------------------
    // 累加器
    //-------------------------------------------------------------------------
    reg [15:0]        cnt;
    reg signed [63:0] pacc;
    reg signed [31:0] oacc;
    reg signed [63:0] psum;
    reg signed [31:0] osum;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt       <= 16'd0;
            pacc      <= 64'sd0;
            oacc      <= 32'sd0;
            psum      <= 64'sd0;
            osum      <= 32'sd0;
            phase_out <= 64'sd0;
            ocxo_out  <= 32'sd0;
            new_flag  <= 1'b0;
            count_out <= 16'd0;
        end else begin
            // 清标志放在前面: 同一拍既清又置时, 置位优先(不丢样本)
            if (clr_new) begin
                new_flag <= 1'b0;
            end

            if (pps_use) begin
                psum <= pacc + phase_in;
                osum <= oacc + ocxo_delta_in;

                if ((cnt + 16'd1) >= t_eff) begin
                    phase_out <= pacc + phase_in;   // 整个窗口的相位差累加
                    ocxo_out  <= oacc + ocxo_delta_in;
                    new_flag  <= 1'b1;
                    cnt       <= 16'd0;
                    pacc      <= 64'sd0;
                    oacc      <= 32'sd0;
                end else begin
                    cnt  <= cnt + 16'd1;
                    pacc <= pacc + phase_in;
                    oacc <= oacc + ocxo_delta_in;
                end
            end

            count_out <= cnt;
        end
    end

endmodule
