//=============================================================================
// gnss_gpsdo_top.v -- AX7010 GPSDO PL top level
//-----------------------------------------------------------------------------
// Ties the measurement chain, the NMEA mailbox and the DAC driver together and
// exposes them to the ARM through one AXI4-Lite slave.
//
//==============================================================================
// CLOCKING -- THE ASSUMPTION THE WHOLE DESIGN RESTS ON
//==============================================================================
//     clk = PS FCLK_CLK0 = 100 MHz, from the on-board 33.333 MHz PS oscillator.
//     It is an INDEPENDENT oscillator, and that is the point.
//
// The OCXO is the device being disciplined, so the OCXO is what gets COUNTED;
// the 100 MHz reference is the ruler. See counter_measure.v for the full argument
// and for why the reverse arrangement (counting a clock derived from the OCXO)
// measures the oscillator against itself and lets GNSS 1PPS jitter straight into
// the result.
//
// Because the reference is independent, the two clocks are unrelated and both
// pps_in and ocxo_in are asynchronous. Each is synchronised on entry. There is no
// MMCM here and no second clock domain: everything runs on FCLK_CLK0.
//
//==============================================================================
// SIGNAL FLOW
//==============================================================================
//   pps_in  ---> counter_measure --> ocxo_delta  --+
//   ocxo_in ---> (counted)                          |
//   pps_in  ---> tdc_carry       --> fine_fraction -+--> phase_error --> 0x00
//
//   uart_rx --> uart_rx --> nmea_mailbox ----------------> 0x04 (W1C handshake)
//
//   AXI write to 0x08 --> dac_wr_pulse --> spi_dac --> sck/mosi/cs_n --> MCP4921
//
//==============================================================================
// PIN BUDGET ON THE AX7010
//==============================================================================
// ocxo_in is an extra input compared with the previous revision. It must land on
// a CLOCK-CAPABLE pin; see the constraint file for which J10 pin was chosen and
// how to verify it in Vivado.
//
//==============================================================================
// RESET
//==============================================================================
// rst_n comes from the PS peripheral_aresetn. It is asynchronous to the PL logic,
// so it is synchronised on release (asserted asynchronously). Without that, the
// reset can be released close enough to a clock edge that some flops leave reset
// one cycle later than others -- which shows up as a state machine starting in an
// illegal state rather than as a clean reset failure.
//
// NOTE: every comment and string in this file is ASCII on purpose.
//=============================================================================

`timescale 1ns / 1ps

module gnss_gpsdo_top #(
    // Nominal 100 MHz reference clocks per 1PPS interval.
    parameter integer NOMINAL_CYCLES = 100_000_000,
    // Nominal OCXO edges per 1PPS interval: 20_000_000 for a 20 MHz OCXO.
    parameter integer NOMINAL_OCXO   = 20_000_000,
    // Frequency of `clk` in Hz, as seen by the UART. Normally equal to
    // NOMINAL_CYCLES. Kept separate only so a simulation can compress the 1PPS
    // period without breaking the clock-to-baud RATIO: the UART's margin depends
    // on oversampling clocks per bit, so shortening the clock without shortening
    // the baud rate silently removes the margin the receiver was designed around.
    parameter integer CLK_FREQ_HZ    = 100_000_000,
    parameter integer CNT_WIDTH      = 48,
    parameter integer NUM_CARRY4     = 128,
    parameter integer FRAC_BITS      = 16,
    parameter integer SCALE_BITS     = 8,
    parameter integer BAUD_RATE      = 9600,
    parameter integer SPI_DIV        = 8,
    // 0 = real CARRY4 delay line (synthesis and hardware).
    // 1 = behavioural delay line (simulation only). CARRY4 has no useful
    //     simulation model because its delay is a physical property of the
    //     device, so a testbench must substitute a line with a known delay or it
    //     can only prove that the code compiles.
    parameter integer SIM_DELAY_MODEL = 0,
    parameter integer SIM_TAP_PS      = 150
)(
    // ---- PL clock and reset (from the PS) ----
    input  wire        clk,
    input  wire        rst_n,

    // ---- physical inputs ----
    input  wire        pps_in,       // GNSS 1PPS          (J10)
    input  wire        ocxo_in,      // OCXO 20 MHz        (J10) -- THE MEASURAND
    input  wire        uart_rx,      // GNSS NMEA TX       (J11)

    // ---- physical outputs ----
    output wire        spi_sck,      // MCP4921 SCK        (J11)
    output wire        spi_mosi,     // MCP4921 SDI        (J11)
    output wire        spi_cs_n,     // MCP4921 CS         (J11)

    // ---- debug / bring-up ----
    output wire [3:0]  led,
    output reg  [31:0] status_dbg,

    // ---- AXI4-Lite slave ----
    input  wire [11:0] S_AXI_AWADDR,
    input  wire [2:0]  S_AXI_AWPROT,
    input  wire        S_AXI_AWVALID,
    output wire        S_AXI_AWREADY,
    input  wire [31:0] S_AXI_WDATA,
    input  wire [3:0]  S_AXI_WSTRB,
    input  wire        S_AXI_WVALID,
    output wire        S_AXI_WREADY,
    output wire [1:0]  S_AXI_BRESP,
    output wire        S_AXI_BVALID,
    input  wire        S_AXI_BREADY,
    input  wire [11:0] S_AXI_ARADDR,
    input  wire [2:0]  S_AXI_ARPROT,
    input  wire        S_AXI_ARVALID,
    output wire        S_AXI_ARREADY,
    output wire [31:0] S_AXI_RDATA,
    output wire [1:0]  S_AXI_RRESP,
    output wire        S_AXI_RVALID,
    input  wire        S_AXI_RREADY
);

    //==========================================================================
    // Reset synchroniser: async assert, sync release
    //==========================================================================
    reg [1:0] rst_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_sync <= 2'b00;
        else        rst_sync <= {rst_sync[0], 1'b1};
    end

    wire rst_n_sync = rst_sync[1];

    //==========================================================================
    // Counters: OCXO frequency (the measurement) + reference interval (diagnostic)
    //==========================================================================
    wire signed [31:0]   ocxo_delta;
    wire [31:0]          ocxo_count;
    wire signed [31:0]   clk_delta;
    wire [31:0]          clk_count;
    wire                 diff_valid;
    wire                 meas_done;
    wire                 pps_seen;
    wire [15:0]          pps_interval;

    //==========================================================================
    // 小数相位（2026-10-26 新增）
    //   复用现有 TDC 的 fine_fraction（PPS 边沿在 10ns 内的位置），
    //   再加一个 100MHz 周期计数器 sub(0..4)，算出 PPS 边沿在一个
    //   完整 OCXO 周期(50ns)内的归一化位置，分辨率 39ps = 0.039 ppb。
    //   不例化第二个 TDC，不增加 CARRY4。
    //==========================================================================
    wire [15:0]          ocxo_frac16;
    wire                 ocxo_frac_valid;
    wire [2:0]           ocxo_sub_dbg;
    wire signed [63:0]   ocxo_delta_frac;


    counter_measure #(
        .NOMINAL_CYCLES (NOMINAL_CYCLES),
        .NOMINAL_OCXO   (NOMINAL_OCXO),
        .CNT_WIDTH      (CNT_WIDTH)
    ) u_counter (
        .clk          (clk),
        .rst_n        (rst_n_sync),
        .pps_in       (pps_in),
        .ocxo_in      (ocxo_in),
        .frac_in      (ocxo_frac16),
        .frac_valid   (ocxo_frac_valid),
        .ocxo_delta   (ocxo_delta),
        .ocxo_count   (ocxo_count),
        .ocxo_delta_frac (ocxo_delta_frac),
        .clk_delta    (clk_delta),
        .clk_count    (clk_count),
        .diff_valid   (diff_valid),
        .meas_done    (meas_done),
        .pps_seen     (pps_seen),
        .pps_interval (pps_interval)
    );

    //==========================================================================
    // TDC: sub-cycle position of the 1PPS edge
    //==========================================================================
    wire [15:0]          tap_value;
    wire                 tap_valid;
    wire                 tap_max;
    wire [FRAC_BITS-1:0] fine_fraction;
    wire                 fine_valid;
    wire                 fine_saturated;
    wire [4*NUM_CARRY4-1:0] tap_raw;

    //==========================================================================
    // 诊断计数器（2026-10-27）
    //   注意：必须放在 fine_valid / tap_valid / ocxo_frac_valid / diff_valid
    //   全部【声明之后】。Verilog 里先用后声明会生成一个新的隐式 1 位网线，
    //   计数器会去数那个悬空的信号 —— 永远读到 0。
    //==========================================================================
    reg  [31:0] dbg_fine_cnt;
    reg  [31:0] dbg_frac_cnt;
    reg  [31:0] dbg_diff_cnt;


    always @(posedge clk or negedge rst_n_sync) begin
        if (!rst_n_sync) begin
            dbg_fine_cnt <= 32'd0;
            dbg_frac_cnt <= 32'd0;
            dbg_diff_cnt <= 32'd0;
        end else begin
            /* 三个都是【事件】计数：
             *   fine_valid / ocxo_frac_valid 本来就是单拍；
             *   meas_done 是"完整测量算完"的单拍事件。
             * 不能用 diff_valid —— 它是电平，首次置高后就再也不降，
             * 数上升沿只会得到 1。 */
            if (fine_valid)      dbg_fine_cnt <= dbg_fine_cnt + 32'd1;
            if (ocxo_frac_valid) dbg_frac_cnt <= dbg_frac_cnt + 32'd1;
            if (meas_done)       dbg_diff_cnt <= dbg_diff_cnt + 32'd1;
        end
    end

    wire [15:0] tap_period;
    wire [15:0] tap_offset;
    wire [31:0] tdc_raw_index;

    tdc_carry #(
        .NUM_CARRY4      (NUM_CARRY4),
        .FRAC_BITS       (FRAC_BITS),
        .SIM_DELAY_MODEL (SIM_DELAY_MODEL),
        .SIM_TAP_PS      (SIM_TAP_PS)
    ) u_tdc (
        .clk            (clk),
        .rst_n          (rst_n_sync),
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
    // Phase error
    //==========================================================================
    wire signed [31:0] phase_error;
    wire               meas_valid;
    wire               locked;
    wire               pps_lost;
    wire               sat_flag;

    phase_error #(
        .NOMINAL_OCXO (NOMINAL_OCXO),
        .FRAC_BITS    (FRAC_BITS),
        .SCALE_BITS   (SCALE_BITS)
    ) u_phase (
        .clk            (clk),
        .rst_n          (rst_n_sync),
        .ocxo_delta     (ocxo_delta),
        .ocxo_delta_frac(ocxo_delta_frac),
        .diff_valid     (diff_valid),
        .meas_done      (meas_done),
        .pps_seen       (pps_seen),
        .fine_fraction  (fine_fraction),
        .fine_valid     (fine_valid),
        .fine_saturated (fine_saturated),
        .phase_error    (phase_error),
        .meas_valid     (meas_valid),
        .locked         (locked),
        .pps_lost       (pps_lost),
        .sat_flag       (sat_flag)
    );

    //==========================================================================
    // NMEA path
    //==========================================================================
    wire [7:0] rx_data;
    wire       rx_valid;
    wire       rx_frame_err;


    // Runtime UART baud divisor (clk cycles per bit). Powers up at
    // CLK_FREQ_HZ/BAUD_RATE and is written by the ARM through AXI 0x40.
    wire [31:0] baud_div;

    // ---- 第二路(慢环) ----
    wire [15:0]        p2_t;            // 0x44 写入的计数周期(秒)
    wire signed [63:0] p2_phase;        // T 内相位差累加 -> 0x48(低)/0x4C(高)
    wire signed [31:0] p2_ocxo;         // 0x4C T 内 ocxo_delta 累加
    wire               p2_new;          // 0x50 bit0
    wire [15:0]        p2_count;        // 0x54 窗口进度
    wire               p2_clr;          // 写 0x50 -> 清标志

    uart_rx #(
        .CLK_FREQ  (CLK_FREQ_HZ),
        .BAUD_RATE (BAUD_RATE)
    ) u_uart (
        .clk       (clk),
        .rst_n     (rst_n_sync),
        .baud_cnt_in (baud_div[15:0]),
        .rx        (uart_rx),
        .data      (rx_data),
        .valid     (rx_valid),
        .frame_err (rx_frame_err)
    );

    wire [31:0] mailbox;
    wire [31:0] drop_count;
    wire [31:0] byte_count;
    wire        frame_err_seen;

    wire [31:0] nmea_ack_wdata;
    wire        nmea_ack_pulse;

    nmea_mailbox u_mailbox (
        .clk            (clk),
        .rst_n          (rst_n_sync),
        .rx_valid       (rx_valid),
        .rx_data        (rx_data),
        .frame_err      (rx_frame_err),
        .ack_pulse      (nmea_ack_pulse),
        .ack_wdata      (nmea_ack_wdata),
        .mailbox        (mailbox),
        .drop_count     (drop_count),
        .byte_count     (byte_count),
        .frame_err_seen (frame_err_seen)
    );

    //==========================================================================
    // DAC path
    //==========================================================================
    wire [11:0] dac_data;
    wire        dac_wr_pulse;
    wire        spi_busy;

    spi_dac #(
        .DAC_BITS (16),
        .DAC_HDR  (8'h00),        // DAC8531: 6 don't care + PD1=0 + PD0=0 (normal)
        .SPI_DIV  (SPI_DIV),
        .LDAC_MODE(0)             // no LDAC pin on the MCP4921
    ) u_spi (
        .clk      (clk),
        .rst_n    (rst_n_sync),
        .start    (dac_wr_pulse),
        .dac_data ({dac_data, 4'b0}),   // 12-bit reg -> 16-bit DAC code (same full scale)
        .sck      (spi_sck),
        .mosi     (spi_mosi),
        .cs_n     (spi_cs_n),
        .ldac_n   (),
        .busy     (spi_busy)
    );

    //==========================================================================
    // Status word
    //   Cleared before every field is assigned, so no bit can be left undefined
    //   when a signal is optimised away or added later.
    //==========================================================================
    always @(*) begin
        status_dbg        = 32'd0;
        status_dbg[0]     = meas_valid;
        status_dbg[1]     = locked;
        status_dbg[2]     = pps_lost;
        status_dbg[3]     = sat_flag;
        status_dbg[4]     = fine_saturated;
        status_dbg[5]     = frame_err_seen;
        status_dbg[6]     = spi_busy;
        status_dbg[7]     = diff_valid;
        status_dbg[8]     = tap_max;
        status_dbg[31:16] = pps_interval;
    end

    //==========================================================================
    // Bring-up LEDs
    //   led[0] : a 1PPS edge has been seen
    //   led[1] : locked
    //   led[2] : phase error is positive (OCXO slow)
    //   led[3] : an NMEA byte is waiting
    // On the AX7010 the PL LEDs are active LOW (the pin sinks current), so the
    // polarity is inverted here rather than in the constraint file.
    //==========================================================================
    assign led[0] = ~pps_seen;
    assign led[1] = ~locked;
    assign led[2] = ~phase_error[31];
    assign led[3] = ~mailbox[8];

    //==========================================================================
    // 第二路(慢环): T 秒区间累加 / 锁存
    //==========================================================================
    path2_measure u_p2 (
        .clk           (clk),
        .rst_n         (rst_n_sync),
        .t_in          (p2_t),
        .pps_in        (pps_in),
        .phase_in      (phase_error),
        .ocxo_delta_in (ocxo_delta),
        .clr_new       (p2_clr),
        .phase_out     (p2_phase),
        .ocxo_out      (p2_ocxo),
        .new_flag      (p2_new),
        .count_out     (p2_count)
    );

    //==========================================================================
    // AXI4-Lite register file
    //==========================================================================
    gnss_axi_ip_slave #(
        .C_S_AXI_DATA_WIDTH (32),
        .C_S_AXI_ADDR_WIDTH (12),
        // Widths must match, so the capture vector is passed straight through
        // with no replication cast. See the note in gnss_axi_ip_slave.v.
        .TDC_RAW_WIDTH      (4*NUM_CARRY4),
        // Power-up UART divisor; the ARM can change it at run time via 0x40.
        .BAUD_DIV_DEFAULT   (CLK_FREQ_HZ / BAUD_RATE)
    ) u_axi (
        .phase_error_in      (phase_error),
        .nmea_data_in        (mailbox),
        .ocxo_count_in       (ocxo_count),
        .ocxo_delta_in       (ocxo_delta),
        .clk_count_in        (clk_count),
        .clk_delta_in        (clk_delta),
        .drop_count_in       (drop_count),
        .byte_count_in       (byte_count),
        .status_in           (status_dbg),
        .tdc_tap_in          ({16'b0, tap_value}),
        .tdc_raw_in          (tap_raw),

        .dbg_fine_cnt_in     (dbg_fine_cnt),
        .dbg_frac_cnt_in     (dbg_frac_cnt),
        .dbg_diff_cnt_in     (dbg_diff_cnt),
        .dbg_frac_val_in     ({16'd0, ocxo_frac16}),
        .dac_data_out        (dac_data),
        .tdc_tap_period_out  (tap_period),
        .tdc_tap_offset_out  (tap_offset),
        .tdc_raw_index_out   (tdc_raw_index),
        .baud_div_out       (baud_div),
        .p2_count_in        (p2_count),
        .p2_new_in          (p2_new),
        .p2_ocxo_in         (p2_ocxo),
        .p2_phase_lo_in     (p2_phase[31:0]),
        .p2_phase_hi_in     (p2_phase[63:32]),
        .p2_t_out           (p2_t),
        .p2_clr_new         (p2_clr),
        .dac_wr_pulse        (dac_wr_pulse),
        .nmea_ack_pulse      (nmea_ack_pulse),
        .nmea_ack_wdata      (nmea_ack_wdata),

        .S_AXI_ACLK          (clk),
        .S_AXI_ARESETN       (rst_n_sync),
        .S_AXI_AWADDR        (S_AXI_AWADDR),
        .S_AXI_AWPROT        (S_AXI_AWPROT),
        .S_AXI_AWVALID       (S_AXI_AWVALID),
        .S_AXI_AWREADY       (S_AXI_AWREADY),
        .S_AXI_WDATA         (S_AXI_WDATA),
        .S_AXI_WSTRB         (S_AXI_WSTRB),
        .S_AXI_WVALID        (S_AXI_WVALID),
        .S_AXI_WREADY        (S_AXI_WREADY),
        .S_AXI_BRESP         (S_AXI_BRESP),
        .S_AXI_BVALID        (S_AXI_BVALID),
        .S_AXI_BREADY        (S_AXI_BREADY),
        .S_AXI_ARADDR        (S_AXI_ARADDR),
        .S_AXI_ARPROT        (S_AXI_ARPROT),
        .S_AXI_ARVALID       (S_AXI_ARVALID),
        .S_AXI_ARREADY       (S_AXI_ARREADY),
        .S_AXI_RDATA         (S_AXI_RDATA),
        .S_AXI_RRESP         (S_AXI_RRESP),
        .S_AXI_RVALID        (S_AXI_RVALID),
        .S_AXI_RREADY        (S_AXI_RREADY)
    );


    //==========================================================================
    // ocxo_frac: PPS 边沿在一个 OCXO 周期内的归一化位置
    //==========================================================================
    ocxo_frac u_ocxo_frac (
        .clk           (clk),
        .rst_n         (rst_n_sync),
        .ocxo_in       (ocxo_in),
        .fine_fraction (fine_fraction),
        .fine_valid    (fine_valid),
        .tap_valid     (tap_valid),
        .pps_in        (pps_in),
        .frac          (ocxo_frac16),
        .frac_valid    (ocxo_frac_valid),
        .sub_dbg       (ocxo_sub_dbg)
    );

endmodule
