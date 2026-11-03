//=============================================================================
// gnss_axi_ip_slave.v -- AXI4-Lite register file for the AX7010 GPSDO
//-----------------------------------------------------------------------------
// This is the ONLY module the ARM talks to. Everything else is behind it.
//
//==============================================================================
// REGISTER MAP  (byte offsets from the IP base address)
//==============================================================================
//  0x00  R    PHASE_ERR      s32, units of reference period/256 (39.06 ps at a
//                            100 MHz reference).
//                            POSITIVE = OCXO SLOW, NEGATIVE = OCXO FAST.
//                            See phase_error.v for the derivation.
//  0x04  R/W  NMEA_DATA      bit8 = NEW_DATA, bits7:0 = ASCII character.
//                            Write 0x0000_0100 to clear bit8 (write-1-to-clear).
//  0x08  R/W  DAC_VALUE      bits11:0 -> MCP4921. A write also fires exactly one
//                            SPI frame.
//  0x0C  R/W  TDC_TAP_PERIOD taps of the delay line spanning one ref period.
//  0x10  R/W  TDC_TAP_OFFSET taps of fixed capture delay to subtract.
//  0x14  R    OCXO_COUNT     OCXO edges in the last 1PPS interval.
//                            THE MEASUREMENT: ideally 20_000_000 for 20 MHz.
//  0x18  R    OCXO_DELTA     s32, OCXO_COUNT - NOMINAL_OCXO (diagnostic:
//                            positive means the OCXO is FAST).
//  0x1C  R    CLK_COUNT      100 MHz reference clocks in the same interval.
//                            Ideally 100_000_000. A DIAGNOSTIC, not the loop
//                            input -- it measures the reference, not the OCXO.
//  0x20  R    CLK_DELTA      s32, CLK_COUNT - NOMINAL_CYCLES (diagnostic).
//  0x24  R    DROP_COUNT     NMEA bytes discarded because the ARM was late.
//  0x28  R    BYTE_COUNT     NMEA bytes accepted.
//  0x2C  R    STATUS         bit0 meas_valid, bit1 locked, bit2 pps_lost,
//                            bit3 sat_flag, bit4 tdc_saturated, bit5 frame_err,
//                            bit6 spi_busy, bit7 diff_valid, bit8 tap_max
//  0x30  R    TDC_TAP        raw tap value of the last measurement
//  0x34  R/W  TDC_RAW_INDEX  selects which 32-bit word of the captured
//                            thermometer pattern the next 0x38 read returns
//  0x38  R    TDC_RAW_DATA   tap_raw[32*index +: 32]
//  0x3C  R    VERSION        identity constant
//  0x40  R/W  BAUD_DIV       clk cycles per UART bit, i.e. CLK_FREQ_HZ/baud.
//                            Powers up at BAUD_DIV_DEFAULT. The ARM writes this
//                            so the UART baud can be changed without rebuilding
//                            the bitstream -- necessary because the GNSS module's
//                            baud is not always 9600 (115200 = 12 x 9600 is very
//                            common, and aliases into plausible-looking garbage).
//
// WHY THE RAW PATTERN IS INDEXED RATHER THAN SPLIT
// The capture is 4*NUM_CARRY4 bits (512 by default). Exposing it as two 32-bit
// registers, as an earlier revision did, silently showed only the first 64 taps
// -- which are the NEAR end of the line, i.e. exactly the taps that carry the
// least information. The far end, where the wavefront actually sits at capture
// time, was unreachable. An index register reaches every tap with only two
// addresses.
//
//==============================================================================
// THE WRITE HANDSHAKE -- AND THE BUG THIS FILE EXISTS TO NOT REPEAT
//==============================================================================
// A write completes only when BOTH the address channel (AWVALID) and the data
// channel (WVALID) have been accepted. The inherited version of this file gated
// its register update on
//
//     if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID)
//
// which looks plausible and is WRONG. WREADY is deasserted for exactly one cycle
// after each accepted beat and is otherwise HIGH, so the term ~axi_wready is
// false on almost every cycle -- including the cycle on which the transaction
// actually commits. The result was that NO register write ever took effect while
// reads worked perfectly, and the symptom was the most confusing possible one:
// the bus handshake completed, BRESP came back OKAY, and the value simply never
// changed.
//
// MEASURED, before the fix (write to 0x08, DAC word 0xABC):
//     [AXI] AWVALID=1 AWREADY=1 WVALID=1 WREADY=1 | commit=1 aw_word=2 dac_pulse=0
// i.e. the transaction committed, decoded to the right word, and still produced
// no strobe.
//
// The replacement uses one flag per channel, set when that channel's beat is
// accepted, and performs the update on the cycle BOTH flags are held. This is
// insensitive to the order in which the master presents AW and W -- which the
// original also got wrong, as it required AWVALID and WVALID in the same cycle,
// so a master presenting them separately would have deadlocked.
//
//==============================================================================
// WHY THE WRITE STROBES ARE ONE CLOCK WIDE
//==============================================================================
// 0x08's write starts an SPI frame and 0x04's write clears a flag. Both are EDGE
// events, not levels: if a strobe stayed high for the several cycles a write can
// occupy, the DAC would be re-programmed several times per write. The strobe is a
// register explicitly cleared every cycle inside the same always block that can
// set it, so exactly one frame goes out per write.
//
// NOTE: every comment and string in this file is ASCII on purpose.
//=============================================================================

`timescale 1ns / 1ps

module gnss_axi_ip_slave #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 12,       // 4 KB of register space
    // Width of the captured TDC thermometer pattern. It MUST match tdc_carry's
    // 4*NUM_CARRY4 exactly.
    //
    // An earlier version hard-coded 64 here while the top level padded a
    // 4*NUM_CARRY4-bit vector down to 64 with
    //     {{(64 - 4*NUM_CARRY4){1'b0}}, tap_raw}
    // which is a NEGATIVE replication count at the real value of NUM_CARRY4 (128)
    // and therefore a synthesis error (Synth 8-693). Matching the widths removes
    // the cast entirely instead of papering over it.
    parameter integer TDC_RAW_WIDTH = 512,
    // Power-up value of the 0x40 BAUD_DIV register, in clk cycles per bit.
    // The ARM writes CLK_FREQ_HZ / wanted_baud to change it at run time.
    parameter integer BAUD_DIV_DEFAULT = 10416
)(
    // ---- user logic: readable status inputs ----
    input  wire signed [31:0] phase_error_in,
    input  wire        [31:0] nmea_data_in,
    input  wire        [31:0] ocxo_count_in,
    input  wire signed [31:0] ocxo_delta_in,
    input  wire        [31:0] clk_count_in,
    input  wire signed [31:0] clk_delta_in,
    input  wire        [31:0] drop_count_in,
    input  wire        [31:0] byte_count_in,
    input  wire        [31:0] status_in,
    input  wire        [31:0] tdc_tap_in,
    input  wire [TDC_RAW_WIDTH-1:0] tdc_raw_in,

    // ---- user logic: writable outputs ----
    // ---- 诊断计数器（2026-10-27 新增，只读）------------------------------
    // 用来判断小数通路卡在哪一级。四个寄存器都是自由运行的计数器/快照，
    // 不影响任何控制逻辑。
    input  wire [31:0]  dbg_fine_cnt_in,   // 0x60: fine_valid 累计脉冲
    input  wire [31:0]  dbg_frac_cnt_in,   // 0x64: frac_valid 累计脉冲
    input  wire [31:0]  dbg_diff_cnt_in,   // 0x68: diff_valid 累计脉冲
    input  wire [31:0]  dbg_frac_val_in,   // 0x6C: 当前 frac 的快照
    output reg         [11:0] dac_data_out,
    output reg         [15:0] tdc_tap_period_out,
    output reg         [15:0] tdc_tap_offset_out,
    output reg         [31:0] tdc_raw_index_out,
    output reg         [31:0] baud_div_out,        // 0x40: clk cycles per bit

    // ---- 第二路(慢环) ----
    input  wire        [15:0] p2_count_in,         // 0x54: 窗口内已过的 1PPS 数
    input  wire               p2_new_in,           // 0x50 bit0
    input  wire signed [31:0] p2_ocxo_in,          // 0x4C
    input  wire        [31:0] p2_phase_lo_in,      // 0x48: 相位差累加 低 32 位
    input  wire        [31:0] p2_phase_hi_in,      // 0x4C: 相位差累加 高 32 位
    output reg         [15:0] p2_t_out,            // 0x44: 计数周期 T(秒)
    output reg                p2_clr_new,          // 写 0x50 -> 清 NEW 标志
    output reg                dac_wr_pulse,         // one-clock pulse on 0x08 write
    output reg                nmea_ack_pulse,       // one-clock pulse on 0x04 write
    output reg         [31:0] nmea_ack_wdata,       // write data, for the W1C mask

    // ---- AXI4-Lite slave ----
    input  wire                          S_AXI_ACLK,
    input  wire                          S_AXI_ARESETN,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR,
    input  wire [2:0]                    S_AXI_AWPROT,
    input  wire                          S_AXI_AWVALID,
    output wire                          S_AXI_AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  wire                          S_AXI_WVALID,
    output wire                          S_AXI_WREADY,
    output wire [1:0]                    S_AXI_BRESP,
    output wire                          S_AXI_BVALID,
    input  wire                          S_AXI_BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR,
    input  wire [2:0]                    S_AXI_ARPROT,
    input  wire                          S_AXI_ARVALID,
    output wire                          S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_RDATA,
    output wire [1:0]                    S_AXI_RRESP,
    output wire                          S_AXI_RVALID,
    input  wire                          S_AXI_RREADY
);

    localparam [31:0] VERSION_VALUE = 32'h0003_0002;   // v3.2: 20MHz nominal OCXO + sub-cycle scaling + sign fix (2026-10-23)

    localparam integer ADDR_LSB          = 2;
    localparam integer OPT_MEM_ADDR_BITS = 5;          // 32 words = 0x00..0x7C

    //==========================================================================
    // WRITE PATH
    //==========================================================================
    reg        aw_flag;
    reg        w_flag;
    reg [11:0] aw_addr_lat;
    reg [31:0] w_data_lat;
    reg        bvalid_r;

    wire aw_accept = S_AXI_AWVALID && !aw_flag;
    wire w_accept  = S_AXI_WVALID  && !w_flag;

    wire wr_commit = (aw_flag || aw_accept) && (w_flag || w_accept);

    wire [11:0] commit_addr = aw_flag ? aw_addr_lat : S_AXI_AWADDR[11:0];
    wire [31:0] commit_data = w_flag  ? w_data_lat  : S_AXI_WDATA;

    assign S_AXI_AWREADY = aw_accept;
    assign S_AXI_WREADY  = w_accept;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            aw_flag            <= 1'b0;
            w_flag             <= 1'b0;
            aw_addr_lat        <= 12'd0;
            w_data_lat         <= 32'd0;
            dac_data_out       <= 12'd0;
            tdc_tap_period_out <= 16'd0;
            tdc_tap_offset_out <= 16'd0;
            tdc_raw_index_out  <= 32'd0;
            baud_div_out       <= BAUD_DIV_DEFAULT[15:0];
            p2_t_out           <= 16'd1;
            p2_clr_new         <= 1'b0;
            dac_wr_pulse       <= 1'b0;
            nmea_ack_pulse     <= 1'b0;
            nmea_ack_wdata     <= 32'd0;
            bvalid_r           <= 1'b0;
        end else begin
            // One-clock strobes: cleared every cycle, set only on a commit.
            dac_wr_pulse   <= 1'b0;
            nmea_ack_pulse <= 1'b0;
            p2_clr_new     <= 1'b0;   // 0x50 的清标志脉冲, 只保持一拍

            if (aw_accept) begin
                aw_flag     <= 1'b1;
                aw_addr_lat <= S_AXI_AWADDR[11:0];
            end
            if (w_accept) begin
                w_flag     <= 1'b1;
                w_data_lat <= S_AXI_WDATA;
            end

            if (wr_commit) begin
                case (commit_addr[ADDR_LSB+OPT_MEM_ADDR_BITS-1 : ADDR_LSB])
                    1: begin
                        // 0x04: record the write data so nmea_mailbox can apply the
                        // write-1-to-clear mask on bit 8. The flag is NOT cleared
                        // here -- only the mailbox knows whether bit 8 was set.
                        nmea_ack_wdata <= commit_data;
                        nmea_ack_pulse <= 1'b1;
                    end
                    2: begin
                        // 0x08: latch the DAC word and fire exactly one frame.
                        dac_data_out <= commit_data[11:0];
                        dac_wr_pulse <= 1'b1;
                    end
                    3: tdc_tap_period_out <= commit_data[15:0];
                    4: tdc_tap_offset_out <= commit_data[15:0];
                    13:   tdc_raw_index_out  <= commit_data[7:0];   // 0x34
                    16:   baud_div_out       <= commit_data;        // 0x40 BAUD_DIV
                    17:   p2_t_out           <= (commit_data[15:0] == 16'd0) ? 16'd1
                                                                    : commit_data[15:0];  // 0x44 P2_T
                    20:   p2_clr_new         <= 1'b1;                // 0x50 P2_STATUS: 写 1 清 NEW
                    default: ;   // read-only or unmapped: accepted and ignored
                endcase

                aw_flag  <= 1'b0;
                w_flag   <= 1'b0;
                bvalid_r <= 1'b1;
            end else if (bvalid_r && S_AXI_BREADY) begin
                bvalid_r <= 1'b0;
            end
        end
    end

    assign S_AXI_BRESP  = 2'b00;      // OKAY
    assign S_AXI_BVALID = bvalid_r;

    //==========================================================================
    // READ PATH
    //==========================================================================
    reg        ar_flag;
    reg [11:0] ar_addr_lat;
    reg        rvalid_r;

    wire ar_accept = S_AXI_ARVALID && !ar_flag;

    assign S_AXI_ARREADY = ar_accept;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            ar_flag     <= 1'b0;
            ar_addr_lat <= 12'd0;
            rvalid_r    <= 1'b0;
        end else begin
            if (ar_accept) begin
                ar_flag     <= 1'b1;
                ar_addr_lat <= S_AXI_ARADDR[11:0];
            end

            if (ar_flag && !rvalid_r) begin
                rvalid_r <= 1'b1;
                ar_flag  <= 1'b0;
            end else if (rvalid_r && S_AXI_RREADY) begin
                rvalid_r <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Indexed view of the captured thermometer pattern
    //
    // A variable part-select of a vector longer than 32 bits is synthesised as a
    // wide multiplexer. The result is REGISTERED so the multiplexer occupies its
    // own clock period instead of sitting combinationally between the address
    // latch and RDATA.
    //
    // Out-of-range indices return zero rather than wrapping, so a software bug
    // reads as "no data" instead of as a plausible-looking wrong word.
    //--------------------------------------------------------------------------
    localparam integer RAW_WORDS = TDC_RAW_WIDTH / 32;
    localparam integer RAW_IDX_W = 4;      // enough for 16 words

    reg [31:0] tdc_raw_word;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            tdc_raw_word <= 32'd0;
        end else if (tdc_raw_index_out[RAW_IDX_W-1:0] < RAW_WORDS) begin
            tdc_raw_word <= tdc_raw_in[tdc_raw_index_out[RAW_IDX_W-1:0]*32 +: 32];
        end else begin
            tdc_raw_word <= 32'd0;
        end
    end

    //--------------------------------------------------------------------------
    // Read multiplexer, combinational on the LATCHED address: stable while
    // RVALID is asserted, so no extra pipeline stage is needed.
    //--------------------------------------------------------------------------
    reg [31:0] rdata_r;

    always @(*) begin
        case (ar_addr_lat[ADDR_LSB+OPT_MEM_ADDR_BITS-1 : ADDR_LSB])
            0: rdata_r = phase_error_in;
            1: rdata_r = nmea_data_in;
            2: rdata_r = {20'b0, dac_data_out};
            3: rdata_r = {16'b0, tdc_tap_period_out};
            4: rdata_r = {16'b0, tdc_tap_offset_out};
            5: rdata_r = ocxo_count_in;
            6: rdata_r = ocxo_delta_in;
            7: rdata_r = clk_count_in;
            8: rdata_r = clk_delta_in;
            9: rdata_r = drop_count_in;
            10: rdata_r = byte_count_in;
            11: rdata_r = status_in;
            12: rdata_r = tdc_tap_in;
            13: rdata_r = tdc_raw_index_out;
            14: rdata_r = tdc_raw_word;
            15: rdata_r = VERSION_VALUE;
            16: rdata_r = baud_div_out;         // 0x40 BAUD_DIV
            17: rdata_r = {16'd0, p2_t_out};    // 0x44 P2_T
            18: rdata_r = p2_phase_lo_in;       // 0x48 P2_PHASE_LO
            19: rdata_r = p2_phase_hi_in;       // 0x4C P2_PHASE_HI
            22: rdata_r = p2_ocxo_in;           // 0x58 P2_OCXO (对照用)
            24: rdata_r = dbg_fine_cnt_in;      // 0x60 fine_valid 计数
            25: rdata_r = dbg_frac_cnt_in;      // 0x64 frac_valid 计数
            26: rdata_r = dbg_diff_cnt_in;      // 0x68 diff_valid 计数
            27: rdata_r = dbg_frac_val_in;      // 0x6C frac 快照
            20: rdata_r = {31'd0, p2_new_in};   // 0x50 P2_STATUS: bit0 = NEW
            21: rdata_r = {16'd0, p2_count_in}; // 0x54 P2_COUNT
            default: rdata_r = 32'd0;
        endcase
    end

    assign S_AXI_RDATA  = rdata_r;
    assign S_AXI_RRESP  = 2'b00;      // OKAY
    assign S_AXI_RVALID = rvalid_r;

endmodule
