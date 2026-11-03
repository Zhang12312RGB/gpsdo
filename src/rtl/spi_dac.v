//=============================================================================
// spi_dac.v -- parameterised SPI DAC driver
//-----------------------------------------------------------------------------
// Drives a 12/16-bit SPI DAC from a value written by the ARM.
//
//==============================================================================
// FIXES relative to the original project file
//==============================================================================
// [FIX-1] MOSI setup time was violated (the DAC received the WRONG data)
//   old: the SEND state updated either `mosi` or `shift_reg` on almost every
//        clock, and the shift happened at the SCK falling edge. The result was
//        that the bit presented during one SCK period was the bit that had
//        already been shifted out, so the 16-bit frame arrived at the DAC with
//        every bit one position late.
//
//        That is the worst class of bug for this project: the DAC still clocks
//        in "a" value, so nothing looks broken -- the oscillator is simply
//        driven by the wrong control word, and the loop never converges.
//
//   new: SCK idles low, MOSI is changed only WHILE SCK IS LOW, and each bit is
//        held for a full SCK period. Data is therefore stable long before the
//        rising edge the DAC samples on, which is SPI mode 0 and is what the
//        MCP492x family requires.
//
// [FIX-2] unconditional re-transmission
//   old: the top level fired `start` every 0.5 ms forever, so the DAC was
//        rewritten 2000 times a second whether or not the value had changed.
//        That is needless activity on the control voltage of the very oscillator
//        being disciplined.
//
//   new: this module only transmits when asked. Deciding WHEN to ask is the top
//        level's job (see gnss_system_top: transmit on a value change, plus a
//        slow heartbeat). A driver should not invent a policy.
//
// [FIX-3] reset style: async assert / sync release, consistent with the project.
//
//==============================================================================
// PARAMETERS -- why this module is parameterised
//==============================================================================
// The DAC part was not fixed when this was written, so nothing about the frame
// is hard-coded:
//
//   DAC_BITS  : number of data bits (12 for MCP4921/MCP4922, 16 for AD5662)
//   DAC_HDR   : the bits that precede the data in the frame. For the MCP492x
//               family this is 4'b0011 = "write to DAC A, unbuffered, 1x gain,
//               active mode". A different part simply passes a different value.
//   SPI_DIV   : system clocks per half SCK period. SCK = CLK_FREQ / (2*SPI_DIV).
//               The default gives about 6.25 MHz at 100 MHz, comfortably inside
//               the 20 MHz limit of the MCP4921 and slow enough to be forgiving
//               about wiring.
//   CS_SETUP  : extra clocks between CS falling and the first SCK edge.
//   CS_HOLD   : extra clocks between the last SCK edge and CS rising.
//
// Changing DAC means changing parameters, never logic.
//
// NOTE: every $display / comment string is ASCII on purpose (xvlog reads the
// source file with the system codepage; UTF-8 Chinese would print as garbage).
//=============================================================================

`timescale 1ns / 1ps

module spi_dac #(
    parameter integer DAC_BITS  = 16,        // data width of the DAC (DAC8531 = 16)
    parameter integer DAC_HDR   = 8'h00,     // DAC8531: 6 don't-care + PD1=0 + PD0=0 (normal)
    parameter integer DAC_HDR_BITS = 8,      // header width (MCP492x used 4)
    parameter integer SPI_DIV   = 32,        // clk / (2*SPI_DIV) = SCK frequency
    parameter integer CS_SETUP  = 4,         // clocks of CS setup before SCK
    parameter integer CS_HOLD   = 4,         // clocks of CS hold after SCK
    // LDAC handling. The pin does not exist on single-channel parts (MCP4921),
    // so the default is "not driven" and the port can be left unconnected.
    //   0 = OFF   : ldac_n held high; the DAC latches on the CS rising edge
    //   1 = FRAME : ldac_n low for the whole frame, which is also the way to
    //               update both channels of a dual part (MCP4922) together
    //
    // Both modes are verified by the testbench. A third "pulse after CS" mode
    // was considered and deliberately not implemented: it would need an extra
    // state, and without the part in hand its timing could not be verified.
    // Shipping an unverifiable option is worse than not shipping it.
    parameter integer LDAC_MODE = 0
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,        // one-clock pulse: send now
    input  wire [DAC_BITS-1:0]     dac_data,     // value to send
    output reg                     sck,
    output reg                     mosi,
    output reg                     cs_n,
    output reg                     ldac_n,       // leave unconnected if unused
    output reg                     busy         // high while a frame is shifting
);

    localparam integer LDAC_OFF   = 0;
    localparam integer LDAC_FRAME = 1;

    // The header is 4 bits for the MCP492x family, so the frame is DAC_BITS + 4.
    // A part with a different header width only needs this constant changed.
    localparam integer FRAME_LEN = DAC_BITS + DAC_HDR_BITS;

    // state encoding
    localparam [2:0] S_IDLE  = 3'd0,
                     S_SETUP = 3'd1,
                     S_LOW   = 3'd2,     // SCK low, present the bit
                     S_HIGH  = 3'd3,     // SCK high, DAC samples
                     S_HOLD  = 3'd4;

    reg [2:0]  state;
    reg [FRAME_LEN-1:0] shift;     // wide enough for any supported frame
    reg [4:0]  bits_left;
    reg [15:0] div_cnt;
    reg [15:0] wait_cnt;

    wire [FRAME_LEN-1:0] frame = {DAC_HDR[DAC_HDR_BITS-1:0], dac_data};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            shift     <= {FRAME_LEN{1'b0}};
            bits_left <= 5'd0;
            div_cnt   <= 16'd0;
            wait_cnt  <= 16'd0;
            sck       <= 1'b0;
            mosi      <= 1'b0;
            cs_n      <= 1'b1;         // idle: chip not selected
            ldac_n    <= 1'b1;         // idle: DAC not being loaded
            busy      <= 1'b0;
        end else begin
            case (state)

                // ---------------- waiting for a request ----------------
                S_IDLE: begin
                    sck    <= 1'b0;
                    cs_n   <= 1'b1;
                    ldac_n <= 1'b1;
                    busy   <= 1'b0;
                    if (start) begin
                        shift     <= frame;
                        bits_left <= FRAME_LEN[4:0];
                        wait_cnt  <= CS_SETUP[15:0];
                        cs_n      <= 1'b0;        // select the DAC
                        // in FRAME mode the load line goes low with the chip
                        // select and stays low for the whole transfer
                        ldac_n    <= (LDAC_MODE == LDAC_FRAME) ? 1'b0 : 1'b1;
                        busy      <= 1'b1;
                        state     <= S_SETUP;
                    end
                end

                // ---------------- CS setup time ----------------
                S_SETUP: begin
                    if (wait_cnt == 16'd0) begin
                        // present the first bit while SCK is low
                        mosi    <= shift[FRAME_LEN-1];
                        div_cnt <= SPI_DIV[15:0];
                        state   <= S_LOW;
                    end else begin
                        wait_cnt <= wait_cnt - 16'd1;
                    end
                end

                // ---------------- SCK low: hold the bit ----------------
                S_LOW: begin
                    sck <= 1'b0;
                    // DAC8531 samples DIN on the FALLING edge, so MOSI must be
                    // stable for the whole low phase. Present the bit HERE, not at
                    // the falling edge (that would shift every bit by one).
                    mosi <= shift[FRAME_LEN-1];
                    if (div_cnt == 16'd0) begin
                        sck     <= 1'b1;          // rising edge: DAC ignores
                        div_cnt <= SPI_DIV[15:0];
                        state   <= S_HIGH;
                    end else begin
                        div_cnt <= div_cnt - 16'd1;
                    end
                end

                // ---------------- SCK high: shift to the next bit ----------------
                S_HIGH: begin
                    if (div_cnt == 16'd0) begin
                        sck     <= 1'b0;
                        shift   <= {shift[FRAME_LEN-2:0], 1'b0};
                        bits_left <= bits_left - 5'd1;

                        if (bits_left == 5'd1) begin
                            wait_cnt <= CS_HOLD[15:0];
                            state    <= S_HOLD;
                        end else begin
                            // MOSI is updated in S_LOW, so it is already stable
                            // when this falling edge latches it.
                            div_cnt <= SPI_DIV[15:0];
                            state   <= S_LOW;
                        end
                    end else begin
                        div_cnt <= div_cnt - 16'd1;
                    end
                end

                // ---------------- CS hold, then done ----------------
                S_HOLD: begin
                    if (wait_cnt == 16'd0) begin
                        cs_n   <= 1'b1;      // in FRAME mode the DAC latches on
                        ldac_n <= 1'b1;      // this rising edge
                        sck    <= 1'b0;
                        busy   <= 1'b0;
                        state  <= S_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt - 16'd1;
                    end
                end

                default: state <= S_IDLE;

            endcase
        end
    end

endmodule
