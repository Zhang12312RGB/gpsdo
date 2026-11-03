//=============================================================================
// uart_rx.v -- 8N1 UART receiver with start-bit validation and frame check
//-----------------------------------------------------------------------------
// Receives one byte at a time from the GNSS module's NMEA stream.
//   Frame format: 1 start bit (low), 8 data bits LSB first, 1 stop bit (high)
//   Baud rate is a parameter; the default matches the usual 9600 of GNSS modules.
//
//==============================================================================
// FIXES relative to the original project file
//==============================================================================
// [FIX-1] the start bit was never validated
//   old:  if (rx_sync[1] == 1'b0) begin      // any low level starts a byte
//   A single glitch started a reception and produced a bogus byte with `valid`
//   asserted, so the ARM parsed garbage.
//   new:  the FSM waits half a bit period and samples again; only a line that
//         is still low is accepted as a real start bit. This is the standard
//         oversampling start-bit check.
//
// [FIX-2] no frame error detection
//   old:  the stop bit was never checked, so a baud mismatch or a disturbed
//         byte was reported as good data.
//   new:  the stop bit is sampled and must be high; otherwise the byte is
//         dropped and `frame_err` pulses. The ARM can then know the stream is
//         unreliable instead of silently consuming corrupt data.
//
// [FIX-3] undocumented one-cycle offset in the sampling instants
//   The old code sampled with a counting scheme that happened to land one
//   clock early, with nothing in the comments explaining it.
//   new:  every sample lands on a BIT CENTRE, so the margin to either edge is
//         half a bit period. See the section below -- this also required
//         compensating for the input synchronizer.
//
// [FIX-4] reset style: now async assert / sync release, like the rest of the
//   project (rst_n comes from the PS peripheral_aresetn).
//
// [FIX-5] synchronizer latency was not compensated  <-- found while writing 3b
//   The two-flop input synchronizer delays the observed line by about two clock
//   cycles. The start-bit centre is therefore detected SYNC_CYCLES later than
//   the real centre, which shifts every later sample by the same amount toward
//   the bit edges.
//
//   At the real operating point this is negligible -- 100 MHz / 9600 baud is
//   about 10416 clocks per bit, so two clocks are 0.02% of a bit. But it
//   becomes a real error whenever the oversampling ratio is small: at 10 clocks
//   per bit those two clocks are 20% of a bit, which is enough to push a sample
//   past a bit boundary.
//
//   The receiver therefore subtracts SYNC_CYCLES when it starts counting to the
//   first data bit's centre, and the testbench uses a realistic oversampling
//   ratio (40) instead of an unrealistically small one. The general lesson:
//   when you shrink a time scale to speed up simulation, keep the RATIOS that
//   the logic depends on -- shrinking the oversampling ratio silently removes
//   the margin the design was relying on.
//
//==============================================================================
// WHY SAMPLING AT THE CENTRE MATTERS
//==============================================================================
// A UART frame has no clock line: the receiver free-runs and must guess where
// each bit sits, using the falling edge of the start bit as its only reference.
// Sampling at bit EDGES lets a small baud mismatch accumulate over the ten bits
// of the frame, and the last data bits get read on the wrong side of a
// transition. Sampling at the CENTRE gives half a bit period of margin in both
// directions, tolerating a baud error of roughly +/-5%.
//
//   bit period      : BAUD_CNT clocks
//   start-bit check : at BAUD_CNT/2 after the line first reads low
//   first data bit  : half a bit later, i.e. exactly at its centre
//   later data bits : one full bit period apart, so also at their centres
//
// NOTE: every $display / comment string is ASCII on purpose (xvlog reads the
// source file with the system codepage; UTF-8 Chinese would print as garbage).
//=============================================================================

`timescale 1ns / 1ps

module uart_rx #(
    parameter integer CLK_FREQ    = 100_000_000,   // system clock, Hz
    parameter integer BAUD_RATE   = 9600,
    parameter integer SYNC_CYCLES = 2              // input synchronizer latency
)(
    input  wire       clk,
    input  wire       rst_n,        // async reset, active low
    // Baud divisor in clk cycles per bit, i.e. CLK_FREQ/BAUD_RATE.
    // Written by the ARM through AXI 0x40; powers up at the compile-time value.
    input  wire [15:0] baud_cnt_in,
    input  wire       rx,           // serial input, idle high
    output reg  [7:0] data,         // received byte, valid when `valid` pulses
    output reg        valid,        // one-clock pulse: a good byte is on `data`
    output reg        frame_err     // one-clock pulse: stop bit was not high
);

    // -------------------------------------------------------------------------
    // Baud divisor -- RUNTIME PROGRAMMABLE  [FIX-8]
    //
    // Why this exists: the module's baud rate is NOT necessarily 9600.  Many
    // GNSS modules and most USB/eval boards ship at 115200, which is exactly
    // 12 x 9600 -- so an FPGA hard-wired to 9600 aliases the stream instead of
    // failing cleanly, and the symptom is a plausible-looking byte stream that
    // is pure garbage (NMEA is ASCII, so bit 7 is never set, but the aliased
    // bytes have bit 7 set about half the time).
    //
    // Rebuilding the bitstream once per candidate baud is far too slow, so the
    // divisor is a register now: the ARM writes CLK_FREQ_HZ / wanted_baud to
    // AXI 0x40 BAUD_DIV and can sweep every candidate in one power-up.
    //
    // It powers up at the compile-time value, so behaviour is unchanged until
    // software writes it, and a silly value is clamped instead of wedging the
    // receiver (a 0 or 1 would otherwise make the bit period a few clocks).
    // -------------------------------------------------------------------------
    localparam [15:0] SYNC_CYCLES16 = SYNC_CYCLES;

    wire [15:0] baud_cnt = (baud_cnt_in < 16'd16) ? 16'd16 : baud_cnt_in;
    wire [15:0] half_cnt = {1'b0, baud_cnt[15:1]};               // baud_cnt / 2
    wire [15:0] to_first = baud_cnt - SYNC_CYCLES16;             // 2*half - SYNC
    //------------------------------------------------------------------------
    // Re-arm threshold: 1/8 of a bit period, NOT half a bit.  [FIX-10]
    //
    // WHY 1/8 AND NOT 1/2
    //   The receiver samples the stop bit at 9.5 bit times and only THEN
    //   returns to S_IDLE, where idle_cnt starts counting.  A NMEA sentence is a
    //   GAPLESS stream, so the next character's start edge arrives at exactly
    //   10 bit times -- i.e. only HALF a bit period after the stop sample.
    //   At 9600 baud that window is 5208 clocks, and the previous threshold was
    //   half_cnt = 5208: the re-arm won by ~8 clocks out of 5200 (0.15%), so
    //   roughly a quarter of all characters were framed from a random bit
    //   position.  Measured symptom: 72% printable bytes at 9600 instead of
    //   100%, frame_err latched, and every other candidate baud gave 0%.
    //   A 1/8 bit threshold keeps a 3.6x margin and still rejects glitches,
    //   because idle_cnt is already forced to 0 while a frame is in progress.
    //------------------------------------------------------------------------
    wire [15:0] rearm    = {3'b000, baud_cnt[15:3]};             // 1/8 bit period

    // -------------------------------------------------------------------------
    // Input synchronizer: rx is asynchronous to clk.
    // -------------------------------------------------------------------------
    reg [1:0] rx_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rx_sync <= 2'b11;            // idle line is high
        else
            rx_sync <= {rx_sync[0], rx};
    end

    wire rx_bit = rx_sync[1];

    // -------------------------------------------------------------------------
    // Receiver FSM
    // -------------------------------------------------------------------------
    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [15:0] cnt;          // baud counter
    reg [3:0]  bit_idx;      // 0..7 data bits, then 8 marks the stop bit
    reg [7:0]  shift;        // assembled byte
    reg [1:0]  state;

    // Clocks from the observed start-bit centre to the first data-bit centre.
    //
    // Work through the cycle counts, because the sign here is easy to get
    // backwards and both directions look plausible:
    //
    //   t=0   : the real falling edge of the start bit (rx goes low)
    //   t=2   : rx_sync has captured it (two flops) -> FSM leaves S_IDLE and
    //           loads cnt = HALF_CNT, so it starts counting at t=2
    //   t=2+HALF_CNT : cnt reaches 0 -> this is the OBSERVED "centre" of the
    //           start bit, which is SYNC_CYCLES later than the true one
    //
    // The true centre of data bit 0 is at t = HALF_CNT + HALF_CNT = 2*HALF_CNT.
    // Counting from the observed centre at t = 2 + HALF_CNT, the remaining
    // distance is therefore
    //
    //   TO_FIRST_DATA = 2*HALF_CNT - (2 + HALF_CNT) = HALF_CNT - SYNC_CYCLES
    //
    // i.e. the shortfall introduced by the synchronizer must be SUBTRACTED.
    // Sampling therefore lands on the centre of data bit 0 and, because every
    // later sample is a full bit period further on, on the centre of every
    // other bit too -- leaving half a bit period of margin on each side.
    //
    // ---------------------------------------------------------------------------
    // CORRECTION -- THE CONSTANT BELOW WAS WRONG AND SILENTLY SHIFTED EVERY BYTE
    // ---------------------------------------------------------------------------
    // The derivation above is sound; the expression that used to follow it was
    // not. This file previously read
    //
    //     localparam integer TO_FIRST_DATA = HALF_CNT - SYNC_CYCLES;
    //
    // which omits the whole remaining half of the start bit. Counting does not
    // begin at the falling edge: it begins at the OBSERVED start-bit centre, which
    // is already HALF_CNT after that edge. From there the distance still to travel
    // to the centre of data bit 0 is
    //
    //     half of the start bit   +   half of data bit 0   -   sync shortfall
    //     HALF_CNT                +   HALF_CNT             -   SYNC_CYCLES
    //
    // so the correct expression is 2*HALF_CNT - SYNC_CYCLES.
    //
    // With HALF_CNT = 5208 at 9600 baud / 100 MHz, the wrong expression gave 5206
    // instead of 10414: data bit 0 was sampled about half a bit period early, i.e.
    // while the start bit was still low, and the whole assembled byte came out
    // shifted by one position.
    //
    // MEASURED, before the fix (first byte of a burst, expecting 0x41 = 'A'):
    //     mailbox held 0x42, byte_count = 1
    // and 0x42 >> 1 == 0x41, which is the signature of a one-bit sampling offset
    // rather than of any handshake or holding problem. A hold-logic bug would have
    // preserved the first byte or lost it entirely; it would not produce a
    // correctly-framed byte with every bit one place out.
    //
    // It survived in the originating project because that project's tests compared
    // against reference data captured with the same defect.
    // ---------------------------------------------------------------------------
    // TO_FIRST_DATA is now derived at runtime (see tow_first above); kept as a

    // -------------------------------------------------------------------------
    // Re-arm guard  [FIX-6]
    //   After a framing error the line is still LOW -- that is exactly what made
    //   the stop bit invalid. If the FSM went straight back to hunting for a
    //   start bit, that leftover low level would be taken for the beginning of a
    //   new frame. The receiver would then sample eight idle-high bits, find a
    //   high stop bit, and report a perfectly legal 0xFF byte out of nothing.
    //
    //   MEASURED, in the isolated framing test:
    //       [RX-ERR  ] framing error #1 at time 395375 ns
    //       [RX-VALID] byte #2 = 0xff at time 487625 ns
    //   i.e. a phantom 0xFF appeared out of a deliberately broken frame.
    //
    //   Two earlier attempts failed and the reasons matter:
    //
    //     1) "set a flag whenever rx_bit is high" -- one cycle after the error
    //        the line is high again, so the flag was already back and the
    //        phantom byte returned.
    //
    //     2) a counter compared against a part-select of a 32-bit localparam --
    //        the guard never became true and the receiver accepted nothing at
    //        all.
    //
    //     3) an `else if` chain in which the "re-arm" term had higher priority
    //        than the "disarm" term -- the disarm could be masked, and the
    //        phantom byte survived (this is the version that produced the
    //        measurement above).
    //
    //   The version below removes every one of those traps:
    //     * the disarm term has PRIORITY and is not part of an else-if chain.
    //
    // ---------------------------------------------------------------------------
    // CORRECTION  [FIX-7]  -- THE GUARD DEMANDED A FULL BIT PERIOD OF IDLE,
    //                         WHICH REAL NMEA TRAFFIC NEVER PROVIDES
    // ---------------------------------------------------------------------------
    // The guard used to be a shift register BAUD_CNT bits wide, so it required a
    // FULL BIT PERIOD of contiguous idle (10416 clocks at 100 MHz / 9600) before
    // another start bit would be accepted:
    //
    //     localparam integer IDLE_CYCLES = BAUD_CNT;   // one full bit period
    //     reg [IDLE_CYCLES-1:0] idle_shift;
    //     wire line_idle = &idle_shift;
    //
    // One stop bit IS one bit period, and a UART transmitter sends characters
    // gaplessly: the next start bit begins the instant the stop bit ends.  The
    // available idle was therefore 10416.67 clocks against a requirement of
    // 10416 -- a margin of 0.67 clocks, i.e. 0.0064%.
    //
    // NMEA makes that the worst case rather than a corner case: every character
    // is ASCII, so bit 7 is ALWAYS 0.  The line is LOW for the last data bit and
    // HIGH only for the stop bit, so there is never a second idle bit to hide in.
    //
    // Hardware consequence: any module whose baud is even ~0.007% fast never
    // re-armed between bytes.  The receiver sat disarmed for the whole sentence
    // and woke only in the (long) gap BETWEEN sentences, so roughly one byte per
    // sentence was captured.  byte_count crawled, mailbox read back as
    // 0x00000000 in almost every ARM poll, and the ARM concluded: no NMEA.
    //
    // It survived simulation because uart_send() in the testbench inserted two
    // extra bit times of idle before every byte, commented 'satisfy the re-arm
    // guard'.  The simulated stream was never gapless, so the one case that
    // mattered was the one case never exercised.
    //
    // The fix needs BOTH changes below; either one alone is wrong:
    //
    //   1. the idle counter only counts while the FSM is in S_IDLE, so a long run
    //      of high bits INSIDE a byte (the top bits of 0xF0 plus its stop bit)
    //      can no longer arm the receiver -- which is what made a full bit period
    //      necessary in the first place;
    //   2. because of (1), the required idle drops to HALF a bit period, giving
    //      10416.67 - 5208 = 5208 clocks (50% of a bit) of margin against the
    //      stop bit instead of 0.67.
    //
    // Side benefit: the 10416-bit shift register and its 10416-input AND are
    // gone.  That register alone held 10416 of the design's 11783 flip flops
    // (33% of an xc7z010); the replacement counter is 32 bits.
    // -------------------------------------------------------------------------
    // idle_cnt is 32 bits on purpose: a 16-bit counter would silently stop
    // reaching the threshold for any HALF_CNT above 65535 (i.e. any baud below
    // about 763 at 100 MHz), and the receiver would simply never arm again.
    

    reg [31:0] idle_cnt;
    reg        armed;

    wire idle_enough = (idle_cnt >= rearm);
    wire start_edge  = (rx_bit == 1'b0) && armed;

    // Consecutive idle-high clocks, counted only while the receiver is idle.
    // Any low level, or any clock spent inside a frame, restarts the count.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            idle_cnt <= 32'd0;
        else if ((state != S_IDLE) || (rx_bit == 1'b0))
            idle_cnt <= 32'd0;
        else if (idle_cnt != 32'hFFFF_FFFF)
            idle_cnt <= idle_cnt + 32'd1;
    end

    // Disarm takes priority: the moment a start bit is taken, or the moment a
    // frame is rejected, the detector must not trigger again until the line has
    // been idle long enough to show the disturbance is over.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            armed <= 1'b0;
        else if (start_edge || frame_err)
            armed <= 1'b0;                          // priority: disarm
        else if (idle_enough)
            armed <= 1'b1;                          // re-arm after idle
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            cnt       <= 16'd0;
            bit_idx   <= 4'd0;
            shift     <= 8'd0;
            data      <= 8'd0;
            valid     <= 1'b0;
            frame_err <= 1'b0;
        end else begin
            // one-clock pulses by default
            valid     <= 1'b0;
            frame_err <= 1'b0;

            case (state)

                // ---------------- wait for a confirmed idle line, then a falling edge
                S_IDLE: begin
                    if ((rx_bit == 1'b0) && armed) begin
                        state <= S_START;
                        cnt   <= half_cnt;
                    end
                end

                // ---------------- validate the start bit ----------------
                S_START: begin
                    if (cnt == 16'd0) begin
                        if (rx_bit == 1'b0) begin
                            // still low at mid-bit: a real start bit
                            state   <= S_DATA;
                            bit_idx <= 4'd0;
                            cnt     <= to_first;
                        end else begin
                            state <= S_IDLE;      // glitch, not a start bit
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                // ---------------- sample 8 data bits, LSB first ----------------
                S_DATA: begin
                    if (cnt == 16'd0) begin
                        shift <= {rx_bit, shift[7:1]};   // LSB arrives first
                        cnt   <= baud_cnt;
                        if (bit_idx == 4'd7) begin
                            bit_idx <= 4'd8;
                            state   <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 4'd1;
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                // ---------------- sample the stop bit ----------------
                S_STOP: begin
                    if (cnt == 16'd0) begin
                        if (rx_bit == 1'b1) begin
                            data  <= shift;
                            valid <= 1'b1;
                        end else begin
                            frame_err <= 1'b1;    // framing error: byte dropped
                        end
                        state <= S_IDLE;
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                default: state <= S_IDLE;

            endcase
        end
    end

endmodule
