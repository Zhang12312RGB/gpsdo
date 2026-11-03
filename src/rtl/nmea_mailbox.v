//=============================================================================
// nmea_mailbox.v -- single-byte NMEA handoff with strict write-1-to-clear
//-----------------------------------------------------------------------------
// REGISTER BEHAVIOUR (offset 0x04)
//
//     bit 8    : NEW_DATA. 1 = a byte is waiting for the ARM.
//     bits 7:0 : the NMEA character.
//
// The two rules, exactly as specified:
//
//   1. When a received byte arrives and NEW_DATA is already 1, the new byte is
//      DISCARDED. The register holds the first unread byte; it is never
//      overwritten. Losing a byte is recoverable -- the next NMEA sentence
//      repeats the same information one second later -- whereas silently
//      replacing a byte mid-sentence corrupts a checksummed message in a way the
//      ARM cannot detect.
//
//   2. NEW_DATA is cleared only by the ARM writing 0x0000_0100. That is a
//      write-1-to-clear on bit 8: the write data is MASKED against bit 8, so
//      writing 0x100 clears the flag and any other write value leaves it alone.
//
//==============================================================================
// WHY THE ACKNOWLEDGE IS MASKED RATHER THAN COMPARED
//==============================================================================
// Comparing the whole word against 0x100 would also work, but it discards
// information in a way that is very hard to debug:
//
//   a read-modify-write by the ARM (read 0x1xx, OR something in, write back)
//   carries the byte value along in bits 7:0. An equality test would then accept
//   or reject the acknowledge depending on WHICH CHARACTER happened to be
//   pending, so the handshake would work for most characters and fail for a few
//   of them.
//
// Masking against bit 8 makes the rule "write a 1 to bit 8 to clear it" hold for
// every write value, which is what ARM-side code naturally assumes.
//
//==============================================================================
// DROP ACCOUNTING
//==============================================================================
// drop_count increments every time a received byte is thrown away. It is the
// only evidence the ARM has that its polling loop is too slow, and it is what
// makes rule 1 observable rather than silent. byte_count counts accepted bytes,
// so the two together account for every byte the UART delivered.
//
// NOTE: every comment and string in this file is ASCII on purpose.
//=============================================================================

`timescale 1ns / 1ps

module nmea_mailbox (
    input  wire        clk,
    input  wire        rst_n,

    // from uart_rx
    input  wire        rx_valid,      // one-clock pulse: a good byte arrived
    input  wire [7:0]  rx_data,
    input  wire        frame_err,     // one-clock pulse: stop bit was not high

    // from the AXI register file
    input  wire        ack_pulse,     // one-clock pulse: ARM wrote 0x04
    input  wire [31:0] ack_wdata,     // the write data, masked against bit 8

    // to the AXI register file
    output reg  [31:0] mailbox,       // {23'b0, NEW_DATA, byte[7:0]}
    output reg  [31:0] drop_count,
    output reg  [31:0] byte_count,
    output reg         frame_err_seen // sticky: at least one framing error
);

    //--------------------------------------------------------------------------
    // The flag is bit 8 of the mailbox register itself, so the pending state
    // lives in exactly one place and the flag and the byte cannot disagree.
    //--------------------------------------------------------------------------
    wire new_data = mailbox[8];

    wire ack = ack_pulse && ack_wdata[8];   // write-1-to-clear, see header

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mailbox    <= 32'd0;
            drop_count <= 32'd0;
            byte_count <= 32'd0;
        end else begin
            // Clear has priority over set, so a byte arriving in the same cycle
            // as the acknowledge is accepted rather than lost: the ARM has just
            // confirmed it took the previous one, so the register is free.
            if (ack)
                mailbox[8] <= 1'b0;

            if (rx_valid) begin
                if (new_data && !ack) begin
                    // Still holding an unread byte: discard the new one.
                    if (drop_count != 32'hFFFF_FFFF)
                        drop_count <= drop_count + 32'd1;
                end else begin
                    mailbox[7:0] <= rx_data;
                    mailbox[8]   <= 1'b1;
                    if (byte_count != 32'hFFFF_FFFF)
                        byte_count <= byte_count + 32'd1;
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Sticky framing-error flag
    //
    // The bad byte is already dropped by uart_rx; this only records that the
    // stream is not clean, so the ARM can distinguish "no NMEA yet" from "the
    // line is mis-wired or the baud rate is wrong".
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)         frame_err_seen <= 1'b0;
        else if (frame_err) frame_err_seen <= 1'b1;
    end

endmodule
