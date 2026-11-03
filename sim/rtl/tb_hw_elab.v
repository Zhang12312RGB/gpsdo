//=============================================================================
// tb_hw_elab.v -- elaboration check for the REAL CARRY4 configuration
//-----------------------------------------------------------------------------
// The functional testbench (tb_gnss_gpsdo_top) runs with SIM_DELAY_MODEL = 1, so
// it elaborates the BEHAVIOURAL delay line and never touches the CARRY4 branch.
// That means a structural error in the synthesised branch -- the one that actually
// goes on the board -- cannot be caught by running the functional tests.
//
// MEASURED: exactly that happened. Vivado synthesis reported
//
//     [Synth 8-373]  invalid index for scalar 'tap_in'          [tdc_carry.v:195]
//     [Synth 8-196]  conditional expression could not be resolved [tdc_carry.v:187]
//     [Synth 8-693]  zero replication count - replication ignored [tdc_carry.v:345]
//
// none of which the functional testbench could see, because it never elaborated
// that branch. This file exists so the synthesised configuration is elaborated
// too, and it is deliberately instantiating the WHOLE top level so that port
// widths and connections are checked as well.
//
// It is NOT a functional test: CARRY4's unisim model is a zero-delay behavioural
// stub, so no real wavefront propagates and tap_value stays 0. That is expected
// and is why the assertions live in the other two testbenches. What this proves
// is that the design ELABORATES in the configuration that gets built.
//
// Run:
//   xvlog  <vivado>/data/verilog/src/glbl.v  ../rtl/*.v  tb_hw_elab.v
//   xelab  -L unisims_ver work.tb_hw_elab work.glbl -s hw_sim
//   xsim   hw_sim -R
//
// glbl is required because the unisim primitive models reference it for global
// set/reset; without it elaboration fails on 'glbl is not declared'.
//
// NOTE: every comment and string in this file is ASCII on purpose.
//=============================================================================

`timescale 1ns / 1ps

module tb_hw_elab;

    // The real build configuration: SIM_DELAY_MODEL = 0, NUM_CARRY4 = 128.
    localparam integer NUM_C4 = 128;

    reg clk    = 1'b0;
    reg rst_n  = 1'b0;
    reg pps    = 1'b0;
    reg ocxo   = 1'b0;
    reg urx    = 1'b1;

    always #5 clk = ~clk;      // 100 MHz

    wire       sck, mosi, cs_n;
    wire [3:0] led;
    wire [31:0] status_dbg;

    // AXI tie-offs: this check is about elaboration, not about bus behaviour.
    reg  [11:0] awaddr  = 12'd0;
    reg  [2:0]  awprot  = 3'd0;
    reg         awvalid = 1'b0;
    wire        awready;
    reg  [31:0] wdata   = 32'd0;
    reg  [3:0]  wstrb   = 4'hF;
    reg         wvalid  = 1'b0;
    wire        wready;
    wire [1:0]  bresp;
    wire        bvalid;
    reg         bready  = 1'b1;
    reg  [11:0] araddr  = 12'd0;
    reg  [2:0]  arprot  = 3'd0;
    reg         arvalid = 1'b0;
    wire        arready;
    wire [31:0] rdata;
    wire [1:0]  rresp;
    wire        rvalid;
    reg         rready  = 1'b1;

    // SIM_DELAY_MODEL is left at its default of 0, so the CARRY4 branch is built.
    gnss_gpsdo_top #(
        .NOMINAL_CYCLES (100_000_000),
        .NOMINAL_OCXO   (10_000_000),
        .CLK_FREQ_HZ    (100_000_000),
        .NUM_CARRY4     (NUM_C4),
        .FRAC_BITS      (16),
        .SCALE_BITS     (8),
        .BAUD_RATE      (9600),
        .SPI_DIV        (8)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .pps_in     (pps),
        .ocxo_in    (ocxo),
        .uart_rx    (urx),
        .spi_sck    (sck),
        .spi_mosi   (mosi),
        .spi_cs_n   (cs_n),
        .led        (led),
        .status_dbg (status_dbg),
        .S_AXI_AWADDR  (awaddr),
        .S_AXI_AWPROT  (awprot),
        .S_AXI_AWVALID (awvalid),
        .S_AXI_AWREADY (awready),
        .S_AXI_WDATA   (wdata),
        .S_AXI_WSTRB   (wstrb),
        .S_AXI_WVALID  (wvalid),
        .S_AXI_WREADY  (wready),
        .S_AXI_BRESP   (bresp),
        .S_AXI_BVALID  (bvalid),
        .S_AXI_BREADY  (bready),
        .S_AXI_ARADDR  (araddr),
        .S_AXI_ARPROT  (arprot),
        .S_AXI_ARVALID (arvalid),
        .S_AXI_ARREADY (arready),
        .S_AXI_RDATA   (rdata),
        .S_AXI_RRESP   (rresp),
        .S_AXI_RVALID  (rvalid),
        .S_AXI_RREADY  (rready)
    );

    integer errors = 0;

    initial begin
        $display("================================================================");
        $display(" tb_hw_elab -- REAL configuration elaboration check");
        $display("   SIM_DELAY_MODEL = 0 (CARRY4), NUM_CARRY4 = %0d (%0d taps)",
                 NUM_C4, 4*NUM_C4);
        $display("================================================================");

        // Exercise the chain a little. The unisim CARRY4 is a zero-delay stub, so
        // no wavefront propagates and tap_value is expected to stay 0; the point
        // is that every branch, part-select and port width is elaborated.
        #100 rst_n = 1'b1;
        #100 pps   = 1'b1;
        #200 pps   = 1'b0;
        #100 ocxo  = 1'b1;
        #100 ocxo  = 1'b0;
        #200;

        $display("  tap_raw width check: NTAP=%0d", 4*NUM_C4);
        if (dut.tap_raw !== {4*NUM_C4{1'b0}} && dut.tap_raw === {4*NUM_C4{1'bx}}) begin
            errors = errors + 1;
            $display("[FAIL] tap_raw is all X -- the chain did not elaborate cleanly");
        end else begin
            $display("[PASS] the CARRY4 chain of %0d taps elaborated", 4*NUM_C4);
        end

        $display("");
        if (errors == 0)
            $display(" RESULT: [PASS] the REAL (synthesised) configuration elaborates");
        else
            $display(" RESULT: [FAIL] %0d problem(s)", errors);
        $display("================================================================");

        $finish;
    end

    initial begin
        #1_000_000;
        $display("[FAIL] timeout");
        $fatal(1, "timeout");
    end

endmodule
