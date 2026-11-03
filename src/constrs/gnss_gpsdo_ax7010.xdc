#=============================================================================
# gnss_gpsdo_ax7010.xdc -- pin and timing constraints, ALINX AX7010
#-----------------------------------------------------------------------------
# Board  : ALINX AX7010, device xc7z010clg400-1
# Job    : GPSDO (GNSS-disciplined OCXO), PL pin-in / pin-out constraints
#
# SOURCES AND PRECEDENCE
# ----------------------
#   1) SIGNAL NAMES   : taken from the project text (this design's port list):
#                       pps_in, ocxo_in, uart_rx, spi_sck, spi_mosi, spi_cs_n,
#                       led[3:0].  Where the text and the drawing disagreed on
#                       a NAME, the text wins.
#   2) PIN LOCATIONS  : taken from 接线图.pdf (硬件连接与引脚分配图 + 接线汇总表).
#                       Where the text and the drawing disagreed on a
#                       PACKAGE_PIN, the drawing wins.
#   3) BOARD KEY      : 按键 KEY1 = N15, taken from 接线汇总表 of the same PDF.
#                       The pasted text listed only the six J11 signals and the
#                       four LEDs and had NO key port at all, so there is no
#                       conflict to resolve here -- it is a gap in the text that
#                       the drawing fills in.
#
# CONFLICT LOG  (old text value  ->  PDF value applied)
# ----------------------------------------------------
#   signal      old text                PDF (APPLIED)         function
#   pps_in      J10 PIN4   W18          J11 PIN5   F20        GNSS 模块 1PPS
#   ocxo_in     J10 PIN8   Y16          J11 PIN16  K17        OCXO 10 MHz OUT
#   uart_rx     J11 PIN4   F16          J11 PIN3   F17        GNSS 模块 TXD
#   spi_sck     J11 PIN12  L19          J11 PIN7   G20        DAC SCLK
#   spi_mosi    J11 PIN14  M19          J11 PIN11  L20        DAC DIN
#   spi_cs_n    J11 PIN16  K17          J11 PIN9   H18        DAC CS_N (low true)
#   led[3:0]    M14 M15 K16 J16         (unchanged)
#   key_in      (取消)                  改用 PS 侧按键 MIO50 / MIO51
#
#   Note the knock-on effect: the old text had spi_cs_n on K17, but the drawing
#   puts the OCXO on K17 and moves CS_N to L20.  Taking the drawing for pins
#   therefore moves BOTH signals, not just one.
#
#   The old text placed pps_in and ocxo_in on J10; the drawing places every
#   signal on J11.  All six J11 signals are BANK35 (J11 = BANK35, 3.3 V LVCMOS).
#
# CROSS-CHECK
# -----------
#   The applied package pins agree with AX7010 用户手册 V1.07:
#     §7.5  J11 table
#       PIN3 = F17   EX_IO2_1N  IO_L6N_T0_35
#       PIN5 = F20   EX_IO2_2N  IO_L15N_T2_35
#       PIN7 = G20   EX_IO2_3N  IO_L18N_T2_35
#       PIN9 = H18   EX_IO2_4N  IO_L14N_T2_35
#       PIN11= L20   EX_IO2_5N  IO_L9N_T1_35
#       PIN16= K17   EX_IO2_7P  IO_L12P_T1_35
#     §7.7  PL 用户按键 / §7.6 PL 用户 LED
#       KEY1 = N15   IO_L21P_T3_35   (低电平有效)
#       LED1..4 = M14 / M15 / K16 / J16   (低电平点亮)
#   Two independent sources (the drawing and the manual) agree on all of them,
#   so the N/P half and the key/LED pins are verified rather than assumed.
#
# J11 AS USED BY THIS DESIGN
# --------------------------
#   PIN  1  GND        GNSS 模块 GND
#   PIN  3  F17        GNSS 模块 TXD        -> PL uart_rx        (3.3 V)
#   PIN  5  F20        GNSS 模块 1PPS       -> PL pps_in / TDC   (3.3 V)
#   PIN  7  G20        DAC SCLK             <- PL spi_sck        (3.3 V)
#   PIN  9  H18        DAC DIN              <- PL spi_mosi       (3.3 V)
#   PIN 11  L20        DAC CS_N             <- PL spi_cs_n       (3.3 V)
#   PIN 16  K17        OCXO 10 MHz OUT      -> PL ocxo_in        (3.3 V)
#   PIN 37/38 GND      DAC / OCXO / 数字地
#   PIN 39/40 +3.3 V   DAC VDD, OCXO VCC, GNSS 模块 VCC
#   (DAC VOUT -> OCXO EFC is an analogue link off-board, no FPGA pin.)
#
# BOARD-LEVEL I/O USED (no connector)
# -----------------------------------
#   按键: 已改由 PS 侧 MIO50 / MIO51 实现, 不经 PL, 无 XDC 约束 (见第 3 节)
#   LED1..4   = M14 / M15 / K16 / J16 -> PL led[3:0]               (低电平点亮)
#   KEY2/KEY3/KEY4 = N16 / T17 / R17 are available but unused by this design.
#
#=============================================================================
# ELECTRICAL WARNING -- READ BEFORE CONNECTING ANYTHING
#=============================================================================
# BANK35 is 3.3 V LVCMOS and is NOT 5 V tolerant.  The user manual repeats this
# for J10/J11: do not connect 5 V devices directly to the expansion port, and
# do not hot-plug it.  A GNSS module or oscillator with a 5 V logic output needs
# a level shifter or a resistive divider before it reaches the FPGA.
#
# The DAC VOUT -> OCXO EFC link is analogue (0 ~ 5 V per the drawing).  Check it
# against the OCXO's EFC input range before power-up; over-driving EFC can pull
# the oscillator off frequency or damage it.
#
# This is the one class of mistake on this board that costs hardware, not time.
#
#=============================================================================
# CLOCKING
#=============================================================================
# There is deliberately NO create_clock for the system clock.  The PL clock is
# FCLK_CLK0, which the PS generates internally, and Vivado derives its period
# from the PS configuration.  A PS output is not a package pin, so there is no
# PACKAGE_PIN and no create_clock to write.  Inventing one for a port that does
# not exist would produce a constraint that silently matches nothing.
#
# ocxo_in is COUNTED, not used as a clock.  It enters as an ordinary input pin
# plus a synchroniser, which is what the design does, so it does not need to be
# clock-capable.  Per the manual's J11 table, K17 = IO_L12P_T1_35 with no
# MRCC/SRCC in the name, i.e. it is a regular I/O -- consistent with that use.
# If a future revision needs the OCXO on a global clock, this pin must move and
# the clock-capable pins of BANK34 are consumed by HDMI on this board, so an
# external clock buffer would be the practical route.
#
#=============================================================================

#------------------------------------------------------------------------------
# 1) Pin locations and I/O standard -- inputs
#------------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN F20   IOSTANDARD LVCMOS33 } [get_ports { pps_in  }]
set_property -dict { PACKAGE_PIN K17   IOSTANDARD LVCMOS33 } [get_ports { ocxo_in }]
set_property -dict { PACKAGE_PIN F17   IOSTANDARD LVCMOS33 } [get_ports { uart_rx }]

#------------------------------------------------------------------------------
# 2) Pin locations and I/O standard -- SPI master to the external DAC
#    MCP4921-class, 3-wire.  CS_N is active low.
#------------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN G20   IOSTANDARD LVCMOS33 } [get_ports { spi_sck  }]
# PIN 11 = L20 = DAC CS_N
#   证据 1 (实现报告): gnss_bd_wrapper_io_placed.rpt 中 L20=spi_cs_n, H18=spi_mosi
#   证据 2 (实测电平): PIN9/H18 读 0V  = spi_mosi 空闲低
#                      PIN11/L20 读 3.3V = spi_cs_n 空闲高
#   证据 3 (功能): 用此映射综合的 bit, hwtest mode 2 测得 g=0.4785 ppb/码、闭环锁 0.2ppb
#   结论: H18 是 DAC DIN, L20 是 DAC CS_N。README 的接线表 / 旧 PDF 此处已过时。
set_property -dict { PACKAGE_PIN L20   IOSTANDARD LVCMOS33 } [get_ports { spi_cs_n }]

# PIN 9 = H18 = DAC DIN
set_property -dict { PACKAGE_PIN H18   IOSTANDARD LVCMOS33 } [get_ports { spi_mosi }]

# Keep CS_N deasserted (high) while the PL is unconfigured, so the DAC does not
# latch a spurious word during configuration or reset.
set_property PULLUP true [get_ports { spi_cs_n }]

#------------------------------------------------------------------------------
# 3) 按键 —— 已改由 PS 侧实现, PL 侧不占用任何引脚
#
# 背景: PL 顶层 gnss_gpsdo_top.v 没有按键输入端口, 而 AXI 的 0x0C 偏移实际是
#       TDC_TAP_PERIOD, 不是按键寄存器。因此"PL 按键"这条路走不通。
#
# 现方案: 用 AX7010 板载的 PS 用户按键 (用户手册 V1.07 §6.9, BANK501)
#           PS KEY1 = PS_MIO50_501 = B13   -> 状态切换 TUNING <-> CALIB
#           PS KEY2 = PS_MIO51_501 = B9    -> 清空自适应 R 采集 / 强制回 TUNING
#         按下为低电平, 由 PS 的 GPIO 控制器 (XGpioPs) 读取, 软件去抖,
#         见 PS 工程 src/ps_key.c / ps_key.h。
#
# 注意: MIO50/MIO51 属于 PS 侧 MIO, 不是 PL 引脚, 因此
#       *** 本文件不需要、也不能为它们写任何约束 ***。
#       只需在 Vivado 的 Zynq PS -> MIO Configuration 里确认 MIO50/MIO51
#       没有被 SD1 等其它外设占用即可。
#
# 板上原本接在 N15 的 PL 按键 KEY1 现已不使用。若将来 PL 恢复按键功能, 需要:
#       (a) RTL 顶层增加按键输入端口;
#       (b) 在 AXI 寄存器表里另选一个空闲偏移做按键寄存器;
#       (c) 在本文件里为那个端口补回 PACKAGE_PIN N15 约束。
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
# 4) Bring-up LEDs -- PL side, ACTIVE LOW
#
# The AX7010 PL LEDs are wired with the FPGA pin sinking current, so driving the
# pin LOW lights the LED and driving it HIGH turns it off.  The polarity is
# inverted in the RTL (see gnss_gpsdo_top) rather than here, so this file stays a
# description of the board and the RTL stays the description of the behaviour.
#
# led[0] = 锁定 (lock)   led[1] = 驯服中 (tuning)   led[2] = 守时 (holdover)
# led[3] = 告警 (fault / no GNSS)
#------------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN M14   IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN M15   IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN K16   IOSTANDARD LVCMOS33 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN J16   IOSTANDARD LVCMOS33 } [get_ports { led[3] }]

#------------------------------------------------------------------------------
# 5) Input register in the IOB
#
# pps_in is asynchronous and is measured by the logic that samples it.  Using the
# dedicated input register in the I/O block gives it a fixed, well-defined delay
# instead of a routing-dependent one.  That matters for the TDC: any delay that
# varies between builds changes the tap offset that has to be calibrated, so
# pinning it down keeps the calibration meaningful across re-runs.
#------------------------------------------------------------------------------
set_property IOB TRUE [get_ports { pps_in }]

# ocxo_in is the other measured edge and would benefit for the same reason, but
# IOB TRUE only applies if the first register on that net sits directly in the
# IOB.  Enable it only if the RTL keeps that register adjacent to the pin:
#     set_property IOB TRUE [get_ports { ocxo_in }]

#------------------------------------------------------------------------------
# 6) Asynchronous inputs
#
# pps_in, ocxo_in and uart_rx are all asynchronous to the PL clock by design.
# pps_in and ocxo_in come from independent oscillators and uart_rx from the
# GNSS module's UART (the PS keys on MIO50/MIO51 are not PL ports and need no
# constraint here).  The
# design measures them with oversampling (two-flop synchronisers plus edge
# detection) rather than by sampling them on a clock edge, so there is no
# setup/hold relationship for the tool to close.
#
# Declaring them false paths is therefore the CORRECT statement of fact rather
# than a way of hiding a violation: it tells the tool not to meet a timing
# relation that does not exist.  It is also why no input delay is specified.
#------------------------------------------------------------------------------
set_false_path -from [get_ports { pps_in  }]
set_false_path -from [get_ports { ocxo_in }]
set_false_path -from [get_ports { uart_rx }]

#------------------------------------------------------------------------------
# 7) SPI outputs -- delay bound only, no invented synchronous relationship
#
# The MCP4921 has NO clock input.  It samples SDI on the rising edge of the SCK
# that this design generates, and that edge is produced by ordinary logic clocked
# at 100 MHz.  There is no real synchronous relationship to a receiving clock, so
# modelling one with set_output_delay against a virtual clock is a fiction -- and
# a harmful one: because the source clock has a 10 ns period, the virtual clock's
# nearest capture edge always lands about one source step after the launch edge,
# so the computed requirement collapses to roughly one clock period no matter what
# period the virtual clock is given.
#
# set_max_delay -datapath_only bounds the routing delay and explicitly does NOT
# invent a clock-edge relationship (-datapath_only drops the clock skew term).
#
# SPI_DIV = 8 gives SCK = 100 MHz / (2*8) = 6.25 MHz, well inside the MCP4921's
# 20 MHz limit, so a 100 ns bound is enormously generous while still preventing a
# pathological route.
#------------------------------------------------------------------------------
create_clock -name clk_spi_virtual -period 160.000

# -from is mandatory here: without it the tool reports
#   [Constraints 18-540] set_max_delay -datapath_only requires -from to be non-empty
# and the 100 ns bound silently never takes effect.
set_max_delay -datapath_only 100.000 -from [get_clocks clk_fpga_0] -to [get_ports { spi_sck spi_mosi spi_cs_n }]
set_false_path -from [get_clocks clk_spi_virtual]
set_false_path -to   [get_clocks clk_spi_virtual]

#------------------------------------------------------------------------------
# 8) SPI clock output
#
# spi_sck is a data output as far as the tool is concerned; nothing inside the
# design is clocked by it, so declaring it a generated clock would be wrong.  The
# virtual clock above already models the far end's sampling instant.  This section
# is kept as a note so the absence is deliberate rather than an oversight.
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
# 9) Unused expansion-header pins
#
# The unused J10/J11 pins and the unused board keys KEY2/KEY3/KEY4 are
# deliberately left unconstrained.  They are connector / board pins, not floating
# FPGA inputs, so leaving them out is correct; adding a PACKAGE_PIN for a port
# that does not exist produces a constraint that matches nothing, which is worse
# than none because it looks like coverage.
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
# 10) Verification checklist after synthesis / implementation
#
#   report_io -file io.rpt                 ;# every port landed where expected
#   report_timing_summary                  ;# WNS/WHS, and no unexpected paths
#   check_timing                           ;# no "no_clock" / "unconstrained"
#
#   Post-synthesis I/O check -- six J11 signals, four LEDs (no PL key):
#     pps_in  -> F20 (J11-5)     ocxo_in  -> K17 (J11-16)
#     uart_rx -> F17 (J11-3)     spi_sck  -> G20 (J11-7)
#     spi_cs_n-> H18 (J11-9)     spi_mosi -> L20 (J11-11)
#     (按键: PS 侧 MIO50 / MIO51, 不经 PL, 无约束)
#     led[0..3] -> M14 M15 K16 J16
#
#   The IOSTANDARD must read LVCMOS33 on all of them; BANK35 on this board is
#   3.3 V, and a mismatch here is reported by DRC as an invalid I/O standard.
#------------------------------------------------------------
#   report_io -file io.rpt                 ;# every port landed where expected
#   report_timing_summary                  ;# WNS/WHS, and no unexpected paths
#   check_timing                           ;# no "no_clock" / "unconstrained"
#
#   Post-synthesis I/O check -- six J11 signals, four LEDs (no PL key):
#     pps_in  -> F20 (J11-5)     ocxo_in  -> K17 (J11-16)
#     uart_rx -> F17 (J11-3)     spi_sck  -> G20 (J11-7)
#     spi_cs_n-> H18 (J11-9)     spi_mosi -> L20 (J11-11)
#     (按键: PS 侧 MIO50 / MIO51, 不经 PL, 无约束)
#     led[0..3] -> M14 M15 K16 J16
#
#   The IOSTANDARD must read LVCMOS33 on all of them; BANK35 on this board is
#   3.3 V, and a mismatch here is reported by DRC as an invalid I/O standard.
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
# TDC_DELAY_LINE_FALSE_PATH
#   pps_in drives a 512-tap CARRY4 delay line. The captures register samples a
#   *physical waveform*, not a logic cone, so static timing on that path is
#   meaningless: the full chain is ~32 ns while one clock period is 10 ns.
#   Without this, Vivado reports a ~22 ns violation and may try to "fix" it.
#------------------------------------------------------------------------------
set_false_path -from [get_ports pps_in]

#------------------------------------------------------------------------------
# TDC_DECODER_QUASI_STATIC
#   tap_raw_r -> thermometer decode -> tap_value.
#
#   The decode is a 512-tap priority chain (~12.7 ns of logic). It is NOT a
#   data path: tap_raw_r is refreshed once per second and is static for the
#   other 10^7 cycles. tdc_carry.v latches tap_value 8 cycles after the
#   capture, so the value is settled by construction.
#
#   A multicycle approach was tried first and is recorded here because it
#   produced a nasty failure: `NAME =~ *tap_value_reg*` also matches the
#   combinational LUTs that synthesis names tap_value_reg[0]_i_100 etc.
#   Constraints 18-401 rejected those as endpoints, the constraint silently
#   did not take effect, and route_design then thrashed in
#   "Phase 5 Rip-up And Reroute" with WHS = -10.176 ns (exactly one
#   period) until it was cancelled.
#
#   IS_SEQUENTIAL keeps only real flops, so the filters below are clean.
#------------------------------------------------------------------------------
set_false_path -from [get_cells -hier -filter {NAME =~ *u_tdc/tap_raw_r_reg* && IS_SEQUENTIAL}] -to [get_cells -hier -filter {NAME =~ *u_tdc/tap_value_reg* && IS_SEQUENTIAL}]
set_false_path -from [get_cells -hier -filter {NAME =~ *u_tdc/cap_dly_reg* && IS_SEQUENTIAL}] -to [get_cells -hier -filter {NAME =~ *u_tdc/tap_value_reg* && IS_SEQUENTIAL}]
