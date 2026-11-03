# 十张系统图索引

2026-11-02复核。当前主报告按五部分模板重排为20页A4纵向页面，采用三张适合正文阅读的系统说明图。完整十张核对后的详图作为独立附件保留，PNG、SVG、draw.io及WPS数据继续可用。

[全部十图PDF](detailed_flowcharts.pdf) · [图表修改清单](history/diagram_changes_20261101.md)

| 图 | 内容 | 文件 |
|---|---|---|
| 01 | 硬件连接与信号方向 | [PNG](figures/fig01_hardware_wiring.png) / [PDF](fig01_hardware_wiring.pdf) / [SVG](figures/fig01_hardware_wiring.svg) / [draw.io](figures/fig01_hardware_wiring.drawio) |
| 02 | PL 与 PS 模块架构 | [PNG](figures/fig02_logic_architecture.png) / [PDF](fig02_logic_architecture.pdf) / [SVG](figures/fig02_logic_architecture.svg) / [draw.io](figures/fig02_logic_architecture.drawio) |
| 03 | PS 状态转移与恢复 | [PNG](figures/fig03_ps_state_machine.png) / [PDF](fig03_ps_state_machine.pdf) / [SVG](figures/fig03_ps_state_machine.svg) / [draw.io](figures/fig03_ps_state_machine.drawio) |
| 04 | 逐秒采集与控制更新流程 | [PNG](figures/fig04_ps_control_pipeline.png) / [PDF](fig04_ps_control_pipeline.pdf) / [SVG](figures/fig04_ps_control_pipeline.svg) / [draw.io](figures/fig04_ps_control_pipeline.drawio) |
| 05 | KF 与 DAC 控制闭环 | [PNG](figures/fig05_ps_pl_ocxo_loop.png) / [PDF](fig05_ps_pl_ocxo_loop.pdf) / [SVG](figures/fig05_ps_pl_ocxo_loop.svg) / [draw.io](figures/fig05_ps_pl_ocxo_loop.drawio) |
| 06 | 仿真台架与验证分层 | [PNG](figures/fig06_testbench_structure.png) / [PDF](fig06_testbench_structure.pdf) / [SVG](figures/fig06_testbench_structure.svg) / [draw.io](figures/fig06_testbench_structure.drawio) |
| 07 | PL 接口状态与结果发布 | [PNG](figures/fig07_pl_state_machine.png) / [PDF](fig07_pl_state_machine.pdf) / [SVG](figures/fig07_pl_state_machine.svg) / [draw.io](figures/fig07_pl_state_machine.drawio) |
| 08 | C 慢环计算与受限接入 | [PNG](figures/fig08_dual_loop_cooperation.png) / [PDF](fig08_dual_loop_cooperation.pdf) / [SVG](figures/fig08_dual_loop_cooperation.svg) / [draw.io](figures/fig08_dual_loop_cooperation.drawio) |
| 09 | 串口屏两页与字段来源 | [PNG](figures/fig09_screen_pages.png) / [PDF](fig09_screen_pages.pdf) / [SVG](figures/fig09_screen_pages.svg) / [draw.io](figures/fig09_screen_pages.drawio) |
| 10 | TDC 与粗分数配对时序 | [PNG](figures/fig10_tdc_measurement.png) / [PDF](fig10_tdc_measurement.pdf) / [SVG](figures/fig10_tdc_measurement.svg) / [draw.io](figures/fig10_tdc_measurement.drawio) |

图09直接核对本地TJC.HMI活动页面：控制页6项，GNSS页8项。控件及坐标证据见figure_sources/screen_layout_evidence.json；未读取屏上已烧写的工程版本。

SPI引脚采用生效XDC及IO报告：SCK=G20，DIN/MOSI=H18，CS_N=L20。报告正文已同步纠正。

本次模板重排与图表说明更新不修改源代码、固件或实验原始值；历史性能与本次v27功能记录分别引用；详图本身不代替实际硬件日志。

新增五张实拍已放入技术总结报告。旧10MHz连接PDF的核对见legacy_wiring_review.md，当前接线继续以本索引图01及生效XDC为准。

2026-11-02更新：当前默认ELF为15:29:18重编译版，v27已记录CALIB保持、失锁返回CALIB及src=2正权重。具体覆盖范围见hardware_acceptance_20261102.md。新增PCB近照与J11/J12、5V说明见hardware_interfaces.md；未提供的模拟元件参数不列为报告待补要求。
