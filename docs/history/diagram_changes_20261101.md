# GPSDO 十图修改清单

修订日期 2026-11-01。以当前可执行 PS/PL 逻辑、有效 XDC 约束和实现 IO 报告为依据。原 picture.docx 含5个WPS原生流程图数据对象，对应图02、03、04、05、08；其余图01、06、07、09、10来自 diagrams.zip。

图中主架构为最终功能配置 MODE2、profile1、Q_FREQ=1e-4、R80、稳态Tu10秒、相位回零项关闭。今天的R标定配置为MODE0、profile0、Q_FREQ=2e-7、P3经验下限开启；两者共同的CALIB均冻结DAC，采原始数据供离线分析。

源文件中的图片预览之外，还保留WPS流程图JSON。新Word含10个对应的流程图数据对象；预览图可直接复制，流程图编辑依赖WPS相应组件。标准Word不把这些对象识别为原生流程图形状。每张图另提供SVG和draw.io，便于继续改文字、节点和连线。

原图保留；经用户确认，本次将修正版同步到当前提交文档，替换前版本另作备份。本套图不宣称绝对频率准确度、几十ps实测分辨率、R自适应已上线或慢环收益已被充分重复确认。

## 图01 硬件连接与信号方向

[修正版PDF](../fig01_hardware_wiring.pdf) · [PNG](../figures/fig01_hardware_wiring.png) · [SVG](../figures/fig01_hardware_wiring.svg) · [draw.io](../figures/fig01_hardware_wiring.drawio)

| 原图位置 | 原内容 | 应替换为 | 连线或逻辑调整 |
|---|---|---|---|
| 中部 J11 表、底部 DAC 小框、接线汇总表 | DIN / CS_N 在几处互换 | SCK=G20；DIN=H18；CS_N=L20。对应 J11 PIN7 / PIN9 / PIN11。 | PL SPI 输出到 DAC，方向由 FPGA 指向 DAC。 |
| OCXO 方框、回送箭头、时钟分配框 | OCXO 10 MHz；PL 时钟输入；本地 1PPS 分频 | OCXO 20 MHz / 50 ns；OUT 输入 K17，被 100 MHz FCLK 采样计数。 | 将 OCXO OUT 画到被测输入，不画成 PS/PL 系统时钟；未实现的本地 1PPS 输出从当前功能中移除。 |
| 底部 DAC 芯片及模拟电压箭头 | 12 位 DAC / VOUT 0～5 V；供电一律 3.3 V | DAC8531 为 16 位器件；系统 12 位命令左移 4 位，24 位 SPI 帧；VOUT 连 EFC。 | 供电、VOUT 范围及模拟滤波以实物电路为准，不能由寄存器位宽推定。 |
| 按键及 LED 说明 | PL N15 KEY1 与 PS KEY1 混在同一功能表 | 实际操作使用 PS KEY1 MIO50、PS KEY2 MIO51；PS LED MIO0 表示频率估计幅度。 | PL LED 的锁定/驯服/守时/告警语义须逐项匹配顶层赋值，不把 PS 闪烁 LED 当作独立精度证据。 |

代码依据：constrs/gnss_gpsdo_ax7010.xdc 生效 set_property；impl_1/gnss_bd_wrapper_io_placed.rpt；gnss_gpsdo_top.v NOMINAL_OCXO；counter_measure.v；gnss_gpsdo_top.v u_spi；spi_dac.v；用户硬件型号；ps_key.c/h；main.c handle_ps_key；led_ctrl.c。

## 图02 PL 与 PS 模块架构

[修正版PDF](../fig02_logic_architecture.pdf) · [PNG](../figures/fig02_logic_architecture.png) · [SVG](../figures/fig02_logic_architecture.svg) · [draw.io](../figures/fig02_logic_architecture.drawio)

| 原图位置 | 原内容 | 应替换为 | 连线或逻辑调整 |
|---|---|---|---|
| 最上层外部设备与信号线 | 10 MHz；MCP4921+EFC；DAC 指向 PL 的 0～5 V | 20 MHz OCXO；DAC8531；PL 输出 SPI 到 DAC，DAC 模拟 VOUT 到 OCXO EFC。 | 反转原错误控制箭头，增加 OCXO OUT 回到 PL 的反馈。 |
| PL 内 counter_measure → tdc_carry → phase_error 串联链 | 整数计数模块串接后驱动 TDC | 同一 PPS 同时进入整数计数和 TDC/sub 快照；ocxo_frac 生成完整周期分数，再配对送 phase_error。 | 整数路径与细分路径并行，在 counter_measure 配对；meas_done 送 phase_error。 |
| PS 自适应R → KF、慢环 → DAC 部分 | 质量指标在线拟合 R；两路分别驱动 DAC | 运行 R 固定 80 ns²，训练关闭；快估计与 C 候选先融合，仅一个 DAC 控制入口。 | GNSS 质量进入校验/状态判断；离线 R 建模作为扩展。 |
| 寄存器列表与右侧黄色说明 | 屏未完成、LED代替；缺少结果序号和小数诊断 | 屏幕两页已实现；增加 0x60/64/68/6C 诊断；0x00 为 0.1953125 ns/LSB 的逐秒增量。 | path2_measure 保留诊断，不能画成最终 C 候选的独立测量源。 |

代码依据：XDC；gnss_gpsdo_top.v；counter_measure.v；ocxo_frac.v；phase_error.v；adaptive_r.c/h；main.c；main.h；bsp.h；screen.c；slow_loop.c。

## 图03 PS 状态转移与恢复

[修正版PDF](../fig03_ps_state_machine.pdf) · [PNG](../figures/fig03_ps_state_machine.png) · [SVG](../figures/fig03_ps_state_machine.svg) · [draw.io](../figures/fig03_ps_state_machine.drawio)

| 原图位置 | 原内容 | 应替换为 | 连线或逻辑调整 |
|---|---|---|---|
| HOLDOVER 外推框与回 TUNING 的长回路 | X0 += X1*T；等待5秒后固定回 TUNING | 守时每秒预测 X 与 P，恢复时要求连续5个可用观测，回到失锁前的 TUNING 或 CALIB。 | 拆成两条恢复箭头；回 CALIB 继续保持 DAC 硬锁。 |
| 底部每秒动作 CALIB 行 | CALIB 调 adaptive_r_update 训练 R | 当前 EXP_R_TRAIN_ENABLE=0；CALIB 保留原始测量和质量日志，在线训练不运行。 | R 标签在线外生成，不接到控制。 |
| enter_state 副作用与按键说明 | 只列 epoch 归零和 adaptive_r_reset | 还需清慢环/C窗口与融合权重；按状态设置 DAC 硬锁；KEY1 单击切状态、两秒内双击强回 TUNING，KEY2 翻页。 | 失锁前状态保存与恢复分支相连。 |

代码依据：main.c task_holdover_second；control_resume_from_holdover；main.c task_calib_second；main.h；main.c enter_state / handle_ps_key。

## 图04 逐秒采集与控制更新流程

[修正版PDF](../fig04_ps_control_pipeline.pdf) · [PNG](../figures/fig04_ps_control_pipeline.png) · [SVG](../figures/fig04_ps_control_pipeline.svg) · [draw.io](../figures/fig04_ps_control_pipeline.drawio)

| 原图位置 | 原内容 | 应替换为 | 连线或逻辑调整 |
|---|---|---|---|
| 顶端 epoch 门控及 CALIB/HOLDOVER 分支 | 所有动作先受 sample_period 门控 | 每秒先统一采样与相位连续性处理；仅 TUNING 的 KF/DAC 更新受 Tu 门控；守时按1秒预测。 | 先画逐秒采集，再画状态调度与 TUNING 到期分支。 |
| meas_valid? 后直接 return | 只检查测量有效位，坏数据直接返回 | 统一快照与 SAT/TORN/DUP/GAP/NO_MEAS 判定；坏秒标记整窗缺口，控制拍只预测不写 DAC。 | 模型补值维持内部参考，但不得混入真实性能评价；当前fix_50ns_glitch仅诊断并原样返回，不作±50ns改值。 |
| R更新框与双路交接大分支 | 先 KF 后设R；active/done 决定谁掌握 DAC；3σ√R；T>1000停止 | 先设 R80，再 KF；C 候选按5σ√S、age/lag与分歧保护后受限融合；满窗继续滚动。 | 删除旧的独占交棒和冻结路径，改为一个融合出口；无效退回快环。 |
| 底部慢环子流程 | 读取0x48/4C，PL NEW推动慢环、动态增长T | 慢环取 PS 累计相位；每秒 slow_tick，300秒采一点，最多3点；C去除DAC历史后拟合。 | path2 NEW 不再作为主慢环计算驱动。 |

代码依据：main.c 主循环、task_tuning_second、task_holdover_second；bsp.c bsp_sample_classify；main.c gap_this_window / fix_50ns_glitch；main.c task_tuning_second；slow_loop.c；slow_loop.c slow_tick / slow_c_sample。

## 图05 KF 与 DAC 控制闭环

[修正版PDF](../fig05_ps_pl_ocxo_loop.pdf) · [PNG](../figures/fig05_ps_pl_ocxo_loop.png) · [SVG](../figures/fig05_ps_pl_ocxo_loop.svg) · [draw.io](../figures/fig05_ps_pl_ocxo_loop.drawio)

| 原图位置 | 原内容 | 应替换为 | 连线或逻辑调整 |
|---|---|---|---|
| 上部计数器与标称值说明 | 每PPS锁存清零；count-10e6 | 自由运行计数器相邻PPS锁存作差，N-20,000,000。 | 不在每次PPS清空自由计数器；增加整数与分数同秒配对。 |
| CARRY4 与 0x00 相位差说明 | 0x00 s32，10 ns/LSB；X1=真实频差 | 0x00 为相邻PPS的 Q8 OCXO 周期增量误差，刻度50/256 ns；X1是相对GNSS的频率估计。 | PL增量先经质量判定并累计为 z，再送KF。 |
| PS控制框、SPI及DAC框 | dac_acc -= X1/0.20；16位0011头；MCP4921 | f_ctrl=(1-w)X1+w*f_C_now；Dacc-=sign*f_ctrl/1.91406；24位0x00头；DAC8531。 | 在KF预测前补已知 DAC 阶跃；相位回零项关闭。 |

代码依据：counter_measure.v；phase_error.v；bsp.h；main.c；control.h/c；main.c；spi_dac.v。

## 图06 仿真台架与验证分层

[修正版PDF](../fig06_testbench_structure.pdf) · [PNG](../figures/fig06_testbench_structure.png) · [SVG](../figures/fig06_testbench_structure.svg) · [draw.io](../figures/fig06_testbench_structure.drawio)

| 原图位置 | 原内容 | 应替换为 | 连线或逻辑调整 |
|---|---|---|---|
| 左上缩时参数与仿真 DUT 参数 | 缩时OCXO标称100，硬件又写10MHz，易误认为同一频率 | 明确缩时测试参数与100MHz FCLK、20MHz被测源不同；改频率时标称计数与PPS门长同时调整。 | 保留激励→DUT→探针/断言结构。 |
| SPI用例与通过状态 | 0xABC→16位，0011头，MCP4921；隐含全部已通过 | 当前SPI检查24位0x00+0xABC0，按下降沿采样；旧tb不当作当前全量回归通过。 | 增加主机门控/恢复/限权测试层，真实上板日志另作证据。 |

代码依据：历史 sim/rtl 测试文件；当前 RTL 参数；gnss_gpsdo_top.v；spi_dac.v；sim/run_host_tests.py。

## 图07 PL 接口状态与结果发布

[修正版PDF](../fig07_pl_state_machine.pdf) · [PNG](../figures/fig07_pl_state_machine.png) · [SVG](../figures/fig07_pl_state_machine.svg) · [draw.io](../figures/fig07_pl_state_machine.drawio)

| 原图位置 | 原内容 | 应替换为 | 连线或逻辑调整 |
|---|---|---|---|
| SPI状态机右侧参数表 | DAC_BITS=12，HDR=0011，FRAME_LEN=16，DAC上升沿采样 | DAC_BITS=16，HDR_BITS=8，HDR=0，FRAME_LEN=24；DAC8531在SCK下降沿采样。 | LOW保持MOSI，HIGH结束时下降沿采样旧位、再移位；剩余位循环，最后HOLD→IDLE。 |
| counter_measure与TDC小框 | PPS到来 count清零；TDC无配对事件 | 自由计数作差；tap_valid/fine_valid/frac_valid/meas_done对应不同流水阶段。 | 测量完成后才发布下游结果，防止上一秒数据。 |
| 邮箱与AXI握手区域 | 将SPI busy、NMEA NEW、AXI flag混读 | NMEA NEW在0x04 bit8；写0x100清除；SPI busy在STATUS bit6；AW/W独立锁存，AR独立返回。 | 邮箱占用且无ACK时丢新字节；ACK同拍可接新字节。 |

代码依据：spi_dac.v 生效逻辑；gnss_gpsdo_top.v 实例参数；counter_measure.v；phase_error.v；nmea_mailbox.v；gnss_axi_ip_slave.v；bsp.h。

## 图08 C 慢环计算与受限接入

[修正版PDF](../fig08_dual_loop_cooperation.pdf) · [PNG](../figures/fig08_dual_loop_cooperation.png) · [SVG](../figures/fig08_dual_loop_cooperation.svg) · [draw.io](../figures/fig08_dual_loop_cooperation.drawio)

| 原图位置 | 原内容 | 应替换为 | 连线或逻辑调整 |
|---|---|---|---|
| 标题与中部 DAC 归属仲裁 | 双路DAC交接；快慢环分别掌舵 | 快慢估计受限融合；f_ctrl=(1-w)X1+w*f_C_now。 | 两路估计先进入融合框，仅融合后的一个控制量写DAC。 |
| 慢环候选与输入来源 | f2直接取历史相位斜率；PL NEW给样本 | C+=g*(D_prev-D_ref)*dt；phi_ref=phi-C；拟合f_ref，再加回当前DAC。 | 使用上一段实际码值积分，当前码值重建；MODE2无有效C时退回快环，不回旧slow.freq。 |
| 底部交接条件和保护阈值 | 3σ√R、T/2近似、T>1000后不再调DAC | 5σ√S；使用真实lag；n≥2且span≥240s；age≤330s、lag≤630s；满窗持续滚动。 | w_target=min(0.1,0.1*w_stat)，每秒上升≤0.1/300，失效立即退出。 |

代码依据：main.c task_tuning_second；slow_loop.c slow_c_advance / slow_c_sample / slow_c_current；slow_loop.h；main.c；slow_c_weight_step。

## 图09 串口屏两页与字段来源

[修正版PDF](../fig09_screen_pages.pdf) · [PNG](../figures/fig09_screen_pages.png) · [SVG](../figures/fig09_screen_pages.svg) · [draw.io](../figures/fig09_screen_pages.drawio)

| 原图位置 | 原内容 | 应替换为 | 连线或逻辑调整 |
|---|---|---|---|
| 页间翻页箭头、CTRL 页示例T | PSkey1翻页；T=5s；空白位置可显示DAC/LOCK | KEY2=MIO51翻页；稳态Tu=10s；DAC主要在日志查看，未实现控件不作为现有功能。 | KEY1=MIO50仅切TUNING/CALIB，双击例外单独说明。 |
| 两页控件分配、行序与慢环T栏 | 第一版修正版依旧注释把卫星数/HDOP/UTC放到控制页，漏掉gpslw；纬经度合成一行。 | 以TJC.HMI活动0.pa/1.pa为准：页0依次gpstxt、gpfrq、gpphs、gpt、gpslw、gppps；页1依次gpsl、gpsn、gphd、gpcn、gpsw、gpsj、gpst、gpsd。 | 页0六控件，下方40像素空白；页1八控件；屏工程128×160，每行20像素。GPS信息不再重复画到页0。 |
| 日期与相位/频率数值 | 显示具体日期；频率与相位均像独立实测真值 | 当前gpsd固定显示--；gpfrq为X1估计，gpphs为软件累计相位估计；gppps为本秒测量增量。gpslw显示slow.active时的slow.T，否则--，不是C参与率。 | 失去定位或HOLDOVER时gpfrq/gpphs显示----；CALIB中X0/X1可以冻结，不得用其绘制实时标定漂移。 |

代码依据：main.c handle_ps_key；screen.h/c；D:/FPGAdesigner/TJC/TJC.HMI活动页面及对象坐标；screen_layout_evidence.json；screen.c screen_send_status。

## 图10 TDC 与粗分数配对时序

[修正版PDF](../fig10_tdc_measurement.pdf) · [PNG](../figures/fig10_tdc_measurement.png) · [SVG](../figures/fig10_tdc_measurement.svg) · [draw.io](../figures/fig10_tdc_measurement.drawio)

| 原图位置 | 原内容 | 应替换为 | 连线或逻辑调整 |
|---|---|---|---|
| tap→fine归一化框 | fine=(tap+offset)/tap_period，Q31倒数实时生效 | 当前生效为固定403对应乘数：fine16=clip(((tap+offset)*41631)>>8)。 | 可写0x0C和倒数计算代码存在，不代表活跃输出会随其动态校准。 |
| 粗细合成框与底部精度说明 | 粗项+(fine_prev-fine_now)；几十ps、提高160倍 | 先算完整OCXO周期分数：(sub_pps+1)*256-fine8；Q16=(N-20M)*65536+frac_now-frac_prev；Q8=(Q16+128)>>>8。 | 整数差与本秒分数配对，结果写入后meas_done；不额外跨圈补一整周期。 |
| 延迟链抽头与输出单位 | 每抽头20～80ps、160倍精度作为确定指标 | 128个CARRY4，共512抽头；0x00数字格式刻度0.1953125ns/LSB。 | 抽头延迟和绝对精度需要独立标定；该刻度不能当作测量准确度。 |

代码依据：tdc_carry.v TAP2FRAC与fine_fraction赋值；ocxo_frac.v；counter_measure.v；phase_error.v；tdc_carry.v 参数；bsp.h PHASE_ERR_LSB_NS。

## 继续编辑

picture_corrected_20261101.docx 是十图集合。单图SVG保留文字；draw.io使用未压缩mxGraph XML；wps.json是WPS流程图数据。source/中保留从原picture提取的5份原生数据及重绘脚本。

硬件接线以当前生效约束为准：F20=1PPS，F17=NMEA，K17=OCXO OUT，G20=SCK，H18=DIN，L20=CS_N。XDC旧注释中的冲突映射不作为依据。

SPI_DIV=8时，本SPI实现用含零计数的等待，稳态SCK周期为2×(8+1)=18个FCLK周期；100MHz下约5.56MHz。旧图6.25MHz是按理想除数8计算，不能当作该RTL的精确值。修正版图只列参数，并以实际计数逻辑为准。

图09已直接核对TJC.HMI活动页面，不再沿用screen.h的旧分页注释。页0六项：状态、频差、累计相位、Tu、慢环T、本秒相位增量；页1八项：定位、卫星、HDOP、CN0、纬度、经度、UTC、日期。页0下方40像素无控件。提取证据见figure_sources/screen_layout_evidence.json。

日期控件gpsd当前固定--。屏幕gpfrq为X[1]估计，gpphs为软件累计相位估计；两者在CALIB可保持，无定位或HOLDOVER显示----。gpslw在slow.active时显示slow.T，否则显示--，不是C融合权重。原始性能分析用REFCHK及有效性判定，模型补值不混入实测。

fix_50ns_glitch保留函数入口，但当前实现在诊断后原样返回p，没有执行±50ns软件改值；图04按当前可执行逻辑表示。图10中的clip16表示将结果限幅到0..65535，周期边界不能靠16位截断回绕。
