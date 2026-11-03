# 最终提交配置

2026年11月2日更新。输出20MHz；当前控制为Q_FREQ=1e-4、P3经验下限关闭、R=80ns²、稳态Tu=10s、相位回零关闭、C慢环MODE2、在线R训练关闭。初始首次更新为1s，此后固定10s。29个PS源与10个PL源本次未修改。

## 版本入口

2026-11-02更新。默认ELF BUILD Nov  2 2026 15:29:18，SHA256 `29717ca20361051e5f3f7cf1d936760821ac88e9ee63a9da41a89c0d7fc2b7e9`；对应v27板上功能记录。PS源码29文件与原冻结源码一致。MODE0回退仍为10月31日20:23:43。逻辑release标签20261031不变。

详见final_configuration.md、hardware_acceptance_20261102.md、reproduction.md与根manifest.json。旧版说明及验证JSON按其生成时间作为历史记录保留，不将旧ELF哈希当作本次默认文件身份。

---

## 运行文件

默认board/firmware/gpsdo_final_c.elf：BUILD Nov  2 2026 15:29:18，SHA256 `29717ca20361051e5f3f7cf1d936760821ac88e9ee63a9da41a89c0d7fc2b7e9`。该文件与本次验收时保留的TRY.elf逐字节相同，启动BUILD及配置匹配v27。原10月31日默认文件已备份，未冒称它接受了本次新ELF的硬件验收。

回退gpsdo_baseline_mode0.elf仍为Oct 31 2026 20:23:43，SHA256 `074d013e94baa5ef48c886e7bed10a4c0e72465e443af7d8b05b133d1848e593`。它关闭慢环，其余正常控制设置相同。重新编译会改变BUILD与ELF哈希，逻辑release标签20261031保持不变。

```text
KFCONF profile=1 QF_e9=100000 p3floor=0 fixed_period=1 phsg=0 slow_mode=2
CFCONF scale_u=100000 cap_u=100000 rise_s=300 age_max_s=330 lag_max_s=630
FINALCONF release=FINAL_C_R80_T10_20261031 R_ns2=80 R_train=0 period_fixed=1 phase=0 slow=2
```

## TDC 与分数刻度

OCXO 为 20 MHz，周期 50 ns；PL 时钟标称 100 MHz。NUM_CARRY4=128 对应 512 个抽头。0x00 接口刻度 50/256=0.1953125 ns/LSB，只是数字刻度；没有独立完成抽头非均匀性、温漂和绝对误差的可溯源标定，不能写为 195 ps 绝对精度。

当前使用统一序号快照与 SAT/TORN/DUP/GAP/NO_MEAS 门控。性能用有效原始 REFCHK 增量，不能把模型填补或 KF 平滑值当独立实测。旧抽头调参过程保留在开发目录 history，不是最终性能规格。

---
## 功能

| 功能 | 配置及已测范围 |
|---|---|
| GNSS与原始测量 | NMEA校验、GGA新鲜度、统一快照及SAT/TORN/DUP/GAP/NO_MEAS门控 |
| 快环 | 两状态KF，已知DAC阶跃补偿，缺口预测及DAC保持 |
| 慢环 | C补偿候选，span≥240s、age≤330s、lag≤630s；真实lag降权，目标min(0.1,0.1*w_stat)，每秒最多增加0.1/300，失效立即退出 |
| CALIB | 命令冻结，原始通道继续；KF显示保留，不用其冻结值计算物理漂移 |
| HOLDOVER恢复 | v27覆盖CALIB→HOLDOVER→CALIB，DAC1851保持，hreset=0；恢复CALIB仅20s，TUNING失锁返回未单测 |
| C实际控制 | v27连续37拍src=2且权重非零，0.3333%～6.0418% |
| 相位项 | 关闭回零控制，但累计相位观测仍用于估计频率；不声称UTC绝对相位同步 |
| 自适应扩展 | 固定R80，旧R训练关闭；Tu自适应分支保留，当前不参与运行 |

v27功能原始DAT及派生CSV见hardware_acceptance_20261102.md，不作为长期稳定度记录。串口配置与归档ELF建立对应，未做板上指令内存逐字节读回。

## 已有稳定度

v24为相同正常控制配置历史记录，分析窗1200000≤ms<3600000，2400个有效逐秒点。平均相对频差−0.01253255ppb；OADEV@100/200/300s为0.207752/0.114758/0.067145ppb，@600s=0.037531ppb仅参考。它与v27功能证据分开。

20MHz下1ppb=20mHz=1ns/s=1e-7%。平均偏移对应−0.250651mHz、−0.01253255ns/s、−1.25326e-9%；OADEV换算ns/s仅为频率波动尺度，不能写成每秒确定漂移，也不是时间偏差TDEV。0.1953125ns/LSB为数字接口刻度，非独立绝对准确度。

v23关/v24开/v25关同窗对照呈部分长周期改善信号，只有一轮开启，未确立可重复收益。不能只看X[1]平滑或候选v=1判断慢环效果。

R研究按v14/v22/v26共331个40s冻结窗口复算，整轮留出常数模型RMSE11.716ns²，线性四参数12.202ns²；未显示四参数稳定优势。固定R80是已测试工程配置，不是拟合得到的纯噪声或最优值。

## 复现边界

PS构建、主机测试、原始CSV及数据复算入口见reproduction.md。PL仍使用历史bit/XSA和实现报告，本次未重新实现或下载；时序报告仍有4项外设延迟约束方法学告警。源快照不能单独证明板上加载bit的逐字节身份。

板卡5V供电、J11/J12、器件近照见hardware_interfaces.md。更换硬件后应重新复验，不把本机相对GNSS测量写成独立绝对准确度。
