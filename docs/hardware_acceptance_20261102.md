# 最终固件板上功能验收

2026年11月2日。此记录检查功能与异常恢复，未将故障段用于稳定度评价。

## 固件和源码对应

原始文件：ReceivedTofile-COM5-2026_11_2_15-28-33.DAT。原始字节归档为 data/raw_support/log_20261102_v27.dat，SHA256 `c815d0bcebfeb9fa95dd829b55fcd79bf4c7b813c645106737c657eb9039a48e`，834215字节。共1154条完整REFCHK及状态记录，ms=1000～1154000，无不完整尾行。

本次默认 board/firmware/gpsdo_final_c.elf 使用操作者运行对应的重编译文件，BUILD为2026-11-02 15:29:18，SHA256 `29717ca20361051e5f3f7cf1d936760821ac88e9ee63a9da41a89c0d7fc2b7e9`。保存时TRY的29个PS源码与提交src/sw全部逐字节相同。旧10月31日默认ELF已保留在项目历史备份；MODE0回退文件不变。启动BUILD与配置对应主机归档ELF，但未执行板上指令存储器逐字节读回，不能仅凭串口证明板上内存哈希。

```text
===== BUILD Nov  2 2026 15:29:18 =====
KFCONF profile=1 QF_e9=100000 p3floor=0 fixed_period=1 phsg=0 slow_mode=2
CFCONF scale_u=100000 cap_u=100000 rise_s=300 age_max_s=330 lag_max_s=630
FINALCONF release=FINAL_C_R80_T10_20261031 R_ns2=80 R_train=0 period_fixed=1 phase=0 slow=2
```

逻辑release标签保留20261031，真实编译时刻以BUILD为准。

## 状态和DAC保持

| 状态 | 起止ms | 完整记录数 | DAC命令 |
|---|---:|---:|---:|
| TUNING | 1000～121000 | 121 | 1752～1851 |
| CALIB | 122000～343000 | 222 | 1851 |
| HOLDOVER | 344008～455000 | 112 | 1851 |
| CALIB | 456000～475000 | 20 | 1851 |
| TUNING | 476000～1154000 | 679 | 1851～1856 |

日志先出现STATE -> CALIB，再进入HOLDOVER，随后“恢复完成, 回到 CALIB”及STATE -> CALIB。CALIB和HOLDOVER所有DAC命令均为1851，验证冻结状态失锁后仍恢复冻结。中间旧提示“平滑5秒后回TUNING”是泛化文字，实际目的状态应看恢复完成和STATE行，未据该提示误判。

恢复后CALIB只保存20秒，短于操作指南建议的60秒；本轮未单独测试TUNING→HOLDOVER→TUNING。实际记录覆盖了重要返回路径，不声称所有异常分支均已验收。

## 实际慢环参与

CFUSE共82次控制更新。ms=787000～1147000连续37次出现SLOWC src=2、v=1、CFUSE reason=0且used_u>0。实际权重为used_u/1e6，范围0.003333～0.060418，即0.3333%～6.0418%，未超过10%上限。首两拍used_u为3333和6667，显示渐增；末拍span=599秒、age=72.010秒、lag=371.427秒、used_u=55432。

该结果证明最终配置的C候选进入了DAC控制融合通路。它不证明慢环长期稳定度收益；长期对照仍使用v23/v24/v25统一2400秒分析窗。

## 门控和时间连续性

全轮hreset=0。参考失效期间最终bad=111、gapfill=111、gapskip=1；这些是故障验收计数，不算正常稳态性能。重新进入TUNING后的679条记录why全部为0，gapfill不再增加。

相邻ms步长1151次为1000、一次为1008、一次为992，属于8ms调度偏移及回归，没有大于1500ms的记录断档。严格等间隔CSV分段仍保留这两个偏移，不将其静默改成1000ms。

末条NMEA chk：ok=10938、badcs=45、badfmt=1、resync=1、gga_age=451、fresh=1。这里只报告解析与新鲜度，不能根据这些计数证明独立频率准确度。

## 复现与材料

data/csv/acceptance/保存主表和6种事件表、csv_index及csv_manifest；sim/results/hardware_acceptance_20261102.json保存完整核算和来源哈希。验收CSV与data/csv/current的13轮历史性能数据分别存放，避免覆盖旧索引或混入训练集。

```text
python skill/scripts/export_raw_csv.py data/raw_support/log_20261102_v27.dat --output data/csv/acceptance
python skill/scripts/check_acceptance.py
```
