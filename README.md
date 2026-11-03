# 基于 CARRY4 延迟链亚纳秒插值GNSS 驯服频率源
本作品以 AX7010（Zynq-7010）为核心，利用 GNSS 1PPS 对20 MHz OCXO进行频率驯服。PL负责计数、分数测量与外设时序，PS负责状态估计、质量门控、快慢环融合及DAC调节。PC用于日志保存和离线分析，实时控制在板上运行。

本目录为2026年11月2日的比赛提交副本。先阅读本页，再按需求进入报告、上板或复现材料。

## 阅读与复现入口

| 需求 | 入口 |
|---|---|
| 了解作品原理与成果 | [技术总结Word](report/design_report.docx)、[文本](report/design_report.md)（PDF 版按赛事要求单独提交） |
| 查看当前配置 | [最终配置](docs/final_configuration.md)、[版本说明](docs/final_configuration.md) |
| 下载与验收 | [上板文件](board/README.md)、[操作步骤](docs/board_acceptance.md)、[本次实测结果](docs/hardware_acceptance_20261102.md) |
| 从源码重建 | [源码说明](src/README.md)、[构建说明](build/README.md)、[完整复现指南](docs/reproduction.md) |
| 核算原始性能 | [数据说明](data/README.md)、[CSV字段](docs/csv_schema.md)、[证据对应表](docs/evidence_map.md) |
| 查看十张详图 | [图索引](docs/diagram_index.md)、[详图PDF](docs/detailed_flowcharts.pdf) |
| 复用分析方法 | [技能包与脚本](skill/README.md) |

## 提交运行配置

| 项目 | 取值 |
|---|---|
| 输出与参考 | 20 MHz OCXO；GNSS 1PPS |
| 主要器件 | AX7010、NEO-M8T-0-11、DAC8531 |
| 供电 | 开发板和底板输入均为5 V |
| 主KF | Q_FREQ=1e-4，关闭P3经验下限，R=80 ns² |
| 控制节拍 | 首次1 s，随后固定Tu=10 s |
| 相位回零项 | 关闭；累计相位观测继续用于估计频率 |
| 慢环 | MODE2，C补偿候选受限融合，真实lag降权、渐增及失效退出 |
| 在线R训练 | 关闭，保留为扩展 |

默认固件为[board/firmware/gpsdo_final_c.elf](board/firmware/gpsdo_final_c.elf)，BUILD **Nov 2 2026 15:29:18**，SHA256：

```text
29717ca20361051e5f3f7cf1d936760821ac88e9ee63a9da41a89c0d7fc2b7e9
```

MODE0回退文件为[gpsdo_baseline_mode0.elf](board/firmware/gpsdo_baseline_mode0.elf)。逻辑release标签保留20261031，真实编译时刻以BUILD为准。重编译会改变BUILD和文件哈希。

## 已有实测结果

性能数据使用同正常控制配置的v24历史记录，统一取开机第20～60分钟：1200000≤ms<3600000，共2400个有效逐秒点。原始通道OADEV@100/200/300 s分别为**0.20775/0.11476/0.06715 ppb**；平均相对频差为−0.01253255 ppb。20 MHz下1 ppb对应20 mHz、1 ns/s和1e-7%。这些是相对当次GNSS的结果，未进行独立绝对准确度标定。

最终固件的v27功能验收记录覆盖CALIB→HOLDOVER→CALIB，DAC命令保持1851；后续连续37次更新由src=2参与，实际权重0.3333%～6.0418%，hreset全程为0。恢复后CALIB仅记录20 s，TUNING失锁返回路径未单独测试。故障验收文件不混入稳定度评价。

慢环参与已获得日志证据；一组关→开→关对照尚不足以确认长期收益的可重复性。R建模统一使用331个冻结窗口，四参数未显示稳定的留出优势，因此运行保留固定R80。

## 文件结构

| 目录 | 内容 |
|---|---|
| src/ | PS源码、PL RTL、XDC、Block Design |
| build/ | 可移植构建脚本、BSP及历史实现报告 |
| board/ | 提交ELF、配套bit/XSA、初始化脚本与实物照片 |
| data/ | 52份DAT、派生CSV、结果快照及索引 |
| docs/ | 接线、配置、复现、验收、流程图和提交说明 |
| report/ | 技术总结、性能图、分析及开发历史 |
| sim/ | 主机验证、历史RTL平台及已有验证结果 |
| skill/ | 可复用分析脚本、工作流和纠错经验 |

## 最短复核流程

以下命令均在本目录执行；前三条仅需要Python标准库。

```text
python skill/scripts/verify_manifest.py
python skill/scripts/check_acceptance.py
python skill/scripts/reproduce_results.py
```

需要重建R标签或绘图时，再安装可选依赖并执行：

```text
python -m pip install -r requirements-analysis.txt
python skill/scripts/reproduce_results.py --r --plots
```

复算默认输出至data/reproduced，保留原始DAT与冻结结果。CSV缺失字段留空；断档、坏样本和模型补值不作为连续实测样本。

manifest.json校验本副本文件；docs/file_map.csv记录文件来源。旧日志、早期结论和不同BUILD不自动代表当前配置。板卡操作按验收指南执行，视频、历史图和主机测试分别注明其证据范围。
