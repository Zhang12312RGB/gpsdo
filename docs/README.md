# 文档导航

本目录共 12 份文档，分四类：配置与版本、复现与上板、数据与证据、提交与素材。
技术总结正文在 [report目录](../report/README.md)。历史说明见 [history/](history/)。

## 一、配置与版本

| 文档 | 内容 |
|---|---|
| [final_configuration.md](final_configuration.md) | **主文档**：默认ELF身份、KFCONF/CFCONF、TDC刻度、功能范围、性能边界 |
| [hardware_interfaces.md](hardware_interfaces.md) | J11/J12、FPGA管脚、5 V供电、显示连接，以及旧图与当前的差异 |

> 配置以板上实际 BUILD 与 KFCONF/CFCONF/FINALCONF 为准。
> 逻辑 release 标签 20261031 不变，真实编译时刻以 BUILD 为准。

## 二、复现与上板

| 文档 | 内容 |
|---|---|
| [reproduction.md](reproduction.md) | 从源码重建 PL/PS、主机测试、结果复算 |
| [board_acceptance.md](board_acceptance.md) | 启动、KEY1 冻结、失锁恢复、慢环参与的步骤，以及闭环到冻结的对照方法 |
| [hardware_acceptance_20261102.md](hardware_acceptance_20261102.md) | v27 实际验收结果与未覆盖范围 |

## 三、数据与证据

| 文档 | 内容 |
|---|---|
| [csv_schema.md](csv_schema.md) | 主 CSV 与事件表的单位、缺失值、分段规则 |
| [evidence_map.md](evidence_map.md) | 报告每条结论对应的原始数据、CSV 与脚本 |
| [diagram_index.md](diagram_index.md) | 十张系统详图与 draw.io/PNG/SVG 源文件 |

## 四、提交与素材

| 文档 | 内容 |
|---|---|
| [submission_checklist.md](submission_checklist.md) | AMD 自主选题初级组逐条核对 + 11月3日本地检查结果 + 待参赛者完成事项 |
| [third_party_notices.md](third_party_notices.md) | 第三方软硬件来源与许可 |
| [video_storyboard.md](video_storyboard.md) | 视频分镜、配音要点、素材来源与剪辑索引 |

## 其他资源

- [detailed_flowcharts.pdf](detailed_flowcharts.pdf) —— 十张详图汇总
- [figures/](figures/) —— PNG / SVG / draw.io 源文件
- [figure_sources/](figure_sources/) —— 屏幕页面核对资料
- [video_subtitles.srt](video_subtitles.srt) / [video_voiceover.txt](video_voiceover.txt) —— 字幕与配音

> 图号 01～10 是详图附件编号，与技术总结正文图号分别使用。

> 视频素材与日志摘录未包含在本提交包内；
> 视频成片按赛事要求单独上传，脚本与分镜见 [video_storyboard.md](video_storyboard.md)。

## 阅读顺序建议

1. 先读 [report/design_report.docx](../report/design_report.docx)（技术总结正文）
2. 再读 [final_configuration.md](final_configuration.md)（当前配置）
3. 需要复现时读 [reproduction.md](reproduction.md)
4. 需要验收时读 [board_acceptance.md](board_acceptance.md) + [hardware_acceptance_20261102.md](hardware_acceptance_20261102.md)
5. 需要核对数据时读 [evidence_map.md](evidence_map.md) + [csv_schema.md](csv_schema.md)

## 证据边界

以下四类材料各有范围，不能相互替代：

| 材料 | 说明 |
|---|---|
| **历史性能记录**（v24） | 与最终固件同正常控制配置，但不是新 ELF 的实测 |
| **新固件功能验收**（v27） | 覆盖 CALIB→HOLDOVER→CALIB、37 拍 C 融合；不用于性能推断 |
| **主机测试** | 证明算法与编译通过，不等于硬件实测 |
| **旧版本说明** | 见 [history/](history/)，保留当时状态，不代表当前配置 |

特别地：稳定度指标相对当次 GNSS 参考，未做独立绝对准确度标定；
OADEV 不能当作定向相位漂移；慢环参与成立不等于收益成立。
