# 分析结果与冻结标签

本目录保存已引用的核算快照及历史探索。重新复算默认输出至data/reproduced，不覆盖本目录。

| 主要文件 | 用途 |
|---|---|
| slow_loop_three_runs.json | v23/v24/v25同窗慢环关开关OADEV |
| v24_recomputed.json、v24_performance.csv | 同配置v24性能及等效换算 |
| r_parameter_reasonableness_labels_20261101.csv | 当前R主核算标签；选择v14/v22/v26的331个冻结窗口 |
| r_parameter_reasonableness_20261101.json | R参数合理性与留出验证 |
| r_v26_segments.csv | 雨天重复记录的分段边界及DAC条件 |
| r_models_all/reduced/nonlinear等JSON | 不同阶段的模型探索，样本集合可能与主核算不同 |

当前结论优先按[证据对应表](../../docs/evidence_map.md)与`reproduce_results.py --r`核对，不将旧探索结果与331窗主集合混写。标签不是纯观测噪声方差；四参数回归未显示稳定优势，固定R80是工程配置。

结果中的作者机器路径保留作历史来源，不是运行要求。所有现行脚本从自身位置解析提交根目录。v27功能结果另见sim/results/hardware_acceptance_20261102.json，不并入这些稳定度或训练结果。
