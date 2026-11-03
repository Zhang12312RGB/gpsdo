# 原始测量数据与派生文件

原始DAT是数据核对依据，raw_index.csv记录52份文件的来源、字节数、SHA256、BUILD及状态行数。保留原始文件和完整运行区间，结果可由包内脚本重新计算。

| 目录 | 内容与用途 |
|---|---|
| raw/ | 5份早期对照记录 |
| raw_support/ | 26份开发、性能与验收支撑记录，包括v14～v27 |
| raw_archive/ | 21份历史采样/归档记录 |
| [csv/current/](csv/current/README.md) | v14～v26共13轮，59616条主记录，保留2条不完整尾行 |
| [csv/acceptance/](csv/acceptance/README.md) | v27功能验收，1154条完整主记录 |
| [results/](results/README.md) | 已引用结果、R标签和历史模型探索 |

## 当前性能核算

v23慢环关、v24受限慢环开、v25慢环关，统一使用1200000≤ms<3600000的2400个逐秒点。分析原始REFCHK raw×0.1953125，而不把KF X[1]作为真实输出稳定度。

在1 s间隔下，增量ns/s与频差ppb数值相同。20 MHz时1 ppb=20 mHz=1 ns/s=1e-7%。OADEV是频率波动指标，不能直接当成累计相位误差或绝对准确度。600 s结果仅参考。

## R与功能数据的边界

R主核算采用v14/v22/v26冻结段的33+165+133=331个40 s窗口，窗口边界由冻结标签定义，不把新增尾部自动并入训练。标签包含测量链和参考等影响，不等于纯观测方差；固定运行R80并非标定所得最优值。

v27含参考失效、HOLDOVER及模型补值，仅验证冻结、恢复和实际慢环参与，不混入OADEV或R训练。v19电脑休眠导致的日志断档按真实时间切分。

## 从DAT到CSV及结果

以下命令在提交根目录执行：

```text
python skill/scripts/export_raw_csv.py --all-recent
python skill/scripts/export_raw_csv.py data/raw_support/log_20261102_v27.dat --output data/csv/acceptance
python skill/scripts/reproduce_results.py
python skill/scripts/check_acceptance.py
```

重建R标签及绘图使用可选依赖：

```text
python -m pip install -r requirements-analysis.txt
python skill/scripts/reproduce_results.py --r --plots
```

输出默认写入data/reproduced。单份导出应指定独立--output，避免覆盖既有集合索引。字段单位、缺失值和序号定义见[CSV字典](../docs/csv_schema.md)。历史归档曾存在CRLF/LF转换，应以包内SHA及文本内容核对，不能只比较文件大小或状态行数。
