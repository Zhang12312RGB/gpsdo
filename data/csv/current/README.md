# 近期13轮派生CSV

本集合由v14～v26原始DAT生成，共59616条REFCHK主记录，其中2条尾部不完整记录保留并由record_complete标记。索引记录源哈希、行数和断档。

| 文件后缀 | 内容 |
|---|---|
| 无后缀主CSV | 同秒REFCHK、状态、原始值、X[0]/X[1]、GNSS质量和门控字段 |
| _slowdbg | 新息、慢环事件等诊断 |
| _slowc | C候选、有效性、span/age/lag及实际来源 |
| _cfuse | 实际权重及退出原因；used_u/1e6为权重 |
| _samp | SAT/TORN/DUP/GAP/NO_MEAS等样本判定 |
| _nmea | 校验、新鲜度和计数 |
| _state_events | 状态事件及相邻REFCHK上下文 |

重新生成：在提交根目录运行`python skill/scripts/export_raw_csv.py --all-recent`。解析不改原始DAT。没有phsg字段的旧日志不能被解释为相位项关闭。事件上下文时间不等于精确硬件事件时刻。

v23/v24/v25性能窗均为1200000≤boot_ms<3600000；只取有效、完整且连续的原始记录。全量CSV与选窗规则同时保留。字段见[字典](../../../docs/csv_schema.md)，原始数据见[说明](../../README.md)。
