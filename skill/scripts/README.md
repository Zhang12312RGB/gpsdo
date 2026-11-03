# 分析脚本使用说明

所有命令在完整提交根目录执行，脚本根据自身位置解析根目录。

| 脚本 | 输入与输出 | 依赖 |
|---|---|---|
| verify_manifest.py | 按manifest逐文件核对SHA及英文路径，不校验板上内存 | Python标准库 |
| export_raw_csv.py | REFCHK原始DAT转主CSV与六种旁表、索引及哈希清单 | Python标准库 |
| analyze_stability.py | 指定完整连续区间，计算原始OADEV与平均频差 | Python标准库 |
| reproduce_results.py | 复算三轮同窗性能；可选重建331窗R标签及绘图 | 基本计算用标准库；--r需NumPy；--plots需Matplotlib |
| check_acceptance.py | 核对归档v27的配置、状态、冻结码值、权重和ELF对应 | Python标准库 |

```text
python skill/scripts/verify_manifest.py
python skill/scripts/export_raw_csv.py --all-recent
python skill/scripts/reproduce_results.py
python skill/scripts/check_acceptance.py
python skill/scripts/analyze_stability.py data/raw_support/log_20261031_v24.dat --start-ms 1200000 --end-ms 3600000 --output data/reproduced/v24.json
```

单份导出使用独立目录：

```text
python skill/scripts/export_raw_csv.py data/raw_support/log_20261102_v27.dat --output data/csv/acceptance
```

--all-recent仅选择v14～v26，不自动加入v27。导出器会重写目标集合索引，不能将单份新日志直接导到已有current集合。原始DAT不改写，缺失字段留空。

稳定度脚本要求完整、有效且连续的单状态分析区间；拒绝坏样本和断档。新息、X[1]波动及原始OADEV分别统计。check_acceptance针对已归档v27，不能对任意新日志宣称相同验收结果。
