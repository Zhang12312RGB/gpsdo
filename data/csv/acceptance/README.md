# v27板上功能验收CSV

来源为[data/raw_support/log_20261102_v27.dat](../../raw_support/log_20261102_v27.dat)，1154条完整主记录。主表和六类事件旁表使用与历史集合相同的字段，csv_manifest.json绑定源与派生文件哈希。

已记录CALIB→HOLDOVER→CALIB，DAC1851保持；返回TUNING后连续37拍src=2、正权重，范围0.3333%～6.0418%，hreset=0。返回CALIB仅20 s，未单独测试TUNING失锁返回。

本集合含故障、缺口和模型补值，不用于OADEV或R训练。ms步长有一次1008与一次992，保留原始调度偏移，不改写成严格1000 ms。

在提交根目录执行：

```text
python skill/scripts/export_raw_csv.py data/raw_support/log_20261102_v27.dat --output data/csv/acceptance
python skill/scripts/check_acceptance.py
```

完整范围见[验收说明](../../../docs/hardware_acceptance_20261102.md)。
