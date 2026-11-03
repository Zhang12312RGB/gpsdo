# 结论与复现材料对应

| 内容 | 原始或源码 | CSV与复现 | 范围 |
|---|---|---|---|
| 默认固件身份 | board/firmware/gpsdo_final_c.elf；src/sw | v27启动四行；sim/results/hardware_acceptance_20261102.json | Nov2 15:29:18；29源相同；未读回板上内存哈希 |
| 板上状态及保持 | data/raw_support/log_20261102_v27.dat | data/csv/acceptance；check_acceptance.py | CALIB→HOLDOVER→CALIB；DAC1851；回CALIB20s |
| 最终C参与 | v27 CFUSE/SLOWC | v27_cfuse.csv、v27_slowc.csv | 连续37拍正权重；0.3333%～6.0418%；不用于性能收益推断 |
| 启动X[1]/DAC | log_20261031_v24.dat | data/csv/current；reproduce_results.py --plots | 历史同正常控制配置 |
| 三轮OADEV | v23、v24、v25 DAT | 对应主CSV；reproduce_results.py | 各2400点，1200000≤ms<3600000；600s仅参考 |
| 历史C性能窗 | v24 CFUSE/SLOWC | 性能窗240/240拍；均重4.4796% | 参与成立，单组收益未确认 |
| R模型 | v14/v22/v26冻结记录 | 331窗；reproduce_results.py --r | 常数/四参数整轮留出；不证明R80最优 |
| 物理连接 | hardware_interfaces.md；当前XDC | board/photos | J11 PL40针、J12 PS12针、5V输入；不编造模拟参数 |
| 报告照片 | board/photos | 正面、斜视、PCB、控制page0、GNSS page1、PC日志 | PC照片fmod=0为界面示例；MODE2证据用v27文本 |
| 绝对准确度 | 尚无独立仪器记录 | 报告P10 | 相对GNSS稳定度与绝对准确度分别说明 |

data/results中的r_models_all、reduced、nonlinear等为历史探索，不替代当前331窗核算。v27故障验收不并入OADEV或R训练。
