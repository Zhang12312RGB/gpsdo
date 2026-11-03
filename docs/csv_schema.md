# 原始日志与CSV字段说明

原始DAT为复现依据，CSV为可重新生成的派生文件。主表每行对应一条REFCHK，不按KF值变化猜测采样时刻。保留板上boot_ms和seq；record_index只是文件内行号，不代替开机时间。

## 主表字段

| 字段 | 含义与单位 |
|---|---|
| boot_ms / boot_s | 板上开机时间，ms/s；缺失留空 |
| seq0 / seq1 | 同一快照前后序号；不相等为撕裂 |
| refchk_line / state_line | 原始DAT中的1起始行号，便于定位 |
| record_complete | 是否具有完整REFCHK和对应状态行；0的尾部行不能用于性能统计 |
| raw_q8 | 0x00带符号原始寄存器整数 |
| raw_phase_increment_ns | raw_q8×0.1953125，单间隔相位增量ns |
| raw_frequency_ppb_1s | 在1 s正常测量间隔下等效频差ppb；异常间隔不可据此当有效频率 |
| ocxo_delta_count / fclk_delta_count | 两路相对标称1 s计数的增量，单位计数，不是ns |
| fraction_q16 / status | 同快照分数和状态原值 |
| dac_snapshot / dac_command | REFCHK采样时与状态行时的命令码，切换边界可能有时序差 |
| X0_ns / X1_ppb | phsm/1000及frqm/1000；为估计器状态，CALIB里可能冻结 |
| Tu_s / R_ns2 | 状态行的控制更新周期s和日志R值ns²，不能视为每秒都更新 |
| mode / phase_control_enabled | fmod与phsg；旧版本没该字段时留空，不能当0 |
| fix / sat / hdop / cn0_dbhz | 质量字段；hdop无量纲，CN0单位dB-Hz；不凭CN0高低改写样本有效性 |
| hdop_display / cn0_display | 原显示字符串，用于追溯旧小数进位日志问题 |
| sample_reason / acc_used / kf_used | SAMP why/acc/used；acc是是否计入相位，used是本秒是否实际更新KF/DAC |
| gapfill_count / hard_reset_count | 模型补值及硬重置累计次数；不等于当前秒发生 |
| nmea_fresh / gga_age_ms | 此REFCHK之前最新NMEA chk的新鲜度与GGA年龄；时间关系保留，不与显示行假定绝对同时 |
| cfuse_used_weight / cfuse_reason | 本更新拍实际融合权重及原因；非更新秒留空 |
| slow_src / slow_candidate_valid | SLOWC来源与候选有效标志；候选有效不等于已参与 |
| slow_span_s / slow_age_s / slow_lag_s | 慢环跨度、末点年龄、拟合中心滞后，单位s |
| continuous_good_segment | 原始快照门控通过且时间/序号/状态连续的段编号；不额外保证GNSS参考质量、热稳定或DAC固定 |

why编码：0=OK，1=SAT，2=TORN，3=DUP，4=GAP，5=NO_MEAS。Tu=10时正常逐秒acc=1，used通常只有每10秒一个，不能要求所有正常秒used=1。

旧日志使用整数与小数分开取整，可能出现cn0=35.10、hdop=0.100。导出保留原显示文本，并按生产者的小数位定义换算为36.0与1.00；不按普通十进制字符串静默读成35.1或0.1。X0/X1沿用毫单位整数字段，避免负零显示问题。

## 旁表与来源

每份日志另有slowdbg、slowc、cfuse、samp、nmea、state_events六个旁表。原字段名称保留，另外增加line_number、context_previous_ref_ms与raw_line。context_previous_ref_ms只是文件流中之前最近一条REFCHK时间，不是新增的板上事件时间；NMEA通常在下一条REFCHK前打印。主表的NMEA质量快照明确取该REFCHK之前最新记录。

SLOWDBG的innov与S已经是本项目打印的ns与ns²；CAND中的同名字段可能带整数缩放，不能不看生产者格式就与SLOWDBG混用。本次主表不使用影子CAND作为物理输出。

## 分析规则

OADEV用完整、状态一致且无拒绝/补值的连续逐秒原始数据；不拼接v19的电脑休眠断档。性能窗固定1200000≤ms<3600000。CALIB的phsm/frqm不能作冻结段输出漂移，必须使用原始增量。

R标签选用已有冻结窗口，不跨状态、DAC和操作边界。r_parameter_reasonableness_labels_20261101.csv含多个诊断集合，当前主检验仅筛run∈{v14,v22,v26}且state=CALIB，共331窗；旧轮和固定DAC的TUNING诊断不并入训练。

## 功能验收集合

v27位于data/csv/acceptance，使用同一字段定义，共1154完整主记录及6类事件表。历史data/csv/current的13轮59616条主记录保持原样。验收记录含故障/模型补值和一次+8ms/-8ms调度偏移，不能当连续等间隔性能样本静默拼接。单份重新导出必须指定--output，避免覆盖历史集合索引。
