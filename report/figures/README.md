# 报告与演示图表

| 文件系列 | 内容 |
|---|---|
| report_overview、report_update_flow、report_c_compensation | 系统组成、逐秒任务及C补偿流程 |
| report_startup、report_oadev | 正文启动和原始通道OADEV |
| submission_config_v24_x1_time | 同正常控制配置v24的X[1]随开机时间变化 |
| submission_config_v24_oadev、slow_loop_oadev | v24及三轮关开关稳定度 |
| summary_fig系列 | 技术总结插图快照；其中旧文件编号不一定等于当前正文图号 |
| r_cross_day_v26、r_cross_validation | R模型跨天/留出验证 |
| report_plot_provenance.json | 曲线源数据、哈希与取窗范围 |

当前稳定度窗为1200000≤ms<3600000，各2400点，使用原始REFCHK通道；图上X[1]是控制估计值，不能替代原始输出稳定度。v24 BUILD为历史17:57:44，和最终版采用相同正常控制配置；v27功能日志不是这些长期曲线的来源。

PNG用于展示，SVG用于矢量编辑。原始数据及复算入口见[数据说明](../../data/README.md)，现行正文编号以design_report为准。
