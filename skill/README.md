# GPSDO可复用分析与验证方法

本目录从工程纠错中整理日志解析、单位检查、稳定度计算和结果复现方法。适用于具有原始记录和闭环估计的项目；脚本输入格式目前对应本GPSDO，不声称通用PYNQ Skill。

| 材料 | 用途 |
|---|---|
| [scripts/](scripts/README.md) | 文件校验、DAT转CSV、OADEV、331窗R复算及功能验收 |
| prompt_workflows.md | 组织输入、审查角色和验收条件的协作模板 |
| lessons.md | 单位、调用节拍、序号恢复、格式检查及统计有效性的经验 |

方法要求保留原始数据、明确时间窗和真实调用节拍，再把可复现反例变成检查项。OADEV使用相隔m点的滑动均值差；可靠序号推进与样本接受分别处理；缺失配置字段不默认视为关闭；候选有效不等于实际参与控制。

常规复核在根目录执行verify_manifest.py、check_acceptance.py和reproduce_results.py。脚本功能与依赖见[scripts说明](scripts/README.md)，完整来源及开发过程见[协作历史](../report/ai_collaboration_history.md)。
