# 技术总结报告与分析材料

| 入口 | 用途 |
|---|---|
| [design_report.docx](design_report.docx) | 可编辑技术总结，已补整机外形尺寸 |
| [design_report.md](design_report.md) | 可检索文本和图片引用 |
| [证据与测量范围](../docs/evidence_map.md) | 实测来源、独立准确度与验证范围说明 |
| [figures/](figures/README.md) | 正文及演示曲线、图片与来源说明 |
| [analyses/](analyses/README.md) | R模型专项分析 |
| [devlog/](devlog/README.md)、[history/](history/README.md) | 开发过程、早期判断与纠错历史 |
| ai_collaboration_history.md / ai_collaboration_update.md | 大模型协作过程及落地说明 |

报告按作品概述、系统组成、完成情况与性能、总结、参考文献五部分展开，包含15幅嵌入图片、13个正文图号。新PCB近景用于装配与器件说明，J11/J12及5 V供电按已确认信息填写。未提供的参考电压、阻容参数和原理图待补项已略去。

长期稳定度取相同正常控制配置v24的20～60分钟原始记录；最终默认ELF BUILD为2026-11-02 15:29:18，其功能验收由v27单列支持。慢环参与已验证，单组长期改善信号不写成确定、普适收益。

编辑Word后应重新导出PDF并同步文本；本次尺寸更新同步Word、PDF与文本，整机外形为245×110×58 mm，高度按最高点计。正式指标和数据入口见[证据表](../docs/evidence_map.md)，校验见根manifest.json。

PDF 版与中英文海报按赛事要求单独提交，不随本工程包分发。
