# CSV集合入口

| 集合 | 来源 | 用途 |
|---|---|---|
| [current/](current/README.md) | v14～v26，共13轮 | 历史性能、观察器及R研究复现 |
| [acceptance/](acceptance/README.md) | v27，11月2日 | 最终配置的板上功能验收 |

两个集合分别保存csv_index.csv与csv_manifest.json，不能混用索引。CSV是由原始DAT解析的派生材料，原始来源及SHA见各集合清单；完整字段解释见[CSV字典](../../docs/csv_schema.md)。

缺失字段留空，不补为0。模型补值、坏样本及时间断档不能直接拼成连续性能样本。所有命令从提交根目录执行。
