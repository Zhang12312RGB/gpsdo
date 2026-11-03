# 已有验证结果

| 文件或系列 | 证据范围 |
|---|---|
| hardware_acceptance_20261102.json | v27原始功能日志核算；默认BUILD Nov2 15:29:18 |
| document_checks.json | 当时报告排版、图片及材料检查，后续手工编辑可能改变文件哈希 |
| host_results_20261102、host_retest | 主机功能测试结果 |
| ps_build_20261102、ps_mode0/2_build日志 | 可移植PS构建记录 |
| submission_restore_20261102 | 本机TRY恢复提交配置的历史过程 |
| binary_checks、verification、build_final/baseline等 | 生成时对应的编译与二进制检查，不自动代表最新文件身份 |
| project_recreate.log | 工程重建/BD验证过程 |

这些材料按生成时点保留。旧JSON或日志中的ELF哈希可属于旧冻结版本；当前文件以根manifest.json和docs/version_manifest.md为准。README更新只刷新交付清单，不改写历史验证事实。

v27包含参考失效及模型补值，用于功能验收，不是长期OADEV来源。重新复核可在根目录运行`python skill/scripts/check_acceptance.py`。
