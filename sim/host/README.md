# 主机测试范围

| 文件 | 检查内容 |
|---|---|
| test_holdover.c | 秒级预测、协方差推进、DAC补偿及恢复路径相关逻辑 |
| test_log.c | 日志格式和小数进位 |
| test_weight.c | 受限C权重、渐增与退出 |

使用sim/run_host_tests.py统一编译运行，包含-Wall、-Wextra、-Wformat=2和-Werror。脚本从slow_loop.c抽取标记内的权重核心，部分测试采用冻结函数摘录及提交头文件，覆盖范围不等同于完整板上程序执行。

输出存sim/generated，不覆盖已有sim/results。运行方法见[验证入口](../README.md)。
