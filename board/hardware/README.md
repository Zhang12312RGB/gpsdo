# 配套PL硬件快照

| 文件 | 用途 |
|---|---|
| gnss_bd_wrapper.bit | 随包可下载的PL配置文件 |
| gnss_bd_wrapper.xsa | Vitis硬件平台描述，包含同版本bit |
| ps7_init.tcl | 从该XSA提取的PS初始化脚本 |

已核对独立bit与XSA内嵌bit一致。此目录保留已构建硬件快照，PL历史报告见[build/reports](../../build/reports/README.md)。不把后续PS重编译视为PL重实现。

新硬件可由build/create_project.tcl与run_impl.tcl生成。地址映射、PS配置或时钟改变后应重新生成配套BSP。下载ELF时避免工具自动装入其它平台副本中的旧bit；完整步骤见[复现指南](../../docs/reproduction.md)。
