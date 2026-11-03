# 功能验证与历史仿真

| 子目录 | 用途 |
|---|---|
| [host/](host/README.md) | 守时恢复、日志和慢环权重的主机测试 |
| [results/](results/README.md) | 已有编译、主机验证、二进制核对及v27功能核算 |
| [rtl/](rtl/README.md) | 历史RTL测试平台，接口需按当前设计复核 |

在提交根目录运行：

```text
python sim/run_host_tests.py --gcc "<本机GCC可执行文件>"
```

此处需要主机GCC，与build/build_ps.py使用的ARM交叉编译器区分。脚本生成sim/generated，测试不下载板卡。

主机测试验证特定逻辑，不能代替板上时序或真实外设。最终固件v27已覆盖CALIB保持、失锁回CALIB和C实际参与；范围见[功能验收](../docs/hardware_acceptance_20261102.md)。历史RTL文件保留，不声明其全部适配当前寄存器并重新通过。
