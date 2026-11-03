# 提交固件

| 文件 | 用途 | BUILD |
|---|---|---|
| gpsdo_final_c.elf | 默认提交及演示，MODE2受限C慢环参与 | Nov 2 2026 15:29:18 |
| gpsdo_baseline_mode0.elf | MODE0慢环关闭的回退/对照 | Oct 31 2026 20:23:43 |

两版均采用Q_FREQ=1e-4、R80、固定稳态Tu10、关闭相位回零。默认ELF的SHA256为：

```text
29717ca20361051e5f3f7cf1d936760821ac88e9ee63a9da41a89c0d7fc2b7e9
```

启动应出现profile=1、QF_e9=100000、p3floor=0、slow_mode=2、R_ns2=80、R_train=0及phsg=0。逻辑release标签20261031保持不变。文件SHA只能核对电脑上的归档文件，串口启动配置不等同于板上内存逐字节读回。

使用同层hardware目录的配套硬件。新编译文件保存至build/rebuilt并另记BUILD及哈希；操作和验收见[上板步骤](../../docs/board_acceptance.md)。
