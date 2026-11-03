> 历史快照：本文保留10月31日身份与验证状态；当前默认ELF和本次上板结果见version_manifest.md及hardware_acceptance_20261102.md。

# 当前版本清单

# 最终提交配置：受限C慢环参与的20MHz GPSDO

冻结日期：2026-10-31。代码默认已更新到PS工程，完整源快照在src/sw/；PL源码快照在src/rtl/。本次没有烧写板卡，没有重新综合/实现PL，也没有改变硬件供电。板上当前程序不能因为电脑端文件已更新就自动视为新版。

## 最终运行文件

- **提交/演示默认**：gpsdo_final_c.elf，BUILD Oct 31 2026 20:23:42，SHA256 `3726d41112b29562ab8f13a1e216cd42b704c5e31b5bba1594b52e2c8d5d1a95`。
- **C关回退/复测**：gpsdo_baseline_mode0.elf，BUILD Oct 31 2026 20:23:43，SHA256 `074d013e94baa5ef48c886e7bed10a4c0e72465e443af7d8b05b133d1848e593`。
- 当前Vitis的`D:/FPGAdesigner/vitis/test/test/TRY/build/TRY.elf`与默认提交ELF逐字节一致，重新运行工程不会再默认选到MODE0。
- 两版均profile1、R80、Tu稳态10秒、相位控制关闭，区别为MODE2与MODE0。首次启动仍有1秒初始化更新，之后固定10秒。
- 原程序与源码备份在before/；此目录中的原程序是本次冻结前的MODE0，不要与新MODE0回退文件混淆。

启动核对：

```text
KFCONF profile=1 QF_e9=100000 p3floor=0 fixed_period=1 phsg=0 slow_mode=2
CFCONF scale_u=100000 cap_u=100000 rise_s=300 age_max_s=330 lag_max_s=630
FINALCONF release=FINAL_C_R80_T10_20261031 R_ns2=80 R_train=0 period_fixed=1 phase=0 slow=2
```



最终配置详见 final_configuration.md；本目录所有图已更新到 20 MHz、DAC8531、固定 R/Tu 与受限 C 慢环。旧文档与原始实验结论在开发目录 history 保存。
