# 历史RTL测试平台

tb_gnss_gpsdo_top.v、tb_hw_elab.v、tb_tdc_carry.v、tb_tdc_recip.v保留顶层、展开和TDC相关测试思路。它们可能使用早期端口、参数或判据，运行前需与当前src/rtl逐项核对。

提交包未将这些文件统一标记为当前版本全套仿真通过。现有确定验证范围见[sim/results](../results/README.md)；Vivado工程重建见[build说明](../../build/README.md)。
