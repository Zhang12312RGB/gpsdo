# 十张详图的图片与编辑源

fig01～fig10依次为硬件接线、PL/PS架构、PS状态机、采集控制流程、KF与DAC闭环、验证结构、PL发布状态、C慢环协作、屏幕两页和TDC配对时序。

每图提供PNG（展示）、SVG（矢量）、drawio（编辑）及wps.json（对应绘图数据）。PDF位于docs目录，汇总为[detailed_flowcharts.pdf](../detailed_flowcharts.pdf)；逐图入口与源码依据见[diagram_index.md](../diagram_index.md)。

详图编号与主报告图号独立。改图时需同步核对控制参数、真实接口和生效代码，不把概念箭头当作新增硬件验收。page0为控制状态，page1为GNSS；SPI SCK/DIN/SYNC为G20/H18/L20。
