# 提交源码

本目录保存比赛提交的PS/PL源码和硬件接口描述。当前配置为Q_FREQ=1e-4、R80、固定Tu10、相位项关闭、MODE2。配置含义见[最终配置](../docs/final_configuration.md)。

| 子目录 | 文件及职责 |
|---|---|
| sw/ | 29个文件，包括C源码、头文件和lscript.ld；裸机PS程序 |
| rtl/ | 10个Verilog文件；测量、结果发布、AXI、NMEA及DAC接口 |
| constrs/ | gnss_gpsdo_ax7010.xdc；板级管脚与时序约束 |
| bd/ | gnss_bd.tcl及gnss_bd.bd；PS7、AXI和模块连接 |

PS主入口为sw/main.c。bsp管理快照/样本判定，gnss解析报文，kalman估计累计相位与频差，slow_loop处理C补偿慢环与权重，control执行DAC控制；screen、ps_key、led_ctrl负责人机交互。cand_obs为只记录的候选观察器。adaptive_r模块保留，当前运行固定R80且训练关闭。

PL顶层为rtl/gnss_gpsdo_top.v。counter_measure、tdc_carry、ocxo_frac及phase_error组成粗细测量路径；gnss_axi_ip_slave发布寄存器；uart_rx/nmea_mailbox接收报文；spi_dac驱动DAC。

OCXO接K17作被计数输入，测量域为独立标称100 MHz FCLK。DAC SCK/DIN/SYNC分别为G20/H18/L20，具体J11脚号见[硬件接口](../docs/hardware_interfaces.md)。

重建使用[build目录](../build/README.md)中的脚本。修改源码后应重新编译并记录BUILD、宏配置和ELF哈希；不能以源码文件名判断板上版本。当前源码快照不单独证明板上bit逐字节身份。
