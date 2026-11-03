# 上板文件与实物材料

| 子目录 | 内容 |
|---|---|
| [firmware/](firmware/README.md) | 默认MODE2及MODE0回退ELF |
| [hardware/](hardware/README.md) | 配套bit、XSA与PS初始化脚本 |
| [photos/](photos/README.md) | 整机、PCB、串口屏及真实日志照片 |

默认gpsdo_final_c.elf的BUILD为2026-11-02 15:29:18，与v27功能日志对应。测试覆盖CALIB冻结、失锁后回CALIB及C慢环实际融合；范围和限制见[本次验收](../docs/hardware_acceptance_20261102.md)。v24用于相同正常控制配置的历史性能核算。

AX7010和底板输入均为5 V。PL信号主要接J11，串口屏使用J12的PS UART0。下载前按[硬件接口表](../docs/hardware_interfaces.md)核对，操作见[上板指南](../docs/board_acceptance.md)。bit/XSA、ELF及平台必须配套，不能仅凭TRY文件名判断版本。
