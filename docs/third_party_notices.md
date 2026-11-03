# 第三方依赖与许可说明

项目原有 LICENSE 采用 MIT，保留原版权声明。本次整理不改变第三方代码或二进制的许可。

build/bsp 中的头文件与静态库来自 AMD Vitis 对所附 XSA 生成的 standalone Cortex-A9 BSP。源文件中的 AMD/Xilinx 版权及 SPDX 声明保留；重建平台需合法安装相应 AMD 工具。项目 MIT 不将 AMD IP、工具或其第三方依赖重新许可。

src/bd 的 Tcl 为 Vivado 导出设计；其底层 PS7/AXI/复位 IP 由本机 Vivado 安装提供，不把工具安装文件打入提交包。所附 bit/XSA 来自本项目既有实现。

附带的竞赛 PDF 用于本地要求核对，不将其作为本项目代码许可证覆盖的内容；提交包不复制整份指南。

开发仓库本地配置的地址是 https://github.com/Zhang12312RGB/gpsdo ，本次没有联网检查其可访问性、版本或许可同步状态，也没有向该仓库推送。
