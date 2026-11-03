# 随包BSP快照

include/为板级支持头文件，lib/包含libxilstandalone、libxiltimer、libxilskey和libxil静态库，xilinx.spec供ARM GCC链接使用。它们与随包硬件快照配套，用于[build_ps.py](../build_ps.py)。

该目录是依赖快照，不是本项目自行实现的驱动源码全集。重新导出不同XSA后应在Vitis中重新生成平台和BSP。第三方来源与许可见[说明](../../docs/third_party_notices.md)。
