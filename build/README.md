# 构建与板级支持

此目录提供从提交源码重建工程的入口。推荐Vivado/Vitis 2026.1，器件xc7z010clg400-1。以下命令在提交根目录执行。

| 文件或目录 | 用途 |
|---|---|
| create_project.tcl | 从RTL、XDC和原生BD Tcl建立Vivado工程并验证BD |
| run_impl.tcl | 运行综合与实现，输出新bit/XSA和报告 |
| build_ps.py | 用ARM GCC和随包BSP编译PS程序，支持MODE2/MODE0 |
| bsp/ | 当前硬件快照配套的头文件、规格文件和静态库 |
| reports/ | 2026-10-28保留的PL实现报告 |

## PL重建

```text
vivado -mode batch -source build/create_project.tcl
vivado -mode batch -source build/run_impl.tcl
```

默认工程位于build/generated/gpsdo.xpr。已有工程时创建脚本拒绝覆盖；可将环境变量GPSDO_BUILD_DIR设为新的输出目录。实现结果存于该工程目录及其reports子目录。两条脚本均不连接下载线、不下载硬件。

## PS重建

```text
python build/build_ps.py --gcc "<Vitis安装目录>/gnu/aarch32/nt/gcc-arm-none-eabi/bin/arm-none-eabi-gcc.exe" --mode 2
python build/build_ps.py --gcc "<Vitis安装目录>/gnu/aarch32/nt/gcc-arm-none-eabi/bin/arm-none-eabi-gcc.exe" --mode 0
```

输出分别为build/rebuilt/mode2/gpsdo.elf与mode0/gpsdo.elf，另保存build.log和commands.json。可用--output指定独立输出目录。脚本不会覆盖board/firmware中的已交付ELF。

使用新XSA时，需重新生成对应Vitis standalone平台和BSP；不混用不同硬件地址映射或PS配置。重新编译时间不同会改变BUILD和哈希。上板文件及已有验证范围见[board说明](../board/README.md)，完整步骤见[复现指南](../docs/reproduction.md)。
