# 从提交目录复现当前版本

## 环境与入口

安装支持 xc7z010clg400-1 的 Vivado 2026.1 与 Vitis 2026.1，准备 Python 3.11 或更新版本。路径可放在任意本地英文目录。命令均在 gpsdo 根目录执行；工具路径使用本机安装值。

先运行 `python skill/scripts/verify_manifest.py`，核对文件 SHA256。它校验交付快照，不校验板上实际程序。仿真/重建产物放到 generated 或 rebuilt 目录，不能覆盖 board/ 中冻结的提交文件。

## PL 工程重建

```text
vivado -mode batch -source build/create_project.tcl
vivado -mode batch -source build/run_impl.tcl
```

第一条从源码与 Vivado 原生导出的 BD Tcl 创建新工程，先加全部 RTL，再恢复 PS7、AXI、复位网络和自定义模块引用，验证 BD 后生成输出及 wrapper。默认项目位置 build/generated/gpsdo.xpr；已有工程时脚本拒绝直接覆盖，重新构建可将 GPSDO_BUILD_DIR 环境变量设为新的空目录。

第二条进行综合、实现并导出新 bit/XSA 和资源/时序报告，不连接下载线。新生成的 XSA 要重新生成平台和 BSP；不能把不同地址映射/PS 配置的 BSP 与它混用。

随包硬件文件是 2026-10-28 已构建版本。board/hardware/gnss_bd_wrapper.bit 与同目录 XSA 内嵌 bit 一致。现有报告资源为 LUT 2363/17600、寄存器 2726/35200、WNS 0.592 ns、WHS 0.037 ns。保留外设延迟约束告警，未宣称此次整理重新通过完整实现。

## PS 编译

可直接对已验证硬件快照执行：

```text
python build/build_ps.py --gcc <Vitis目录>/gnu/aarch32/nt/gcc-arm-none-eabi/bin/arm-none-eabi-gcc.exe --mode 2
```

脚本使用 src/sw 全部 C 文件、原链接脚本、Cortex-A9 hard-float 参数和 build/bsp/include、lib 中的板级支持库，输出 build/rebuilt/mode2/gpsdo.elf。`--mode 0` 编译回退比较版。重新编译时间不同会改变 BUILD 与 ELF 哈希，不要求与冻结 ELF 字节相同。提交的冻结固件不被覆盖。

重新从 XSA 建立 Vitis 平台时，选 standalone、ps7_cortexa9_0；导入 src/sw 中 .c/.h 与 lscript.ld，按原平台包含 xilstandalone、xiltimer、xilskey、xil 库。此项目使用裸机 C，不要求迁移为 PYNQ。

## 主机功能测试

```text
python sim/run_host_tests.py --gcc <主机GCC>
```

覆盖守时按秒推进、协方差增长、DAC 输入仅补一次、累计相位恢复、坏样本恢复延迟、返回 CALIB 保持硬锁、日志进位、慢环权重渐增与退出等。此测试不需要 FPGA。sim/rtl 为历史测试平台，接口和判据需要按当前 RTL 核对，不把它们当新一轮全通过证据。

## 上板操作

1. 接好既有 AX7010、GNSS、20 MHz OCXO、DAC8531 和调试串口，沿用已验证供电与接口电平。PL 引脚以 src/constrs 的实际 PACKAGE_PIN 设置为准。
2. 确认使用同一组 board/hardware bit/XSA 与对应平台。用 Vivado Hardware Manager 下载该 bit；Vitis 下载 board/firmware/gpsdo_final_c.elf 时，避免平台副本自动重新覆盖为别的旧 bit。
3. 调试串口使用 115200 8N1，保存未包装的原始文本。SSCOM 原始 DAT 可直接供分析；带时间戳 CSV 的每行封装需先还原，不能直接混用。
4. 核对 BUILD Nov  2 2026 15:29:18 和以下配置。

```text
KFCONF profile=1 QF_e9=100000 p3floor=0 fixed_period=1 phsg=0 slow_mode=2
FINALCONF release=FINAL_C_R80_T10_20261031 R_ns2=80 R_train=0 period_fixed=1 phase=0 slow=2
```

5. GNSS 质量持续有效后，先做短时功能验收。CALIB 应冻结 DAC；HOLDOVER 恢复回失锁前状态。拔天线不保证接收机立即停止 PPS，要同时看 GGA 新鲜度和系统状态。
6. 单独另开稳定度文件，至少采 60 min，前 20 min 作启动/热稳定观察，统一分析后 40 min。禁用电脑自动休眠；熄屏本身不等于测量停止。
7. 坏样本及模型补值不能算作真实输出性能；序号/时间断档必须切段。不能只看 X[1] 更平滑就宣布晶振变稳。

## 结果复算

```text
python skill/scripts/analyze_stability.py data/raw_support/log_20261031_v24.dat --start-ms 1200000 --end-ms 3600000 --output data/results/v24_recomputed.json
```

对 v23、v25 使用同样区间。算法先计算 m 点滑动均值，再取相隔 m 点均值之差；相邻滑动均值直接作差不是正确 OADEV。1 s 相位增量的数值 ns 与平均频差 ppb 在数值上相同；1 ppb 等于 20 MHz 下 20 mHz、1 ns/s 和 1e-7%。600 s 结果仅作参考。


## 本次新增的原始日志到CSV复现

在解压后的gpsdo目录运行：

```text
python skill/scripts/verify_manifest.py
python skill/scripts/export_raw_csv.py --all-recent
python skill/scripts/reproduce_results.py
```

第一条校验交付文件；第二条从v14～v26共13份DAT重新生成data/csv/current中的主表、SLOWDBG、SLOWC、CFUSE、SAMP、NMEA校验和状态事件表；第三条检查原始与CSV哈希、连续性，并重算三轮2400点OADEV及融合参与率。缺失字段留空，两个尾部不完整记录仍导出并标记，不补造成实测。电脑休眠导致的时间断档保留，不能跨段做OADEV。

R与绘图的可选复现使用NumPy和Matplotlib，本机验证版本列于requirements-analysis.txt。需要时先在独立Python环境安装依赖，再运行：

```text
python -m pip install -r requirements-analysis.txt
python skill/scripts/reproduce_results.py --r --plots
```

输出为data/reproduced，保留data/results里的冻结核算快照。R入口从原始导出的逐秒CSV重建331个冻结40秒窗口，检查局部残差标签与质量特征，再重算整轮留出的常数/四参数比较。不会随机打散相邻窗口，也不会把新增尾部数据自动吸收为旧训练集。

单份新的功能验收DAT请先放入data/raw_support，例如：

```text
python skill/scripts/export_raw_csv.py data/raw_support/log_20261102_v27.dat --output data/csv/acceptance
python skill/scripts/check_acceptance.py
```

新的验收记录不自动属于v14～v26历史性能样本。详细板上操作与通过条件见board_acceptance.md；功能故障测试完成后另开性能记录。

## 11月2日本机恢复说明

TRY/src的两个实验宏已恢复提交值，29个提交源文件与src/sw逐字节一致，工作区RTL与10个提交RTL文件一致。IDE默认TRY.elf与冻结gpsdo_final_c.elf一致。R标定配置及原ELF另备份，不混入提交快照。当前默认ELF已改用v27实际验收对应的15:29:18重编译文件，旧默认保留历史备份。普通重建产物仍应另存，不自动代替已测文件。

本包同时保留标准build/build_ps.py入口：可在任意解压路径重建。Vitis工程原构建规则未改；作者机器原生size后处理的问题只在临时构建包装中绕过，不能将该机器包装文件当作跨机器必需依赖。

功能CSV另放data/csv/acceptance。check_acceptance.py检查源哈希、配置、DAC冻结、状态回返与实际融合。功能故障文件不进入旧13轮CSV索引或331窗R训练集合。新增照片和物理排针复现见hardware_interfaces.md。
