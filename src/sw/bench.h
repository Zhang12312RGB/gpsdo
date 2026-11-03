#ifndef BENCH_H
#define BENCH_H

#include <stdint.h>

//=============================================================================
// 台架模式(无外设自测)
//-----------------------------------------------------------------------------
// 目的: 不接 GNSS 模块、没有 1PPS、没有 TDC 的情况下, 也能把整条控制逻辑跑起来
//       并在 UART1 上看到每一步的数值, 用来验证代码而不是验证硬件。
//
// 做法: 把 PL 提供的那几个量在软件里造出来 ——
//       0x00 PHASE_ERR  : 一个一阶被控对象(OCXO 固有偏差 + DAC 修正 + 噪声)
//       0x2C meas_valid : 恒 1
//       0x44..0x58 第二路: 按 T 秒窗口产生相位差增量样本, 并置 NEW
//       GNSS 状态       : fix=有效, sat=9, hdop=0.90, cn0=41.0
//       (全部在 bsp.c 里按 BENCH_ENABLE 分流, main.c 一行都不用改)
//
// 被控对象模型:
//   实际频率偏差 = BENCH_BIAS_PPB + (dac - DAC_CENTER) * BENCH_DAC_GAIN
//   相位         = 频率偏差 * 1s 的累加  (1ppb = 1ns/s, 正好 1:1)
//
// 注意 BENCH_DAC_GAIN (重要)
//   control.c 的 DAC 律是【绝对式】: dac = 2048 - X1/DAC_SCALE, 不是积分式。
//   代入被控对象后闭环固定点是:
//       X1 = bias / (1 + BENCH_DAC_GAIN/DAC_SCALE)
//   也就是残留误差只取决于 台架模型 DAC 灵敏度 与 DAC_SCALE 的比值:
//       比值 1    -> 残留 bias/2     (只压掉一半)
//       比值 10   -> 残留 bias/11    (刚够进 X1<10ppb 的交接门限)
//       比值 1000 -> 残留 bias/1001  (能一路走到 T>1000s 的 done)
//   这里取 BENCH_DAC_GAIN = 50.0 (= 1000 x DAC_SCALE) -> 残留约 0.10ppb,
//   对应 T = 31.62 * 0.1^-1.5 = 1000s, 所以【调整完成】这条路也能走到。
//   ! 这不是真实器件的标定值。真实硬件的 DAC_SCALE 必须自己测:
//     给 DAC 一个固定增量, 量 OCXO 频率变化, 才能得到真实的 ppb/code。
//=============================================================================

#define BENCH_ENABLE        0        // 0 = 真实硬件(比赛/上板); 1 = 台架自测

#define BENCH_BIAS_PPB      100.0    // 模拟 OCXO 的固有频率偏差
#define BENCH_DAC_GAIN      0.050    // 台架 DAC 灵敏度(ppb/code) = DAC_SCALE
#define BENCH_NOISE_NS      2.0      // 相位测量噪声幅度(+-ns)
#define BENCH_STEP_AT_S     180U     // 第几秒注入一次相位阶跃(去触发保护)
#define BENCH_STEP_NS       300.0    // 阶跃幅度(ns)
#define BENCH_OCXO_HZ       20000000.0

/* 台架替身的 GNSS 状态。注意: 不再去改 gnss 结构体, 而是在 main.c 的消费点
 * 用 gnss_fix()/gnss_sat()/... 分流 —— 这样 gnss 里【永远】是真实模块解析
 * 出来的数据, 打印时不会出现'上一秒的替身冒充真实值'。 */
#define BENCH_FIX           1
#define BENCH_SAT           9
#define BENCH_HDOP          0.90
#define BENCH_CN0           41.0

void     bench_init(void);           // 复位模型
void     bench_tick(void);           // 每秒推进一次模型
void     bench_gnss_override(void);  // 把 GNSS 状态改成'有定位'

double   bench_phase_ns(void);       // 相当于 0x00 的测量值(含噪声)
int      bench_meas_valid(void);     // 恒 1

int      bench_p2_take_new(void);    // 相当于 0x50 bit0 + 写 1 清
int64_t  bench_p2_phase(void);       // 相当于 0x48/0x4C 拼接
int32_t  bench_p2_ocxo(void);        // 相当于 0x58
void     bench_p2_set_t(uint16_t t); // 相当于 0x44
uint16_t bench_p2_get_t(void);
uint16_t bench_p2_count(void);       // 相当于 0x54

double   bench_true_ppb(void);       // 真值: 当前实际频率偏差(ppb), 只用于打印
double   bench_true_phase_ns(void);  // 真值: 当前实际相位(ns)


#endif