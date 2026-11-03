#ifndef PS_KEY_H
#define PS_KEY_H

#include "xil_types.h"

//=============================================================================
// ps_key.h -- PS 端用户按键模块（不依赖 PL）
//-----------------------------------------------------------------------------
// 硬件（ALINX AX7010 用户手册 V1.07 §6.9，BANK501）：
//     PS KEY1 = PS_MIO50_501 = B13
//     PS KEY2 = PS_MIO51_501 = B9
//     按下 = 低电平, 未按下 = 高电平（低有效）
//
// 为什么用 PS 按键而不用 PL 按键：
//     PL 顶层的 gnss_gpsdo_top.v 里根本没有按键输入端口，
//     AXI 从机的 0x0C 实际是 TDC_TAP_PERIOD，不是 KEY。
//     直接用 PS 的 MIO50/MIO51，不需要 A 改任何 RTL。
//
// 去抖与边沿检测全在软件里做，时间基准取自 bsp_millis()（SCU 硬件定时器），
// 与主循环速度无关。
//=============================================================================

#define PS_KEY_NUM          2

// 按键下标
#define PS_KEY_IDX_1        0
#define PS_KEY_IDX_2        1

// 事件码（ps_key_get_event 的返回值）
#define PS_KEY_NONE         0
#define PS_KEY_EVENT_1      1   // KEY1 按下
#define PS_KEY_EVENT_2      2   // KEY2 按下

// 默认动作（在 main.c 的 handle_ps_key 里使用，可自行改）
//   KEY1: TUNING <-> CALIB 手动切换（守时状态下不响应，保持守时）
//   KEY2: 清空自适应 R 采集点, 并把判定窗口归零后强制回 TUNING

#define PS_KEY_DEBOUNCE_MS  20U    // 去抖时间
#define PS_KEY_REPEAT_MS    0U     // 长按重复间隔, 0 = 不重复

void     ps_key_init(void);        // 初始化 GPIOMIO50/51 为输入
int      ps_key_ready(void);       // 1 = 按键可用
int      ps_key_raw(int idx);      // 原始电平: 1 = 按下
int      ps_key_get_event(void);   // 取一个按键事件, 无则 PS_KEY_NONE

void     ps_key_poll(void);        // 主循环高频调用: 去抖 + 生成事件
int      ps_key_level(int idx);    // 去抖后的电平: 1 = 按下

#endif // PS_KEY_H
