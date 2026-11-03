#ifndef DBG_UART_H
#define DBG_UART_H

#include "xil_types.h"

/*=============================================================================
 * dbg_uart.h -- 调试串口模块（独占 PS UART1）
 *-----------------------------------------------------------------------------
 * 物理连接：UART1 = MIO48(B12) / MIO49(C12) -> TXS0102 -> CP2102GM -> Micro USB
 *           基地址 0xE0001000
 *
 * 与串口屏（UART0 / 0xE0000000，见 screen.c）完全独立，两个模块各驱动
 * 自己的 UART 实例，互不影响。
 *
 * 本模块不覆盖 BSP 的 outbyte/inbyte：
 *   - 若 BSP 的 stdout 已指向 ps7_uart_1，xil_printf 可直接用；
 *   - 若没配（xparameters.h 里没有 STDOUT_BASEADDRESS），就用本模块的
 *     dbg_printf()，效果一样，且不依赖 BSP 设置。
 *===========================================================================*/

#define DBG_BAUDRATE   115200U
#define DBG_BUF_SIZE   300   /* 中文诊断行可能到 150+ 字节 */

void dbg_init(void);                        /* 初始化 UART1 */
int  dbg_ready(void);                       /* 1 = UART1 可用 */
void dbg_putc(char c);                      /* 发一个字符(原始流用) */
void dbg_puts(const char *s);               /* 发一段字符串 */
/* 2026-10-28：加上 GCC 格式检查属性 —— 自研 printf 家族默认不受 -Wformat
 * 保护，之前 TUNING 日志漏了 4 个参数也编译通过，运行时打出垃圾值。
 * 加这一行后，格式符与参数数量/类型不符会在【编译期】报错。 */
void dbg_printf(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));      /* 格式化输出，替代 xil_printf */

#endif /* DBG_UART_H */
