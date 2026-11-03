#ifndef SCREEN_H
#define SCREEN_H

#include "xil_types.h"

/*=============================================================================
 * screen.h -- 串口屏显示模块（独占 PS UART0）
 *-----------------------------------------------------------------------------
 * 物理连接（接线图 J12 = PS PMOD）：
 *     J12 PIN9  (C5, MIO14) = UART0 TX  ->  串口屏 RX
 *     J12 PIN3  (C8, MIO15) = UART0 RX  <-  串口屏 TX
 *     J12 PIN5 / PIN11      = GND          <-> 串口屏 GND
 *     J12 PIN6 / PIN12      = +3.3V        <-> 串口屏 VCC
 *
 *     UART0 基地址 0xE0000000；调试口 UART1 在 0xE0001000，
 *     由 dbg_uart.c 单独管理，两个模块互不干扰。
 *
 * 发送内容：经纬度、X[0] 相位差(ns)、X[1] 频差(ppb)、卫星数、HDOP、
 * 显示内容(每帧 14 条 <对象>.txt="值" + FF FF FF):
*   频率差 X[1] (ppb) / 相位差 X[0] (ns) / 卡尔曼采样周期 T (s)
*   经纬度 / 卫星数 / HDOP / 定位质量 / C/N0
*   1PPS 与本地差距(本秒实测相位差, ns) / 此时慢环的 T (s)
*   时间 / 日期 / 状态文字
 *
 * 本模块不覆盖 BSP 的 outbyte/inbyte，只用 XUartPs 驱动自己的实例。
 *===========================================================================*/

/* 输出协议二选一 */
#define SCREEN_PROTO_TEXT   0   /* 一行可读文本 */
#define SCREEN_PROTO_TJC    1   /* 陶晶驰 USART HMI: t0.txt="..." */
/* 默认走陶晶驰(TJC/淘晶驰) USART HMI 指令集:
 *     <对象>.txt="<值>"  然后三个 0xFF 结尾
 * 参照 ltxuartscreen.v 的写法(gpstxt/gpsl/gpsn/gpsj/gpsw/gpsd/gpst 那一套)。
 * 所有【值】一律用 ASCII —— 中文标签由屏工程里的控件自己负责, 我们不发 GBK,
 * 免得到处埋编码坑。 */
#define SCREEN_PROTO        SCREEN_PROTO_TJC

/* 屏对象名: 按你屏工程里的控件名改这里就行 */
#define SCR_O_LAT    "gpsw"    /* 纬度  N/S */
#define SCR_O_LON    "gpsj"    /* 经度  E/W */
#define SCR_O_SAT    "gpsn"    /* 卫星数    */
#define SCR_O_QUAL   "gpsl"    /* 定位质量  */
#define SCR_O_TXT    "gpstxt"  /* 状态文字  */
#define SCR_O_TIME   "gpst"    /* 时间      */
#define SCR_O_DATE   "gpsd"    /* 日期      */
#define SCR_O_FRQ    "gpfrq"   /* 频率差 ppb       */
#define SCR_O_PHS    "gpphs"   /* 相位差 ns        */
#define SCR_O_T      "gpt"     /* 采样周期 T (s)   */
#define SCR_O_HDOP   "gphd"    /* HDOP             */
#define SCR_O_CN0    "gpcn"    /* C/N0             */
#define SCR_O_PPS    "gppps"   /* 1PPS 与本地差距  */
#define SCR_O_SLOW   "gpslw"   /* 慢环 T (s)       */

/* 波特率：必须与串口屏里的设置一致 */
#define SCREEN_BAUDRATE     9600U    /* 和参考工程 ltxuartscreen.v 一致 */

void screen_init(void);                       /* 初始化 UART0 */
int  screen_ready_flag(void);                 /* 1 = UART0 可用 */
void screen_send_status(void);                /* 发一帧完整状态（每秒调一次） */
void screen_send_str(const char *s);          /* 发一段原始字符串 */

/*-----------------------------------------------------------------------------
 * 分页
 *   1.8 寸 160x128 的屏放不下全部 14 个控件，分两页：
 *     页 0 = 控制状态（状态/频率差/相位差/T/1PPS差/卫星/HDOP/时间）
 *     页 1 = GNSS 信息（定位质量/卫星/HDOP/C-N0/纬度/经度/时间/日期）
 *   切页只发 TJC 的 page 指令；控件值跨页保留，不需要重绘。
 *---------------------------------------------------------------------------*/
#define SCREEN_PAGE_CNT     2
#define SCR_PAGE_CTRL       0    /* 页 0: 控制状态 */
#define SCR_PAGE_GNSS       1    /* 页 1: GNSS 信息 */

int         screen_page_get(void);
int         screen_page_next(void);   /* 切到下一页，下发 page 指令，返回新页号 */
const char *screen_page_name(void);   /* "CTRL" / "GNSS" */

/* --- 以下为可选的"屏幕回传命令"功能，不用可以整段删掉 --- */

int  screen_get_cmd(char *out, int maxlen);   /* 取一条完整命令，无则返回 0 */
void screen_poll_rx(void);

#endif /* SCREEN_H */
