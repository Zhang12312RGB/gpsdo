#include "dbg_uart.h"
#include "bsp.h"   /* bsp_idle(): 发送间隙抽干 NMEA 邮箱 */
#include "xparameters.h"
#include "xuartps.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/*-----------------------------------------------------------------------------
 * 配置：只认 UART1（0xE0001000），与串口屏 UART0 完全分开
 *---------------------------------------------------------------------------*/
#ifndef DBG_UART_BASE
#define DBG_UART_BASE   XPS_UART1_BASEADDR
#endif

extern XUartPs_Config XUartPs_ConfigTable[];

static XUartPs dbg_uart;
static int     dbg_ok = 0;

static XUartPs_Config *dbg_lookup_cfg(void)
{
    int i;
    for (i = 0; i < XPAR_XUARTPS_NUM_INSTANCES; i++) {
        if (XUartPs_ConfigTable[i].BaseAddress == DBG_UART_BASE) {
            return &XUartPs_ConfigTable[i];
        }
    }
    return NULL;
}

void dbg_init(void)
{
    XUartPs_Config *cfg;

    dbg_ok = 0;
    cfg = dbg_lookup_cfg();
    if (cfg == NULL) {
        return;
    }
    if (XUartPs_CfgInitialize(&dbg_uart, cfg, cfg->BaseAddress) != XST_SUCCESS) {
        return;
    }
    XUartPs_SetBaudRate(&dbg_uart, DBG_BAUDRATE);
    dbg_ok = 1;
}

int dbg_ready(void)
{
    return dbg_ok;
}

void dbg_putc(char c)
{
    if (!dbg_ok) {
        return;
    }
    XUartPs_SendByte(dbg_uart.Config.BaseAddress, (u8)c);
    bsp_idle();
}

void dbg_puts(const char *s)
{
    if ((!dbg_ok) || (s == NULL)) {
        return;
    }
    while (*s != '\0') {
        XUartPs_SendByte(dbg_uart.Config.BaseAddress, (u8)(*s));
        s++;
        bsp_idle();
    }
}

/* 格式化输出：先 snprintf 到缓冲，再整块发出，避免逐字符 printf */
void dbg_printf(const char *fmt, ...)
{
    char buf[DBG_BUF_SIZE];
    va_list ap;
    int n, i;

    if (!dbg_ok) {
        return;
    }
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }
    if (n > (int)sizeof(buf) - 1) {
        n = (int)sizeof(buf) - 1;
    }
    for (i = 0; i < n; i++) {
        XUartPs_SendByte(dbg_uart.Config.BaseAddress, (u8)buf[i]);
        bsp_idle();
    }
}
