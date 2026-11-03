#include "screen.h"
#include "main.h"        /* sys_state, sample_period, STATE_* */
#include "gnss.h"        /* 经纬度 / 卫星数 / HDOP / C_N0 / fix_valid */
#include "kalman.h"      /* X[0] 相位差 ns, X[1] 频差 ppb */
#include "bsp.h"       /* bsp_meas_valid / read_phase_error_ns */
#include "control.h"     /* control_get_last_dac() */
#include "slow_loop.h"   /* slow.T 慢环计数周期 */

#include "xparameters.h"
#include "xuartps.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/*-----------------------------------------------------------------------------
 * 配置：只认 UART0（0xE0000000），与调试口 UART1 完全分开
 *---------------------------------------------------------------------------*/
#ifndef SCREEN_UART_BASE
#define SCREEN_UART_BASE   XPS_UART0_BASEADDR
#endif
#define SCREEN_TX_MAX      256

/* 驱动生成的配置表（xuartps_g.c），按基地址找实例，不依赖 DEVICE_ID 宏 */
extern XUartPs_Config XUartPs_ConfigTable[];

static XUartPs screen_uart;
static int     screen_ready = 0;

static XUartPs_Config *screen_lookup_cfg(void)
{
    int i;
    for (i = 0; i < XPAR_XUARTPS_NUM_INSTANCES; i++) {
        if (XUartPs_ConfigTable[i].BaseAddress == SCREEN_UART_BASE) {
            return &XUartPs_ConfigTable[i];
        }
    }
    return NULL;
}

/*-----------------------------------------------------------------------------
 * 初始化：8N1，波特率见 screen.h。找不到 UART0 就静默降级，不影响其它功能。
 *---------------------------------------------------------------------------*/
void screen_init(void)
{
    XUartPs_Config *cfg;

    screen_ready = 0;
    cfg = screen_lookup_cfg();
    if (cfg == NULL) {
        return;
    }
    if (XUartPs_CfgInitialize(&screen_uart, cfg, cfg->BaseAddress) != XST_SUCCESS) {
        return;
    }
    XUartPs_SetBaudRate(&screen_uart, SCREEN_BAUDRATE);
    screen_ready = 1;
}

int screen_ready_flag(void)
{
    return screen_ready;
}

/*-----------------------------------------------------------------------------
 * 内部：逐字节发送（阻塞等发送 FIFO；一帧一百多字节，115200 下约 11ms）
 *---------------------------------------------------------------------------*/
static void screen_put(const char *s)
{
    if ((!screen_ready) || (s == NULL)) {
        return;
    }
    while (*s != '\0') {
        XUartPs_SendByte(screen_uart.Config.BaseAddress, (u8)(*s));
        s++;
        /* 发屏是阻塞的(9600 下一帧约 0.3 s), 这里每个字节间隙抽干一次 NMEA 邮箱 */
        bsp_idle();
    }
}

void screen_send_str(const char *s)
{
    screen_put(s);
}
/*-----------------------------------------------------------------------------
 * 定点格式化，避免 %f（xil_printf 不支持浮点）。
 *   放大成整数再插小数点，按 int 范围自动限幅防溢出。
 *
 * 这里**刻意不用** snprintf 的 "%0*d" 动态宽度：GCC 只看到宽度参数是 int、
 * 范围 [0, 2147483647]，无法证明上界，于是报
 *     note: directive argument in the range [0, 2147483647]
 *     note: 'snprintf' output between 5 and 2147483651 bytes ...
 * 所以改成手工拼字符串。既没有格式串风险，也不再需要 snprintf。
 *----------------------------------------------------------------------------*/
static void fmt_fixed(char *dst, int dstlen, double v, int dec)
{
    int scale = 1;
    int i, s, ip, fp, n = 0, neg;
    char frac[12];
    char ib[12];
    unsigned int u;
    double lim;

    if ((dst == 0) || (dstlen <= 0)) {
        return;
    }
    if (dec < 0) { dec = 0; }
    if (dec > 9) { dec = 9; }               /* 本工程最多用 6 位小数 */

    for (i = 0; i < dec; i++) {
        scale *= 10;
    }
    lim = (double)(2147483647 / scale);     /* int 安全上限 */
    if (v >  lim) { v =  lim; }
    if (v < -lim) { v = -lim; }

    s   = (int)((v >= 0.0) ? (v * (double)scale + 0.5)
                           : (v * (double)scale - 0.5));
    ip  = s / scale;
    fp  = s % scale;
    neg = (s < 0);
    if (fp < 0) { fp = -fp; }

    /* 小数位：从最低位往前写，不足的高位自然补 '0' */
    for (i = 0; i < dec; i++) {
        frac[i] = (char)('0' + (fp % 10));
        fp /= 10;
    }

    /* 整数部分（-(ip+1)+1 避免 INT_MIN 取负溢出） */
    u = (ip < 0) ? ((unsigned int)(-(ip + 1)) + 1U) : (unsigned int)ip;
    i = 0;
    do {
        ib[i++] = (char)('0' + (u % 10U));
        u /= 10U;
    } while (u != 0U);

    if (neg && (n < dstlen - 1)) {
        dst[n++] = '-';                     /* -0.5 也要带负号 */
    }
    while ((i > 0) && (n < dstlen - 1)) {
        dst[n++] = ib[--i];
    }
    if ((dec > 0) && (n < dstlen - 1)) {
        dst[n++] = '.';
    }
    for (i = dec - 1; (i >= 0) && (n < dstlen - 1); i--) {
        dst[n++] = frac[i];
    }
    dst[n] = '\0';
}

static const char *screen_state_name(int st)
{
    switch (st) {
    case STATE_TUNING:   return "TUNING";
    case STATE_CALIB:    return "CALIB";
    case STATE_HOLDOVER: return "HOLDOVER";
    default:             return "UNKNOWN";
    }
}

/*=============================================================================
 * 对外：把当前状态打包发给串口屏
 *   经纬度      gnss.latitude / gnss.longitude + lat_dir / lon_dir
 *   X[0] 相位差 kalman.X[0]  (ns)
 *   X[1] 频差   kalman.X[1]  (ppb)
 *   卫星数      gnss.satellites
 *   HDOP        gnss.hdop
 *   DAC 值      control_get_last_dac()
 *   C/N0        gnss.cn0_avg
 *   状态机状态  sys_state
 *===========================================================================*/
/*-----------------------------------------------------------------------------
 * 发送一条 <对象>.txt="<值>" + FF FF FF
 *   不拼 snprintf, 直接分段发: 没有格式串风险, 也不用担心转义。
 *-----------------------------------------------------------------------------*/
static void scr_txt(const char *obj, const char *val)
{
    screen_put(obj);
    screen_put(".txt=\"");
    screen_put(val);
    screen_put("\"");
    screen_put("\xFF\xFF\xFF");
}

/* 整数 -> 十进制字符串(不带符号处理, 只用于卫星数/周期这类正值) */
static void scr_int(char *dst, int dstlen, int v)
{
    char  t[12];
    int   n = 0, k = 0;
    unsigned int u;

    if ((dst == 0) || (dstlen <= 0)) { return; }
    if (v < 0) { v = 0; }
    u = (unsigned int)v;
    do { t[n++] = (char)('0' + (u % 10U)); u /= 10U; } while ((u != 0U) && (n < 11));
    while ((n > 0) && (k < dstlen - 1)) { dst[k++] = t[--n]; }
    dst[k] = '\0';
}

/* 经纬度: 数值 + 半球字母, 例如 39.5812N */
static void scr_coord(const char *obj, double v, char pos, char neg)
{
    char b[24];
    int  n;

    fmt_fixed(b, (int)sizeof(b), fabs(v), 4);
    n = (int)strlen(b);
    if (n < (int)sizeof(b) - 1) {
        b[n]     = (v >= 0.0) ? pos : neg;
        b[n + 1] = '\0';
    }
    scr_txt(obj, b);
}

/*=============================================================================
 * 每秒刷一帧（陶晶驰 USART HMI / TJC 指令集，9600 8N1）
 *
 *   gpfrq = 频率差 X[1] (ppb)        gpcn  = C/N0
 *   gpphs = 相位差 X[0] (ns)         gppps = 1PPS 与本地差距(本秒实测, ns)
 *   gpt   = 卡尔曼采样周期 T (s)      gpslw = 此时慢环的 T (s)
 *   gpsj  = 经度 E/W                 gpsw  = 纬度 N/S
 *   gpsn  = 卫星数                   gpsl  = 定位质量
 *   gphd  = HDOP                     gpstxt= 状态文字
 *   gpst  = 时间                     gpsd  = 日期
 *===========================================================================*/
void screen_send_status(void)
{
    char b[24];

    if (!screen_ready) {
        return;
    }

    /* ---- 频率差 / 相位差 / 采样周期 T ----------------------------------
     * 失锁（无定位或 HOLDOVER）时这两栏显示 "----"：
     *   没有 GNSS 参考，OCXO 的固有频偏会积分进 X[0]，每 ppb 每秒涨 1 ns，
     *   ppm 级偏差几秒就能涨到微秒量级 —— 屏上跳一个越来越大的数没有意义，
     *   反而会让人以为系统在工作。真实数值仍然照常打到 UART1 日志里。
     * -------------------------------------------------------------------- */
    {
        int locked = (gnss.fix_valid != 0) && (sys_state != STATE_HOLDOVER);

        if (locked != 0) {
            double v;

            v = kalman.X[1];
            if (v >  999.0) { v =  999.0; }
            if (v < -999.0) { v = -999.0; }
            fmt_fixed(b, (int)sizeof(b), v, 3);
            scr_txt(SCR_O_FRQ, b);      /* X[1] = 频率差 ppb */

            /* 相位差显示【绝对相位 X[0]】。
             * 原来这里和下面的 1PPS 差距都填 read_phase_error_ns()（本秒增量），
             * 两栏永远一模一样，而且绕过了 main.c 里的 ±50 ns 修正，
             * 屏上会看到粗计数 ±1 造成的尖峰。 */
            v = kalman.X[0];
            if (v >  99999.0) { v =  99999.0; }
            if (v < -99999.0) { v = -99999.0; }
            fmt_fixed(b, (int)sizeof(b), v, 3);
            scr_txt(SCR_O_PHS, b);      /* X[0] = 绝对相位 ns */
        } else {
            scr_txt(SCR_O_FRQ, "----");
            scr_txt(SCR_O_PHS, "----");
        }
    }

    scr_int(b, (int)sizeof(b), sample_period);
    scr_txt(SCR_O_T, b);

    /* ---- 1PPS 与本地差距: 本秒实测相位增量 ---- */
    if (bsp_meas_valid() != 0) {
        /* 用 main_last_phase_ns 而不是 raw 的 0x00：
         * 它已经过 main.c 的 fix_50ns_glitch() 修正，屏上不再有 ±50 ns 尖峰。 */
        fmt_fixed(b, (int)sizeof(b), main_last_phase_ns, 3);
    } else {
        b[0] = '-'; b[1] = '-'; b[2] = '\0';
    }
    scr_txt(SCR_O_PPS, b);

    /* ---- 此时慢环的 T ---- */
    if (slow.active != 0) {
        scr_int(b, (int)sizeof(b), (int)slow.T);
    } else {
        b[0] = '-'; b[1] = '-'; b[2] = '\0';
    }
    scr_txt(SCR_O_SLOW, b);

    /* ---- 经纬度(没定位时显示 __) ---- */
    if (gnss.fix_valid != 0) {
        scr_coord(SCR_O_LAT, gnss.latitude,  'N', 'S');
        scr_coord(SCR_O_LON, gnss.longitude, 'E', 'W');
    } else {
        scr_txt(SCR_O_LAT, "--");
        scr_txt(SCR_O_LON, "--");
    }

    /* ---- 定位质量 / 卫星数 / HDOP / C/N0 ---- */
    scr_txt(SCR_O_QUAL, (gnss.fix_valid != 0) ? "FIX" : "NOFIX");

    scr_int(b, (int)sizeof(b), gnss.satellites);
    scr_txt(SCR_O_SAT, b);

    fmt_fixed(b, (int)sizeof(b), gnss.hdop, 2);
    scr_txt(SCR_O_HDOP, b);

    fmt_fixed(b, (int)sizeof(b), gnss.cn0_avg, 1);
    scr_txt(SCR_O_CN0, b);

    /* ---- 时间: GNSS 结构体只给 utc_time (hhmmss.ss), 没有日期字段 ---- */
    if (gnss.utc_time > 0.0) {
        int hh = (int)(gnss.utc_time / 10000.0);
        int mi = ((int)(gnss.utc_time / 100.0)) % 100;
        int ss = ((int)gnss.utc_time) % 100;

        scr_int(&b[0], 3, hh); b[2] = ':';
        scr_int(&b[3], 3, mi); b[5] = ':';
        if (ss < 10) { b[6] = '0'; scr_int(&b[7], 3, ss); }
        else         { scr_int(&b[6], 3, ss); }
        scr_txt(SCR_O_TIME, b);
    } else {
        scr_txt(SCR_O_TIME, "--");
    }

    /* 日期: gnss.c 目前不解析日期, 固定发 -- (要的话在 GI 里加 date 字段) */
    scr_txt(SCR_O_DATE, "--");
    /* ---- 状态文字 ---- */
    scr_txt(SCR_O_TXT, screen_state_name(sys_state));
}
/*=============================================================================
 * 可选部分：串口屏回传命令（UART0 接收中断）
 *   不用的话，从 screen_init_irq() 往下整段删掉即可。
 *===========================================================================*/
#define SCREEN_RX_SIZE  64
static volatile u8  screen_rx_buf[SCREEN_RX_SIZE];
static volatile int screen_rx_head = 0;
static volatile int screen_rx_tail = 0;

static void screen_rx_push(u8 b)
{
    int next = (screen_rx_head + 1) % SCREEN_RX_SIZE;
    if (next != screen_rx_tail) {
        screen_rx_buf[screen_rx_head] = b;
        screen_rx_head = next;
    }
}

int screen_get_cmd(char *out, int maxlen)
{
    static char line[SCREEN_RX_SIZE];
    static int  n = 0;
    int i;

    if ((out == NULL) || (maxlen <= 0)) {
        return 0;
    }
    while (screen_rx_tail != screen_rx_head) {
        u8 b = screen_rx_buf[screen_rx_tail];
        screen_rx_tail = (screen_rx_tail + 1) % SCREEN_RX_SIZE;

        if ((b == '\n') || (b == '\r')) {
            if (n > 0) {
                line[n] = '\0';
                for (i = 0; (i < n) && (i < maxlen - 1); i++) {
                    out[i] = line[i];
                }
                out[i] = '\0';
                n = 0;
                return 1;
            }
        } else if (n < (int)sizeof(line) - 1) {
            line[n++] = (char)b;
        }
    }
    return 0;
}




void screen_poll_rx(void)
{
    u32 base;

    if (!screen_ready) {
        return;
    }
    base = screen_uart.Config.BaseAddress;

    while (XUartPs_IsReceiveData(base)) {
        screen_rx_push((u8)XUartPs_RecvByte(base));
    }
}

//=============================================================================
// 页码切换
//-----------------------------------------------------------------------------
// TJC 指令格式:  page <n>  后跟三个 0xFF
// 1.8 寸 160x128 放不下 14 个控件，所以分两页（见 screen.h 的说明）。
// 控件值跨页保留，所以切页后不需要重绘，下一秒 screen_send_status() 会自动刷。
//=============================================================================
static int s_page = SCR_PAGE_CTRL;

int screen_page_get(void)
{
    return s_page;
}

const char *screen_page_name(void)
{
    return (s_page == SCR_PAGE_GNSS) ? "GNSS" : "CTRL";
}

int screen_page_next(void)
{
    char b[8];

    s_page = (s_page + 1) % SCREEN_PAGE_CNT;

    b[0] = 'p';
    b[1] = 'a';
    b[2] = 'g';
    b[3] = 'e';
    b[4] = ' ';
    b[5] = (char)('0' + s_page);
    b[6] = '\0';

    screen_put(b);
    screen_put("\xFF\xFF\xFF");

    return s_page;
}