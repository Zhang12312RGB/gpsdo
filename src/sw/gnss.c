#include "gnss.h"
#include "bsp.h"
#include "xil_io.h"
#include <string.h>
#include <stdlib.h>
#include "dbg_uart.h"

GI gnss;                       /* 全局实例 */

static char g_nmea_buffer[128];
static unsigned int g_nmea_len;
static int g_pack_busy;
static GNSS_NMEA_DIAG g_nmea_diag;
static int g_have_gga;
static uint32_t g_last_gga_ms;

int gnss_quality_fresh(void)
{
    return g_have_gga &&
           (uint32_t)(bsp_millis() - g_last_gga_ms) <= GNSS_QUALITY_TIMEOUT_MS;
}

void gnss_get_nmea_diag(GNSS_NMEA_DIAG *out)
{
    if (out == NULL) { return; }
    *out = g_nmea_diag;
    out->gga_age_ms = g_have_gga ? (uint32_t)(bsp_millis() - g_last_gga_ms)
                               : UINT32_MAX;
    out->quality_fresh = g_have_gga && out->gga_age_ms <= GNSS_QUALITY_TIMEOUT_MS;
}


#define GSV_CN0_ALPHA  0.3     /* C/N0 滑动平均新值权重 */

/* 原始字节钩子: 定义在 main.c, 由 gnss_set_raw_hook() 注册 */
static GNSS_RAW_HOOK g_raw_hook = 0;

/* 从 NMEA 寄存器读一个字符: 有新数据返回字符并写 0x100 清零, 否则返回 -1 */
static int read_nmea_char(void)
{
    uint32_t reg_val = Xil_In32(REG_NMEA_DATA);

    if ((reg_val >> 8) & 0x01) {
        unsigned char c = (unsigned char)(reg_val & 0xFF);      /* 低8位字符 */
        Xil_Out32(REG_NMEA_DATA, NMEA_NEW_FLAG); /* 写 0x100 清标志 */

        /* 先交给原始数据钩子(main.c 里的 UART1 原始流), 再喂给解析器 ——
         * 钩子只做输出, 不消费数据, 所以状态机照样拿到全部字节。 */
        if (g_raw_hook != 0) {
            g_raw_hook(c);
        }

        if (gnss_debug_level >= GNSS_DBG_CHAR) {   /* 1 字符级回显 */
            dbg_putc(c);
        }
        return (int)c;
    }
    return -1;                                 /* 无新数据 */
}

int gnss_debug_level = GNSS_DBG_OFF;     /* 默认关 */


void gnss_set_raw_hook(GNSS_RAW_HOOK fn)
{
    g_raw_hook = fn;
}

GNSS_RAW_HOOK gnss_get_raw_hook(void)
{
    return g_raw_hook;
}



/* 初始化 GNSS 结构与拼包状态 */
void gnss_task_init(void)
{
    memset(&gnss, 0, sizeof(gnss));
    memset(&g_nmea_diag, 0, sizeof(g_nmea_diag));
    g_nmea_len = 0U;
    g_pack_busy = 0;
    g_have_gga = 0;
    g_last_gga_ms = 0U;
    gnss.hdop    = 99.9;     /* 无定位时的默认 HDOP */
    gnss.cn0_avg = 40.0;     /* 收到 GSV 前的默认 C/N0 */
}

/*=============================================================================
 * NMEA 字段读取 —— 不能用 strtok！
 *-----------------------------------------------------------------------------
 * strtok(p, ",") 会把【连续的逗号当成一个分隔符】，并且跳过开头的逗号。
 * 但 NMEA 里空字段是有意义的：没有定位时经纬度、HDOP 全是空，
 * 例如  $GPGGA,,,,,,0,00,99.99,,,,,,*48
 *       字段: 1 2 3 4 5 6 7 8 ...
 *                  ^空^        ^空^
 * 用 strtok 会得到  0 / 00 / 99.99 当作第 1/2/3 个字段,
 * 于是 field==6/7/8 的判断永远不命中 -> fix/sat/hdop 全都不被赋值,
 * 而函数末尾无条件的 last_update_tick 照写 -> 出现「upd 在涨但字段不变」的假象。
 * 下面按【逗号位置】逐个取字段, 空字段保留为空串。
 *===========================================================================*/
static char *nmea_field(char *p, int n, int *len)
{
    char *s = p;
    int   i;

    for (i = 0; i < n; i++) {
        char *c = strchr(s, ',');
        if (c == NULL) {
            *len = 0;
            return NULL;
        }
        s = c + 1;
    }
    {
        char *e = strchr(s, ',');
        *len = (e != NULL) ? (int)(e - s) : (int)strlen(s);
    }
    return s;
}

/* 取第 n 个字段(0 基)的数值; 空字段/越界 -> 0.0 */
static double nmea_num(char *p, int n)
{
    int   len = 0;
    char *s = nmea_field(p, n, &len);
    char  buf[24];

    if ((s == NULL) || (len <= 0) || (len >= (int)sizeof(buf))) {
        return 0.0;
    }
    memcpy(buf, s, (size_t)len);
    buf[len] = '\0';
    return atof(buf);
}

/* 取第 n 个字段的首字符; 空字段 -> '\0' */
static char nmea_ch(char *p, int n)
{
    int   len = 0;
    char *s = nmea_field(p, n, &len);

    return ((s != NULL) && (len > 0)) ? s[0] : '\0';
}

/* ddmm.mmmm / dddmm.mmmm -> 十进制度 */
static double nmea_deg(char *p, int n)
{
    double v   = nmea_num(p, n);
    double deg = (double)(int)(v / 100.0);
    double min = v - deg * 100.0;

    return deg + min / 60.0;
}
/* RMC: UTC时间、定位状态、经纬度(度分转十进制度) */
static void parse_rmc(char *p)
{
    gnss.utc_time  = nmea_num(p, 0);                       /* UTC hhmmss.ss */
    gnss.fix_valid = (nmea_ch(p, 1) == 'A') ? 1 : 0;       /* A=有效 V=无效 */
    gnss.latitude  = nmea_deg(p, 2);                       /* ddmm.mmmm     */
    gnss.lat_dir   = nmea_ch(p, 3);                        /* N / S         */
    gnss.longitude = nmea_deg(p, 4);                       /* dddmm.mmmm    */
    gnss.lon_dir   = nmea_ch(p, 5);                        /* E / W         */

    gnss.last_update_tick = get_system_tick();
}

/* GGA: 卫星数(字段7)、HDOP(字段8, atof)、海拔(字段9) */
static void parse_gga(char *p)
{
    /* 字段(0 基): 0=UTC 1=lat 2=N/S 3=lon 4=E/W 5=fix 6=sats 7=HDOP 8=alt */
    gnss.fix_valid  = (nmea_num(p, 5) > 0.0) ? 1 : 0;      /* 定位质量     */
    gnss.satellites = (int)nmea_num(p, 6);                 /* 卫星数       */
    gnss.hdop       = nmea_num(p, 7);                      /* HDOP         */
    gnss.altitude   = nmea_num(p, 8);                      /* 海拔 m       */
    g_last_gga_ms = bsp_millis();
    g_have_gga = 1;

    gnss.last_update_tick = get_system_tick();
}

/* GSV: 提取各卫星 C/N0 做 0.7/0.3 滑动平均 (字段 7,11,15,19) */
static void parse_gsv(char *p)
{
    /* 字段(0 基): 0=本次句数 1=第几句 2=可视卫星数,
     * 之后每颗星 4 个字段: PRN / 仰角 / 方位 / C/N0
     *   -> C/N0 落在 6, 10, 14, 18
     * 注意: 没卫星时这些字段是空的, 必须用 nmea_field 判空,
     *       不能像原来那样用 strtok (会吃掉空字段导致错位)。 */
    int    cnt = 0;
    double sum = 0.0;
    double avg;
    int    k;

    for (k = 0; k < 4; k++) {
        int   len = 0;
        int   idx = 6 + 4 * k;

        if (nmea_field(p, idx, &len) == NULL) {
            break;                            /* 字段不存在: 后面没有了 */
        }
        if (len <= 0) {
            continue;                         /* 空字段: 该卫星没报 C/N0 */
        }
        sum += nmea_num(p, idx);
        cnt++;
    }
    if (cnt == 0) {
        return;                               /* 一句 GSV 都没带 C/N0 */
    }

    avg = sum / (double)cnt;
    if (gnss.cn0_avg <= 0.0) {
        gnss.cn0_avg = avg;                   /* 第一次: 直接取平均 */
    } else {
        gnss.cn0_avg = 0.7 * gnss.cn0_avg + GSV_CN0_ALPHA * avg;
    }
}

/* Check the complete frame before touching any GNSS field or freshness time. */
static int nmea_hex(char c)
{
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    return -1;
}

static char *nmea_checksum(char *sentence)
{
    char *star, *p, *tail;
    unsigned int sum = 0U;
    int hi, lo;
    if (sentence[0] != '$') { goto malformed; }
    star = strchr(sentence + 1, '*');
    if (star == NULL || star[1] == '\0' || star[2] == '\0') { goto malformed; }
    hi = nmea_hex(star[1]);
    lo = nmea_hex(star[2]);
    if (hi < 0 || lo < 0) { goto malformed; }
    tail = star + 3;
    if (*tail == '\r') { tail++; }
    if (*tail == '\n') { tail++; }
    if (*tail != '\0') { goto malformed; }
    for (p = sentence + 1; p < star; p++) {
        unsigned char ch = (unsigned char)*p;
        if (ch < 32U || ch > 126U) { goto malformed; }
        sum ^= ch;
    }
    if (sum != (unsigned int)((hi << 4) | lo)) {
        g_nmea_diag.bad_checksum++;
        return NULL;
    }
    return star;
malformed:
    g_nmea_diag.bad_format++;
    return NULL;
}

static int nmea_fields_at_least(const char *p, int count)
{
    int fields = 1;
    for (; *p != '\0'; p++) {
        if (*p == ',') { fields++; }
    }
    return fields >= count;
}

/* Standard two-letter talker + three-letter type; preserve empty fields. */
static void parse_nmea_sentence(char *sentence)
{
    char *star = nmea_checksum(sentence);
    char *comma, *payload;
    if (star == NULL) { return; }
    *star = '\0';
    comma = strchr(sentence, ',');
    if (comma == NULL || comma != sentence + 6) {
        g_nmea_diag.bad_format++;
        return;
    }
    payload = comma + 1;
    if (memcmp(sentence + 3, "RMC", 3U) == 0) {
        if (!nmea_fields_at_least(payload, 7)) { goto malformed; }
        parse_rmc(payload);
    } else if (memcmp(sentence + 3, "GGA", 3U) == 0) {
        if (!nmea_fields_at_least(payload, 9)) { goto malformed; }
        parse_gga(payload);
    } else if (memcmp(sentence + 3, "GSV", 3U) == 0) {
        if (!nmea_fields_at_least(payload, 3)) { goto malformed; }
        parse_gsv(payload);
    }
    g_nmea_diag.ok++;
    return;
malformed:
    g_nmea_diag.bad_format++;
}

/* '$' starts a new frame; only a complete newline-terminated frame is parsed. */
void gnss_pack_task(void)
{
    int c;
    /* Debug/raw hooks may call bsp_idle(), whose hook is this same function. */
    if (g_pack_busy) { return; }
    g_pack_busy = 1;
    while ((c = read_nmea_char()) >= 0) {
        if (c == '$') {
            if (g_nmea_len != 0U) { g_nmea_diag.resync++; }
            g_nmea_len = 0U;
            g_nmea_buffer[g_nmea_len++] = (char)c;
            continue;
        }
        if (g_nmea_len == 0U) { continue; }
        if ((c < 32 && c != '\r' && c != '\n') || c > 126) {
            g_nmea_diag.bad_format++;
            g_nmea_len = 0U;
            continue;
        }
        if (g_nmea_len >= sizeof(g_nmea_buffer) - 1U) {
            g_nmea_diag.overlong++;
            g_nmea_len = 0U;
            continue;
        }
        g_nmea_buffer[g_nmea_len++] = (char)c;
        if (c == '\n') {
            g_nmea_buffer[g_nmea_len] = '\0';
            if (gnss_debug_level >= GNSS_DBG_SENTENCE) {
                dbg_puts(g_nmea_buffer);
                dbg_puts("\r\n");
            }
            parse_nmea_sentence(g_nmea_buffer);
            g_nmea_len = 0U;
        }
    }
    g_pack_busy = 0;
}
