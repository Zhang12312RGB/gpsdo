#ifndef GNSS_H
#define GNSS_H

#include "xil_types.h"

/* GNSS 信息结构体 */
typedef struct GnssInfo_t
{
    /* --- 必须的数据 --- */
    int    fix_valid;          /* 1=有效(A), 0=无效(V)              */
    int    satellites;         /* 卫星数                            */
    double hdop;               /* 水平精度因子                      */
    double cn0_avg;            /* 平均载噪比 dB-Hz                  */

    /* --- 补充的数据 --- */
    double utc_time;           /* UTC时间 hhmmss.ss                 */
    double latitude;           /* 纬度, 十进制度                    */
    char   lat_dir;            /* 'N' / 'S'                         */
    double longitude;          /* 经度, 十进制度                    */
    char   lon_dir;            /* 'E' / 'W'                         */
    double altitude;           /* 海拔, m                           */

    /* --- 状态 --- */
    uint32_t last_update_tick; /* 最后成功更新的软件计数            */
} GI;

extern GI gnss;                /* 定义在 gnss.c                     */

/*---------------------------------------------------------------------------
 * NMEA 调试回显级别（通过 dbg_uart 打到 UART1）
 *   OFF      关（正常运行时用这个）
 *   CHAR     收到的每个字符都打 —— 查"PL 有没有把数据送进 0x04 寄存器"
 *   SENTENCE 打完整语句         —— 查"拼包对不对"
 *   FIELDS   再打解析出的字段   —— 查"解析对不对"
 * 在 main.c 里 dbg_init() 之后设置：gnss_debug_level = GNSS_DBG_SENTENCE;
 *-------------------------------------------------------------------------*/
#define GNSS_DBG_OFF       0
#define GNSS_DBG_CHAR      1
#define GNSS_DBG_SENTENCE  2
#define GNSS_DBG_FIELDS    3

extern int gnss_debug_level;

/*---------------------------------------------------------------------------
 * 原始字节钩子
 *   每从 PL 的 0x04 寄存器取出一个字符, 就在"喂给解析器之前"回调一次。
 *   用来把原始 NMEA 原样转给 UART1 / SD 卡 / 上位机, 而解析器仍然拿到
 *   全部数据 —— 两边不会互相抢数据。
 *   传 NULL 关闭。
 *-------------------------------------------------------------------------*/
typedef void (*GNSS_RAW_HOOK)(char c);

void          gnss_set_raw_hook(GNSS_RAW_HOOK fn);
GNSS_RAW_HOOK gnss_get_raw_hook(void);

/* Valid GGA normally arrives every second. RMC/GSV must not refresh its age. */
#define GNSS_QUALITY_TIMEOUT_MS 5000U

typedef struct {
    uint32_t ok;              /* complete frames passing format + checksum */
    uint32_t bad_checksum;
    uint32_t bad_format;      /* missing/invalid checksum, malformed/truncated */
    uint32_t overlong;
    uint32_t resync;          /* new '$' before previous frame completed */
    uint32_t gga_age_ms;      /* UINT32_MAX until first accepted GGA */
    int quality_fresh;
} GNSS_NMEA_DIAG;

int  gnss_quality_fresh(void);
void gnss_get_nmea_diag(GNSS_NMEA_DIAG *out);

/* Parser lifecycle and byte ingestion. */
void gnss_task_init(void);
void gnss_pack_task(void);


#endif /* GNSS_H */