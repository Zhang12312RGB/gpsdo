#ifndef BSP_H
#define BSP_H

#include "xil_types.h"
#include "xparameters.h"

#define GNSS_BASE_ADDR  XPAR_GNSS_GPSDO_TOP_0_BASEADDR

/* ---- PL 寄存器映射, 与 rtl/gnss_axi_ip_slave.v 的表逐条对应 ---- */
#define REG_PHASE_ERR      (GNSS_BASE_ADDR + 0x00)  /* R   s32 相位差             */
#define REG_NMEA_DATA      (GNSS_BASE_ADDR + 0x04)  /* R/W bit8=新数据, 写0x100清 */
#define REG_DAC_DATA       (GNSS_BASE_ADDR + 0x08)  /* R/W 12bit, 写触发一帧 SPI  */
#define REG_TDC_TAP_PERIOD (GNSS_BASE_ADDR + 0x0C)  /* R/W TDC标定: 每周期抽头数  */
#define REG_TDC_TAP_OFFSET (GNSS_BASE_ADDR + 0x10)  /* R/W TDC标定: 固定延迟抽头  */
#define REG_OCXO_COUNT     (GNSS_BASE_ADDR + 0x14)  /* R   本 1PPS 间隔内 OCXO 边沿 */
#define REG_OCXO_DELTA     (GNSS_BASE_ADDR + 0x18)  /* R   s32 OCXO_COUNT - 标称  */
#define REG_CLK_COUNT      (GNSS_BASE_ADDR + 0x1C)  /* R   100MHz 参考计数(诊断)  */
#define REG_CLK_DELTA      (GNSS_BASE_ADDR + 0x20)  /* R   s32 CLK_COUNT - 标称   */
#define REG_DROP_COUNT     (GNSS_BASE_ADDR + 0x24)  /* R   被丢弃的 NMEA 字节数   */
#define REG_BYTE_COUNT     (GNSS_BASE_ADDR + 0x28)  /* R   收下的 NMEA 字节数     */
#define REG_STATUS         (GNSS_BASE_ADDR + 0x2C)  /* R   状态字(见下)           */
#define REG_TDC_TAP        (GNSS_BASE_ADDR + 0x30)  /* R   上次测量的抽头值       */
#define REG_TDC_RAW_INDEX  (GNSS_BASE_ADDR + 0x34)  /* R/W 原始图索引             */
#define REG_TDC_RAW_DATA   (GNSS_BASE_ADDR + 0x38)  /* R   原始图数据             */
#define REG_VERSION        (GNSS_BASE_ADDR + 0x3C)  /* R   版本号 0x00030000      */
/* 诊断计数器（2026-10-27 新增，只读）*/
#define REG_DBG_FINE_CNT   (GNSS_BASE_ADDR + 0x60)  /* R   fine_valid 累计脉冲    */
#define REG_DBG_FRAC_CNT   (GNSS_BASE_ADDR + 0x64)  /* R   frac_valid 累计脉冲    */
#define REG_DBG_DIFF_CNT   (GNSS_BASE_ADDR + 0x68)  /* R   diff_valid 累计脉冲    */
#define REG_DBG_FRAC_VAL   (GNSS_BASE_ADDR + 0x6C)  /* R   当前 frac 快照         */
#define REG_BAUD_DIV       (0x43C00000 + 0x40)  /* R/W UART 分频 = 参考时钟/波特率 */

/* ---- 第二路(慢环) ---- */
#define REG_P2_T           (GNSS_BASE_ADDR + 0x44)  /* R/W 计数周期 T(秒)          */
#define REG_P2_PHASE_LO    (GNSS_BASE_ADDR + 0x48)  /* R   相位差累加 低 32 位     */
#define REG_P2_PHASE_HI    (GNSS_BASE_ADDR + 0x4C)  /* R   相位差累加 高 32 位     */
#define REG_P2_STATUS      (GNSS_BASE_ADDR + 0x50)  /* R/W bit0=NEW, 写 1 清       */
#define REG_P2_COUNT       (GNSS_BASE_ADDR + 0x54)  /* R   窗口进度(0..T-1)        */
#define REG_P2_OCXO        (GNSS_BASE_ADDR + 0x58)  /* R   T 内 ocxo_delta 累加    */
#define P2_NEW_FLAG        (1U << 0)

/* STATUS(0x2C) 位定义 */
#define STATUS_MEAS_VALID  (1U << 0)
#define STATUS_LOCKED      (1U << 1)
#define STATUS_PPS_LOST    (1U << 2)
#define STATUS_SAT_FLAG    (1U << 3)
#define STATUS_TDC_SAT     (1U << 4)
#define STATUS_FRAME_ERR   (1U << 5)   /* 曾经出现帧错误: 波特率/接线 问题 */
#define STATUS_SPI_BUSY    (1U << 6)
#define STATUS_DIFF_VALID  (1U << 7)
#define STATUS_TAP_MAX     (1U << 8)
#define NMEA_NEW_FLAG     0x100      /* NMEA 第8位新数据标志        */
#define PHASE_ERR_LSB_NS  0.1953125  /* 1 raw LSB = 50ns/256 = 0.1953125 ns (20 MHz) */

/*-----------------------------------------------------------------------------
 * TDC 标定量（hwtest mode 3 实测后填）
 *   TDC_TAP_PERIOD_MEASURED : 每 10 ns 周期对应多少抽头。0 = 沿用 RTL 默认 160
 *   TDC_TAP_OFFSET_MEASURED : 固定同步器延迟（抽头数）。一般填 0
 * 写入位置见 bsp_set_tdc_calib()，由 main() 上电时调用。
 *---------------------------------------------------------------------------*/
#define TDC_TAP_PERIOD_MEASURED   0
#define TDC_TAP_OFFSET_MEASURED   0

void bsp_set_tdc_calib(uint16_t period, uint16_t offset);
int32_t  read_phase_error(void);
double   read_phase_error_ns(void);
uint32_t bsp_status(void);            /* 读 0x2C STATUS            */
int      bsp_meas_valid(void);
/* 1PPS 是否真的存在（0x2C 的 pps_lost 取反）。
 * 和 bsp_meas_valid() 的区别：meas_valid 只表示"0x00 里有一个锁存值"，
 * 接收机完全没输出 1PPS 时它仍可能是 1，此时 0x00 是【上一次的陈旧值】。
 * 拔掉天线实测就是这样：sat=0、fix=0，但 meas_valid 仍是 1，
 * 于是每秒都把同一个陈旧值累加进 main_phase_acc_ns，最后撞到 int32 上限。 */
int bsp_pps_ok(void);         /* STATUS bit0: 0x00 的相位差有效 */

/* 测量样本质量检查（2026-10-30）
 *   raw  : 0x00 的原始值
 *   st   : 0x2C 状态字
 *   seq0 : 读之前 REG_DBG_DIFF_CNT
 *   seq1 : 读之后 REG_DBG_DIFF_CNT
 * 返回 1 = 可用；0 = 应跳过（饱和/撕裂/间隔异常）。
 * 调用方在返回 0 时【跳过本拍、不要清零相位累加器】。 */
int bsp_sample_good(int32_t raw, uint32_t st, uint32_t seq0, uint32_t seq1);

/*-----------------------------------------------------------------------------
 * 统一测量快照 + 质量判定（2026-10-30）
 *   一份快照里包含 0x00 / 0x2C / 0x18 / 0x20 / 0x30 / 0x6C / 0x68 全部字段，
 *   保证「相位累加」与「KF/DAC 控制」看到的是同一拍的数据。
 *-----------------------------------------------------------------------------*/
#define BSP_Q_OK       0   /* 可用 */
#define BSP_Q_SAT      1   /* 饱和 */
#define BSP_Q_TORN     2   /* 撕裂（读期间来新 PPS）*/
#define BSP_Q_DUP      3   /* 重复（序号未变）*/
#define BSP_Q_GAP      4   /* 间隔异常（粗异常，±0.5%）*/
#define BSP_Q_NO_MEAS  5   /* 无测量（meas_valid=0 或 pps_lost=1）*/

typedef struct {
    int32_t  raw;        /* 0x00 相位增量（Q8）*/
    uint32_t status;     /* 0x2C 状态字 */
    int32_t  ocxo_delta; /* 0x18 */
    int32_t  clk_delta;  /* 0x20 */
    uint32_t tap;        /* 0x30 */
    uint32_t frac;       /* 0x6C */
    uint32_t seq;        /* 0x68 */
    uint32_t seq0;
    uint32_t seq1;
    uint32_t ms;
} bsp_snapshot_t;

void bsp_snapshot_take(bsp_snapshot_t *s);
int  bsp_sample_classify(const bsp_snapshot_t *s);
void bsp_snapshot_mark_used(const bsp_snapshot_t *s);
void bsp_snapshot_reset_seq(void);

void     bsp_p2_set_t(uint16_t t);     /* 写 0x44 计数周期 T(秒)      */
uint16_t bsp_p2_get_t(void);           /* 读 0x44                     */
uint16_t bsp_p2_count(void);           /* 读 0x54 窗口进度            */
int32_t  bsp_p2_read_ocxo(void);       /* 读 0x58 T 内 OCXO 边沿累加  */
int64_t  bsp_p2_read_phase(void);      /* 0x48(低)+0x4C(高) 拼 64 位  */
int      bsp_p2_take_new(void);        /* 有 NEW 则清标志并返回 1     */
uint32_t bsp_millis(void);
void     bsp_set_baud_div(uint32_t div);   /* 写 0x40 */
uint32_t bsp_get_baud_div(void);           /* 读 0x40 */
void     bsp_timer_init(void);
void     bsp_tick(void);
uint32_t get_system_tick(void);
int      one_second_elapsed(void);

/*-----------------------------------------------------------------------------
 * 阻塞式外设输出（串口屏 UART0 / 调试口 UART1）在等发送 FIFO 时的空闲钩子。
 * main.c 注册 gnss_pack_task 之后，发屏/打日志的每个字节间隙都会去抽干 PL 的
 * NMEA 邮箱 —— 否则一帧屏占用 0.3 s，期间到达的字节全被丢（实测 drop≈37%）。
 *---------------------------------------------------------------------------*/
void bsp_set_idle_hook(void (*fn)(void));
void bsp_idle(void);

#endif /* BSP_H */