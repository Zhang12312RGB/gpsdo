#include "bench.h"
#include "bsp.h"
#include "control.h"
#include "gnss.h"
#include "dbg_uart.h"
#include "xil_io.h"

static double   b_phase;       // 累计相位真值(ns)
static double   b_meas;        // 含噪声的测量值(ns)
static double   b_true;        // 当前实际频率偏差(ppb)
static uint32_t b_t;           // 台架秒计数
static uint32_t b_seed = 12345U;

static double   b_p2_base;     // 本窗口起点的相位(ns)
static uint16_t b_p2_t = 1;
static uint16_t b_p2_cnt;
static int      b_p2_new;
static int64_t  b_p2_delta;    // 本窗口的相位差增量, 单位为 PHASE_ERR 原始计数
static int32_t  b_p2_ocxo;     // 本窗口的 OCXO 边沿偏差

// 线性同余伪随机, -1.0 .. +1.0; 不用 rand() 免得引入 stdlib
static double b_rand(void)
{
    b_seed = b_seed * 1103515245U + 12345U;
    return ((double)((b_seed >> 16) & 0x7FFFU) / 32767.0) - 1.0;
}

void bench_init(void)
{
    b_phase   = 0.0;
    b_meas    = 0.0;
    b_true    = BENCH_BIAS_PPB;
    b_t       = 0U;
    b_seed    = 12345U;
    b_p2_base = 0.0;
    b_p2_t    = 1U;
    b_p2_cnt  = 0U;
    b_p2_new  = 0;
    b_p2_delta = 0;
    b_p2_ocxo  = 0;

    dbg_printf("\r\n---- 台架模式(无外设自测) ----\r\n");
    dbg_printf("     被控对象: bias=%.1fppb, DAC 灵敏度=%.3fppb/code, 噪声=%.1fns\r\n",
                BENCH_BIAS_PPB, BENCH_DAC_GAIN, BENCH_NOISE_NS);
    dbg_printf("     第 %us 注入 +%.0fns 相位阶跃, 用来检验 |innov|>3sqrt(R) 保护\r\n",
                (unsigned)BENCH_STEP_AT_S, BENCH_STEP_NS);
    dbg_printf("     把 bench.h 的 BENCH_ENABLE 改成 0 就回到真实硬件\r\n\r\n");
}

void bench_tick(void)
{
    int    dac;
    double corr;

    b_t++;

    if (b_t == BENCH_STEP_AT_S) {
        b_phase += BENCH_STEP_NS;
        dbg_printf("[BENCH] t=%us 注入相位阶跃 +%.0fns\r\n", (unsigned)b_t, BENCH_STEP_NS);
    }

    // 被控对象: 实际频率偏差 = 固有偏差 + DAC 带来的修正
    dac  = (int)(Xil_In32(REG_DAC_DATA) & 0xFFFU);
    corr = ((double)dac - (double)DAC_CENTER) * BENCH_DAC_GAIN;
    b_true = BENCH_BIAS_PPB + corr;

    // 1ppb 在 1s 里正好是 1ns
    b_phase += b_true;
    b_meas   = b_phase + (b_rand() * BENCH_NOISE_NS);

    // ---- 第二路的 T 秒窗口: 到点产生一个样本 ----
    if (b_p2_cnt < b_p2_t) {
        b_p2_cnt++;
    }
    if (b_p2_cnt >= b_p2_t) {
        double d = b_phase - b_p2_base;

        b_p2_delta = (int64_t)(d / PHASE_ERR_LSB_NS);
        b_p2_ocxo  = (int32_t)((d / 1e9) * BENCH_OCXO_HZ);
        b_p2_base  = b_phase;
        b_p2_cnt   = 0U;
        b_p2_new   = 1;
    }
}

/* 保留这个函数只为兼容调用点; 现在【不再修改 gnss 结构体】。
 * 替身值由 bench.h 的 BENCH_FIX/SAT/HDOP/CN0 提供, main.c 在消费点分流。 */
void bench_gnss_override(void)
{
}

double bench_phase_ns(void)      { return b_meas; }
int    bench_meas_valid(void)    { return 1; }
double bench_true_ppb(void)      { return b_true; }
double bench_true_phase_ns(void) { return b_phase; }

int bench_p2_take_new(void)
{
    if (b_p2_new == 0) { return 0; }
    b_p2_new = 0;
    return 1;
}
int64_t bench_p2_phase(void)     { return b_p2_delta; }
int32_t bench_p2_ocxo(void)      { return b_p2_ocxo; }
void    bench_p2_set_t(uint16_t t)
{
    b_p2_t = (t == 0U) ? 1U : t;
    b_p2_cnt = 0U;
    b_p2_base = b_phase;      // 新窗口从当前相位起算
    b_p2_new = 0;
}
uint16_t bench_p2_get_t(void)   { return b_p2_t; }
uint16_t bench_p2_count(void)   { return b_p2_cnt; }
