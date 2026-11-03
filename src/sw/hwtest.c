//=============================================================================
// hwtest.c -- 上板硬件标定与自测
//-----------------------------------------------------------------------------
// 输出全部走 UART1（dbg_printf），不碰串口屏、不碰状态机。
// 全程开环：DAC 直接写 0x08，不经过 control_apply_freq()。
//=============================================================================

#include "control.h"   // control_get_last_dac() for the TDC calibration park point
#include "hwtest.h"
#include "bsp.h"
#include "xil_io.h"
#include "dbg_uart.h"
#include <math.h>

//-----------------------------------------------------------------------------
// 定点小数打印（带符号 + 补前导零）
//-----------------------------------------------------------------------------
static void hw_fix(double v, int dec, int scale)
{
    char fs[5];
    int  neg = (v < 0.0);
    double a = neg ? -v : v;
    int  ip  = (int)a;
    int  fp  = (int)((a - (double)ip) * (double)scale + 0.5);
    int  k;

    for (k = dec - 1; k >= 0; k--) { fs[k] = (char)('0' + (fp % 10)); fp /= 10; }
    fs[dec] = '\0';
    dbg_printf("%s%d.%s", neg ? "-" : "", ip, fs);
}

//-----------------------------------------------------------------------------
// 等下一个 1 秒节拍；期间调 bsp_idle() 让 NMEA 照常被取走
//   （main 已把 gnss_pack_task 注册成空闲钩子，这里不用知道它的可链接性）
//-----------------------------------------------------------------------------
static void hw_wait_tick(void)
{
    while (one_second_elapsed() == 0) {
        bsp_idle();
    }
    bsp_idle();
}

static void hw_dac(int code)
{
    if (code < 0)    { code = 0; }
    if (code > 4095) { code = 4095; }
    Xil_Out32(REG_DAC_DATA, (uint32_t)code);
}

//-----------------------------------------------------------------------------
// 寄存器快照
//-----------------------------------------------------------------------------
void hwtest_dump(void)
{
    uint32_t st = Xil_In32(REG_STATUS);

    dbg_printf("\r\n========== 寄存器快照 ==========\r\n");
    dbg_printf(" ver 0x3C        = 0x%08x\r\n", (unsigned)Xil_In32(REG_VERSION));
    dbg_printf(" 0x00 PHASE_ERR  = %ld\r\n", (long)(int32_t)Xil_In32(REG_PHASE_ERR));
    dbg_printf(" 0x08 DAC_DATA   = %lu\r\n", (unsigned long)Xil_In32(REG_DAC_DATA));
    dbg_printf(" 0x0C TAP_PERIOD = %lu\r\n", (unsigned long)Xil_In32(REG_TDC_TAP_PERIOD));
    dbg_printf(" 0x10 TAP_OFFSET = %lu\r\n", (unsigned long)Xil_In32(REG_TDC_TAP_OFFSET));
    dbg_printf(" 0x14 OCXO_COUNT = %lu\r\n", (unsigned long)Xil_In32(REG_OCXO_COUNT));
    dbg_printf(" 0x18 OCXO_DELTA = %ld\r\n", (long)(int32_t)Xil_In32(REG_OCXO_DELTA));
    dbg_printf(" 0x1C CLK_COUNT  = %lu\r\n", (unsigned long)Xil_In32(REG_CLK_COUNT));
    dbg_printf(" 0x20 CLK_DELTA  = %ld\r\n", (long)(int32_t)Xil_In32(REG_CLK_DELTA));
    dbg_printf(" 0x24 DROP_COUNT = %lu\r\n", (unsigned long)Xil_In32(REG_DROP_COUNT));
    dbg_printf(" 0x28 BYTE_COUNT = %lu\r\n", (unsigned long)Xil_In32(REG_BYTE_COUNT));
    dbg_printf(" 0x2C STATUS     = 0x%08x\r\n", (unsigned)st);
    dbg_printf("      meas=%d locked=%d pps_lost=%d tdc_sat=%d tap_max=%d frame_err=%d\r\n",
               (int)((st & STATUS_MEAS_VALID) ? 1 : 0),
               (int)((st & STATUS_LOCKED) ? 1 : 0),
               (int)((st & STATUS_PPS_LOST) ? 1 : 0),
               (int)((st & STATUS_TDC_SAT) ? 1 : 0),
               (int)((st & STATUS_TAP_MAX) ? 1 : 0),
               (int)((st & STATUS_FRAME_ERR) ? 1 : 0));
    dbg_printf(" 0x30 TDC_TAP    = %lu\r\n", (unsigned long)Xil_In32(REG_TDC_TAP));
    dbg_printf(" 0x50 P2_STATUS  = 0x%08x\r\n", (unsigned)Xil_In32(REG_P2_STATUS));
    dbg_printf("================================\r\n");
}

//-----------------------------------------------------------------------------
// 开环测频：DAC 钉死，采 n 点
//   主用 0x18 OCXO_DELTA（量纲无歧义：1 个计数 = 1 个 OCXO 边沿/秒 = 100 ppb）
//   同时记 0x00 的平均值，做量纲交叉验证
//-----------------------------------------------------------------------------
static void hw_measure(int dac, int n, const char *tag,
                       double *out_delta, double *out_phase, int *out_bad)
{
    int    i, bad = 0;
    double sd = 0.0, sp = 0.0;

    hw_dac(dac);
    dbg_printf("\r\n[%s] DAC = %d，等 %d s 稳定…\r\n", tag, dac, HT_POL_SETTLE);
    for (i = 0; i < HT_POL_SETTLE; i++) { hw_wait_tick(); }

    dbg_printf("[%s] 开始采样 %d 点：\r\n", tag, n);
    for (i = 0; i < n; i++) {
        hw_wait_tick();
        if (bsp_meas_valid() == 0) { bad++; }
        sd += (double)(int32_t)Xil_In32(REG_OCXO_DELTA);
        sp += (double)(int32_t)Xil_In32(REG_PHASE_ERR);
        if (((i + 1) % 10) == 0) {
            dbg_printf("   %2d/%d  0x18=%ld  0x00=%ld\r\n",
                       i + 1, n,
                       (long)(int32_t)Xil_In32(REG_OCXO_DELTA),
                       (long)(int32_t)Xil_In32(REG_PHASE_ERR));
        }
    }
    *out_delta = sd / (double)n;
    *out_phase = sp / (double)n;
    *out_bad   = bad;
    dbg_printf("[%s] 平均: 0x18=", tag);
    hw_fix(*out_delta, 3, 1000);
    dbg_printf(" 个计数   0x00=");
    hw_fix(*out_phase, 3, 1000);
    dbg_printf(" raw   meas 无效 %d/%d\r\n", bad, n);
}

//-----------------------------------------------------------------------------
// 极性 + 灵敏度
//-----------------------------------------------------------------------------
static void hwtest_polarity(void)
{
    double dA, dB, phA, phB, gc, gppb, ghz, gscale;
    int    badA, badB;

    dbg_printf("\r\n########## 极性 / 灵敏度标定 ##########\r\n");
    dbg_printf("两点法：DAC %d 与 %d，各采 %d 点\r\n", HT_DAC_A, HT_DAC_B, HT_POL_SAMPLES);
    dbg_printf("开环：DAC 钉死，不调用 dac_update()\r\n");

    hw_measure(HT_DAC_A, HT_POL_SAMPLES, "低点", &dA, &phA, &badA);
    hw_measure(HT_DAC_B, HT_POL_SAMPLES, "高点", &dB, &phB, &badB);

    if ((badA > 0) || (badB > 0)) {
        dbg_printf("\r\n!! 有 %d/%d 个点 meas_valid=0，数据不可信\r\n", badA + badB, HT_POL_SAMPLES * 2);
        dbg_printf("!! 先确认：1PPS 到了吗（0x2C pps_lost）？OCXO 接了吗（0x14 应≈2e7）？\r\n");
        return;
    }

    gc  = (dB - dA) / (double)(HT_DAC_B - HT_DAC_A);   /* 计数/码 */
    gppb = gc * 50.0;                                   /* ppb/码 (1 计数 = 50 ppb @20MHz) */
    ghz  = gppb * 0.02;                                 /* Hz/码（20 MHz 基准） */
    gscale = (gppb < 0.0) ? -gppb : gppb;

    dbg_printf("\r\n---------- 结果 ----------\r\n");
    dbg_printf("0x18 @ %d = ", HT_DAC_A); hw_fix(dA, 3, 1000); dbg_printf(" 计数\r\n");
    dbg_printf("0x18 @ %d = ", HT_DAC_B); hw_fix(dB, 3, 1000); dbg_printf(" 计数\r\n");
    dbg_printf("斜率      = "); hw_fix(gc, 6, 1000000); dbg_printf(" 计数/码\r\n");
    dbg_printf("灵敏度 g  = "); hw_fix(gppb, 5, 100000); dbg_printf(" ppb/码\r\n");
    dbg_printf("          = "); hw_fix(ghz, 7, 20000000); dbg_printf(" Hz/码  (20 MHz 基准)\r\n");
    dbg_printf("极性      = %s\r\n", (gc > 0.0) ? "正极性（DAC↑ -> 频率↑）" : "负极性（DAC↑ -> 频率↓）");

    dbg_printf("\r\n---- 填回 control.h ----\r\n");
    dbg_printf("#define DAC_GAIN   "); hw_fix(gscale, 5, 100000); dbg_printf("   /* = |g| */\r\n");
    dbg_printf("#define DAC_SCALE  "); hw_fix(gscale * 4.0, 5, 100000); dbg_printf("   /* = 4 x |g| */\r\n");
    dbg_printf("#define LOOP_SIGN_DEFAULT   (%+d)\r\n", (gc > 0.0) ? 1 : -1);

    dbg_printf("\r\n---- 量纲交叉验证 ----\r\n");
    dbg_printf("0x00 平均: 低="); hw_fix(phA, 1, 10); dbg_printf("  高="); hw_fix(phB, 1, 10); dbg_printf(" raw\r\n");
    dbg_printf("若 0x00/256  ≈ 0x18 -> 1 raw LSB = 0.1953 ppb（bsp.h 应为 0.1953125）\r\n");
    dbg_printf("若 0x00/2560 ≈ 0x18 -> 1 raw LSB = 3.9 ppb（说明 coarse 项没左移 8 位）\r\n");
    dbg_printf("本次: 0x00/256="); hw_fix(phA / 256.0, 2, 100); dbg_printf(" vs 0x18="); hw_fix(dA, 2, 100);
    dbg_printf("   %s\r\n", (fabs(phA / 256.0 - dA) < 2.0) ? "-> 前者成立" : "-> 需人工判断");

    hw_dac(HT_DAC_MID);
    dbg_printf("\r\nDAC 已恢复到 %d。\r\n", HT_DAC_MID);
}

//-----------------------------------------------------------------------------
// TDC 标定：靠相位自然漂移，找回绕跳变反推「每周期抽头数」
//-----------------------------------------------------------------------------
static void hwtest_tdc(void)
{
    int      i, n = HT_TDC_SAMPLES, wraps = 0;
    uint32_t tap[320];
    int32_t  d[320];
    int32_t  dmax = 0, step = 0;
    uint32_t tmin = 0xFFFFFFFFu, tmax = 0;
    int      period_est;
    int32_t  sorted[320];
    int      m = 0;

    dbg_printf("\r\n########## TDC 标定（0x0C / 0x10）##########\r\n");
    dbg_printf("当前 0x0C = %lu, 0x10 = %lu\r\n",
               (unsigned long)Xil_In32(REG_TDC_TAP_PERIOD),
               (unsigned long)Xil_In32(REG_TDC_TAP_OFFSET));
    /* WHY NOT HT_DAC_A (1024):
     *   Pinning the DAC far from the null point leaves a big frequency offset.
     *   Measured 2026-10-25: DAC=1024 gives about -348 ppb, so the phase walks
     *   348 ns per second = 35 whole periods per second. Sampled once a second
     *   the tap value lands on a random position, no wrap is ever seen, and
     *   "taps per period" can only be guessed from the observed span.
     *
     *   Pinning at the current loop output keeps the offset near zero (a few
     *   tenths of a ppb), so the phase walks a few ns per MINUTE and the wrap
     *   from the line end back to 0 becomes directly observable -- the only way
     *   to measure tap_period properly.
     */
    {
        int park = control_get_last_dac();
        dbg_printf("\r\n准备：DAC 钉在 %d（当前锁定点），相位慢漂；采 %d 点\r\n", park, n);
        dbg_printf("提示：偏差 0.2 ppb 时每秒走 0.2 ns，绕完 10 ns 周期约需 50 s\r\n");
        hw_dac(park);
    }
    if (n > 300) { n = 300; }

    for (i = 0; i < n; i++) {
        hw_wait_tick();
        tap[i] = Xil_In32(REG_TDC_TAP) & 0xFFFFu;
        if (tap[i] < tmin) { tmin = tap[i]; }
        if (tap[i] > tmax) { tmax = tap[i]; }
        if (((i + 1) % 10) == 0) {
            dbg_printf("   %2d/%d  tap=%lu  sat=%d\r\n", i + 1, n,
                       (unsigned long)tap[i],
                       (int)((Xil_In32(REG_STATUS) & STATUS_TDC_SAT) ? 1 : 0));
        }
    }

    for (i = 1; i < n; i++) { d[i] = (int32_t)tap[i] - (int32_t)tap[i - 1]; }

    /* 取中位数作「每拍正常步长」，最大跳变视作回绕 */
    for (i = 1; i < n; i++) { sorted[m++] = (int32_t)((d[i] < 0) ? -d[i] : d[i]); }
    for (i = 1; i < m; i++) {
        int32_t v = sorted[i], j = i - 1;
        while ((j >= 0) && (sorted[j] > v)) { sorted[j + 1] = sorted[j]; j--; }
        sorted[j + 1] = v;
    }
    step = (m > 4) ? sorted[m / 4] : 0;          /* 下四分位 ≈ 正常步长 */

    for (i = 1; i < n; i++) {
        int32_t ad = (int32_t)((d[i] < 0) ? -d[i] : d[i]);
        if (ad > dmax) { dmax = ad; }
        if (ad > 10 * step + 8) { wraps++; }
    }

    dbg_printf("\r\n---------- 统计 ----------\r\n");
    dbg_printf("tap 范围 %lu .. %lu （跨度 %lu）\r\n",
               (unsigned long)tmin, (unsigned long)tmax, (unsigned long)(tmax - tmin));
    dbg_printf("正常步长(下四分位) = %ld 抽头/秒\r\n", (long)step);
    dbg_printf("最大相邻跳变 = %ld，检测到回绕 %d 次\r\n", (long)dmax, wraps);

    if ((wraps == 0) && ((tmax - tmin) < 32u)) {
        dbg_printf("\r\n!! 没看到相位漂移，无法标定。请依次确认：\r\n");
        dbg_printf("!! 1) 0x2C 的 pps_lost 是否为 0（1PPS 到了吗）\r\n");
        dbg_printf("!! 2) 0x30 是否在变化（不变 = 没有新边沿）\r\n");
        dbg_printf("!! 3) OCXO 与标称的频差是否太小（漂太慢，加长采样时间）\r\n");
        hw_dac(HT_DAC_MID);
        return;
    }

    /* WHY THERE IS NO "+ step" HERE (removed 2026-10-25)
     *
     *   The old form added one nominal step to the observed span. There is no
     *   physical reason for that: when the phase aliases (even a 0.2 ppb offset
     *   moves it 0.2 ns per second against a 10 ns period, and a 1752-code park
     *   is ~6 ppb off the null, i.e. 0.67 periods per sample) the 300 samples
     *   cover [0, T) essentially uniformly, so
     *
     *       span = max - min  ~=  (n-1)/n * T
     *
     *   Two independent runs (DAC=1024 and DAC=1752) both gave span = 400,
     *   which puts T near 403. The old formula reported 461 -- 14% high, which
     *   scales the whole fine_fraction by 14% and costs 1.4 ns, i.e. 60x the
     *   22 ps tap resolution it was supposed to deliver.
     */
    if (wraps > 0) { period_est = (int)dmax; }
    else           { period_est = (int)(tmax - tmin); }
    if ((period_est > 0) && (n > 1)) {
        period_est = (period_est * (n + 1)) / n;      /* undo the (n-1)/n bias */
    }

    if (period_est < 16)   { period_est = 16; }
    if (period_est > 4000) { period_est = 4000; }

    dbg_printf("\r\n估计「每 10 ns 周期 = %d 个抽头」-> 每抽头约 ", period_est);
    hw_fix(10000.0 / (double)period_est, 1, 10);
    dbg_printf(" ps\r\n");

    Xil_Out32(REG_TDC_TAP_PERIOD, (uint32_t)period_est);
    Xil_Out32(REG_TDC_TAP_OFFSET, 0u);
    dbg_printf("已写入 0x0C = %d, 0x10 = 0\r\n", period_est);

    /* 复核：看是否还饱和 */
    {
        int satcnt = 0, tapmaxcnt = 0;
        for (i = 0; i < HT_VERIFY_SAMPLES; i++) {
            uint32_t st;
            hw_wait_tick();
            st = Xil_In32(REG_STATUS);
            if (st & STATUS_TDC_SAT) { satcnt++; }
            if (st & STATUS_TAP_MAX) { tapmaxcnt++; }
        }
        dbg_printf("\r\n复核 %d 点：tdc_sat=%d  tap_max=%d\r\n", HT_VERIFY_SAMPLES, satcnt, tapmaxcnt);
        if ((satcnt > 0) || (tapmaxcnt > 0)) {
            dbg_printf("!! 仍饱和 -> 说明 period_est 偏小。建议把 0x0C 调到 %d 左右再复核\r\n",
                       period_est + (period_est / 4));
        } else {
            dbg_printf("OK：未饱和。0x10 保持 0 即可；若 fine 读数恒在低位，再逐步加大 0x10。\r\n");
        }
    }

    hw_dac(HT_DAC_MID);
    dbg_printf("DAC 已恢复到 %d。\r\n", HT_DAC_MID);
}

//-----------------------------------------------------------------------------
// 入口
//-----------------------------------------------------------------------------
void hwtest_run(int mode)
{
    dbg_printf("\r\n\r\n############################################\r\n");
    dbg_printf("##  上板硬件标定  (mode = %d)\r\n", mode);
    dbg_printf("##  1=dump  2=极性  3=TDC  9=全部\r\n");
    dbg_printf("############################################\r\n");

    if ((mode == HWTEST_DUMP) || (mode == HWTEST_ALL)) { hwtest_dump(); }

    if ((mode == HWTEST_POLARITY) || (mode == HWTEST_ALL)) { hwtest_polarity(); }

    if ((mode == HWTEST_TDC) || (mode == HWTEST_ALL)) {
        dbg_printf("\r\n按 KEY1 可随时重跑 TDC；否则等采样结束…\r\n");
        hwtest_tdc();
    }

    dbg_printf("\r\n########## 测试结束 ##########\r\n");
    dbg_printf("按 KEY1 重新跑一遍，或改 TEST_MODE 重烧。\r\n");
}
