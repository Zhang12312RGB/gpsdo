#include "control.h"
#include "slow_loop.h"
#include "main.h"
#include "bench.h"
#include "bsp.h"
#include "kalman.h"
#include "xil_io.h"
#include <math.h>

static int    last_dac = DAC_CENTER;              /* 最近写入的码值 */
static int    g_dac_center = DAC_CENTER;          /* 中心值(标定后可改)      */
static int    g_loop_sign  = LOOP_SIGN_DEFAULT;                  /* 环路基性: +1 / -1       */
static double dac_acc  = (double)DAC_CENTER;      /* 增量累加器(保留小数, 防死区) */

/* Report the command event after the original register write. The old held
 * DAC is integrated up to this timestamp; the new code applies afterward. */
static void control_write_dac(int code)
{
    Xil_Out32(REG_DAC_DATA, (uint32_t)code);
#if SLOW_C_ENABLE
    slow_c_dac_changed(bsp_millis(), code);
#endif
}

/* 初始化: DAC 写中心值 */
void control_init(void)
{
    g_loop_sign = LOOP_SIGN_DEFAULT;      /* 上电按默认极性 */
    last_dac = g_dac_center;
    dac_acc  = (double)g_dac_center;
    control_write_dac(last_dac);
}

//=============================================================================
// 增量式(积分)调 DAC —— 这条律是被台架测出来的, 换掉了原来的绝对式
//-----------------------------------------------------------------------------
// 原来的绝对式:  dac = DAC_CENTER - X1/DAC_SCALE
//   代进被控对象 true = bias + (dac-CENTER)*g, 得到闭环
//       true_{k+1} = bias - (g/DAC_SCALE) * X1_k
//   增益比 g/DAC_SCALE >= 1 就振荡, 越大越猛:
//       比值 1   -> 每拍 true -> bias - true, 等幅振荡
//       比值 1000-> 直接跑飞(台架上从 +100ppb 甩到 -83100ppb)
//   而且固定点残留 = bias/(1+g/DAC_SCALE), 增益小的时候收敛不到 0。
//   一句话: 绝对式既不能又稳又准。
//
// 增量式:  dac -= X1/DAC_SCALE
//   代进去得到 true_{k+1} = (1 - g/DAC_SCALE) * true_k
//   只要 0 < g/DAC_SCALE < 2 就收敛, 而且 g == DAC_SCALE 时【一拍到位】。
//   对真实硬件也宽容得多: DAC_SCALE 只要和实际灵敏度差不到 2 倍就稳。
//=============================================================================
//-----------------------------------------------------------------------------
// 增量式(积分)调 DAC —— 方向由 g_loop_sign 决定
//
// 约定（需要 PL 侧 phase_error.v 为「正 = 快」的版本，已重综合）：
//     0x00 PHASE_ERR 正 = OCXO 快（本地超前）  =>  X[1] 正 = 超前
//
// 所以：
//     g_loop_sign = +1 （g>0，抬高 DAC 让频率变快）：
//         超前(X1>0) -> 压低 DAC 把它拉回来 -> dac_acc -= X1/S
//     g_loop_sign = -1 （g<0，抬高 DAC 反而变慢）：
//         超前(X1>0) -> 抬高 DAC -> dac_acc += X1/S
//
// 闭环推导（f = 真实频率偏差, 正 = 快;  X1 ≈ +f）：
//     Δdac = -SIGN * X1 / S = -SIGN * f / S
//     Δf   = g * Δdac = -g*SIGN*f/S = -|g|*f/S        (因为 SIGN = sign(g))
//     f_{k+1} = f (1 - |g|/S)                          => 0 < |g|/S < 2 收敛
//-----------------------------------------------------------------------------
/* 2026-10-30：DAC 硬冻结开关。
 * 置 1 后 control_apply_freq() / control_bump_setpoint() 都直接返回，
 * 不写任何 DAC 寄存器。用于换天线等需要绝对保持工作点的实验。
 * 由 main.c 在 CALIB 期间置位，TUNING 期间清零。*/
static int s_dac_hard_lock = 0;

void control_set_dac_lock(int on) { s_dac_hard_lock = (on != 0) ? 1 : 0; }
int  control_get_dac_lock(void)   { return s_dac_hard_lock; }

void control_apply_freq(double freq_ppb)
{
    if (s_dac_hard_lock != 0) { return; }   /* 硬冻结：连累加器都不动 */

    dac_acc -= (double)g_loop_sign * freq_ppb / DAC_SCALE;

    /* 饱和时把累加器也钳住, 避免积分饱和(anti-windup) */
    if (dac_acc < (double)DAC_MIN) { dac_acc = (double)DAC_MIN; }
    if (dac_acc > (double)DAC_MAX) { dac_acc = (double)DAC_MAX; }

    last_dac = (int)(dac_acc + 0.5);
    control_write_dac(last_dac);
}

/*=============================================================================
 * 双环（Route B）—— 仅测试工程启用
 *===========================================================================*/
static double dac_sp = (double)DAC_CENTER;   /* 慢环积分出的 DAC 设定点 */

double control_get_setpoint(void) { return dac_sp; }

/* 慢环: 每 T_slow 秒推进一次设定点。这是系统里【唯一】的积分器。
 * 用和单环同一条增量式律，保证两版之间的可比性。 */
void control_bump_setpoint(double freq_ppb)
{
    dac_sp -= (double)g_loop_sign * freq_ppb / DAC_SCALE;

    if (dac_sp < (double)DAC_MIN) { dac_sp = (double)DAC_MIN; }
    if (dac_sp > (double)DAC_MAX) { dac_sp = (double)DAC_MAX; }

    dac_acc = dac_sp;      /* holdover / resume 仍按 dac_acc 的语义工作 */
}

/* 快环: 每秒在设定点上做比例修正，本身不含积分。
 *   dac = dac_sp - sign * X1 * Kp
 * Kp = 0.42 码/ppb = 0.2/g，相当于每秒修正 20% 的频率偏差，
 * 剩下的 80% 由慢环的积分在 T_slow 秒尺度上消除 —— 这就是经典双环分工。 */
void control_apply_prop(double freq_ppb)
{
    double v = dac_sp - (double)g_loop_sign * freq_ppb * DAC_KP;
    int    nd;

    if (v < (double)DAC_MIN) { v = (double)DAC_MIN; }
    if (v > (double)DAC_MAX) { v = (double)DAC_MAX; }

    nd = (int)(v + 0.5);
    last_dac = nd;
    control_write_dac(nd);
}

/* X[1]>0 本地快 -> DAC 减小; 限幅 0~4095, 保存 last_dac */
void dac_update(void)
{
    control_apply_freq(kalman.X[1]);   /* X[1] = 频率 ppb */
}

/* HOLDOVER: 重写 last_dac, 保持输出 */
void control_holdover_tick(void)
{
    control_write_dac(last_dac);
}

/* GNSS 恢复: 用观测相位重置 X[0], 重置 P, X[1] 保留 */
void control_resume_from_holdover(void)
{
    kalman.X[0] = main_phase_acc_ns;   /* 对齐到当前绝对相位 */

    /* 守时期间 X[1] 是自由外推的。如果它已经顶到限幅（说明失锁前
     * 状态就已经不可信），恢复后继续沿用会把 DAC 推向错误方向 ——
     * 实测拔天线后 X[1] 停在 -5.000 ppb（= KALMAN_X1_MAX）。
     * 这种情况直接清零，让滤波器重新学。 */
    if ((kalman.X[1] >=  (KALMAN_X1_MAX - 0.01)) ||
        (kalman.X[1] <= -(KALMAN_X1_MAX - 0.01))) {
        kalman.X[1] = 0.0;
    }

    kalman.P[0] = 1e4;
    kalman.P[1] = 0;
    kalman.P[2] = 0;
    kalman.P[3] = 1e2;
    dac_acc = (double)last_dac;   // 增量累加器对齐当前输出
}
 int control_get_last_dac(void)
        {
            return last_dac;
        }

/* 诊断用：积分累加器的小数值 */
double control_get_dac_acc(void)
{
    return dac_acc;
}

/*-----------------------------------------------------------------------------
 * 标定用：中心值 / 环路基性
 *----------------------------------------------------------------------------*/
int control_get_center(void)
{
    return g_dac_center;
}

void control_set_center(int code)
{
    if (code < DAC_MIN) { code = DAC_MIN; }
    if (code > DAC_MAX) { code = DAC_MAX; }
    g_dac_center = code;
}

int control_get_loop_sign(void)
{
    return g_loop_sign;
}

int control_set_loop_sign(int s)
{
    g_loop_sign = (s < 0) ? -1 : +1;      /* 只允许 +1 / -1 */
    return g_loop_sign;
}