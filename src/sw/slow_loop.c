#include "slow_loop.h"
#include <math.h>
#include "bsp.h"
#include "kalman.h"
#include "control.h"
#include "main.h"
#include "dbg_uart.h"   /* dbg_printf: SLOWWIN 诊断行 */          /* main_phase_acc_ns: 已修正的累计相位 */
#include "xil_io.h"

/* BEGIN LIMITED C WEIGHT CORE: no hardware access or estimator mutation. */
void slow_c_weight_reset(SLOW_C_WEIGHT *s, int reason)
{
    if (s == 0) { return; }
    s->used = 0.0; s->stat = 0.0; s->target = 0.0;
    s->last_ms = 0U; s->have_time = 0; s->reason = reason;
}

double slow_c_weight_step(SLOW_C_WEIGHT *s, uint32_t now_ms,
                         double w_stat, double age_s, double lag_s,
                         int reject_reason)
{
    uint32_t dt_ms;
    double target, step;
    if (s == 0) { return 0.0; }
    if (reject_reason != SLOW_CW_OK) {
        slow_c_weight_reset(s, reject_reason); return 0.0;
    }
    if (!isfinite(w_stat) || !isfinite(age_s) || !isfinite(lag_s) ||
        w_stat < 0.0 || age_s < 0.0 || lag_s < 0.0) {
        slow_c_weight_reset(s, SLOW_CW_NUMERIC); return 0.0;
    }
    if (age_s > SLOW_C_MAX_AGE_S) {
        slow_c_weight_reset(s, SLOW_CW_AGE); return 0.0;
    }
    if (lag_s > SLOW_C_MAX_LAG_S) {
        slow_c_weight_reset(s, SLOW_CW_LAG); return 0.0;
    }
    if (w_stat > 1.0) { w_stat = 1.0; }
    target = SLOW_C_WEIGHT_SCALE * w_stat;
    if (target > SLOW_C_WEIGHT_MAX) { target = SLOW_C_WEIGHT_MAX; }
    s->stat = w_stat; s->target = target; s->reason = SLOW_CW_OK;
    if (!s->have_time) {
        s->used = 0.0; s->last_ms = now_ms; s->have_time = 1;
        return 0.0; /* first eligible update starts the ramp at zero */
    }
    dt_ms = now_ms - s->last_ms; /* allow normal uint32 timer rollover */
    s->last_ms = now_ms;
    if (dt_ms > SLOW_C_WEIGHT_MAX_DT_MS) {
        slow_c_weight_reset(s, SLOW_CW_CLOCK); return 0.0;
    }
    step = SLOW_C_WEIGHT_MAX / SLOW_C_WEIGHT_RISE_S * ((double)dt_ms * 0.001);
    if (s->used > target) {
        s->used = target; /* confidence decreases take effect immediately */
    } else {
        s->used += step;
        if (s->used > target) { s->used = target; }
    }
    return s->used;
}
/* END LIMITED C WEIGHT CORE */

SLOW_LOOP slow;

/* C is an independent shadow estimate. It never changes legacy slow.freq. */
void slow_c_reset_window(void)
{
    slow.c_npts = 0;
    slow.c_valid = 0;
    slow.c_f_ref = 0.0;
    slow.c_f_now = 0.0;
    slow.c_span_s = 0.0;
    slow.c_fit_end_s = 0.0;
    slow.c_center_s = 0.0;
}

void slow_c_init(uint32_t now_ms, int reference_dac, double signed_gain)
{
    slow_c_reset_window();
    slow.c_initialized = 1;
    slow.c_dac_ref = reference_dac;
    slow.c_dac_prev = reference_dac;
    slow.c_gain = signed_gain;
    slow.c_last_ms = now_ms;
    slow.c_elapsed_s = 0.0;
    slow.c_phase_ns = 0.0;
    slow.c_phi_ref = 0.0;
    slow.c_obs_control_ns = 0.0;
    slow.c_obs_s = 0.0;
    slow.c_obs_valid = 0;
    slow.c_have_seq = 0;
}

static void slow_c_advance(uint32_t now_ms)
{
    uint32_t delta;
    double dt;
    if (!slow.c_initialized) { return; }
    delta = now_ms - slow.c_last_ms;  /* unsigned arithmetic handles timer wrap */
    slow.c_last_ms = now_ms;
    if (delta > 0x7fffffffU) {
        slow_c_reset_window();
        slow.c_obs_valid = 0;
        return;
    }
    dt = (double)delta * 0.001;
    slow.c_phase_ns += slow.c_gain * (double)(slow.c_dac_prev - slow.c_dac_ref) * dt;
    slow.c_elapsed_s += dt;
}

void slow_c_dac_changed(uint32_t now_ms, int applied_dac)
{
#if SLOW_C_ENABLE
    if (!slow.c_initialized) { return; }
    slow_c_advance(now_ms);          /* integrate the OLD held code first */
    slow.c_dac_prev = applied_dac;   /* the new code applies only afterward */
    if (slow.c_valid) {
        slow.c_f_now = slow.c_f_ref + slow.c_gain * (double)(applied_dac - slow.c_dac_ref);
    }
#else
    (void)now_ms; (void)applied_dac;
#endif
}

void slow_c_observe(uint32_t now_ms, double phase_ns, int valid, uint32_t sequence)
{
#if SLOW_C_ENABLE
    int fresh = !slow.c_have_seq || (uint32_t)(sequence - slow.c_last_seq) == 1U;
    slow_c_advance(now_ms);
    slow.c_last_seq = sequence;
    slow.c_have_seq = 1;
    slow.c_obs_s = slow.c_elapsed_s;
    slow.c_obs_control_ns = slow.c_phase_ns;
    slow.c_phi_ref = phase_ns - slow.c_obs_control_ns;
    slow.c_obs_valid = slow.c_initialized && valid && fresh;
    if (!slow.c_obs_valid) {
        slow_c_reset_window();
        return;
    }
    /* After a gap, seed only a new valid observation, without resetting legacy. */
    if (slow.active && slow.c_npts == 0) { slow_c_sample(); }
#else
    (void)now_ms; (void)phase_ns; (void)valid; (void)sequence;
#endif
}

void slow_c_sample(void)
{
#if SLOW_C_ENABLE
    int k, capacity;
    double mean_t=0.0, mean_y=0.0, den=0.0, num=0.0, center_num=0.0;
    double origin, t, y;
    if (!slow.c_obs_valid) { return; }
    if (slow.c_npts && slow.c_obs_s <= slow.c_time[slow.c_npts - 1]) { return; }
    capacity = (int)(SLOW_T_MAX / (slow.active ? slow.T : SLOW_T_FIXED));
    if (capacity < 2) { capacity = 2; }
    if (capacity > SLOW_N_MAX) { capacity = SLOW_N_MAX; }
    if (slow.c_npts == capacity) {
        for (k=1; k<capacity; k++) {
            slow.c_time[k-1] = slow.c_time[k];
            slow.c_values[k-1] = slow.c_values[k];
        }
        slow.c_npts--;
    }
    slow.c_time[slow.c_npts] = slow.c_obs_s;
    slow.c_values[slow.c_npts] = slow.c_phi_ref;
    slow.c_npts++;
    if (slow.c_npts < 2) { return; }
    origin = slow.c_time[0];
    for (k=0; k<slow.c_npts; k++) {
        mean_t += slow.c_time[k] - origin;
        mean_y += slow.c_values[k];
    }
    mean_t /= (double)slow.c_npts;
    mean_y /= (double)slow.c_npts;
    for (k=0; k<slow.c_npts; k++) {
        t = slow.c_time[k] - origin;
        y = slow.c_values[k] - mean_y;
        den += (t - mean_t) * (t - mean_t);
        num += (t - mean_t) * y;
        center_num += (t - mean_t) * t * t;
    }
    if (den <= 0.0) { slow.c_valid = 0; return; }
    slow.c_f_ref = num / den;
    slow.c_span_s = slow.c_time[slow.c_npts-1] - origin;
    slow.c_fit_end_s = slow.c_obs_s;
    /* Effective center for a locally linear frequency drift, including uneven t. */
    slow.c_center_s = origin + center_num / (2.0 * den);
    slow.c_valid = 1;
    slow.c_f_now = slow.c_f_ref + slow.c_gain * (double)(slow.c_dac_prev - slow.c_dac_ref);
#endif
}

double slow_c_current(uint32_t now_ms, int applied_dac)
{
#if SLOW_C_ENABLE
    slow_c_dac_changed(now_ms, applied_dac);
    if (slow.c_valid) {
        slow.c_f_now = slow.c_f_ref + slow.c_gain * (double)(applied_dac - slow.c_dac_ref);
    }
#else
    (void)now_ms; (void)applied_dac;
#endif
    return slow.c_f_now;
}



//=============================================================================
// 牛顿迭代开方
//   工程链接里没有 -lm, 用 sqrt() 会链接失败; VSQRT 又依赖编译器是否肯生成
//   (默认 -fmath-errno 下 GCC 会改成库调用)。这里自己迭代, 1Hz 调用量下
//   60 次迭代的开销可以忽略, 而且结果对 1e-6 .. 1e6 的输入都能收敛。
//=============================================================================
double slow_sqrt(double x)
{
    double g;
    int    k;

    if (x <= 0.0) { return 0.0; }
    g = (x >= 1.0) ? x : 1.0;
    for (k = 0; k < 60; k++) {
        g = 0.5 * (g + x / g);
    }
    return g;
}

//=============================================================================
// T = floor(31.62 * X[1]^-1.5) = floor(31.62 / (X[1]*sqrt(X[1])))
//   X[1]=10ppb -> 1s; 1ppb -> 31.6s; 0.1ppb -> 1000s
//=============================================================================
uint32_t slow_calc_T(double x1_ppb)
{
    double t;

    if (x1_ppb <= 0.0) { return 65535U; }
    t = 31.62 / (x1_ppb * slow_sqrt(x1_ppb));
    if (t < (double)SLOW_T_MIN) { t = (double)SLOW_T_MIN; }
    if (t > 65535.0)            { t = 65535.0; }
    return (uint32_t)t;
}

// N 由 T 决定: 1..4s -> 30, 5..9s -> 6, 10..20s -> 3, >20s -> 2
static int slow_n_from_t(uint32_t T)
{
    if (T <= 4U)  { return 30; }
    if (T <= 9U)  { return 6;  }
    if (T <= 20U) { return 3;  }
    return 2;
}

void slow_init(void)
{
    int k;

    slow.T = SLOW_T_MIN;
    slow.N = SLOW_N_MAX;
    slow.cnt = 0;
    slow.phase = 0.0;
    slow.freq = 0.0;
    slow.freq_ocxo = 0.0;
    slow.active = 0;
    slow.done = 0;
    slow.samples = 0U;
    slow.sec_cnt = 0U;
    slow.npts    = 0;
    slow_c_reset_window();
    for (k = 0; k < SLOW_N_MAX; k++) { slow.X[k] = 0.0; }
    slow_c_init(bsp_millis(), control_get_last_dac(),
                DAC_GAIN * (double)control_get_loop_sign());
    bsp_p2_set_t((uint16_t)SLOW_T_MIN);
    (void)bsp_p2_take_new();
}

void slow_enter(double x1_ppb)
{
    int k;

    slow.active = 1;
    slow.done = 0;
    slow.cnt = 0;
    slow.samples = 0U;
    slow.sec_cnt = 0U;
    slow.npts    = 0;
    slow_c_reset_window();
    slow.phase   = main_phase_acc_ns;   /* 从当前绝对相位起步 */
    slow.freq = x1_ppb;      // 从第一路当前工作点起步, DAC 不会跳变
    slow.T = SLOW_T_FIXED;
    slow.N = slow_n_from_t(slow.T);

    /* 先把整个窗口清零，再种第 0 个点。
     * 顺序不能反：原来这里是「先种 X[0] 再清零」，结果是 X[0] 恒为 0，
     * 第一个 T 秒窗口算出的 freq = (φ(t) - 0)/T，比真值多出 φ(起点)/T。
     * 实测起点相位约 -2.9 ns、T = 300 s，对应 +0.16 ppb 的偏置
     * （日志 slow = +0.124，而按 pps 累加反算是 -0.037），
     * 而且这个零点会一直参与后续最小二乘，偏置不会自动消失。 */
    for (k = 0; k < SLOW_N_MAX; k++) { slow.X[k] = 0.0; }

    /* 种一个初始点：这样第一个 T 秒窗口到期就有 2 个点，
     * 加权融合在 T 秒后即可启用，而不是等 2T。 */
    slow.X[0]    = slow.phase;
    slow_c_sample();
    slow.npts    = 1;
    /* 必须和 slow.T 一致：PL 按这个秒数累加 phase_out，PS 按 slow.T 做除数，
     * 两边不一致会让慢环的频率估计差一个 T 的倍数。 */
    bsp_p2_set_t((uint16_t)slow.T);
    (void)bsp_p2_take_new();              // 丢掉可能残留的旧样本
}

void slow_abort(void)
{
    slow.active = 0;
    slow.done = 0;
    slow.T = SLOW_T_FIXED;
    slow.N = SLOW_N_MAX;
    slow.sec_cnt = 0U;
    slow.npts    = 0;
    slow_c_reset_window();
    (void)bsp_p2_take_new();   /* 丢掉 PL 里可能残留的 P2 样本 */
}

//=============================================================================
// 每收到一个 T 边界样本: 压栈 -> 算频率差 -> 调 DAC -> 更新下一轮 T
//=============================================================================
/* 环境变化检测触发：清空窗口重新累积，T 与 active 保持不变 */
void slow_reset_window(void)
{
    int k;
    slow.done    = 0;      /* 窗口不再满 */
    slow.npts    = 0;
    slow_c_reset_window();
    slow.sec_cnt = 0U;
    slow.phase   = main_phase_acc_ns;
    for (k = 0; k < SLOW_N_MAX; k++) { slow.X[k] = 0.0; }
}

void slow_tick(void)
{
    int     k, n;
    double  sx, sy, sxx, sxy, num, den, sl;

    if (slow.active == 0) { return; }
    /* 注意：done 只表示"基线已达 SLOW_T_MAX"，【不停止工作】。
     * 原设计里 done 会让 slow_tick 直接 return，slow.freq 就冻结在最后一次的值，
     * 而融合还在读它 —— 那是个隐患。现在 done 只是状态位。 */

    /* 每 1 秒被主循环调一次，攒够 T 秒才采一个点 */
    if (++slow.sec_cnt < slow.T) { return; }
    slow.sec_cnt = 0;

    /* ---- 关键：用 PS 侧【已经过 fix_50ns_glitch() 修正】的累计相位 ----
     * 原实现读 PL 的 0x48/0x4C（path2_measure 累加），那条路上没有 ±50 ns
     * 修正，粗计数 ±1 的毛刺会原样进慢环，实测把 frq 摆到 ±1.2 ppb。
     * 改成直接取 main_phase_acc_ns，和快环用同一份干净数据，
     * 这样"双环 vs 单环"的对比才是公平的。
     * ------------------------------------------------------------------ */
    slow.phase = main_phase_acc_ns;

    slow_c_sample();

    /* 基线长度上限 SLOW_T_MAX（1000 s）：
     *   nmax = SLOW_T_MAX / T，即最多存这么多个点，
     *   基线 = (nmax-1)*T 不会超过 SLOW_T_MAX。
     *   T=300 时 nmax=3，基线 600 s；T=250 时 nmax=4，基线 750 s。 */
    {
        int nmax = (int)(SLOW_T_MAX / slow.T);
        if (nmax < 2)            { nmax = 2; }
        if (nmax > SLOW_N_MAX)   { nmax = SLOW_N_MAX; }
        slow.N = nmax;

        if (slow.npts < nmax) {
            slow.X[slow.npts] = slow.phase;
            slow.npts++;
        } else {
            for (k = 1; k < nmax; k++) { slow.X[k - 1] = slow.X[k]; }
            slow.X[nmax - 1] = slow.phase;
        }
        /* 窗口已满：置状态位（不影响后续更新） */
        if (slow.npts >= nmax) { slow.done = 1; }
    }
    slow.samples++;

    if (slow.npts < 2) { return; }

    n = slow.npts;
    if (n >= 3) {
        /* 3 点以上做最小二乘（nmax=SLOW_T_MAX/T，T=300 时只有 3 个点，
         * 阈值取 6 的话这段永远用不到） */
        sx = sy = sxx = sxy = 0.0;
        for (k = 0; k < n; k++) {
            sx  += (double)k;
            sy  += slow.X[k];
            sxx += (double)k * (double)k;
            sxy += (double)k * slow.X[k];
        }
        num = (double)n * sxy - sx * sy;
        den = (double)n * sxx - sx * sx;
        sl  = (den != 0.0) ? (num / den) : 0.0;
        slow.freq = sl / (double)slow.T;

    } else {
        /* 2~5 点：首尾差 */
        slow.freq = (slow.X[n - 1] - slow.X[0]) / ((double)(n - 1) * (double)slow.T);
    }

    /* ---- 一次性诊断：把窗口首尾点和累加器一起打出来 ----------------
     * 用来核对 slow.freq 到底是怎么算出来的（窗口可能跨越了重置）。 */
    if ((slow.npts == 2) || (slow.npts == 3) || (slow.npts == 1)) {
        /* 全部用【毫单位整数】打印：%d.%03d 对 -1<v<0 会丢负号 */
        dbg_printf("SLOWWIN n=%d T=%u X0m=%d XNm=%d accm=%d psum=%d dphim=%d freqm=%d\r\n",
                   slow.npts, (unsigned)slow.T,
                   (int)(slow.X[0] * 1000.0),
                   (int)(slow.X[slow.npts - 1] * 1000.0),
                   (int)(main_phase_acc_ns * 1000.0),
                   (int)(main_p_sum * 1000.0),
                   (int)((slow.X[slow.npts - 1] - slow.X[0]) * 1000.0),
                   (int)(slow.freq * 1000.0));
    }

    /* 不再直接调 DAC：slow.freq 由 main.c 的加权融合读取。
     * 原 Route B 里这里调 control_bump_setpoint()，会和主环的积分律
     * 构成两个积分器，慢环的量被算两次。 */
}