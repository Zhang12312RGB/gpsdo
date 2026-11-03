#include "adaptive_r.h"
#include "kalman.h"        /* 读 kalman.Pm[0] 做 S→R 修正 */
#include "dbg_uart.h"
#include <math.h>
#include <stdlib.h>

AdaptiveR ar;              /* 全局实例 */

/* 内部函数声明 */
static void   ar_push_phase(double p);
static double ar_phase_variance(void);
static int    ar_solve3(double A[3][3], double b[3], double x[3]);

/* 初始化: 人工标定初值 */
void adaptive_r_init(void)
{
    /* 2026-10-25：d 由 150 改成 50。
     * 原值是 TDC 失效时代（相位量化 50 ns）的估值。
     * TDC 修好后实测 pps 标准差只有 5.00 ns，对应 R = 25 ns²；
     * 按典型 GNSS 条件（hdop 1.11 / sat 8.5 / cn0 32.1）代入：
     *     80*1.11 - 15*8.5 + 400/32.1 + d = 88.8 - 127.0 + 12.5 + d = -25.7 + d
     * d = 50 时 R = 24.3 ns² -> sigma = 4.9 ns，与实测 5.00 ns 吻合。
     * d = 150 时 R = 124 ns² -> sigma = 11.1 ns，偏大 5 倍，
     * 导致卡尔曼增益偏小、X[1] 的开机暂态 20 分钟都没衰减完。 */
    ar.a = 80.0;
    ar.b = -15.0;
    ar.c = 400.0;
    ar.d = 50.0;
    adaptive_r_reset();
}

/* 清空数据与消抖状态, 保留系数 */
void adaptive_r_reset(void)
{
    int i;

    ar.n = 0;
    for (i = 0; i < ADAPT_POINTS; i++) {
        ar.pts[i].hdop   = 0.0;
        ar.pts[i].sat    = 0.0;
        ar.pts[i].cn0inv = 0.0;
        ar.pts[i].var    = 0.0;
    }
    for (i = 0; i < PHASE_WIN; i++) {
        ar.phase_win[i] = 0.0;
    }
    ar.win_cnt    = 0;
    ar.win_idx    = 0;
    ar.have_cur   = 0;
    ar.cur_hdop   = 0.0;
    ar.cur_sat    = 0;
    ar.cur_cn0    = 0.0;
    ar.cand_hdop  = 0;
    ar.cand_sat   = 0;
    ar.cand_cn0   = 0;
    ar.stable_cnt = 0;
    ar.var_pre    = 0.0;
    ar.var_valid  = 0;
    ar.ready      = 0;
}

/* 新息样本入窗 */
static void ar_push_phase(double p)
{
    ar.phase_win[ar.win_idx] = p;
    ar.win_idx = (ar.win_idx + 1) % PHASE_WIN;
    if (ar.win_cnt < PHASE_WIN) ar.win_cnt++;
}

/* 窗口内新息方差 */
static double ar_phase_variance(void)
{
    int    i;
    double mean = 0.0, var = 0.0;

    if (ar.win_cnt < 2) return 0.0;
    for (i = 0; i < ar.win_cnt; i++) mean += ar.phase_win[i];
    mean /= (double)ar.win_cnt;
    for (i = 0; i < ar.win_cnt; i++) {
        double dv = ar.phase_win[i] - mean;
        var += dv * dv;
    }
    return var / (double)ar.win_cnt;
}

/* phase 传新息 diff = z - Xm[0], 不是原始相位差 */
void adaptive_r_update(double hdop, int sat, double cn0, double phase)
{
    int t_h, t_s, t_c, trig, others_ok, stable;

    ar_push_phase(phase);

    /* 首次建立基准 */
    if (!ar.have_cur) {
        ar.cur_hdop  = hdop;
        ar.cur_sat   = sat;
        ar.cur_cn0   = cn0;
        ar.have_cur  = 1;
        ar.cand_hdop = ar.cand_sat = ar.cand_cn0 = 0;
        ar.stable_cnt = 0;
        ar.var_valid  = 0;
        return;
    }

    /* ---- 1. 相对"基准"连续越阈值计数(按键同款消抖思路) ---- */
    if (fabs(hdop - ar.cur_hdop) > HDOP_THRESH) ar.cand_hdop++; else ar.cand_hdop = 0;
    if (abs(sat   - ar.cur_sat)  >= SAT_THRESH) ar.cand_sat++;  else ar.cand_sat  = 0;
    if (fabs(cn0  - ar.cur_cn0)  > CN0_THRESH)  ar.cand_cn0++;  else ar.cand_cn0  = 0;

    stable = (ar.cand_hdop == 0) && (ar.cand_sat == 0) && (ar.cand_cn0 == 0);

    /* ---- 2. 非稳定期样本不进窗口, 保证变化前方差干净 ---- */
    if (!stable) {
        ar.win_cnt = 0;
        ar.win_idx = 0;
    }

    /* ---- 3. 稳定窗口快照: 扣掉 Pm[0], 得到 R 的估计 ---- */
    if (stable) {
        if (ar.stable_cnt < 0x7FFFFFFF) ar.stable_cnt++;
        if (ar.stable_cnt >= SETTLE_N) {
            double v = ar_phase_variance() - kalman.Pm[0];
            if (v < R_MIN) v = R_MIN;
            ar.var_pre   = v;
            ar.var_valid = 1;
        }
    } else {
        ar.stable_cnt = 0;
    }

    /* ---- 4. 恰好一个变量连续越阈 DEBOUNCE_N 拍才确认 ---- */
    t_h = (ar.cand_hdop >= DEBOUNCE_N);
    t_s = (ar.cand_sat  >= DEBOUNCE_N);
    t_c = (ar.cand_cn0  >= DEBOUNCE_N);
    trig = t_h + t_s + t_c;

    others_ok = 0;
    if (trig == 1) {
        if (t_h)      others_ok = (ar.cand_sat == 0) && (ar.cand_cn0 == 0);
        else if (t_s) others_ok = (ar.cand_hdop == 0) && (ar.cand_cn0 == 0);
        else          others_ok = (ar.cand_hdop == 0) && (ar.cand_sat == 0);
    }

    /* 记"变化前工作点 + 变化前方差" */
    if ((trig == 1) && others_ok && ar.var_valid && (ar.n < ADAPT_POINTS)) {
        ar.pts[ar.n].hdop   = ar.cur_hdop;
        ar.pts[ar.n].sat    = (double)ar.cur_sat;
        ar.pts[ar.n].cn0inv = (ar.cur_cn0 > 1.0) ? (1.0 / ar.cur_cn0) : 1.0;
        ar.pts[ar.n].var    = ar.var_pre;
        ar.n++;
    }

    /* ---- 5. 确认(或丢弃)后才移动基准, 否则计数累不起来 ---- */
    if (trig >= 1) {
        ar.cur_hdop   = hdop;
        ar.cur_sat    = sat;
        ar.cur_cn0    = cn0;
        ar.cand_hdop  = ar.cand_sat = ar.cand_cn0 = 0;
        ar.stable_cnt = 0;
        ar.var_valid  = 0;
    }

    if (ar.n >= ADAPT_POINTS) {
        ar.ready = 1;
        adaptive_r_fit();
    }
}

/* ============================================================================
 * 2026-10-30 固定 R 对照实验
 * ----------------------------------------------------------------------------
 * 原公式: R = a*HDOP + b*Sat + c/C_N0 + d，限幅 10~50000
 *   实测该公式会把 R 推到负值再钳到下限 10
 *   （v11 实例: 80*0.89 - 15*9 + 400/31.9 + 50 = -1.2 -> 10），
 *   使新息门限 5*sqrt(S) 突然收紧，触发 5sigma 保护误清窗。
 *
 * 离线重放（保持实测 DAC 轨迹）：
 *   原公式   全程 mean(z^2) = 3.71  (v11 闭环)
 *   R=50                  = 1.59
 *   R=80                  = 1.00    <- 最匹配
 *   R=100                 = 0.80
 *
 * 本实验把 R 固定为 80，绕过 ar.a/b/c/d，
 * 消除输入字段变化造成的 R 骤降。
 * adaptive_r_fit() 仍会运行（打印 ADAPT R fit），但不再影响 R 输出。
 *
 * 回滚: 恢复为下面的原公式；备份在 history/src_20261030_1720_before_R80/
 * ========================================================================== */
double adaptive_r_get(double hdop, int sat, double cn0)
{
    (void)hdop;
    (void)sat;
    (void)cn0;
    return ADAPTIVE_R_FIXED_NS2;       /* final operational R, ns^2 */
}

/* 3x3 列主元高斯消元, 病态返回 0 */
static int ar_solve3(double A[3][3], double b[3], double x[3])
{
    int i, j, k;

    for (k = 0; k < 3; k++) {
        int piv = k;
        for (i = k + 1; i < 3; i++) {
            if (fabs(A[i][k]) > fabs(A[piv][k])) piv = i;
        }
        if (fabs(A[piv][k]) < 1e-12) return 0;
        if (piv != k) {
            for (j = 0; j < 3; j++) {
                double t = A[k][j]; A[k][j] = A[piv][j]; A[piv][j] = t;
            }
            { double t = b[k]; b[k] = b[piv]; b[piv] = t; }
        }
        for (i = k + 1; i < 3; i++) {
            double f = A[i][k] / A[k][k];
            for (j = k; j < 3; j++) A[i][j] -= f * A[k][j];
            b[i] -= f * b[k];
        }
    }
    for (i = 2; i >= 0; i--) {
        double s = b[i];
        for (j = i + 1; j < 3; j++) s -= A[i][j] * x[j];
        x[i] = s / A[i][i];
    }
    return 1;
}

/* 联合最小二乘(中心化)拟合 a,b,c,d; 跳变>300% 拒绝 */
void adaptive_r_fit(void)
{
    double mx1 = 0, mx2 = 0, mx3 = 0, my = 0;
    double S[3][3], T[3], sol[3];
    double a_new, b_new, c_new, d_new;
    int    i, j, k, reject = 0;

    if (ar.n < ADAPT_POINTS) return;

    /* 均值 */
    for (i = 0; i < ar.n; i++) {
        mx1 += ar.pts[i].hdop;
        mx2 += ar.pts[i].sat;
        mx3 += ar.pts[i].cn0inv;
        my  += ar.pts[i].var;
    }
    mx1 /= ar.n; mx2 /= ar.n; mx3 /= ar.n; my /= ar.n;

    for (j = 0; j < 3; j++) { T[j] = 0; for (k = 0; k < 3; k++) S[j][k] = 0; }

    /* 中心化设计矩阵: Σ(xi-mx)(xj-mx), Σ(xi-mx)(y-my) */
    for (i = 0; i < ar.n; i++) {
        double dx[3], dy;
        dx[0] = ar.pts[i].hdop   - mx1;
        dx[1] = ar.pts[i].sat    - mx2;
        dx[2] = ar.pts[i].cn0inv - mx3;
        dy    = ar.pts[i].var    - my;
        for (j = 0; j < 3; j++) {
            T[j] += dx[j] * dy;
            for (k = 0; k < 3; k++) S[j][k] += dx[j] * dx[k];
        }
    }

    if (!ar_solve3(S, T, sol)) {
        dbg_printf("ADAPT R fit: ill-conditioned, discard\r\n");
        ar.n = 0;
        return;
    }
    a_new = sol[0];
    b_new = sol[1];
    c_new = sol[2];
    d_new = my - a_new * mx1 - b_new * mx2 - c_new * mx3;   /* 截距 */

    if ((fabs(ar.a) > 1e-9) && (fabs(a_new - ar.a) / fabs(ar.a) > JUMP_THRESH)) reject = 1;
    if ((fabs(ar.b) > 1e-9) && (fabs(b_new - ar.b) / fabs(ar.b) > JUMP_THRESH)) reject = 1;
    if ((fabs(ar.c) > 1e-9) && (fabs(c_new - ar.c) / fabs(ar.c) > JUMP_THRESH)) reject = 1;
    if ((fabs(ar.d) > 1e-9) && (fabs(d_new - ar.d) / fabs(ar.d) > JUMP_THRESH)) reject = 1;

    if (!reject) {
        ar.a = a_new; ar.b = b_new; ar.c = c_new; ar.d = d_new;
        dbg_printf("ADAPT R fit: a=%d b=%d c=%d d=%d (x1000)\r\n",
                   (int)(ar.a * 1000), (int)(ar.b * 1000),
                   (int)(ar.c * 1000), (int)(ar.d * 1000));
    } else {
        dbg_printf("ADAPT R fit rejected (jump > 300%%)\r\n");
    }

    ar.n = 0;   /* 重新收集 */
}