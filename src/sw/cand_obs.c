#include "cand_obs.h"
#include "kalman.h"
#include "control.h"
#include <math.h>

/*=============================================================================
 * 候选观察器（2026-10-31，补齐版）
 *-----------------------------------------------------------------------------
 * 与主 KF 保持【同等信息】，只改 Q_FREQ，不加 P3 下限：
 *   · 每次【真实更新拍】喂同一个 z
 *   · 同步做 DAC 阶跃补偿（du = Δdac × DAC_GAIN × g_loop_sign）
 *   · 缺测时同步做 predict_only（与主路径一致）
 *   · 只在真实更新拍打日志（避免重复打印旧值）
 *
 * 硬约束：
 *   · 候选不写 DAC、不改保护、不清慢环窗口
 *   · 候选状态完全独立于主 kalman
 *===========================================================================*/

#define CAND_N  2

static const double s_q[CAND_N] = { 1.0e-4, 3.0e-4 };
static const char  *s_tag[CAND_N] = { "Q1e4", "Q3e4" };

static Kalman s_kf[CAND_N];
static int    s_prev_dac[CAND_N];     /* 各候选自己的 DAC 记账（实际都跟主路径同值） */
static double s_innov[CAND_N];
static double s_z[CAND_N];
static int    s_upd_cnt[CAND_N];

void cand_obs_init(void)
{
    int i;
    for (i = 0; i < CAND_N; i++) {
        kalman_init(&s_kf[i]);
        s_kf[i].R      = 80.0;              /* 与主路径一致 */
        s_prev_dac[i]  = control_get_last_dac();
        s_innov[i]     = 0.0;
        s_z[i]         = 0.0;
        s_upd_cnt[i]   = 0;
    }
}

/* 预测步（与 kalman_predict_only 等价，但 Q_FREQ 用候选值） */
static void cand_predict(int i, double T)
{
    Kalman *kf = &s_kf[i];

    kf->F[0] = 1.0;   kf->F[1] = T;
    kf->F[2] = 0.0;   kf->F[3] = 1.0;

    kf->Q[0] = KALMAN_Q_PHASE * T;   kf->Q[1] = 0.0;
    kf->Q[2] = 0.0;                  kf->Q[3] = s_q[i] * T;   /* ★ 候选 Q */

    kf->Xm[0] = kf->X[0] + T * kf->X[1];
    kf->Xm[1] = kf->X[1];

    kf->Pm[0] = kf->P[0] + 2.0*T*kf->P[2] + T*T*kf->P[3] + kf->Q[0];
    kf->Pm[1] = kf->P[1] + T * kf->P[3];
    kf->Pm[2] = kf->Pm[1];
    kf->Pm[3] = kf->P[3] + kf->Q[3];
}

/* ---- ① DAC 阶跃补偿（与主路径同一条式子）---- */
static void cand_apply_dac(int i)
{
    int cur = control_get_last_dac();
    double du = (double)(cur - s_prev_dac[i]) * DAC_GAIN
                * (double)control_get_loop_sign();
    if (du != 0.0) {
        s_kf[i].X[1] += du;       /* 频率：DAC 阶跃直接改变频率 */
        s_prev_dac[i] = cur;
    }
}

/* 真实更新拍：DAC 补偿 → 预测 → 观测更新 */
static void cand_update_one(int i, double z, double T)
{
    Kalman *kf = &s_kf[i];
    double diff;

    cand_apply_dac(i);            /* ① 先补 DAC 阶跃（必须在预测前）*/
    cand_predict(i, T);

    diff = z - kf->Xm[0];
    s_innov[i] = diff;
    s_z[i]     = z;

    kf->S    = kf->Pm[0] + kf->R;
    kf->K[0] = kf->Pm[0] / kf->S;
    kf->K[1] = kf->Pm[2] / kf->S;

    kf->X[0] = kf->Xm[0] + kf->K[0] * diff;
    kf->X[1] = kf->Xm[1] + kf->K[1] * diff;

    kf->P[0] = (1 - kf->K[0]) * kf->Pm[0];
    kf->P[1] = (1 - kf->K[0]) * kf->Pm[1];
    kf->P[2] = -kf->K[1] * kf->Pm[0] + kf->Pm[2];
    kf->P[3] = -kf->K[1] * kf->Pm[1] + kf->Pm[3];

    if (kf->X[1] >  KALMAN_X1_MAX) { kf->X[1] =  KALMAN_X1_MAX; }
    if (kf->X[1] < -KALMAN_X1_MAX) { kf->X[1] = -KALMAN_X1_MAX; }

    /* ⚠️ 不加 P3 下限 —— 候选要看的就是没有下限时 Q 的作用 */
    s_upd_cnt[i]++;
}

void cand_obs_update(double z, double T)
{
    int i;
    for (i = 0; i < CAND_N; i++) { cand_update_one(i, z, T); }
}

/* ---- ② 缺测时同步预测（与主路径 kalman_predict_only 一致）---- */
void cand_obs_predict_only(double T)
{
    int i;
    for (i = 0; i < CAND_N; i++) {
        cand_apply_dac(i);
        cand_predict(i, T);
        s_kf[i].X[0] = s_kf[i].Xm[0];
        s_kf[i].X[1] = s_kf[i].Xm[1];
        s_kf[i].P[0] = s_kf[i].Pm[0];
        s_kf[i].P[1] = s_kf[i].Pm[1];
        s_kf[i].P[2] = s_kf[i].Pm[2];
        s_kf[i].P[3] = s_kf[i].Pm[3];
        s_kf[i].S    = 0.0;
        s_kf[i].K[0] = 0.0;
        s_kf[i].K[1] = 0.0;
    }
}

double cand_obs_get_x1(int i)    { return (i>=0&&i<CAND_N) ? s_kf[i].X[1] : 0.0; }
double cand_obs_get_innov(int i) { return (i>=0&&i<CAND_N) ? s_innov[i]  : 0.0; }
double cand_obs_get_S(int i)     { return (i>=0&&i<CAND_N) ? s_kf[i].S    : 0.0; }
double cand_obs_get_P3(int i)    { return (i>=0&&i<CAND_N) ? s_kf[i].P[3] : 0.0; }
double cand_obs_get_K1(int i)    { return (i>=0&&i<CAND_N) ? s_kf[i].K[1] : 0.0; }
double cand_obs_get_q(int i)     { return (i>=0&&i<CAND_N) ? s_q[i]       : 0.0; }

extern void dbg_printf(const char *fmt, ...);

/* ---- ③ 只在真实更新拍打印（调用方保证）---- */
void cand_obs_log(void)
{
    int i;
    for (i = 0; i < CAND_N; i++) {
        dbg_printf("CAND %s n=%ld q=%ld z=%ld innov=%ld S=%ld P3u=%ld K1u=%ld x1=%ld\r\n",
                   s_tag[i],
                   (long)s_upd_cnt[i],
                   (long)(s_q[i] * 1e7),
                   (long)(s_z[i] * 1000.0),
                   (long)(s_innov[i] * 1000.0),
                   (long)(s_kf[i].S * 1000.0),
                   (long)(s_kf[i].P[3] * 1000000.0),
                   (long)(s_kf[i].K[1] * 1000000000.0),
                   (long)(s_kf[i].X[1] * 1000000.0));
    }
}
