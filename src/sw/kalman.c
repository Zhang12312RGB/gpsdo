#include "kalman.h"

Kalman kalman;   /* 全局实例 */

/* 初始化: X=[0,0], P=diag(1e6,1e3) */
void kalman_init(Kalman *kf)
{
    kf->X[0] = 0;      kf->X[1] = 0;
    kf->Xm[0] = 0;     kf->Xm[1] = 0;
    kf->P[0] = 1e6;    kf->P[1] = 0;
    kf->P[2] = 0;      kf->P[3] = 1e3;
    kf->Pm[0] = 0;     kf->Pm[1] = 0;
    kf->Pm[2] = 0;     kf->Pm[3] = 0;
    kf->F[0] = 1;      kf->F[1] = 0;
    kf->F[2] = 0;      kf->F[3] = 1;
    kf->Q[0] = 0;      kf->Q[1] = 0;
    kf->Q[2] = 0;      kf->Q[3] = 0;
    kf->R    = 100;    /* 初值, 之后每个 epoch 由 adaptive_r_get 覆盖 */
    kf->K[0] = 0;      kf->K[1] = 0;
    kf->S    = 1;
}

/* 一步预测 + 更新. z: 相位观测 ns; T: 采样周期 s (1/5/10) */
/*=============================================================================
 * 只做预测、不做观测更新（2026-10-30）
 *-----------------------------------------------------------------------------
 * 用途：本拍没有可信观测（缺测 / 坏样本 / 整窗口有缺口）时调用。
 *   · 状态按 F 推进：X0 += T*X1，X1 不变
 *   · 协方差按 F*P*F^T + Q 增长（不确定性变大，下次观测权重提高）
 *   · 不计算新息、不改 S/K
 *===========================================================================*/
void kalman_predict_only(Kalman *kf, double T)
{
    if (kf == 0) { return; }

    kf->F[0] = 1.0;    kf->F[1] = T;
    kf->F[2] = 0.0;    kf->F[3] = 1.0;

    kf->Q[0] = KALMAN_Q_PHASE * T;   kf->Q[1] = 0.0;
    kf->Q[2] = 0.0;                  kf->Q[3] = KALMAN_Q_FREQ * T;

    kf->Xm[0] = kf->X[0] + T * kf->X[1];
    kf->Xm[1] = kf->X[1];

    kf->Pm[0] = kf->P[0] + 2.0 * T * kf->P[2] + T * T * kf->P[3] + kf->Q[0];
    kf->Pm[1] = kf->P[1] + T * kf->P[3];
    kf->Pm[2] = kf->Pm[1];
    kf->Pm[3] = kf->P[3] + kf->Q[3];

    kf->X[0] = kf->Xm[0];
    kf->X[1] = kf->Xm[1];
    kf->P[0] = kf->Pm[0];
    kf->P[1] = kf->Pm[1];
    kf->P[2] = kf->Pm[2];
    kf->P[3] = kf->Pm[3];

    kf->S    = 0.0;
    kf->K[0] = 0.0;
    kf->K[1] = 0.0;
}

void kalman_update(Kalman *kf, double z, double T)
{
    double diff;

    /* F = [1 T; 0 1] —— 必须按 T 动态构造 */
    kf->F[0] = 1.0;    kf->F[1] = T;
    kf->F[2] = 0.0;    kf->F[3] = 1.0;

    /* Q = diag(KALMAN_Q_PHASE*T, KALMAN_Q_FREQ*T)，系数在 kalman.h */
    kf->Q[0] = KALMAN_Q_PHASE * T;   kf->Q[1] = 0.0;
    kf->Q[2] = 0.0;                  kf->Q[3] = KALMAN_Q_FREQ * T;

    /* 预测状态 Xm = F * X */
    kf->Xm[0] = kf->X[0] + T * kf->X[1];
    kf->Xm[1] = kf->X[1];

    /* 预测协方差 Pm = F*P*F^T + Q, T^2 项不能漏 */
    kf->Pm[0] = kf->P[0] + 2.0 * T * kf->P[2] + T * T * kf->P[3] + kf->Q[0];
    kf->Pm[1] = kf->P[1] + T * kf->P[3];
    kf->Pm[2] = kf->Pm[1];
    kf->Pm[3] = kf->P[3] + kf->Q[3];

    /* 新息 H=[1 0] */
    diff = z - kf->Xm[0];

    kf->S    = kf->Pm[0] + kf->R;
    kf->K[0] = kf->Pm[0] / kf->S;
    kf->K[1] = kf->Pm[2] / kf->S;

    /* 更新状态 */
    kf->X[0] = kf->Xm[0] + kf->K[0] * diff;
    kf->X[1] = kf->Xm[1] + kf->K[1] * diff;

    /* 更新协方差 P = (I - K*H) * Pm */
    kf->P[0] = (1 - kf->K[0]) * kf->Pm[0];
    kf->P[1] = (1 - kf->K[0]) * kf->Pm[1];
    kf->P[2] = -kf->K[1] * kf->Pm[0] + kf->Pm[2];
    kf->P[3] = -kf->K[1] * kf->Pm[1] + kf->Pm[3];

    /* ---- 旧B基线的经验下限（仅BASELINE配置启用）-------------------
     * 依据：sigma_f >= sigma_phi / T_eff  =>  P3 >= R / T_eff^2
     *   不设下限时 P3 会掉到 1e-4（sigma 0.010 ppb），比 5ns/160s 的
     *   物理下限还小 3 倍，卡尔曼增益趋 0，X[1] 冻结（实测 90% 的样本
     *   相邻值完全相同，停在 +0.19 而真值 -0.045）。
     *   随 R 动态的好处：GNSS 差(R 大)时允许更不确定，信号好(R 小)时
     *   收紧下限，比写死一个常数更符合物理。
     * KALMAN_P3_MIN 只是绝对兜底。
     * ---------------------------------------------------------------- */
#if KALMAN_P3_FLOOR_ENABLE
    {
        double p_floor = kf->R / (KALMAN_T_EFF * KALMAN_T_EFF);
        if (p_floor < KALMAN_P3_MIN) { p_floor = KALMAN_P3_MIN; }
        if (kf->P[3] < p_floor)      { kf->P[3] = p_floor; }
    }
#endif

    if (kf->X[1] >  KALMAN_X1_MAX) { kf->X[1] =  KALMAN_X1_MAX; }
    if (kf->X[1] < -KALMAN_X1_MAX) { kf->X[1] = -KALMAN_X1_MAX; }
}