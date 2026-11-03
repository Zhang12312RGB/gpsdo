#ifndef KALMAN_H
#define KALMAN_H

/* 2026-10-31: select one complete observer configuration.
 * BASELINE reproduces v19. Q1E4 matches the passive Q1e4 observer.
 * Override with -DKALMAN_PROFILE=0 for a baseline build, or change the
 * default below and rebuild. Candidate observers remain passive. */
#define KALMAN_PROFILE_BASELINE  0
#define KALMAN_PROFILE_Q1E4      1
#ifndef KALMAN_PROFILE
#define KALMAN_PROFILE KALMAN_PROFILE_Q1E4
#endif

/*-----------------------------------------------------------------------------
 * 过程噪声系数（单位：每秒钟的方差）
 *   KALMAN_Q_PHASE : 相位的随机游走强度   Q[0] = KALMAN_Q_PHASE * T
 *   KALMAN_Q_FREQ  : 频率的随机游走强度   Q[3] = KALMAN_Q_FREQ  * T
 *
 * 2026-10-25 改动：KALMAN_Q_FREQ 由 1e-10 提到 1e-6。
 * 依据：实测 X[1] 与真实频差符号相反、偏差 0.177 ppb，且 37 分钟都收不回来
 *       —— 时间常数超过 2000 s。对二阶跟踪器，时间常数约与 Q^(-1/3) 成比例，
 *       提 1e4 倍 => 时间常数缩到约 1/21，即 100 s 量级。
 *----------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------
 * 协方差经验下限与状态限幅
 *
 * KALMAN_T_EFF  : B基线的经验协方差膨胀参数，P3 >= R/T_eff^2。
 *                 这不是普适的硬件精度下界；仅BASELINE配置启用。
 * KALMAN_P3_MIN : B基线的绝对下限。Q1E4配置不钳制P3。
 * KALMAN_X1_MAX : 频率估计的合理范围，只防开机跳变等异常，修不了偏置。
 *----------------------------------------------------------------------------*/
#define KALMAN_T_EFF     160.0
#define KALMAN_P3_MIN    3e-4
/* 限幅取 50 ppb：
 *   · 5 ppb 太小 —— 真偏置 -46 ppb 时环路收不进来，DAC 只走 1 码（10-25 实测）
 *   · 1000 ppb 太大 —— 噪声 ±0.3 让环路剧烈响应，DAC 冲到 1855（103 码），
 *     频率摆动 ±49 ppb，反而把底层 50 ppb 量化噪声暴露出来（10-26 实测）
 *   · 50 ppb 对应 105 码，够 acquire 一个真实偏差，又不至于被噪声推飞
 * 真正的稳定靠慢环（300 s 最小二乘，已验证 0.0000 ppb），限幅只是兜底。 */
#define KALMAN_X1_MAX    50.0

#define KALMAN_Q_PHASE   1e-6
/* 2026-10-25 二次调整：1e-10 太小（X[1] 冻结，37 分钟收不回来），
 * 1e-6 太大（Allan@1s 由 0.032 劣化到 0.054）。取中间 2e-7：
 * 时间常数约 (1e-10/2e-7 的立方根关系) -> 2000 / 12.6 ≈ 160 s。 */
/* 2026-10-25 第三次调整：2e-7 -> 1e-5
 * 实测（R 已修正到 28 之后）：P3 稳态掉到 1e-4，sqrt 只有 0.010 ppb，
 * 滤波器过度自信 -> 卡尔曼增益几乎为 0 -> X[1] 冻在 +0.19（真实 +0.006），
 * 进而 X[0] 与 z 脱节 -> 新息超 3 sigma -> slow_abort 反复触发、n 归零。
 * 目标 P3 稳态 0.003~0.01（sigma 0.05~0.1 ppb）。P3 稳态约与 sqrt(Q) 成比例，
 * 从 1e-4 提到 0.005 需要 Q 提约 (0.005/1e-4)^2 = 2500 倍；
 * 取 50 倍（1e-5）先看，宁可偏慢也不要过度自信。 */
/* 2026-10-25 定稿：2e-7
 *
 * 扫参记录（同一套 R 修正 + 年龄衰减 + 5 sigma 中止门限）：
 *   Q = 1e-10 -> P3 冻结在 1e-4 以下，X[1] 完全不跟，符号都能反
 *   Q = 2e-7  -> P3 稳在 1e-4，Allan@1s = 0.0273  【迄今最好】
 *   Q = 1e-6  -> Allan@1s = 0.0544，偏吵
 *   Q = 1e-5  -> P3 只涨到 8e-4（提 50 倍 Q 只换 8 倍 P3），
 *                Allan@1s = 0.0443，更吵
 *
 * 结论：P3 稳态约与 Q^0.55 成比例，继续提 Q 换不到想要的 P3，
 * 只会让频率估计变吵。2e-7 是这条曲线上的最优点。 */
/* The preceding 2026-10-25 comments are historical tuning notes.
 * Active values are selected together here; do not change Q alone. */
#if KALMAN_PROFILE == KALMAN_PROFILE_BASELINE
#define KALMAN_Q_FREQ             2e-7
#define KALMAN_P3_FLOOR_ENABLE    1
#elif KALMAN_PROFILE == KALMAN_PROFILE_Q1E4
#define KALMAN_Q_FREQ             1e-4
#define KALMAN_P3_FLOOR_ENABLE    0
#else
#error "Unsupported KALMAN_PROFILE"
#endif

typedef struct _Kalman
{
    double X[2];      /* X[0] 相位差 ns, X[1] 频率差 ppb       */
    double Xm[2];     /* 预测状态                               */
    double P[4];      /* 协方差 [p00 p01 p10 p11]               */
    double Pm[4];     /* 预测协方差                             */
    double F[4];      /* 状态转移 [1 T; 0 1], 按 T 动态算       */
    double Q[4];      /* 过程噪声 diag(Q_PHASE*T, Q_FREQ*T) */
    double R;         /* 观测噪声                               */
    double K[2];      /* 卡尔曼增益                             */
    double S;         /* 新息协方差                             */
} Kalman;

extern Kalman kalman;  /* 定义在 kalman.c */

void kalman_init(Kalman *kf);
void kalman_update(Kalman *kf, double z, double T);
/* 只做预测、不做观测更新。无可信观测时调用。*/
void kalman_predict_only(Kalman *kf, double T);

#endif /* KALMAN_H */