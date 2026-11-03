#ifndef SLOW_LOOP_H
#define SLOW_LOOP_H

#include <stdint.h>

//=============================================================================
// 第二路(慢环)
//-----------------------------------------------------------------------------
// 与第一路(卡尔曼)的分工:
//   X[1] >= 10ppb : 第一路掌握 DAC, 正常卡尔曼 + 自适应 R(按键生效)
//   X[1] <  10ppb : 第二路接管 DAC, 用 T 秒长窗口测相位/频率; 第一路仍在跑,
//                   但只监视新息: |innov| > 3*sqrt(R) 就强行终止第二路, T 回 1
//   T > 1000s     : 认为调整完成, DAC 冻结
//
// T 的来源: T = floor(31.62 * |freq2|^-1.5), freq2 是【第二路自己算出的频率差】。
// 这是一个闭环, slow_loop.c 里有样本数/符号/每拍翻倍三道闸防止发散。
//=============================================================================

#define SLOW_N_MAX      30      // X[30]
#define SLOW_X1_GATE    10.0     // 基础门限（启动 10 分钟后自动放宽，见 main.c）
                                 // 慢环不依赖快环精度（直接对相位做 300 s 最小二乘），放宽无害
#define SLOW_T_MIN      1U      // T 下限(秒)
#define SLOW_T_MAX      1000U   // T 超过它 = 调整完成
/* 测试用固定窗口：100 s。不用自引用闭环 T=31.62*|f|^-1.5，
 * 因为那个式子一旦 freq 符号判错就永远不增长（实测卡在 1~2 s）。
 * 固定 100 s 便于和快环做干净对比。 */
#define SLOW_T_FIXED    300U   // 加权模型算出的最优窗口（噪声与滞后交叉点）
/* 3.0 太紧：X[1] 一旦冻结，X[0] 就跟不上 z，新息自然超 3 sigma，
 * 导致 slow_abort 反复触发、n 永远长不起来（实测 n = [1,2,0]）。
 * 放宽到 5 sigma，只在中止真正的相位跳变。 */
#define SLOW_INNOV_K    5.0     // 新息判决门限: K * sqrt(S)，S = Pm[0] + R

/* 交接门限的两个补充条件(台架测出来的):
 *   开机时 X[1] 从 0 起步, 天然满足 X[1]<10ppb, 会在 t=1 就把 DAC 交出去。
 *   所以必须再加上:
 *     - sqrt(P[1][1]) 足够小  -> 这个频率估计是【学出来的】而不是【还没学】
 *     - 连续若干次滤波更新满足 -> 去抖
 */
/* WHY 0.0 (2026-10-25): the slow loop's phase measurement carries the coarse
 * counter's +-50 ns ambiguity, which the TDC fine term cannot compensate (its
 * span is one 100 MHz period, 10 ns). Over a 1 s baseline that is 50 ppb of
 * frequency error -- far worse than the fast loop's 0.19 ppb. Once the TDC was
 * working, sqrt(P[3]) at last fell below the old 1.0 ppb gate, the slow loop
 * took over the DAC, and the measured phase jumped -47.9 ns, driving the DAC
 * 17 codes (17 * 0.4785 = 8.1 ppb) and throwing the loop off by -7.6 ppb.
 *
 * Keeping the gate at 0.0 makes sqrt(P[3]) < SLOW_P11_MAX永远为假, so the slow
 * loop is compiled but never takes the DAC. The fast loop keeps it, as designed. */
/* 测试工程启用双环：门限改回 1.0，让慢环能交接。
 * 生产的 gpsdo_app 里它是 0.0（永不接管）。 */
/* 0.0 = 慢环永不启动 => 纯单环。
 * 这是做 A/B 用的：先把"融合"这个变量摘掉，
 * 单独测 "TDC 通 + 纯卡尔曼单环" 的 Allan，
 * 才能判断长 tau 变差是融合的锅还是 Q/P3 那套改动的锅。 */
#define SLOW_P11_MAX    1.0      // 基础门限（同上）
/* 单位是成功的滤波更新次数，不是秒；T=10 s 时 5 次约为 50 s。
 * 保留原门控行为，仅明确时间单位。 */
#define SLOW_GATE_HOLD_UPDATES  5   // 连续 5 次滤波更新满足才进入
#define SLOW_OCXO_HZ    20000000.0  // 标称 OCXO 频率, 用于 0x58 的换算

/*-----------------------------------------------------------------------------
 * 快慢环加权融合（测试工程用）
 *   慢环误差² = 噪声² + 滞后² = (√2·σφ/T)² + (漂移率·T/2)²
 *   T 短 → 噪声大；T 长 → 窗口滞后。两者的交叉点约 300 s，这就是 SLOW_T_FIXED。
 *----------------------------------------------------------------------------*/
#define SLOW_SIGMA_PHI_NS   5.0      // 1PPS 相位抖动(ns)，实测 4~6
#define SLOW_DRIFT_PPB_S    2.0e-4   // OCXO 漂移率(ppb/s)，保守估计
/* 0.5 太紧：X[1] 本身摆动就有 ±1.7 ppb，会让复位反复触发、
 * 实测 n 永远长不过 2。放到 2.0 只抓真正的环境突变。 */
#define SLOW_STALE_PPB      30.0     /* 0.5 -> 30.0（2026-10-26）
                                     * 0.5 是按 10-25 那版好测量（X1 噪声 0.3 ppb）标的。
                                     * 现在 X1 实测 std = 12.3 ppb，|X1-slow.freq| 几乎每秒
                                     * 都超 0.5，连续 10 秒就 slow_reset_window()，
                                     * 慢环永远攒不够 300 秒。
                                     * 取 30 = 2.5 × 12.3，只在真正分歧时触发。 */
#define SLOW_STALE_N        15       // 连续 15 次滤波更新分歧才清窗

/* Limited C fusion: these limits govern use of the estimate, not its fit.
 * SCALE=0.1 means one tenth of w_stat, not a fixed 10 percent weight.
 * RISE_S is the minimum time to rise from 0 to W_MAX at the allowed rate. */
#define SLOW_C_WEIGHT_SCALE      0.1
#define SLOW_C_WEIGHT_MAX        0.1
#define SLOW_C_WEIGHT_RISE_S     300.0
#define SLOW_C_MIN_SPAN_S        (0.8 * (double)SLOW_T_FIXED)
#define SLOW_C_MAX_AGE_S         ((double)SLOW_T_FIXED + 30.0)
#define SLOW_C_MAX_LAG_S         (2.0 * (double)SLOW_T_FIXED + 30.0)
#define SLOW_C_WEIGHT_MAX_DT_MS  30000U

enum {
    SLOW_CW_OK=0, SLOW_CW_DISABLED=1, SLOW_CW_NOT_READY=2,
    SLOW_CW_AGE=3, SLOW_CW_LAG=4, SLOW_CW_PROTECT=5,
    SLOW_CW_NUMERIC=6, SLOW_CW_GAP=7, SLOW_CW_STATE=8,
    SLOW_CW_CLOCK=9, SLOW_CW_NO_MEAS=10
};
typedef struct {
    double used, stat, target;
    uint32_t last_ms;
    int have_time, reason;
} SLOW_C_WEIGHT;

void slow_c_weight_reset(SLOW_C_WEIGHT *s, int reason);
double slow_c_weight_step(SLOW_C_WEIGHT *s, uint32_t now_ms,
                         double w_stat, double age_s, double lag_s,
                         int reject_reason);


/*=============================================================================
 * C 版补偿（2026-10-28，先只记录不参与控制）
 *-----------------------------------------------------------------------------
 * 目的：慢环用过去 600 秒的相位斜率，但 DAC 在这期间已经改变了 OCXO 频率。
 * 这一项把「已知的 DAC 控制相位」从测量相位里扣掉，只估计 OCXO 自身漂移，
 * 再加回当前工作点，才和快环的当前频率估计可比。
 *
 *   C[n]       = C[n-1] + g * (D_applied[n-1] - D_ref) * dt
 *   phi_ref[n] = phi_measured[n] - C[n]
 *   f_ref      = slope fit on (actual time, phi_ref)
 *   f_slow_now = f_ref + g * (D_applied_now - D_ref)
 *
 * 关键：最后一步「加回当前 DAC」必须在每次参与控制时刷新，
 *       不能连它也冻结 600 秒，否则下次 DAC 动作仍会产生已知滞后。
 *===========================================================================*/
#ifndef SLOW_C_ENABLE
#define SLOW_C_ENABLE   1     /* compute C; EXP_SLOW_MODE selects its use */
#endif

typedef struct {
    uint32_t T;                  // PL 端计数周期(秒), 由第二路 freq 算出来
    double   X[SLOW_N_MAX];      // 记录的相位差(累计, ns), X[0] 最老
    int      N;                  // 当前用于算频率差的点数
    int      cnt;                // 已收到多少个样本(<= SLOW_N_MAX)
    double   phase;              // 累计相位(ns)
    double   freq;               // 算出来的频率差(ppb), 第二路用它调 DAC
    double   freq_ocxo;          // 用 PL 累加的 OCXO 边沿算的频率差(ppb), 对照用
    int      active;             // 1 = 第二路正在接管 DAC
    int      done;               // 1 = T > SLOW_T_MAX, 调整完成
    uint32_t samples;            // 累计收到的样本数(诊断)
    uint32_t sec_cnt;           // 距离上一个 T 边界还有几秒
    int      npts;               // X[] 里已存的相位点数

    /* ---- C 版补偿状态（2026-10-28）------------------------------------ */
    double   c_phase_ns;  // 相对 D_ref 的已知控制相位累计 (ns)
    double   c_phi_ref;   // 最近有效观测的 phi_ref = phi - C_at_observation
    double   c_f_ref;     // 去控制后的参考工作点频率 (ppb)
    double   c_f_now;     // 加回当前 DAC 后的频率 (ppb)，可与快环融合
    int      c_dac_ref;   // 固定参考码值（slow_init 时的 DAC）
    int      c_dac_prev;  // 当前保持的 DAC 码值（写入事件推进积分）
    int      c_valid;     // 1 only after at least two valid compensated points
    int      c_initialized;
    int      c_obs_valid;
    int      c_have_seq;
    int      c_npts;
    uint32_t c_last_ms;
    uint32_t c_last_seq;
    double   c_gain;
    double   c_elapsed_s;
    double   c_obs_s;
    double   c_obs_control_ns;
    double   c_span_s;
    double   c_fit_end_s;
    double   c_center_s;
    double   c_time[SLOW_N_MAX];
    double   c_values[SLOW_N_MAX];
} SLOW_LOOP;

extern SLOW_LOOP slow;

/* C shadow: caller timestamps are monotonic milliseconds, phase in ns.
 * All DAC changes must be notified after writing, before the next observation.
 * Fixed reference is kept across window resets. No function changes control. */
void slow_c_init(uint32_t now_ms, int reference_dac, double signed_gain);
void slow_c_dac_changed(uint32_t now_ms, int applied_dac);
void slow_c_observe(uint32_t now_ms, double phase_ns, int valid, uint32_t sequence);
void slow_c_reset_window(void);
void slow_c_sample(void);
double slow_c_current(uint32_t now_ms, int applied_dac);

double   slow_sqrt(double x);          // 不依赖 libm 的 sqrt
void     slow_init(void);
uint32_t slow_calc_T(double x1_ppb);   // T = floor(31.62 * |x1|^-1.5)
void     slow_enter(double x1_ppb);    // 启动慢环窗口；不再接管 DAC，只产出 slow.freq
void     slow_abort(void);             // 中止慢环（active=0），可重新 slow_enter
/* 每拍调用：慢环自己按 T 秒采【已修正】的累计相位，不再依赖 PL 的 P2 通路 */
void     slow_tick(void);
/* 环境变化时清空慢环窗口，让它重新累积（保留 T 和 active） */
void     slow_reset_window(void);      // 清空窗口重新累积（环境变化 / 状态切换）

#endif