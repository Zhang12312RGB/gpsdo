//=============================================================================
// main.c -- GPSDO (GNSS 驯服 OCXO) PS 端主程序
//-----------------------------------------------------------------------------
// 硬件分工
//   PL: gnss_gpsdo_top @ 0x43C0_0000
//       0x00 PHASE_ERR  相位差, 10ns/LSB, 有符号, 正 = 本地 1PPS 超前
//       0x04 NMEA_DATA  bit[7:0] 字符, bit8 新数据标志, 写 0x100 清零
//       0x08 DAC_DATA   12 bit, 0..4095
//       0x0C KEY        bit0 按键事件, 写 1 清零
//   PS: UART0 (0xE0000000, MIO14/15 -> J12)  -> 串口屏
//       UART1 (0xE0001000, MIO48/49 -> USB)  -> 调试 / 数据日志
//       MIO0  (E6, PS LED1)                  -> |X1| 越大闪得越快
//       SCU 私有定时器                        -> 精确 1 秒节拍
//
// 状态机
//   TUNING   闭环驯服: 每 sample_period 秒读相位 -> 卡尔曼 -> 调 DAC
//   CALIB    标定: DAC 冻结, 只收集自适应 R 的工作点
//   HOLDOVER 守时: GNSS 失锁, X[1] 冻结, 只外推 X[0] 并保持 DAC
//   切换: 按键 / 串口屏命令 TUNING<->CALIB; 连续失锁 5 秒 -> HOLDOVER;
//         GNSS 恢复 -> R=5000 平滑 5 秒 -> 重置 X[0]/P(保留 X[1]) -> TUNING
//=============================================================================

#include "main.h"
#include "bsp.h"
#include "xil_io.h"
#include <math.h>
#include "gnss.h"
#include "kalman.h"
#include "cand_obs.h"
#include "adaptive_r.h"
#include "control.h"
#include "slow_loop.h"
#include "bench.h"
#include "screen.h"
#include "dbg_uart.h"
#include "led_ctrl.h"
#include "ps_key.h"
#include "hwtest.h"

//-----------------------------------------------------------------------------
// 运行期日志模式（三选一）
//   QUIET : 只发串口屏, UART1 全静默（正式演示/比赛现场用）
//   TEXT  : UART1 每秒一行人可读状态（调试用）
//   CSV   : UART1 每秒一行 CSV, 直接存盘做 ADEV / 稳态误差分析
//-----------------------------------------------------------------------------
#define MAIN_LOG_QUIET   0
#define MAIN_LOG_TEXT    1
#define MAIN_LOG_CSV     2

/*=============================================================================
 * 中心值 / OCXO 极性标定
 *   1 = 上电后【接管整个程序】，只做标定，不进状态机、不跑闭环
 *   0 = 正常比赛/运行模式
 * 标定做完把下面两个数按打印结果填回 control.h，并把本开关改回 0。
 *===========================================================================*/
#define CENTER_CALIB_ENABLE 0
#define CAL_N_POINT         30      /* 每个测点采多少点(1 s 一点)        */
#define CAL_DAC_CENTER      2048    /* 中心值：先假定 2048，标定后按实测改 */
static int main_log = MAIN_LOG_TEXT;

/* NMEA 通路诊断(0x24 DROP_COUNT / 0x28 BYTE_COUNT + 0x2C STATUS)的打印周期, 秒.
 * 0 = 关闭。注意: 它**不受 main_log 影响** —— 开着原始 NMEA dump 时
 * main_log 是 QUIET, 如果放在 TEXT 分支里就永远打不出来。 */
#define DIAG_NMEA_PERIOD_S  1

//-----------------------------------------------------------------------------
// 全局状态
//-----------------------------------------------------------------------------
int sys_state     = STATE_TUNING;
int sample_period = 1;
double main_phase_acc_ns = 0.0;  /* 绝对相位（给滤波器用） */
/* ---- 2026-10-30：统一门控的诊断字段 -------------------------------- */
static int      sample_why            = BSP_Q_OK; /* 本秒判定 */
static int      sample_acc            = 0;        /* 本秒是否计入累计相位 */
static int      g_kf_used             = 0;        /* 本拍是否真的参与 KF/DAC */
static int      g_last_why            = BSP_Q_OK;
static int      g_last_acc            = 0;
static uint32_t g_last_seq            = 0U;
static uint32_t s_bad_sample_cnt      = 0U;       /* 累计坏样本 */
static uint16_t s_bad_sample_run      = 0U;       /* 连续坏样本 */
static uint32_t s_hard_reset_cnt      = 0U;       /* 硬保护次数 */
static uint32_t s_gap_filled          = 0U;       /* 用 X[1] 补齐相位的次数 */
static uint32_t s_gap_skips           = 0U;       /* 因缺口跳过的控制拍数 */
static uint32_t gap_this_window       = 0U;       /* 本 10 秒窗口内是否有缺口 */
static uint16_t gap_count_this_window = 0U;       /* 本窗口缺口秒数 */
static uint16_t gap_count_last_window = 0U;       /* 上一窗口缺口秒数 */
static uint32_t s_slow_win_reset      = 0U;       /* 因缺口清慢环窗口的次数 */
static uint32_t slow_gap_hold         = 0U;       /* 本秒是否暂停 slow_tick */
static double   g_phase_term          = 0.0;      /* 本拍相位项（日志用，不受开关影响）*/
/* 2026-10-30：记住失锁前的状态。
 * 原来 task_holdover_second() 恢复时写死回 STATE_TUNING，
 * 结果是：CALIB(冻结) -> 失锁 -> HOLDOVER -> 恢复 -> TUNING(开始推 DAC)。
 * 换天线做实验时一旦失锁，冻结的 DAC 就保不住。
 * 现在改成恢复到【失锁前的状态】。*/
static int      prev_state            = STATE_TUNING;   /* 上一个状态 */
static int      state_before_holdover = STATE_TUNING;
double main_p_sum        = 0.0;  /* 诊断: 独立累加 p，核对累加器是否被别处改过 */

/* ±50 ns 粗计数修正累计触发次数（诊断用，日志里打 f50=） */
unsigned long main_fix50_cnt = 0;
/* f50r: 被判定为真实阶跃、因此放弃修正的次数 */
unsigned long main_fix50_real = 0;
double main_last_phase_ns = 0.0;   /* 本秒实测相位增量 (ns) */

/* 双路交接用: 最近一次卡尔曼新息(ns)与测量噪声方差, 由 task_tuning_second 写入 */
static double g_innov = 0.0;
static double g_R     = 1.0;

static int epoch            = 0;   // 采样周期内的秒计数
static int invalid_count    = 0;   // GNSS 连续无效计数
static int gnss_lost        = 0;   // 失锁标志
static int holdover_seconds = 0;   // 守时 / 恢复计时
static int resume_flag      = 0;   // GNSS 恢复后的平滑标志
static uint32_t log_seconds = 0;   // CSV 时间戳(秒)

/* 去抖单位是成功的滤波更新次数，不是每秒任务调用次数。 */
static int slow_gate_cnt = 0;
static uint32_t g_tuning_update = 0;
static uint32_t g_tuning_update_second = 0;
static int g_tuning_update_period = 0;

enum {
    SLOW_EV_NONE = 0,
    SLOW_EV_ENTER = 1,
    SLOW_EV_ABORT_INNOV = 2,
    SLOW_EV_RESET_STALE = 3,
    SLOW_EV_DONE = 4,          /* 窗口首次满；慢环仍继续工作 */
    SLOW_EV_RESET_INNOV = 5,   /* 融合保护先清窗，之后可能再触发中止 */
    SLOW_EV_RESET_STATE = 6
};

//=============================================================================
// 小工具
//=============================================================================

static const char *state_name(int st)
{
    switch (st) {
    case STATE_TUNING:   return "TUNING";
    case STATE_CALIB:    return "CALIB";
    case STATE_HOLDOVER: return "HOLDOVER";
    default:             return "UNKNOWN";
    }
}

// 取小数部分并放大: frac_of(-1.234, 1000) = 234
static int frac_of(double v, int scale)
{
    double f = v - (double)(int)v;
    if (f < 0.0) { f = -f; }
    return (int)(f * (double)scale + 0.5);
}

//=============================================================================
// 状态切换（唯一入口：统一清 epoch 与自适应 R 采集）
//=============================================================================
/* 每次成功滤波更新记录一次 ev=0，状态事件在发生处立即记录。
 * upd=0 表示尚无滤波样本；age 是 innov/S 距当前日志秒数的年龄。
 * Tu 是产生这组 innov/S 的实际周期，不是自适应后的下一个周期。
 * n0 为操作前点数，cnt 为进入去抖次数，sec 为慢环窗口内秒计数。
 * 仅 TEXT 模式输出，避免向 CSV 或静默模式插入额外记录。 */
static void log_slow_debug(int event, int n_before)
{
    double sigma;
    double normalized;
    if (main_log != MAIN_LOG_TEXT) { return; }
    sigma = slow_sqrt(kalman.S);
    normalized = (sigma > 0.0) ? (g_innov / sigma) : 0.0;
    dbg_printf("SLOWDBG t=%lu upd=%lu age=%lu Tu=%d ev=%d st=%d "
               "act=%d n0=%d n=%d cnt=%d sec=%lu done=%d "
               "innov=%.9g S=%.9g K=%.1f lim=%.9g z=%.6g R=%.9g\r\n",
               (unsigned long)log_seconds, (unsigned long)g_tuning_update,
               (unsigned long)(log_seconds - g_tuning_update_second),
               g_tuning_update_period, event, sys_state,
               slow.active, n_before, slow.npts, slow_gate_cnt,
               (unsigned long)slow.sec_cnt, slow.done,
               g_innov, kalman.S, SLOW_INNOV_K, SLOW_INNOV_K * sigma,
               normalized, g_R);
}

/* Capture before the DAC command, transmit afterward so UART does not delay it.
 * Cns/phiRef are paired at the last phase observation; age/lag are seconds. */
/* ---- C 完整快照（2026-10-28）-------------------------------------------
 * 目的：让【控制、保护、日志】对应同一时刻、同一组 C 状态。
 * 只保存 fNow 不够 —— 融合保护可能在取快照和打印之间把窗口清掉，
 * 那样 v / fRef / 点数 / 窗口信息就会变成"清窗后"的值，与 fNow 不配套。
 * ---------------------------------------------------------------------- */
typedef struct {
    int    valid;       /* 取快照时刻的 slow.c_valid                 */
    int    npts;        /* slow.c_npts                               */
    double cns;         /* slow.c_obs_control_ns（与相位观测配对）   */
    double phi_ref;     /* slow.c_phi_ref                            */
    double f_ref;       /* slow.c_f_ref                              */
    double f_now;       /* slow_c_current() 的返回值                 */
    int    dref;        /* slow.c_dac_ref                            */
    int    dac;         /* 调用时的 DAC（调节前）                    */
    double gain;        /* slow.c_gain                               */
    double span_s;      /* slow.c_span_s                             */
    double age;         /* snapshot_s − slow.c_fit_end_s             */
    double lag;         /* snapshot_s − slow.c_center_s              */
    double snapshot_s;  /* slow.c_elapsed_s                          */
} c_snap_t;

/* 每拍都取（不论 MODE），调用后再判有效 —— 避免用到调用前的旧 valid */
static void c_snap_take(c_snap_t *cs)
{
    /* ★ 先清零：SLOW_C_ENABLE=0 时下面的 #else 只做 (void)cs，
     *   不会初始化内容；而 MODE=2 仍会读 cs.valid，必须明确得到"无效"。 */
    *cs = (c_snap_t){0};
#if SLOW_C_ENABLE
    cs->dac        = control_get_last_dac();
    cs->f_now      = slow_c_current(bsp_millis(), cs->dac);
    cs->valid      = slow.c_valid;
    cs->npts       = slow.c_npts;
    cs->cns        = slow.c_obs_control_ns;
    cs->phi_ref    = slow.c_phi_ref;
    cs->f_ref      = slow.c_f_ref;
    cs->dref       = slow.c_dac_ref;
    cs->gain       = slow.c_gain;
    cs->span_s     = slow.c_span_s;
    cs->snapshot_s = slow.c_elapsed_s;
    cs->age        = (cs->valid != 0) ? (cs->snapshot_s - slow.c_fit_end_s) : 0.0;
    cs->lag        = (cs->valid != 0) ? (cs->snapshot_s - slow.c_center_s)  : 0.0;
#else
    (void)cs;
#endif
}


static void log_slow_c(const c_snap_t *cs, double w_used, int src_used, int prot_ev)
{
#if SLOW_C_ENABLE
    if (main_log != MAIN_LOG_TEXT) { return; }
    /* 全部字段取自同一份快照；w_used/src_used/prot_ev 是【本拍实际】结果，
     * 与候选有效标志 cs->valid 分开 —— 避免把"候选有效"误认为"已参与控制"。 */
    dbg_printf("SLOWC t=%lu upd=%lu v=%d n=%d Cns=%.9g phiRef=%.9g "
               "fRef=%.9g fNow=%.9g fast=%.9g Dref=%d D=%d g=%.7g "
               "span=%.6g age=%.6g lag=%.6g src=%d w=%.4f pev=%d\r\n",
               (unsigned long)log_seconds, (unsigned long)g_tuning_update,
               cs->valid, cs->npts, cs->cns, cs->phi_ref,
               cs->f_ref, cs->f_now, kalman.X[1], cs->dref, cs->dac, cs->gain,
               cs->span_s, cs->age, cs->lag, src_used, w_used, prot_ev);
#else
    (void)cs; (void)w_used; (void)src_used; (void)prot_ev;
#endif
}

static void c_fusion_reset(int reason);

static void enter_state(int st)
{
    if (st == sys_state) {
        return;
    }
    c_fusion_reset(SLOW_CW_STATE);
    prev_state = sys_state;      /* 2026-10-30: 记住上一个状态 */
    sys_state  = st;
    epoch      = 0;

    /* 切换状态时清空慢环窗口。
     * 不清的后果：慢环的 X[] 里会跨越一整段 HOLDOVER —— 那段时间要么
     * PPS 失效、累加器冻结（斜率算成 0），要么 OCXO 自由振荡（频率和
     * 驯服时完全不同）。恢复 TUNING 后前 600 秒，slow.freq 会是这段
     * 无效数据算出来的，却照常以 89% 的权重进融合。 */
    {
        int n_before = slow.npts;
        slow_reset_window();
        log_slow_debug(SLOW_EV_RESET_STATE, n_before);
    }

    /* 2026-10-30：CALIB 与 HOLDOVER 期间硬冻结 DAC */
    control_set_dac_lock((st == STATE_TUNING) ? 0 : 1);

    if (st == STATE_HOLDOVER) {
        /* 记下从哪个状态进来的，恢复时回到原状态 */
        /* prev_state在sys_state改写前保存；入口已排除重复切换。 */
        state_before_holdover = prev_state;
        holdover_seconds = 0;
        resume_flag      = 0;
    } else {
        adaptive_r_reset();     // 回到可观测状态: 重新收集工作点
    }

    if (main_log != MAIN_LOG_QUIET) {
        dbg_printf("STATE -> %s\r\n", state_name(st));
    }
}

//=============================================================================
// 每秒: GNSS 失锁 / 恢复判定
//=============================================================================
//=============================================================================
// GNSS 状态的统一入口
//   台架模式下返回 bench.h 里的替身值, 真实硬件下返回 gnss 结构体。
//   注意 gnss 结构体本身【永远】是真实模块解析出来的 —— 台架不去改它,
//   所以 log_bench 里读 gnss.* 一定是真实值(没天线 -> fix=0 sat=0)。
//=============================================================================
static int gnss_fix(void)
{
#if (BENCH_ENABLE)
    return BENCH_FIX;
#else
    /* Fresh RMC alone must not keep stale GGA quality valid after bad frames. */
    return gnss.fix_valid && gnss_quality_fresh();
#endif
}

static int gnss_sat(void)
{
#if (BENCH_ENABLE)
    return BENCH_SAT;
#else
    return gnss.satellites;
#endif
}

static double gnss_hdop(void)
{
#if (BENCH_ENABLE)
    return BENCH_HDOP;
#else
    return gnss.hdop;
#endif
}

static double gnss_cn0(void)
{
#if (BENCH_ENABLE)
    return BENCH_CN0;
#else
    return gnss.cn0_avg;
#endif
}

static void check_gnss_lost(void)
{
    int bad = ((gnss_fix() == 0) || (gnss_sat() < SATELLITE_MIN)) ? 1 : 0;

    if (bad) {
        if (invalid_count < 1000) {
            invalid_count++;
        }
        if (sys_state == STATE_HOLDOVER) {
            resume_flag = 0;            // 恢复途中又丢: 取消平滑, 继续守时
            gnss_lost   = 1;
        } else if (invalid_count >= GNSS_LOST_LIMIT) {
            gnss_lost = 1;
            enter_state(STATE_HOLDOVER);
        }
    } else {
        invalid_count = 0;
        if ((sys_state == STATE_HOLDOVER) && (gnss_lost != 0)) {
            resume_flag      = 1;
            holdover_seconds = 0;
            gnss_lost        = 0;
            if (main_log != MAIN_LOG_QUIET) {
                dbg_printf("GNSS 恢复, 平滑 %d 秒后回 TUNING\r\n",
                           HOLDOVER_RESUME_SECONDS);
            }
        }
    }
}

//=============================================================================
// 采样周期自适应: 1 -> 5 -> 10, 受扰反向
//=============================================================================
static void update_sample_period(void)
{
    double ap, af;

    /* ---- 开机前 5 分钟强制 T = 10，不让采样周期自适应 ------------------
     * 自适应判据用的是 P11 和新息，而开机阶段这两个量本身还在收敛。
     * 实测（2026-10-25 30 分钟数据）会在 T = 1 -> 5 -> 1 -> 5 -> 10 之间
     * 反复跳，每次切换都改变 Q = KALMAN_Q_FREQ * T，导致 P3 突变、卡尔曼
     * 增益突变，X[1] 出现 +-2 ppb 的跳变（实测 t=6 s 从 +0.003 崩到 -2.069，
     * t=7 s 又跳回 +0.309）—— 这把 Allan@1s 从 0.022 抬到 0.054。
     * 强制 T = 10 同时也让 Q 大 10 倍，开机收敛反而更快。
     * ------------------------------------------------------------------ */
    /* 对照实验：固定周期时直接返回 10 秒，跳过整条自适应判据 */
    if (EXP_FIXED_PERIOD != 0) {
        sample_period = 10;
        return;
    }

    if (log_seconds < 300U) {
        sample_period = 10;
        return;
    }

    ap = fabs(g_innov);              /* 用新息判收敛: X0 在锁频环里没有"收敛"含义 */
    af = fabs(kalman.X[1]);           /* af = 频差 (ppb) */

    if (sample_period == 1) {
        if ((ap < 50.0) && (af < 1.0)) {
            sample_period = 5;
        }
    } else if (sample_period == 5) {
        if ((ap < 20.0) && (af < 0.3)) {
            sample_period = 10;
        } else if ((ap > 100.0) || (af > 2.0)) {
            sample_period = 1;
        }
    } else if (sample_period == 10) {
        if ((ap > 40.0) || (af > 0.6)) {
            sample_period = 5;
        }
    }
}

//=============================================================================
// 每秒任务
//=============================================================================

/* 上一次已经写进 DAC 的码值。用来在卡尔曼预测里补上【控制输入项】。
 * 见 task_tuning_second() 里的 B 项说明。 */
static int s_prev_dac = DAC_CENTER;

/* ---- 融合快照（2026-10-28）：给日志用，不参与控制 --------------- */
static double g_fuse_w   = 0.0;   /* 本次融合权重             */
static int    g_fuse_src = 0;     /* 0=无 1=slow.freq 2=C fNow */
static double g_fuse_fsl = 0.0;   /* 本次用的慢环频率(ppb)    */
static SLOW_C_WEIGHT g_c_weight;

static void c_fusion_reset(int reason)
{
    slow_c_weight_reset(&g_c_weight, reason);
    g_fuse_w = 0.0; g_fuse_src = 0; g_fuse_fsl = 0.0;
}

static void log_c_fusion(void)
{
    if (main_log != MAIN_LOG_TEXT) { return; }
    dbg_printf("CFUSE upd=%lu wstat_u=%ld target_u=%ld used_u=%ld reason=%d\r\n",
               (unsigned long)g_tuning_update,
               (long)(g_c_weight.stat * 1000000.0 + 0.5),
               (long)(g_c_weight.target * 1000000.0 + 0.5),
               (long)(g_c_weight.used * 1000000.0 + 0.5), g_c_weight.reason);
}



// TUNING: 每 sample_period 秒读一次相位 -> 自适应 R -> 卡尔曼 -> 调 DAC
static int s_stale_cnt = 0;   /* 快慢环分歧的连续滤波更新次数 */
static void task_tuning_second(void)
{
    double z, R, innov;

    epoch++;
    if (epoch < sample_period) {
        return;
    }
    epoch = 0;

    /* ---- 2026-10-30：先补 DAC 阶跃，再看缺口 ---------------------------
     * 必须放在任何 return 之前。上一拍已写新 DAC 码，它对频率的影响是
     * 【已知】的，必须补进 X[1]；否则预测会用旧频率。
     * 按当前增益，每漏 1 码补偿，10 秒预测相位就差 4.7852 ns。
     * -------------------------------------------------------------------- */
    {
        int cur = control_get_last_dac();
        double du = (double)(cur - s_prev_dac) * DAC_GAIN
                    * (double)control_get_loop_sign();
        if (du != 0.0) {
            kalman.X[1] += du;      /* 频率：DAC 阶跃直接改变频率 */
            s_prev_dac   = cur;
        }
    }

    /* ---- 统一门控：本 10 秒窗口内任一秒缺测 → 跳过本拍 KF 更新与 DAC ----
     * 跳过时：协方差按预测增长；相位参考保持（缺口期间已用 X[1] 补齐）；
     *         不重建参考、不清窗口。
     * -------------------------------------------------------------------- */
    if (gap_this_window != 0U) {
        uint16_t gaps_here = gap_count_this_window;
        gap_count_last_window = gaps_here;
        gap_this_window       = 0U;
        gap_count_this_window = 0U;
        s_gap_skips++;
        if (main_log == MAIN_LOG_TEXT) {
            dbg_printf("GAPSKIP upd=%d 本窗口缺测 %u 秒, 跳过本拍 KF/DAC\r\n",
                       (int)g_tuning_update, (unsigned)gaps_here);
        }
        c_fusion_reset(SLOW_CW_GAP);
        log_c_fusion();
        kalman_predict_only(&kalman, (double)sample_period);
        cand_obs_predict_only((double)sample_period);   /* 2026-10-31: 候选同步预测 */
        return;
    }

    /* 没有有效测量就不要喂卡尔曼。
     * meas_valid=0 时 0x00 里的相位差是 0 或上一次的陈旧值, 把它当测量喂进去
     * 等于告诉滤波器"误差恰好是 0": X[0] 会被强行拉向 0, 而且下面的 dac_update()
     * 会按这个假误差去动 DAC。1PPS 短暂丢失但卫星数仍够(所以没进 HOLDOVER)时
     * 就会踩到这条路。直接返回, 让协方差按预测步自然增长。
     * 顺带也避免了"X 恒为 0 被误判成极度稳定, 采样周期一路升到 10s"。 */
    if (bsp_meas_valid() == 0) {
        c_fusion_reset(SLOW_CW_NO_MEAS);
        log_c_fusion();
        kalman_predict_only(&kalman, (double)sample_period);
        cand_obs_predict_only((double)sample_period);
        return;
    }

    /* 观测 z = 绝对相位 (ns)
     * PL 的 0x00 是【每秒增量】，这里累加成绝对相位，
     * 才能让卡尔曼的 X[0]=相位(ns)、X[1]=频率(ppb) 成立。
     * 屏幕与日志显示的“相位差”用 read_phase_error_ns()（本秒增量），不用这个值。
     */
    z = main_phase_acc_ns;
    R = adaptive_r_get(gnss_hdop(), gnss_sat(), gnss_cn0());
    kalman.R = R;

    /* ---- 控制输入项 B·u ------------------------------------------------
     * 标准卡尔曼状态方程是  x_{k+1} = F·x_k + B·u_k。
     * 这里 u_k 就是「上一次我们改了多少 DAC」：它按 g 直接改变 OCXO 频率，
     * 是一个【已知】阶跃，不是随机扰动。
     *
     * 不补会怎样（台架实测）：DAC 从 384 砸到 0 = 频率突变 -19.2 ppb，
     * 滤波器仍按 X1=+55 预测相位，新息 = -57 ns ≈ -5.8σ（R=96），
     * 它判定「测量异常」而选择相信模型 -> X0 越跑越偏、P11 越缩越小 -> 发散。
     * 表现：X0 比实测高 20 ns、X1 符号都反了，DAC 被一直压到限幅 0。
     *
     * 量纲: du = Δdac × DAC_GAIN [ppb] = [ns/s]，直接加在 X1 上；X0 由 T·X1 自然累积。
     * 符号: 约定 dac↑ = 频率↑，故 du>0 表示变快。 */
    g_tuning_update_period = sample_period;
    kalman_update(&kalman, z, (double)sample_period);
    g_tuning_update++;
    g_kf_used = 1;      /* 本拍确实参与了 KF/DAC */

    /* ---- 2026-10-31：候选观察器（只记录，不影响控制）------------------
     * 三路共享同一个 z（来自主快照），各自维护独立状态。
     * ⚠️ 候选【不写 DAC】【不改保护】【不清慢环窗口】。*/
    cand_obs_update(z, (double)sample_period);
    g_tuning_update_second = log_seconds;
    innov = z - kalman.Xm[0];       // 新息(预测之后、更新之前的残差)
#if EXP_R_TRAIN_ENABLE
    adaptive_r_update(gnss_hdop(), gnss_sat(), gnss_cn0(), innov);
#endif

    /* 采样周期自适应必须【每个 epoch 都跑】。
     * 原来它只在 (slow.done == 0 && slow.active == 0) 的交接门控里调用，
     * 慢环一旦启动，这个门控就再也不进 —— T 永远冻结在 slow_enter 那一刻
     * 的值。更糟的是 task_holdover_second() 恢复时会把 sample_period 直接
     * 置 1，若此时慢环已 active，T 就永久卡在 1，Q 小 10 倍，P3 立刻塌。
     * 放到这里，任何路径都不会漏。 */
    /* 2026-10-28 复核修正：g_innov / g_R 必须在 update_sample_period() 之前
     * 写入，否则周期判据用的是【上一次】的新息。
     * 复核用 638 次真实更新重放：改动前后有 2 次决策不同
     * （t=1035 5->10，t=2435 10->5），确认是时序不一致。 */
    g_innov = innov;
    g_R     = R;

    update_sample_period();

    /* ---- 快慢环加权融合（仅测试工程）-----------------------------------
     * 原单环 : control_apply_freq(kalman.X[1])
     *          短 τ 好（0.046 ppb @1 s），长 τ 差 —— 实测 τ>200 s 连简单平均
     *          都不如（1000 s: 0.0275 vs 0.0122），因为 Q 让滤波器一直在
     *          "跟踪"而不是"平均"，长基线的信息被不断打掉。
     * 现在   : 还是同一条积分律，只把【输入】换成加权值
     *          慢环没准备好 -> 纯快环（短 τ 一点不损失）
     *          慢环攒够点   -> 按逆方差加权，长 τ 交给慢环的长基线
     * 慢环误差² = 噪声² + 滞后² = (√2·σφ/T)² + (漂移率·T/2)²
     *          T=100 s 噪声主导，T=1000 s 滞后主导，交叉点约 300 s -> SLOW_T_FIXED
     * -------------------------------------------------------------------- */
    {
        double f_ctrl = kalman.X[1];

        /* ---- 慢环输入源选择（2026-10-28 C 反馈实验）--------------------
         * MODE=0 : 不参与控制，但【仍照原样】维护窗口与保护
         * MODE=1 : 原始 slow.freq
         * MODE=2 : C 补偿后的 fNow
         * 任一来源无效 -> 纯快环（不是退回 slow.freq）
         * ---------------------------------------------------------- */
        /* ---- ① 先取完整快照（不论 MODE，每拍都取）---------------------- */
        c_snap_t cs;
        int      prot_ev = 0;      /* 本拍触发的保护事件（0=无） */
        c_snap_take(&cs);

        {
            double f_slow_use = 0.0;
            int    slow_src   = 0;   /* 0=无效 1=原始 2=C 补偿 */
            int c_reason = SLOW_CW_NOT_READY;

            if (EXP_SLOW_MODE == 2) {
                /* C 只在有有效拟合时提供值；否则本拍纯快环 */
                if (cs.valid != 0 && cs.npts >= 2 &&
                    isfinite(cs.span_s) && cs.span_s >= SLOW_C_MIN_SPAN_S &&
                    isfinite(cs.f_now)) {
                    f_slow_use = cs.f_now;    /* ★ 不再二次调用 */
                    slow_src   = 2;
                    c_reason = SLOW_CW_OK;
                } else if (cs.valid != 0 &&
                           (!isfinite(cs.f_now) || !isfinite(cs.span_s))) {
                    c_reason = SLOW_CW_NUMERIC;
                }
            } else if ((slow.active != 0) && (slow.npts >= 2)) {
                f_slow_use = slow.freq;
                slow_src   = 1;
            }

            if (slow_src != 0) {
                double s_noise  = 1.41421356 * SLOW_SIGMA_PHI_NS / (double)slow.T;
                /* C lag已包含回归中心和age，不再套用名义T/2。 */
                double lag_use  = (slow_src == 2) ? cs.lag : (double)slow.T * 0.5;
                double s_lag    = SLOW_DRIFT_PPB_S * lag_use;
                double var_slow = s_noise * s_noise + s_lag * s_lag;
                double var_fast = kalman.P[3];
                double w;
                double innov_sig = slow_sqrt(kalman.S);

                if (var_fast < 0.002) { var_fast = 0.002; }
                w = var_fast / (var_fast + var_slow);
                double w_stat = w;

                /* ---- 用【本次选中的来源】跑保护 ---- */
                if (fabs(innov) > (SLOW_INNOV_K * innov_sig)) {
                    int n_before = slow.npts;
                    w = 0.0;
                    slow_reset_window();          /* 内部已同时清 C 窗口 */
                    prot_ev = SLOW_EV_RESET_INNOV;
                    log_slow_debug(SLOW_EV_RESET_INNOV, n_before);
                } else if (fabs(kalman.X[1] - f_slow_use) > SLOW_STALE_PPB) {
                    if (++s_stale_cnt >= SLOW_STALE_N) {
                        int n_before = slow.npts;
                        w = 0.0;
                        slow_reset_window();
                        s_stale_cnt = 0;
                        prot_ev = SLOW_EV_RESET_STALE;
                        log_slow_debug(SLOW_EV_RESET_STALE, n_before);
                    }
                } else {
                    s_stale_cnt = 0;
                }

                if (EXP_SLOW_MODE == 2) {
                    w = slow_c_weight_step(&g_c_weight, bsp_millis(), w_stat,
                                          cs.age, cs.lag,
                                          (prot_ev != 0) ? SLOW_CW_PROTECT : c_reason);
                } else {
                    slow_c_weight_reset(&g_c_weight, SLOW_CW_DISABLED);
                    if (w > 0.95) { w = 0.95; }
                }
                if (EXP_SLOW_MODE == 0) { w = 0.0; }   /* B 基线：只置零，不改窗口 */

                f_ctrl = (1.0 - w) * kalman.X[1] + w * f_slow_use;
                g_fuse_w    = w;
                /* ★ src 表示【实际参与控制】的来源：w=0 时（保护清窗或
                 *   MODE=0）没有任何慢环数据进入 f_ctrl，应记 0。
                 *   候选来源 slow_src 由 src 之外的 v/n 字段间接体现。 */
                g_fuse_src  = (w > 0.0) ? slow_src : 0;
                g_fuse_fsl  = f_slow_use;
            } else {
                slow_c_weight_reset(&g_c_weight,
                    (EXP_SLOW_MODE == 2) ? c_reason : SLOW_CW_DISABLED);
                if ((slow.active == 0) || (slow.npts < 2)) { s_stale_cnt = 0; }
                g_fuse_w   = 0.0;
                g_fuse_src = 0;
                g_fuse_fsl = 0.0;
            }
        }

        /* ---- 相位反馈项 ------------------------------------------------
         * 积分律原来只看得见 f_ctrl（频率估计）。而 X[1] 的时间常数约
         * 2000 秒，22 分钟只收到真值的 35%，导致积分器推得极慢、相位
         * 单调漂移，Allan 随 tau 上升。
         * 这里把【相位本身】也按比例加进控制量：相位一漂立刻纠正，
         * 不必等 X[1] 收敛。这就是「相位 P + 频率 I」。
         * 它就是 X[0]，即卡尔曼的累计相位（已过 ±50 ns 修正）。
         * -------------------------------------------------------------- */
        {
            /* 2026-10-30：相位项改成可开关（EXP_PHASE_TERM_EN）
             *   =1  正常闭环（历史行为）
             *   =0  纯频率控制：仍然【计算并记录】相位项，但不加进 f_ctrl
             *       这样日志里的 phs_term 仍能看到"假如启用会是多少"，
             *       便于事后对比，也不影响 KF 内部的 X[0] 状态。*/
            double phs_term = K_PHASE * kalman.X[0];
            if (phs_term >  K_PHASE_MAX) { phs_term =  K_PHASE_MAX; }
            if (phs_term < -K_PHASE_MAX) { phs_term = -K_PHASE_MAX; }

            g_phase_term = phs_term;        /* 日志用 */

#if (EXP_PHASE_TERM_EN)
            f_ctrl += phs_term;
#endif
        }

        /* ---- ④ 写 DAC，然后输出【同一份快照】+ 本拍实际融合结果 -------- */
        control_apply_freq(f_ctrl);
        log_slow_c(&cs, g_fuse_w, g_fuse_src, prot_ev);
        log_c_fusion();
    }

    if (slow.done == 0) {
        if (slow.active == 0) {
            /* 交接必须三个条件同时满足:
             *   1) X[1] < 10ppb                  —— 频率偏差已经够小
             *   2) sqrt(P[1][1]) < SLOW_P11_MAX  —— 这个估计是【学出来的】,
             *      而不是【还没学到】。开机时 X[1] 从 0 起步, 天生满足条件 1,
             *      台架上就是它把交接提前到了 t=1(DAC 直接跳到 384, 环路跑飞)。
             *   3) 连续 SLOW_GATE_HOLD_UPDATES 次滤波更新满足 —— 去抖
             */
            int ok = 0;

            /* ---- 分级门控 ----------------------------------------------------
             * 开机前 10 分钟用严格门限：此时 X[1] 还在从 0 收敛，P3 也很大，
             * 若放宽会让慢环在 t=1 就抢跑（台架上实测把 DAC 直接推到 384，
             * 环路跑飞）—— 这是原来那条注释里记录的教训。
             * 10 分钟之后环路已经稳了，放宽门限让慢环安全接管：
             * 它对相位做 300 秒最小二乘，不依赖卡尔曼的精度。
             * ---------------------------------------------------------------- */
            double x1_gate  = (log_seconds < 600U) ? SLOW_X1_GATE  : 200.0;
            double p11_gate = (log_seconds < 600U) ? SLOW_P11_MAX  : 100.0;

            if ((fabs(kalman.X[1]) < x1_gate) &&
                (slow_sqrt(kalman.P[3]) < p11_gate)) {
                if (slow_gate_cnt < SLOW_GATE_HOLD_UPDATES) { slow_gate_cnt++; }
                if (slow_gate_cnt >= SLOW_GATE_HOLD_UPDATES) { ok = 1; }
            } else {
                slow_gate_cnt = 0;
            }

            if (ok != 0) {
                int n_before = slow.npts;
                slow_enter(kalman.X[1]);        /* 启动慢环的 T 秒窗口积分 */
                log_slow_debug(SLOW_EV_ENTER, n_before);
            }
        } else if (fabs(innov) > (SLOW_INNOV_K * slow_sqrt(kalman.S))) {
            /* 2026-10-27 修正：新息的标准差是 sqrt(P00 + R)，不是 sqrt(R)。
             * 这里原来是【慢环运行中】的唯一中止分支，按 sqrt(R) 算的门限
             * 只有 16~34 ns，正常相位波动就会命中 ——
             * 第三轮日志里有 126 次 n=1→0，就是这条造成的。
             * 而 SLOW_STALE_PPB 只在 n>=2 之后才检查，管不到这个阶段。 */
            int n_before = slow.npts;
            slow_gate_cnt = 0;
            slow_abort();                       /* 新息异常: 重新累积 */
            log_slow_debug(SLOW_EV_ABORT_INNOV, n_before);
        }
    }
    log_slow_debug(SLOW_EV_NONE, slow.npts);
}

// CALIB: DAC 冻结, 读相位差喂自适应 R 的采集
static void task_calib_second(void)
{
#if EXP_R_TRAIN_ENABLE
    double z, innov;

    /* 同 TUNING: 没有有效测量就不采集, 否则自适应 R 的统计会被 z=0 污染。 */
    if (bsp_meas_valid() == 0) {
        return;
    }

    z     = read_phase_error_ns();
    innov = z - kalman.X[0];

    adaptive_r_update(gnss_hdop(), gnss_sat(), gnss_cn0(), innov);
#endif
    /* Final CALIB only freezes DAC; raw snapshots remain available for offline labels. */
}

// HOLDOVER: X[1] 冻结只外推 X[0]; 恢复则 R=5000 平滑 5 秒后回 TUNING
static void task_holdover_second(void)
{
    /* Apply the last commanded DAC step once before a one-second prediction. */
    {
        int cur = control_get_last_dac();
        double du = (double)(cur - s_prev_dac) * DAC_GAIN
                    * (double)control_get_loop_sign();
        kalman.X[1] += du;
        s_prev_dac = cur;
    }
    holdover_seconds++;

    if (resume_flag != 0) {
        kalman.R = 5000.0;
        /* 有有效测量才做平滑, 否则 z 是陈旧值/0, 会把恢复过程带偏。
         * holdover_seconds 照常累加, 不因为暂时没有测量而卡住恢复流程。 */
        if ((bsp_meas_valid() != 0) && (g_last_why == BSP_Q_OK) && (g_last_acc != 0)) {
            /* z and X[0] must share the accumulated phase reference. */
            kalman_update(&kalman, main_phase_acc_ns, 1.0);
        } else {
            kalman_predict_only(&kalman, 1.0);
            holdover_seconds = 0; /* require consecutive usable recovery observations */
        }
        cand_obs_predict_only(1.0); /* passive diagnostics only */
        control_holdover_tick();

        if (holdover_seconds >= HOLDOVER_RESUME_SECONDS) {
            control_resume_from_holdover();   // 重置 X[0]/P, 保留 X[1]
            resume_flag   = 0;
            invalid_count = 0;
            sample_period = 1;

            /* 2026-10-30：回到【失锁前】的状态，而不是写死 TUNING。
             * 从 CALIB 进来的失锁必须回 CALIB —— 否则一回 TUNING
             * 就开始 control_apply_freq()，把冻结的 DAC 推走。*/
            {
                int back = state_before_holdover;
                state_before_holdover = STATE_TUNING;   /* 复位，下次重新记 */
                if (main_log != MAIN_LOG_QUIET) {
                    dbg_printf("恢复完成, 回到 %s\r\n", state_name(back));
                }
                enter_state(back);
            }
        }
    } else {
        kalman_predict_only(&kalman, 1.0); /* this task runs once per second */
        cand_obs_predict_only(1.0);
        control_holdover_tick();                              // DAC 保持
    }
}

//=============================================================================
// 数据日志（UART1）
//   CSV 列: t_s,state,phase_ns,freq_ppb,dac,sat,hdop,cn0
//=============================================================================
static void log_csv(void)
{
    dbg_printf("%u,%s,%d,%d,%d,%d,%d,%d\r\n",
               (unsigned)log_seconds,
               state_name(sys_state),
               (int)(kalman.X[0] * 1000.0),        // 相位 ns x1000
               (int)(kalman.X[1] * 1000000.0),     // 频差 ppb x1000
               control_get_last_dac(),             // 12 bit 码值
               gnss_sat(),
               (int)(gnss_hdop() * 100.0),           // x100
               (int)(gnss_cn0() * 10.0));        // dB-Hz x10
}

static void log_text(void)
{
    int hdop100 = (int)(gnss_hdop() * 100.0 + 0.5);
    int cn010 = (int)(gnss_cn0() * 10.0 + 0.5);
    int dac100 = (int)(control_get_dac_acc() * 100.0 + 0.5);
    /* 字段和串口屏一 一对应, 方便两边对照 */
    /* 注意：不能用 "%d.%03d" 打印可能落在 (-1,0) 的数 —— (int)(-0.15)=0，
     * 负号会丢。frq / pps / phs 全部改成毫单位整数（符号安全）。 */
    dbg_printf("[%s] T=%d 慢环T=%u | frqm=%d phsm=%ld ppsm=%d | "
               "fix=%d sat=%d hdop=%d.%02d cn0=%d.%01d | dac=%d acc=%d.%02d | "
                "slow=%dm n=%d | P3u=%d R=%d f50=%d f50r=%d | fmod=%d fsrc=%d fw=%d fsl=%d phsg=%d pt=%d\r\n",
               state_name(sys_state), sample_period, (unsigned)slow.T,
               (int)(kalman.X[1] * 1000.0),
                (long)(kalman.X[0] * 1000.0),
               (int)(main_last_phase_ns * 1000.0),
               gnss_fix(), gnss_sat(),
               hdop100 / 100, hdop100 % 100,
               cn010 / 10, cn010 % 10,
               control_get_last_dac(),
               dac100 / 100, dac100 % 100,
               (int)(slow.freq * 1000.0), slow.npts,
                (int)(kalman.P[3] * 1000000.0), (int)g_R, (int)main_fix50_cnt, (int)main_fix50_real,
                (int)EXP_SLOW_MODE, g_fuse_src,
                (int)(g_fuse_w * 1000.0), (int)(g_fuse_fsl * 1000.0),
                 (int)EXP_PHASE_TERM_EN, (int)(g_phase_term * 1000.0));
}
//=============================================================================
// NMEA 数据通路诊断（关键：区分 3 种完全不同的原因）
//   byte_count > 0            -> 字节在进来, 问题在握手/读法
//   byte_count = 0 + frame_err-> 有信号但每帧都错 -> 波特率不匹配
//   byte_count = 0 + 无 frame  -> F17 上根本没有信号 -> 模块/接线
//=============================================================================
static void diag_nmea(void)
{
    GNSS_NMEA_DIAG nd;
    gnss_get_nmea_diag(&nd);   /* one snapshot before printing invokes idle hooks */
    uint32_t bc = Xil_In32(REG_BYTE_COUNT);
    uint32_t dc = Xil_In32(REG_DROP_COUNT);
    uint32_t st = Xil_In32(REG_STATUS);

    static uint32_t last_bc = 0;                 // 上次累计值, 用来算增量
    static uint32_t last_dc = 0;
    uint32_t dbc = bc - last_bc;                 // 本周期新增字节数 <- 最关键
    uint32_t ddc = dc - last_dc;

    last_bc = bc;
    last_dc = dc;

    dbg_printf("NMEA diag: byte=%u(+%u) drop=%u(+%u) ver=0x%08x | "
               "frame_err=%d pps_seen=%d locked=%d meas=%d spi_busy=%d | %s\r\n",
               (unsigned)bc, (unsigned)dbc, (unsigned)dc, (unsigned)ddc,
               (unsigned)Xil_In32(REG_VERSION),
               (int)((st & STATUS_FRAME_ERR) ? 1 : 0),
               (int)((st & STATUS_PPS_LOST) ? 0 : 1),
               (int)((st & STATUS_LOCKED) ? 1 : 0),
               (int)((st & STATUS_MEAS_VALID) ? 1 : 0),
               (int)((st & STATUS_SPI_BUSY) ? 1 : 0),
               (dbc > 0U) ? "OK: bytes arriving" :
               ((st & STATUS_FRAME_ERR) ? "SIGNAL present, framing errors -> check baud"
                                        : "NO SIGNAL on F17 -> check module / wiring"));
    dbg_printf("NMEA chk: ok=%lu badcs=%lu badfmt=%lu long=%lu resync=%lu gga_age=%lu fresh=%d\r\n",
               (unsigned long)nd.ok, (unsigned long)nd.bad_checksum,
               (unsigned long)nd.bad_format, (unsigned long)nd.overlong,
               (unsigned long)nd.resync, (unsigned long)nd.gga_age_ms,
               nd.quality_fresh);
}
//=============================================================================
// 串口屏命令（屏 -> PS，UART0 轮询接收）
//   T 回 TUNING   C 进 CALIB   R 清空自适应R采集
//   D 立即刷屏   S 立即回一行状态
//=============================================================================
static void handle_screen_cmd(void)
{
    char cmd[16];

    if (!screen_get_cmd(cmd, (int)sizeof(cmd))) {
        return;
    }

    switch (cmd[0]) {
    case 'T': case 't':
        sample_period = 1;
        enter_state(STATE_TUNING);
        break;

    case 'C': case 'c':
        enter_state(STATE_CALIB);
        break;

    case 'R': case 'r':
        adaptive_r_reset();
        if (main_log != MAIN_LOG_QUIET) {
            dbg_printf("ADAPT R: 采集清空, 重新开始\r\n");
        }
        break;

    case 'D': case 'd':
        screen_send_status();
        break;

    case 'S': case 's':
        log_text();
        break;

    default:
        break;
    }
}

//=============================================================================
// 上电自检：确认 PL 通路、两路串口、LED 引脚
//=============================================================================
//=============================================================================
// PS 按键 -> 状态机
//   KEY1 (MIO50, B13): 单击 = TUNING <-> CALIB 手动切换（守时中不响应）
//                      双击（2 s 内按两次）= 清自适应R采集 + 强制回 TUNING
//   KEY2 (MIO51, B9) : 切换串口屏页码（页0 控制状态 / 页1 GNSS）
//   注：本屏 TJC1612T118_011N 无触摸，所有操作都在两个键上。
//                      原「清自适应R+强回TUNING」已挪到屏命令 'R' / 'T'
//=============================================================================
/* KEY1 双击判定窗口（ms）：2 s 内按两次 = 复位 */
#define KEY1_DBLCLICK_MS   2000U

/* 上一次 KEY1 事件发生的时刻（ms）；0 = 还没有前一次 */
static uint32_t s_key1_last_ms = 0U;

static void handle_ps_key(void)
{
    int ev = ps_key_get_event();

    if (ev == PS_KEY_EVENT_1) {
        uint32_t now = bsp_millis();

        /* ---- KEY1 双击 = 原 KEY2 的复位动作 ----------------------------
         * 2 s 内第二次按下：清自适应 R 采集 + 判定窗口归零 + 强制回 TUNING。
         * 第一次按下【立即】照常切 TUNING/CALIB，不延迟，所以没有手感滞后；
         * 双击时状态会先切一次再被复位回 TUNING，最终停在 TUNING，符合预期。
         * 双击后 last_ms 清零，第三次按会重新当作单击。
         * 注意：ps_key.c 的事件队列深度为 1，两次按键请间隔 200 ms 以上，
         *       且别正好落在发屏那 0.3 s 里（漏了再按一次即可）。 */
        if ((s_key1_last_ms != 0U) &&
            ((uint32_t)(now - s_key1_last_ms) <= KEY1_DBLCLICK_MS)) {
            s_key1_last_ms = 0U;

            adaptive_r_reset();
            sample_period = 1;
            invalid_count = 0;          /* 重新给 5 秒判定窗口 */
            epoch         = 0;
            enter_state(STATE_TUNING);

            if (main_log != MAIN_LOG_QUIET) {
                dbg_printf("KEY1 x2: 清空自适应R采集, 强制回 TUNING\r\n");
            }
            return;
        }
        s_key1_last_ms = now;

        /* ---- 单击：TUNING <-> CALIB ---- */
        if (sys_state == STATE_TUNING) {
            enter_state(STATE_CALIB);
        } else if (sys_state == STATE_CALIB) {
            enter_state(STATE_TUNING);
        } else if (main_log != MAIN_LOG_QUIET) {
            dbg_printf("KEY1: 守时中, 忽略(等 GNSS 恢复)\r\n");
        }
    } else if (ev == PS_KEY_EVENT_2) {
        /* KEY2 = 串口屏切页（页 0 控制状态 / 页 1 GNSS 信息）
         *
         * 本屏 TJC1612T118_011N 无触摸，所以不用屏命令，只靠两个键：
         *     KEY1 单击 = TUNING <-> CALIB
         *     KEY1 双击 = 清自适应R + 强回 TUNING（见上面 KEY1 分支）
         *     KEY2      = 切页 */
        int pg = screen_page_next();
        if (main_log != MAIN_LOG_QUIET) {
            dbg_printf("KEY2: 屏切到第 %d 页 (%s)\r\n", pg, screen_page_name());
        }
    }
}


static void self_test(void)
{
    uint32_t rb;
    int i;

    dbg_printf("\r\n");
    dbg_printf("========== GPSDO PS 自检 ==========\r\n");
    dbg_printf("AXI base    = 0x%08x\r\n", (unsigned)GNSS_BASE_ADDR);

    rb = Xil_In32(REG_DAC_DATA) & 0xFFFU;
    dbg_printf("DAC register (read only) = 0x%03x\r\n", (unsigned)rb);

    dbg_printf("UART0(屏)   = %s\r\n", screen_ready_flag() ? "OK" : "FAIL");
    dbg_printf("UART1(调试) = %s\r\n", dbg_ready()        ? "OK" : "FAIL");
    dbg_printf("PS LED 引脚 = %s\r\n", led_pin_ready()    ? "OK" : "FAIL");
    dbg_printf("PS 按键     = %s\r\n", ps_key_ready()     ? "OK" : "FAIL");

    dbg_printf("寄存器初值:\r\n");
    for (i = 0; i < 4; i++) {
        dbg_printf("  +0x%02X = 0x%08x\r\n", i * 4,
                   (unsigned)Xil_In32(GNSS_BASE_ADDR + (uint32_t)(i * 4)));
    }
    dbg_printf("提示: +0x04(NMEA) 长期为 0 -> 查 PL UART(F17) 与 GNSS 模块接线/波特率\r\n");
    diag_nmea();
    dbg_printf("===================================\r\n\r\n");
}
//=============================================================================
// 原始 NMEA -> UART1
//-----------------------------------------------------------------------------
// 用法: main() 里调 nmea_dump_init() 打开, nmea_dump_stop() 关闭,
//       nmea_dump_stats() 随时查累计统计。
//
// 原理: gnss.c 每从 PL 的 0x04 寄存器取出一个字符, 就在交给解析器之前回调
//       nmea_dump_char()。看到的是模块吐出来的完整原始字节流 —— 不加工、
//       不丢字节; 同时状态机照常解析, 两边不抢数据。
//
// 带宽: GNSS 模块 9600 8N1, 每秒最多 960 字符; UART1 跑 115200, 完全够。
//       dbg_putc() 是阻塞发送(每字节约 87us), 满速时约占 8% CPU。
//
// 注意: 打开 dump 后 main_log 会被置成 MAIN_LOG_QUIET, 否则每秒的状态行会
//       和原始 NMEA 混在一起。要区分很简单: NMEA 语句都以美元符号开头。
//=============================================================================
#define NMEA_DUMP_ENABLE    0       // 1 = 上电就把原始 NMEA 全部打到 UART1
#define NMEA_DUMP_LINE_MAX  128
/* 1 = 每个字节按两位十六进制打印(查波特率/位偏移用), 0 = 原样输出原始 NMEA。
 * 查波特率时先置 1: 如果收到的是 24 4E 52 4D 43 ... 说明其实是对的, 只是终端
 * 显示问题; 如果是 {\xNN} 之类的规律图案, 就是位偏移/波特率不匹配。 */
#define NMEA_DUMP_HEX    0

/*-----------------------------------------------------------------------------
 * 比赛/生产模式: 下面这一大段(原始 NMEA 转储 + 波特率自扫描)只在上板调试时用。
 * 两个开关都为 0 时整段不参与编译, 也就不会产生 -Wunused 警告。
 * 要重新调试: 把 NMEA_DUMP_ENABLE / BAUD_SCAN_ENABLE 之一置 1 即可。
 *-----------------------------------------------------------------------------*/
#if (NMEA_DUMP_ENABLE) || (BAUD_SCAN_ENABLE)

static char     nmea_line[NMEA_DUMP_LINE_MAX];   // 最近一条完整语句
static int      nmea_line_len = 0;
static uint32_t nmea_byte_cnt = 0;               // 累计原始字节数
static uint32_t nmea_line_cnt = 0;               // 累计完整语句数
static int      nmea_dump_on  = 0;
static int      nmea_dump_print = 1;   // 0 = 只统计不输出(扫描波特率时用)
static uint32_t scan_n   = 0;          // 本窗口收到的字节数
static uint32_t scan_asc = 0;          // 其中看起来像 NMEA 的字节数

// 每收到一个原始字符调用一次(由 gnss.c 回调, 在解析之前)
static void nmea_dump_char(char c)
{
    if (nmea_dump_print) {
#if (NMEA_DUMP_HEX)
        static const char hx[] = "0123456789ABCDEF";
        static int col = 0;
        dbg_putc(hx[(c >> 4) & 0x0F]);
        dbg_putc(hx[c & 0x0F]);
        dbg_putc(' ');
        if (++col >= 32) { col = 0; dbg_puts("\r\n"); }
#else
        dbg_putc(c);                            // 原样送 UART1
#endif
    }

    scan_n++;
    if ((c == '\r') || (c == '\n') || ((c >= 0x20) && (c <= 0x7E))) { scan_asc++; }

    if (c == '\n') {                            // 2) 本地留一份最近语句
        nmea_line_cnt++;
        nmea_line[nmea_line_len] = '\0';
        nmea_line_len = 0;
    } else if ((c != '\r') && (nmea_line_len < NMEA_DUMP_LINE_MAX - 1)) {
        nmea_line[nmea_line_len++] = c;
    }
    nmea_byte_cnt++;
}

// 打开: 把原始 NMEA 全部转到 UART1
void nmea_dump_init(void)
{
    nmea_line_len = 0;
    nmea_byte_cnt = 0;
    nmea_line_cnt = 0;
    gnss_set_raw_hook(nmea_dump_char);
    nmea_dump_on  = 1;
    dbg_printf("\r\n---- NMEA raw dump -> UART1 ON ----\r\n");
}

// 关闭: 打印累计统计
void nmea_dump_stop(void)
{
    gnss_set_raw_hook(0);
    nmea_dump_on = 0;
    dbg_printf("\r\n---- NMEA raw dump OFF: %u bytes / %u sentences ----\r\n",
               (unsigned)nmea_byte_cnt, (unsigned)nmea_line_cnt);
}

// 查询: 累计统计 + 最近一条语句
void nmea_dump_stats(void)
{
    dbg_printf("NMEA dump %s: %u bytes, %u sentences, last=[%s]\r\n",
               (nmea_dump_on != 0) ? "ON" : "OFF",
               (unsigned)nmea_byte_cnt, (unsigned)nmea_line_cnt, nmea_line);
}

//=============================================================================
// UART 波特率自动匹配
//-----------------------------------------------------------------------------
// 背景: GNSS 模块的出厂波特率不一定是 9600。115200 = 12 x 9600 非常常见, 而
//       FPGA 若写死 9600, 收到的不是没有数据, 而是一堆看着像数据的乱码
//       (NMEA 是 ASCII, bit7 恒为 0; 错位采样出来的字节约一半 bit7 是 1)。
//
// 现在 uart_rx 的分频是运行时寄存器(AXI 0x40 BAUD_DIV = 参考时钟/波特率),
// 所以上电扫一遍就能定死, 不必反复重新综合。
//
// 判据: 收到的字节里像 NMEA 的字节(可打印 ASCII 或 CR/LF)所占比例
//         波特率对 -> 接近 100%
//         波特率错 -> 约 40~60% (bit7 随机)
//       只看速率不行: 模块每秒吐多少字节由它的占空比决定, 与波特率无关。
//=============================================================================
#define BAUD_SCAN_ENABLE    0            // 1 = 上电自动扫一遍
#define BAUD_SCAN_MS       1000U        // 每个候选观察多久(覆盖一整秒的 NMEA 周期)
#define BAUD_SCAN_MIN_N    50U          // 样本太少不算数
#define BAUD_SCAN_MIN_ASC  90U          // 可打印比例下限(%)
#define BAUD_CLK_HZ        100000000U   // PL 参考时钟 FCLK_CLK0

static const uint32_t baud_cand[] = { 9600U, 115200U, 38400U, 57600U, 19200U, 4800U, 230400U };
#define BAUD_CAND_N  ((int)(sizeof(baud_cand) / sizeof(baud_cand[0])))

static int      scan_idx   = -1;        // -1 = 不在扫描
static uint32_t scan_t0    = 0;
static uint32_t scan_bc0   = 0;
static uint32_t baud_found = 0;

static uint32_t nmea_total_bytes(void)
{
    // byte_count 只统计 ARM 真正读走的, drop_count 统计被丢的;
    // 两者之和 = UART 成功成帧的总字节数, 与 ARM 轮询快慢无关。
    return Xil_In32(REG_BYTE_COUNT) + Xil_In32(REG_DROP_COUNT);
}

static void baud_scan_start(void)
{
    uint32_t rb;

    /* ---- 先做一次 BAUD_DIV 回环自检 --------------------------------
     * 写一个魔数再读回。如果 PL 里还是旧 bitstream(最常见的原因:
     * 这个 IP 是 OOC 综合的, 它的综合运行没重跑, 顶层拿的是缓存的旧网表),
     * 这里读回来的就不是魔数 —— 那么后面 7 秒的扫描毫无意义, 直接跳过。
     * ---------------------------------------------------------------- */
    bsp_set_baud_div(0x0001ABCDU);
    rb = bsp_get_baud_div();
    dbg_printf("\r\n---- BAUD_DIV loopback: wrote 0x0001ABCD, read 0x%08x -> %s\r\n",
               (unsigned)rb,
               (rb == 0x0001ABCDU) ? "OK (new PL bitstream)"
                                   : "FAIL -> PL IS STILL THE OLD BITSTREAM");
    if (rb != 0x0001ABCDU) {
        dbg_printf("---- scan skipped. Re-run the IP OOC synthesis: ----\r\n");
        dbg_printf("----   reset_run gnss_bd_gnss_gpsdo_top_0_0_synth_1  ----\r\n\r\n");
        nmea_dump_print = 1;
        scan_idx = -1;
        return;
    }

    scan_idx   = 0;
    baud_found = 0U;
    nmea_dump_print = 0;                // 扫描期间只统计, 不刷屏
    scan_n = 0; scan_asc = 0;
    bsp_set_baud_div(BAUD_CLK_HZ / baud_cand[0]);
    scan_bc0 = nmea_total_bytes();
    scan_t0  = bsp_millis();
    dbg_printf("---- UART baud scan (%d candidates) ----\r\n", BAUD_CAND_N);
}

// 主循环高频调用, 非阻塞
static void baud_scan_tick(void)
{
    uint32_t now, dt, tot, ratio, rate;

    if ((scan_idx < 0) || (baud_found != 0U)) { return; }

    now = bsp_millis();
    dt  = now - scan_t0;
    if (dt < BAUD_SCAN_MS) { return; }

    tot   = nmea_total_bytes();
    rate  = (tot - scan_bc0) * 1000U / dt;
    ratio = (scan_n != 0U) ? (scan_asc * 100U / scan_n) : 0U;

    dbg_printf("  %6u baud (div=%5u): %5u B/s, ASCII %3u%% (n=%u)\r\n",
               (unsigned)baud_cand[scan_idx],
               (unsigned)(BAUD_CLK_HZ / baud_cand[scan_idx]),
               (unsigned)rate, (unsigned)ratio, (unsigned)scan_n);

    if ((ratio >= BAUD_SCAN_MIN_ASC) && (scan_n >= BAUD_SCAN_MIN_N)) {
        baud_found = baud_cand[scan_idx];
        bsp_set_baud_div(BAUD_CLK_HZ / baud_found);
        dbg_printf("---- FOUND: %u baud (BAUD_DIV = %u) ----\r\n\r\n",
                   (unsigned)baud_found, (unsigned)(BAUD_CLK_HZ / baud_found));
        nmea_dump_print = 1;
        scan_idx = -1;
        return;
    }

    scan_idx++;
    if (scan_idx >= BAUD_CAND_N) {
        dbg_printf("---- NO baud matched: check module power / TX wiring ----\r\n");
        dbg_printf("---- falling back to %u baud ----\r\n\r\n", (unsigned)baud_cand[0]);
        bsp_set_baud_div(BAUD_CLK_HZ / baud_cand[0]);
        nmea_dump_print = 1;
        scan_idx = -1;
        return;
    }

    bsp_set_baud_div(BAUD_CLK_HZ / baud_cand[scan_idx]);
    scan_bc0 = nmea_total_bytes();
    scan_n = 0; scan_asc = 0;
    scan_t0  = now;
}


#endif  /* NMEA_DUMP_ENABLE || BAUD_SCAN_ENABLE */

/* 诊断行里 "扫描进行中就不要打" 的那个条件。生产模式下恒为 1。 */
#if (NMEA_DUMP_ENABLE) || (BAUD_SCAN_ENABLE)
#  define SCAN_IDLE  (scan_idx < 0)
#else
#  define SCAN_IDLE  1
#endif

#if (BENCH_ENABLE)
//=============================================================================
// 台架模式的观察行: 真值 / 估计 / 双路状态 / 真实模块状态 一起打
//=============================================================================
static void log_bench(void)
{
    int dac = (int)(Xil_In32(REG_DAC_DATA) & 0xFFFU);

    dbg_printf("[BENCH] t=%u 真值=%d.%03dppb 相位=%d.%03dns | "
               "X0=%d.%03d X1=%d.%03d R=%d | DAC=%d | L1=%s L2=%s T=%us N=%d done=%d\r\n",
               (unsigned)log_seconds,
               (int)bench_true_ppb(), frac_of(bench_true_ppb(), 1000),
               (int)bench_true_phase_ns(), frac_of(bench_true_phase_ns(), 1000),
               (int)(kalman.X[0] * 1000.0),
               (int)(kalman.X[1] * 1000.0),
               (int)kalman.R,
               dac,
               (slow.active == 0) ? "DAC" : "监视",
               (slow.active == 0) ? "off" : "接管",
               (unsigned)slow.T, slow.N, slow.done);

    /* 这一行读的是 gnss 结构体, 也就是【真实模块】解析出来的数据 */
    dbg_printf("[BENCH]      真实模块: fix=%d sat=%d hdop=%d.%02d cn0=%d.%01d"
               " | 频率估计可信度 sqrt(P11)=%d.%03dppb (门限 %d.%03d)\r\n",
               gnss.fix_valid, gnss.satellites,
               (int)gnss.hdop, frac_of(gnss.hdop, 100),
               (int)gnss.cn0_avg, frac_of(gnss.cn0_avg, 10),
               (int)slow_sqrt(kalman.P[3]), frac_of(slow_sqrt(kalman.P[3]), 1000),
               (int)SLOW_P11_MAX, 0);
}
#endif

#if (CENTER_CALIB_ENABLE)

//=============================================================================
// 定点小数打印: 带符号 + 固定位数(补前导零), 不用 %0*d 免得触发 snprintf 警告
//=============================================================================
static void cal_p(const char *name, double v, int dec, int scale)
{
    char fs[5];
    int  neg = (v < 0.0);
    double a = neg ? -v : v;
    int  ip  = (int)a;
    int  fp  = frac_of(a, scale);
    int  k;

    for (k = dec - 1; k >= 0; k--) {
        fs[k] = (char)('0' + (fp % 10));
        fp /= 10;
    }
    fs[dec] = '\0';

    dbg_printf("%s%s%d.%s", name, neg ? "-" : "", ip, fs);
}

//=============================================================================
// 在固定 DAC 上采 N 点，打印相位差与 X[1]，返回收敛后的 X[1] (ppb)
//-----------------------------------------------------------------------------
//   约定（板上的 bitstream，PL 未改动）：
//       0x00 PHASE_ERR 正 = OCXO 慢      =>  X[1] 正 = 越来越慢
//
//   * DAC 用 Xil_Out32 直接钉死，全程【不调 control_apply_freq()】---
//     这就是开环：相位才会随频率偏差线性漂移，X[1] 才有物理意义。
//   * 每点先 kalman_init()，前 2/3 是收敛过程丢掉，只平均后 1/3。
//   * R 固定 96（不跑自适应），否则两次测量不可比。
//=============================================================================
static double cal_measure_at(int dac)
{
    int    n = 0, n_fail = 0, n_avg = 0;
    double sum = 0.0;
    double z;

    kalman_init(&kalman);
    Xil_Out32(REG_DAC_DATA, (uint32_t)dac);      /* 钉死，之后绝不动它 */

    dbg_printf("\r\n[CAL] ===== DAC = %d, 采集 %d 点（每点 1 PPS）=====\r\n", dac, CAL_N_POINT);

    while (n < CAL_N_POINT) {
        while (!one_second_elapsed()) {
            gnss_pack_task();                   /* 别把 NMEA 饿死 */
        }
        gnss_pack_task();

        if (bsp_meas_valid() == 0) {
            n_fail++;
            dbg_printf("[CAL]   %2d/%d  meas_valid=0（无 1PPS），跳过\r\n", n + 1, CAL_N_POINT);
            if (n_fail >= 60) {
                dbg_printf("[CAL] 连续 60 s 没有有效 1PPS，放弃本测点\r\n");
                return 0.0;
            }
            continue;
        }
        n_fail = 0;

        z = read_phase_error_ns();
        kalman.R = 96.0;
        kalman_update(&kalman, z, 1.0);
        n++;

        dbg_printf("[CAL]   %2d/%d   相位差=", n, CAL_N_POINT);
        cal_p("", z, 3, 1000);
        dbg_printf(" ns    X0=");
        cal_p("", kalman.X[0], 3, 1000);
        dbg_printf("    X1=");
        cal_p("", kalman.X[1], 3, 1000);
        dbg_printf(" ppb    sqrt(P11)=");
        cal_p("", slow_sqrt(kalman.P[3]), 3, 1000);
        dbg_printf("\r\n");

        if (n > (CAL_N_POINT * 2 / 3)) {          /* 只统计后 1/3 */
            sum += kalman.X[1];
            n_avg++;
        }
    }

    dbg_printf("[CAL] DAC=%d 收敛后 X1 平均（末 %d 点）= ", dac, n_avg);
    cal_p("", sum / (double)n_avg, 3, 1000);
    dbg_printf(" ppb\r\n");

    return sum / (double)n_avg;
}

//=============================================================================
// 标定主流程：中心值处采 30 点，打印相位差与 X1，并给出极性判读方法
//===========================================================================*/
static void center_calib_run(void)
{
    double x1;
    int    center;

    /* main.c 里的设定 -> control 模块（两边保持一致） */
    control_set_center(CAL_DAC_CENTER);
    center = control_get_center();

    dbg_printf("\r\n########## 中心值标定（单点，30 x 1PPS）##########\r\n");
    dbg_printf("固件        : %s %s\r\n", __DATE__, __TIME__);
    dbg_printf("中心值      : DAC = %d  (control_get_center())\r\n", center);
    dbg_printf("环路基性    : LOOP_SIGN = %+d\r\n", control_get_loop_sign());
    dbg_printf("采样点数    : %d\r\n", CAL_N_POINT);
    dbg_printf("开环        : DAC 钉死，不调用 control_apply_freq()\r\n");

    x1 = cal_measure_at(center);

    dbg_printf("\r\n########## 结果 ##########\r\n");
    dbg_printf("相位差终值  : "); cal_p("", kalman.X[0], 3, 1000); dbg_printf(" ns\r\n");
    dbg_printf("X1 平均     : "); cal_p("", x1, 3, 1000); dbg_printf(" ppb   （正 = 超前/快）\r\n");

    dbg_printf("\r\n---- 怎么判极性 ----\r\n");
    dbg_printf("把 CAL_DAC_CENTER 改成一个差 1024 的值（例如 %d），再跑一次：\r\n", center + 1024);
    dbg_printf("  X1 变大（更正） -> 抬高 DAC 让频率变快 -> 正极性 -> LOOP_SIGN = +1\r\n");
    dbg_printf("  X1 变小（更负） -> 抬高 DAC 让频率变慢 -> 负极性 -> LOOP_SIGN = -1\r\n");
    dbg_printf("两次的 X1 之差 / 1024 就是灵敏度 g（ppb/code）。\r\n");

    dbg_printf("\r\n---- 怎么判中心 ----\r\n");
    dbg_printf("X1 应该接近 0。若明显非 0，说明 %d 不是真实中心：\r\n", center);
    dbg_printf("真实中心 = %d - X1/g，g 由上面两次测量得到。\r\n", center);

    dbg_printf("\r\n标定完成。把 CENTER_CALIB_ENABLE 改回 0 再烧一次。\r\n");
}

#endif /* CENTER_CALIB_ENABLE */

//=============================================================================
// 主程序
//=============================================================================
//=============================================================================
// 上板标定模式（TEST_MODE）
//-----------------------------------------------------------------------------
//   0   正常运行（比赛 / 展示，默认）
//   1   只打寄存器快照
//   2   只测 OCXO 极性 + 灵敏度 g（ppb/码、Hz/码）
//   3   只测 TDC 标定（写 0x0C / 0x10）
//   9   全部跑一遍
//
// 所有输出走 UART1，参数在 hwtest.h 里改。
// 设成非 0 后，main() 在 self_test() 之后直接进入标定并停在那里，
// 不会跑闭环，所以 DAC 不会被控制器改动，读数才有意义。
//=============================================================================
/* 2026-10-25 15:xx: 3 -> 0
 * 双环（Route B）改完后要采 1 小时闭环数据，必须回到正常调谐模式；
 * 非 0 时 main() 会在 self_test() 后直接进标定并停在那里，不跑闭环。 */
#define TEST_MODE   0

/*-----------------------------------------------------------------------------
 * fix_50ns_glitch -- 修正粗计数器的 ±1 误计（±50 ns）
 *
 * 成因：粗计数 LSB 是一个 OCXO 周期（20 MHz 下 50 ns），而 TDC 细项只补偿
 *       一个 100 MHz 周期（10 ns），二者 5:1。1PPS 边沿落进 OCXO 边沿的
 *       模糊窗口时粗计数偶发 ±1，幅值约 50 ns，细项补不回来。
 *       2026-10-25 实测占比约 10%，幅值 40~50 ns。
 *
 * 修法：这是确定性偏移，直接把 50 ns 加回去。比丢弃样本好 —— 丢弃会破坏
 *       相位累加器的连续性，实测会直接把环路锁死。
 *-----------------------------------------------------------------------------*/
static double        s_prev_inc  = 0.0;
static double        s_prev_raw  = 0.0;   /* 上一次的【原始】增量（未修正），供 ±50 判据用 */

/* ---- ±50 ns 修正器的"真实阶跃"判定 ------------------------------------
 * 单次毛刺：下一秒 d 反向，计数清零，永远不会误判。
 * 真实阶跃：之后每秒 d 都还是同向 50，连续 N 次即判定为真实变化。
 * N=5 时，即使毛刺率高达 50%，误判概率也只有 (0.25)^5 = 0.10%。
 * 判定为真实变化后锁定 FIX50_LOCK_S 秒不再修正，让原始值直接进卡尔曼。
 * ---------------------------------------------------------------------- */
#define FIX50_RUN_MAX   5       /* 连续同向次数门限 */
#define FIX50_LOCK_S    120U    /* 判定后锁定多少秒不修正 */

static int      s_fix_dir  = 0;      /* 当前同向方向 +1/-1/0 */
static int      s_fix_run  = 0;      /* 已连续同向次数 */
static uint32_t s_fix_lock = 0U;     /* 剩余锁定秒数 */

static double fix_50ns_glitch(double p)
{
    /* ---- 2026-10-26：±50 ns 修正器【已停用】------------------------------
     * 这个修正器是 10-25 为了补 PL 侧那个 5:1 量程失配而加的补丁：
     *   粗计数 1 格 = 50 ns，而 TDC 只有 10 ns 量程，补不满。
     * PL 侧的 ocxo_frac 上线后，0x00 的量化噪声从 50 ppb 降到 0.039 ppb，
     * 不会再出现整 50 ns 的跳变 —— 此时任何 ±50 都是【真实变化】，
     * 再按毛刺去修就会把真值改掉，反而制造误差。
     *
     * 这里保留函数和 s_prev_raw 的更新（对外接口不变、日志字段保留），
     * 但不再修改 p。日志里的 f50 会恒为 0 —— 万一哪天 PL 回退，
     * 一眼就能从 f50 重新涨起来看出问题。
     * -------------------------------------------------------------------- */
    s_prev_raw = p;
    s_prev_inc = p;
    return p;
}
int main(void)
{
    // ---- 1. 初始化（顺序有讲究）----
    dbg_init();            // UART1 必须最先: 后面所有打印都靠它
    bsp_timer_init();      // 1 秒硬件节拍 (SCU 私有定时器)
    screen_init();         // UART0 -> 串口屏
    led_ctrl_init();       // PS LED1 (E6/MIO0)
    ps_key_init();         // PS 按键 KEY1= MIO50 / KEY2= MIO51

#if (NMEA_DUMP_ENABLE)
    /* 原始 NMEA 全部转到 UART1 —— 放在这里, 保证第一次 gnss_pack_task()
     * 之前钩子已经注册好。想改成 parse-only 就把 NMEA_DUMP_ENABLE 置 0。 */
    main_log         = MAIN_LOG_QUIET;   // UART1 上只留原始 NMEA, 便于直接存盘
    gnss_debug_level = GNSS_DBG_OFF;     // 关掉 gnss.c 自带回显, 免得重复输出
    nmea_dump_init();
#else
    /* 不开 dump 时, 想要哪种回显把下面这行改成 CHAR / SENTENCE / FIELDS */
    gnss_debug_level = GNSS_DBG_OFF;
#endif

    gnss_task_init();

    /* NMEA 调试级别：OFF = 只解析不回显；
     * 需要看原始语句时改成 GNSS_DBG_SENTENCE（UART1 会打印每条语句）。 */
    gnss_debug_level = GNSS_DBG_OFF;

    /* 把 NMEA 抽干挂到屏/日志的发送间隙上（见 bsp_idle 的说明） */
    bsp_set_idle_hook(gnss_pack_task);
    kalman_init(&kalman);
    cand_obs_init();          /* 2026-10-31: 候选观察器（必须在主循环前）*/
    adaptive_r_init();
    control_init();        // DAC 写中心值 DAC_CENTER(1752)
    slow_init();           // 第二路(慢环): T=1s, 未接管 DAC
    c_fusion_reset(SLOW_CW_DISABLED);

#if (BENCH_ENABLE)
    bench_init();          // 台架模式: 无外设自测
#endif

    sys_state     = STATE_TUNING;
    sample_period = 1;

    /*---------------------------------------------------------------
     * 构建时间戳横幅
     *   __DATE__ / __TIME__ 是【本文件被编译的时刻】。
     *   所以：
     *     - 只要横幅变了      -> 板子上确实换了新 elf
     *     - 横幅没变          -> Vitis 没真重编, 或下载没生效
     *     - 横幅变了但行为没变 -> 代码逻辑问题
     *   比对着 hdop 判断可靠得多（那一行有可能被后面的打印冲掉）。
     *--------------------------------------------------------------*/
    dbg_printf("\r\n===== BUILD " __DATE__ " " __TIME__ " =====\r\n");
    dbg_printf("KFCONF profile=%d QF_e9=%ld p3floor=%d fixed_period=%d phsg=%d slow_mode=%d\r\n",
               (int)KALMAN_PROFILE, (long)(KALMAN_Q_FREQ * 1000000000.0 + 0.5),
               (int)KALMAN_P3_FLOOR_ENABLE, (int)EXP_FIXED_PERIOD,
               (int)EXP_PHASE_TERM_EN, (int)EXP_SLOW_MODE);
    dbg_printf("CFCONF scale_u=%ld cap_u=%ld rise_s=%ld age_max_s=%ld lag_max_s=%ld\r\n",
               (long)(SLOW_C_WEIGHT_SCALE * 1000000.0 + 0.5),
               (long)(SLOW_C_WEIGHT_MAX * 1000000.0 + 0.5),
               (long)SLOW_C_WEIGHT_RISE_S, (long)SLOW_C_MAX_AGE_S,
               (long)SLOW_C_MAX_LAG_S);
    dbg_printf("FINALCONF release=%s R_ns2=%d R_train=%d period_fixed=%d phase=%d slow=%d\r\n",
               FINAL_RELEASE_TAG, (int)ADAPTIVE_R_FIXED_NS2, (int)EXP_R_TRAIN_ENABLE,
               (int)EXP_FIXED_PERIOD, (int)EXP_PHASE_TERM_EN, (int)EXP_SLOW_MODE);



    self_test();


    /* TDC 标定量：hwtest mode 3 实测后改 bsp.h 的两个宏 */
    bsp_set_tdc_calib((uint16_t)TDC_TAP_PERIOD_MEASURED,
                      (uint16_t)TDC_TAP_OFFSET_MEASURED);
#if (TEST_MODE != 0)
    /*=====================================================================
     * 上板标定模式：接管整个程序，不进状态机、不跑闭环
     *   测完不会返回，按 KEY1 可重跑一次
     *===================================================================*/
    hwtest_run(TEST_MODE);

    while (1) {
        bsp_idle();                       /* 保持 NMEA 通道畅通 */
        ps_key_poll();
        if (ps_key_get_event() == PS_KEY_EVENT_1) {
            hwtest_run(TEST_MODE);        /* KEY1 重跑 */
        }
    }
#endif /* TEST_MODE */

#if (BAUD_SCAN_ENABLE)
    baud_scan_start();     // 上电自动匹配 GNSS 模块的波特率(仅上板调试)
#else
    /* 比赛/生产模式: 不扫描, 直接锁定实测到的波特率。
     * BAUD_DIV = FCLK_CLK0 / 波特率 = 100 MHz / 9600 = 10416。
     * 显式写一次, 这样即使以后改动 IP 里 BAUD_RATE 的默认值也不会受影响。 */
    bsp_set_baud_div(10416U);
#endif

#if (CENTER_CALIB_ENABLE)
    // ---- 标定模式：接管整个程序，不进状态机 ----
    center_calib_run();

    // 标定完停在这里，方便把 UART1 的输出抄下来
    while (1) {
        gnss_pack_task();
    }
#else
    // ---- 2. 主循环 ----
    while (1) {
        // 2.1 高频任务
        gnss_pack_task();      // 0x04 -> 拼 NMEA 语句 -> 解析
#if (BAUD_SCAN_ENABLE)
        baud_scan_tick();      // 非阻塞波特率扫描(仅上板调试)
#endif
        screen_poll_rx();      // 串口屏回传字节入队
        handle_screen_cmd();   // 命令 -> 状态机
        led_ctrl_tick();       // LED 到半周期翻转
        bsp_tick();            // 软件计数(仅给 gnss.last_update_tick 用)

        // 2.2 PS 按键: 高频采样 + 软件去抖 + 事件
        ps_key_poll();
        handle_ps_key();

        // 2.3 每秒任务（硬件定时, 与主循环速度无关）
        if (one_second_elapsed()) {
#if (BENCH_ENABLE)
            /* 必须在 check_gnss_lost() 之前: 它要看到'有定位'才不会进 HOLDOVER */
            bench_tick();                  // 推进台架模型(相位/第二路样本)
            bench_gnss_override();         // 把 GNSS 状态改成有定位(先存真值做对照)
#endif
            check_gnss_lost();                 // 失锁 / 恢复判定
#if (!BENCH_ENABLE)
            uint32_t c_sample_ms = bsp_millis();
            int c_sample_ok = 0;
            /* 0x00 = 本秒相位增量(ns/s)，累加成绝对相位给滤波器 */
            /* 必须同时满足两个条件才累加：
             *   meas_valid : 0x00 里有一个锁存值
             *   pps_ok     : 接收机确实在输出 1PPS（0x2C 的 pps_lost 为 0）
             * 只判 meas_valid 不够 —— 拔掉天线时 sat=0/fix=0，但 meas_valid
             * 仍可能是 1，此时 0x00 是【上一次的陈旧值】，每秒累加同一个值
             * 会让相位撞到 int32 上限（实测 phsm = -2147483648）。 */
            /* ---- 2026-10-30：加样本质量门 ----------------------------------
             * 实测 t=547 秒出现过一次 2 秒间隔，PL 把相位钳到 INT32_MAX
             * (0x7FFFFFFF)，原代码只把 SAT 位打进日志、从不用它拒绝，
             * 于是 0x7FFFFFFF × 0.1953125 = 419.4 ms 被累加进相位，
             * 相位项直接顶到 K_PHASE_MAX 限幅，DAC 被错误推走。
             *
             * 现在先过 bsp_sample_good()：
             *   ① 饱和  ② 撕裂（seq0!=seq1）③ 间隔异常（±10%）
             * 命中则【跳过本拍、不清零】—— 相位历史保留，坏样本不污染。
             * ---------------------------------------------------------- */
            {
                /* ---- 2026-10-30 统一门控 ----------------------------------
                 * 相位累加与 KF/DAC 控制共用同一份快照、同一个判定。
                 * 好样本：累加相位、标记已消费。
                 * 坏样本：不累加，但用 X[1] 补上这一秒的相位增量，保证
                 *         累加器与真实相位连续（否则下个窗口会出现一个
                 *         大小为 freq*1s 的假新息，1 ppb 就是 -1 ns）。
                 *         同时置 gap 标记，让控制跳过一次。
                 * ---------------------------------------------------------- */
                bsp_snapshot_t snap;
                bsp_snapshot_take(&snap);

                sample_why = bsp_sample_classify(&snap);
                sample_acc = 0;
                g_last_seq = snap.seq0;

                if (sample_why == BSP_Q_OK) {
                    double p = fix_50ns_glitch((double)snap.raw * PHASE_ERR_LSB_NS);
                    sample_acc        = 1;
                    c_sample_ok       = 1;
                    g_last_why        = BSP_Q_OK;
                    g_last_acc        = 1;
                    main_last_phase_ns = p;
                    main_phase_acc_ns += p;
                    main_p_sum        += p;
                    s_bad_sample_run   = 0U;
                    bsp_snapshot_mark_used(&snap);
                } else {
                    /* ---- 相位连续性补偿（2026-10-30 修订）------------------
                     * 原来只加 kalman.X[1]，但这漏了【尚未补进 KF 的 DAC 阶跃】。
                     * DAC 补偿是在 10 秒节拍才做的，而补值每秒都做，
                     * 所以 kalman.X[1] 可能仍是旧频率。
                     *
                     * 反例：X[1]=1 ppb，上一拍 DAC 加 1 码，然后缺测 10 秒
                     *   当前补值 = 1 × 10          = 10.0000 ns
                     *   应该补值 = (1 + 1×0.47852) × 10 = 14.7852 ns
                     *   恢复后假新息 = -4.7852 ns
                     *
                     * 修法：补值时把这段 DAC 变化也算进去。
                     * ⚠️ 这仍是【模型预测】，不能当实测相位用于性能评价。
                     * ------------------------------------------------------ */
                    double f_hold = kalman.X[1]
                                  + (double)(control_get_last_dac() - s_prev_dac)
                                    * DAC_GAIN * (double)control_get_loop_sign();
                    main_phase_acc_ns += f_hold;
                    main_p_sum        += f_hold;
                    s_gap_filled++;

                    c_sample_ok       = 0;
                    g_last_why        = sample_why;
                    g_last_acc        = 0;
                    main_last_phase_ns = 0.0;

                    gap_this_window = 1U;
                    if (gap_count_this_window < 0xFFFFU) { gap_count_this_window++; }
                    if (s_bad_sample_cnt < 0xFFFFFFFFU) { s_bad_sample_cnt++; }
                    if (s_bad_sample_run < 0xFFFFU)     { s_bad_sample_run++; }

                    /* ---- 2026-10-30：清旧慢环窗口与 C 拟合窗口 --------------
                     * 缺口期间相位参考与实测之间断了连续性，旧窗口里的点
                     * 对应的参考已经不可信，直接清掉让它们重新攒。
                     * 保留：累计相位、DAC 历史、KF 状态（KF 只跳过更新）。
                     * C 窗口 slow_c_observe() 收到无效样本本来就会清，
                     * 这里显式调一次是为了让【慢环】窗口也一起清。
                     * ------------------------------------------------------ */
                    if (slow.active != 0 || slow.npts > 0) {
                        slow_reset_window();
                        s_slow_win_reset++;
                    }
                    slow_gap_hold = 1U;      /* 本秒不推进 slow_tick */
                }
            }

            /* ---- 相位越界保护（2026-10-30 修订）----------------------------
             * 软保护 ±100 µs：只标记本次不进 C 窗口，相位保留（让控制器拉回）
             * 硬保护 ±10 ms ：确实病态，才清零并重建参考
             * -------------------------------------------------------------- */
            if (main_phase_acc_ns > 1e5 || main_phase_acc_ns < -1e5) {
                c_sample_ok = 0;            /* 软保护：本次不进 C 窗口 */
            }
            if (main_phase_acc_ns > 1e7 || main_phase_acc_ns < -1e7) {
                /* 硬保护：±10 ms 以上明确病态，重建参考 */
                main_phase_acc_ns = 0.0;
                main_p_sum        = 0.0;
                kalman.X[0]       = 0.0;
                c_sample_ok       = 0;
                s_hard_reset_cnt++;
            }

#if SLOW_C_ENABLE
            {
                double gain = DAC_GAIN * (double)control_get_loop_sign();
                if (!slow.c_initialized || gain != slow.c_gain) {
                    slow_c_init(c_sample_ms, control_get_last_dac(), gain);
                }
                /* 2026-10-30：序号【必须复用快照】，不能再读硬件。
                 * c_sample_ok / main_phase_acc_ns 都来自本秒快照，
                 * 若这里重新读 REG_DBG_DIFF_CNT，中途来新 PPS 就会把
                 * 「本秒相位」配上「下一秒序号」，C 按错误序号判连续性 -> 误清窗。*/
                slow_c_observe(c_sample_ms, main_phase_acc_ns,
                               c_sample_ok && sys_state == STATE_TUNING,
                               g_last_seq);
            }
#else
            (void)c_sample_ms; (void)c_sample_ok;
#endif

            /* ---- 原始寄存器诊断（每秒一行）------------------------------------
             * 用来判断 ±50 ns 的跳变到底来自哪里：
             *   0x00 = 粗计数<<8 + TDC 细项，是"相位差"的完整测量
             *   0x18 = 本 1PPS 间隔内 OCXO 边沿数 - 标称值   ← 独立的频率测量
             *   0x30 = 上次测量的 TDC 抽头值                ← 细项是否在变
             *   0x2C = 状态字（tdc_sat / tap_max / meas_valid）
             *
             * 判读：
             *   · 0x00 跳 50ns 而 0x18 同时跳 1  → 真实频率变化
             *   · 0x00 跳 50ns 而 0x18 不动      → 粗计数误计（软件修正对象）
             *   · 0x00 的【低 8 位】在跳          → TDC 细项在工作
             *   · 0x00 的低 8 位恒为 0            → TDC 没接上
             * ---------------------------------------------------------------- */
            if (main_log == MAIN_LOG_TEXT) {
                uint32_t ref_seq0 = Xil_In32(REG_DBG_DIFF_CNT);
                int32_t  r00 = (int32_t)Xil_In32(REG_PHASE_ERR);
                int32_t  r18 = (int32_t)Xil_In32(REG_OCXO_DELTA);
                uint32_t r30 = Xil_In32(REG_TDC_TAP);
                uint32_t r2c = Xil_In32(REG_STATUS);
                int32_t  ref_clk  = (int32_t)Xil_In32(REG_CLK_DELTA);
                uint32_t ref_frac = Xil_In32(REG_DBG_FRAC_VAL);
                int      ref_dac  = control_get_last_dac();
                uint32_t ref_ms   = bsp_millis();
                uint32_t ref_fine_cnt = Xil_In32(REG_DBG_FINE_CNT);
                uint32_t ref_frac_cnt = Xil_In32(REG_DBG_FRAC_CNT);
                uint32_t ref_seq1 = Xil_In32(REG_DBG_DIFF_CNT);
                dbg_printf("REFCHK seq0=%lu seq1=%lu ocxo=%ld clk=%ld raw=%ld frac=%lu D=%d ms=%lu status=%lu\r\n",
                           (unsigned long)ref_seq0, (unsigned long)ref_seq1,
                           (long)r18, (long)ref_clk, (long)r00,
                           (unsigned long)ref_frac, ref_dac,
                           (unsigned long)ref_ms, (unsigned long)r2c);

                dbg_printf("RAW 0x00=%ld c=%ld f=%lu 0x18=%ld 0x30=%lu 0x2C=%s%s%s\r\n",
                           (long)r00, (long)(r00 / 256), (unsigned long)(r00 & 0xFF),
                           (long)r18, (unsigned long)r30,
                           ((r2c & STATUS_MEAS_VALID) ? "M" : "-"),
                           ((r2c & STATUS_TDC_SAT)    ? "S" : "-"),
                           ((r2c & STATUS_TAP_MAX)    ? "X" : "-"));

                /* 小数通路诊断（只读计数器）*/
                dbg_printf("DBG fine=%lu frac=%lu diff=%lu fracval=%lu\r\n",
                           (unsigned long)ref_fine_cnt,
                           (unsigned long)ref_frac_cnt,
                           (unsigned long)ref_seq1,
                           (unsigned long)ref_frac);
            }
#endif

            if (sys_state == STATE_TUNING) {
                task_tuning_second();
            } else if (sys_state == STATE_CALIB) {
                task_calib_second();
            } else {
                task_holdover_second();
            }

            /* ---- 2026-10-30：统一门控诊断（放在控制【之后】，反映本拍实际结果）
             *   why   : 最近一秒样本判定  0=OK 1=SAT 2=TORN 3=DUP 4=GAP 5=NO_MEAS
             *   used  : 本拍是否真的参与了 KF/DAC（1=参与 0=跳过）
             *   acc   : 最近一秒样本是否计入了累计相位
             *   gap   : 上一窗口的缺口秒数
             *   bad   : 累计坏样本    badrun: 连续坏样本
             *   hreset: 硬保护次数    gapskip: 因缺口跳过的控制拍数
             *   gapfill: 用 X[1] 补齐相位的次数
             * ------------------------------------------------------------------ */
            if (main_log == MAIN_LOG_TEXT) {
                dbg_printf("SAMP why=%d used=%d acc=%d gap=%u bad=%lu badrun=%u "
                           "hreset=%lu gapskip=%lu gapfill=%lu\r\n",
                           g_last_why, g_kf_used, g_last_acc,
                           (unsigned)gap_count_last_window,
                           (unsigned long)s_bad_sample_cnt,
                           (unsigned)s_bad_sample_run,
                           (unsigned long)s_hard_reset_cnt,
                           (unsigned long)s_gap_skips,
                           (unsigned long)s_gap_filled);
            }
            /* 2026-10-31：候选日志只在【真实更新拍】打印，避免重复旧值 */
            if (g_kf_used != 0) {
                cand_obs_log();
            }
            g_kf_used = 0;

            {
                int was_done = slow.done;
                int n_before = slow.npts;
                if (slow_gap_hold != 0U) {
                    slow_gap_hold = 0U;        /* 2026-10-30: 缺口这一秒不推进慢环 */
                } else {
                    slow_tick();               // 慢环更新频率估计，供主环融合
                }
                if ((was_done == 0) && (slow.done != 0)) {
                    log_slow_debug(SLOW_EV_DONE, n_before);
                }
            }

            /* 本秒实测相位增量: 屏幕的 gppps / gpphs 和日志的 pps= 都用它 */
#if (BENCH_ENABLE)
            main_last_phase_ns = (bsp_meas_valid() != 0) ? read_phase_error_ns() : 0.0;
#endif

            led_update();                      // |X1| -> 闪烁频率
            screen_send_status();              // 串口屏刷一帧

            log_seconds++;

#if (BENCH_ENABLE)
            log_bench();                   // 台架: 真值 vs 估计 vs 双路状态
#endif

            if (main_log == MAIN_LOG_TEXT) {
                log_text();
            } else if (main_log == MAIN_LOG_CSV) {
                log_csv();
            }

            /* NMEA 通路诊断: 不受 main_log 影响, 一定要打 —— 否则开着原始
             * dump(main_log = QUIET)时完全看不到 byte_count / drop_count。 */
#if (DIAG_NMEA_PERIOD_S > 0)
            if (((log_seconds % (uint32_t)DIAG_NMEA_PERIOD_S) == 0U) && SCAN_IDLE) {
                diag_nmea();
            }
#endif
        }
    }
#endif /* CENTER_CALIB_ENABLE */

    return 0;
}