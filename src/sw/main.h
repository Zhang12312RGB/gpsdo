#ifndef MAIN_H
#define MAIN_H

#define STATE_TUNING     0    /* 正常闭环驯服              */
#define STATE_CALIB      1    /* 标定: DAC 冻结            */
#define STATE_HOLDOVER   2    /* 守时: GNSS 失锁           */

#define GNSS_LOST_LIMIT          5   /* 连续失锁次数          */
#define SATELLITE_MIN            4   /* 最低卫星数            */
#define HOLDOVER_RESUME_SECONDS  5   /* 恢复平滑秒数          */

/*=============================================================================
 * 对照实验开关（2026-10-28）
 *-----------------------------------------------------------------------------
 * 目的：用单变量对照判断「慢环历史平均频率」对 100~300 秒波动的贡献。
 *
 *   EXP_FIXED_PERIOD  0 = 自适应周期（当前行为）
 *                     1 = 固定 10 秒（便于同长度比较；已知它本身不消除
 *                         Allan 上升，只是去掉一个变量）
 *
 *   EXP_SLOW_CONTROL  0 = 慢环【不参与】控制，只继续记录 slow.freq
 *                     1 = 正常融合（慢环按权重 w 进入 f_ctrl）
 *
 * 建议的 A/B：
 *   A 版：FIXED=1, SLOW=1   固定周期 + 慢环参与
 *   B 版：FIXED=1, SLOW=0   固定周期 + 慢环只记录
 * 两版其余参数（K_PHASE / R / Q / 各门限）完全相同。
 *===========================================================================*/
#define EXP_FIXED_PERIOD   1
/*-----------------------------------------------------------------------------
 * 慢环输入源选择（2026-10-28）
 *   0 = 慢环不参与控制，只维护窗口和保护逻辑   ← B 版基线
 *   1 = 用原始 slow.freq                      ← A 版（已知有 100 s 波动）
 *   2 = 用 C 补偿后的 fNow                    ← C 反馈（本次实验）
 *
 * 语义（按 2026-10-28 复核）：
 *   · MODE=2 时若 C 无有效拟合 → 【纯快环】，不退回 slow.freq。
 *     否则会在两个慢环之间切换，无法判断收益来自哪里。
 *   · MODE=0 仍保持原有的窗口维护和保护行为，只把融合权重置零，
 *     避免重构后悄悄改变 B 基线。
 *   · 计算顺序固定为：选来源 → 判有效 → 用该来源跑保护 → 融合。
 *----------------------------------------------------------------------------*/
#ifndef EXP_SLOW_MODE
#define EXP_SLOW_MODE      2
#endif

/*-----------------------------------------------------------------------------
 * 2026-10-30 单变量试验：DAC 端是否使用相位项 K_PHASE × X[0]
 *
 *   1 = 使用相位反馈；当前提交配置关闭此项
 *   0 = 不使用（纯频率控制，但【KF 内部仍保留 X[0] 状态】）
 *
 * ⚠️ 只影响 DAC 控制输入，不影响滤波器结构。
 *    关掉后：OADEV@200 预计改善约 20%，X[1] std 降约 17%，
 *            但平均频差会偏离（相位漂移约束减弱）。
 *    模型预测，需实测验证。
 *-----------------------------------------------------------------------------*/
#define EXP_PHASE_TERM_EN  0

/* Legacy alias; EXP_SLOW_MODE is the only control-source selector. */
#define EXP_SLOW_CONTROL   (EXP_SLOW_MODE != 0)

/* Final release: legacy online R training is retained for development only.
 * The CALIB/TUNING label definitions must be redesigned before enabling it. */
#ifndef EXP_R_TRAIN_ENABLE
#define EXP_R_TRAIN_ENABLE 0
#endif
#define FINAL_RELEASE_TAG "FINAL_C_R80_T10_20261031"

extern int sys_state;
extern int sample_period;

/* 本秒实测的相位差(ns), 给串口屏和日志用; meas_valid=0 时为 0 */
extern double main_last_phase_ns;

/* Accumulated phase relative to the software initial reference, in ns.
 * X[0]/phsm use this accumulated reference; it is not calibrated UTC phase.
 * main_last_phase_ns/ppsm are one-interval increments. */
extern double main_phase_acc_ns;
/* 诊断用: 单独累加 p，用来核对 main_phase_acc_ns 是不是真的只被这一处加过 */
extern double main_p_sum;

#endif /* MAIN_H */