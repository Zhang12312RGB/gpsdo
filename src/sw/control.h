#ifndef CONTROL_H
#define CONTROL_H

#define DAC_CENTER   1752      /* 锁定点 ≈ 1752（实测 g=0.4785 时，+141.6 ppb 需 -296 码） */
/* DAC 律的环路增益 S。真实灵敏度记作 g = df/d(dac)，则闭环特征根
 *     λ = 1 - g/S
 * 所以：
 *     S = g      -> λ = 0，死拍：一步到位，但对估计误差零容忍（会撞轨）
 *     S = 4g     -> λ = 0.75，每步收敛 25%，5~6 步到位，对 g 的标定误差有 4 倍余量
 * 实测标定 g 之后，本值取 4g。台架里 g = 0.050，故取 0.20。 */
#define DAC_SCALE    1.91406   /* 环路增益 = 4 x g，λ = 1 - g/S = 0.75（实测）       */
/* 真实被控对象灵敏度 g = df/d(dac)，ppb/code。
 * 用途：卡尔曼的控制输入项 B·u（du = Δdac × g）。
 * 注意：这里必须用 g，不能用 DAC_SCALE —— DAC_SCALE 是环路增益 S = 4g，
 *       用它会把已知扰动放大 4 倍，反而把滤波器推偏。
 * 标定方法：DAC 写 1024 / 3072 各测一次 OCXO 频率，g = Δf/Δdac。 */
/* 环路基性默认值：+1 或 -1。运行中用 control_set_loop_sign() 改。
 *     +1 -> OCXO 在 DAC↑ 时变快（正极性）
 *     -1 -> OCXO 在 DAC↑ 时变慢（负极性）
 * 标定程序会打印该填哪个。 */
#define LOOP_SIGN_DEFAULT   (+1)   /* 实测正极性：DAC↑ -> 频率↑ */
#define DAC_GAIN     0.47852   /* 被控对象灵敏度 0.4785 ppb/码（hwtest mode 2）  */
#define DAC_MIN     0
#define DAC_MAX     4095

/*-----------------------------------------------------------------------------
 * 【标定用】两个可运行时设置的量
 *-----------------------------------------------------------------------------
 * g_dac_center : DAC 中心值。默认 2048，标定后按实测改成「真实中心」。
 * g_loop_sign  : 环路基性，只能是 +1 或 -1。它同时吸收两件事：
 *                  (1) OCXO 极性 —— DAC 码值↑ 是让频率↑ 还是 ↓
 *                  (2) PL 相位差符号 —— 需要 phase_error.v 是「0x00 正 = OCXO 快」
 *                所以：
 *                  OCXO 在 DAC↑ 时变快 (g>0) -> +1  （超前 -> 压低 DAC 拉回来）
 *                  OCXO 在 DAC↑ 时变慢 (g<0) -> -1
 *                标定程序会把该填哪个值打印出来。
 *----------------------------------------------------------------------------*/
int  control_get_center(void);
void control_set_center(int code);          /* 越界会被夹到 DAC_MIN..DAC_MAX */
int  control_get_loop_sign(void);
int  control_set_loop_sign(int s);          /* 只接受 +1 / -1，返回设置后的值 */

void control_init(void);
void dac_update(void);                        /* 用卡尔曼 X[1] 调 DAC */
/* DAC 硬冻结（2026-10-30）：1=完全不动 DAC，用于换天线等实验 */
void control_set_dac_lock(int on);
int  control_get_dac_lock(void);

void control_apply_freq(double freq_ppb);     /* 按频率估计增量式调 DAC */
void control_holdover_tick(void);
void control_resume_from_holdover(void);
int control_get_last_dac(void);
/* 积分累加器的【小数】值：诊断用，看积分器有没有在输出 */
double control_get_dac_acc(void);



/*-----------------------------------------------------------------------------
 * 双环（Route B）接口 —— 仅测试工程启用
 *   control_bump_setpoint : 慢环每 T_slow 秒推进一次设定点，系统里【唯一】的积分器
 *   control_apply_prop    : 快环每秒在设定点上做比例修正，本身不积分
 *----------------------------------------------------------------------------*/
#define DAC_KP      0.42     /* 快环比例增益 码/ppb = 0.2/g，约 20%/s 的修正速度 */

/*-----------------------------------------------------------------------------
 * 相位反馈项（2026-10-25 新增）
 *
 * 背景：原来的积分律只看得见 X[1]（频率估计）。而 X[1] 在 Q=2e-7 下的
 * 时间常数约 2000 秒，22 分钟只能收到真值的 35%。结果积分器推得极慢，
 * 频率长期偏着，相位一路单调漂（实测 22.6 分钟漂了 -116 ns），
 * 反映到 Allan 上就是「随 tau 上升」而不是下降。
 *
 * 加一个对相位本身的【比例】反馈：相位一漂就立刻产生纠正量，
 * 不必等 X[1] 收敛。这就把环路从「只跟频率」变成「相位 P + 频率 I」。
 *
 * 取值约束：
 *   闭环增益 = K_PHASE * g / DAC_SCALE = 0.002 * 0.4785 / 1.914
 *            = 0.0005 / 次更新  （每 10 秒一次）
 *   稳定性要求 0 < K_PHASE*g/DAC_SCALE < 2，这里远小于 2 ✓
 *
 * 量级校验（相位 -> 折算成频率纠正量）：
 *     10 ns  -> 0.020 ppb
 *     50 ns  -> 0.100 ppb
 *    116 ns  -> 0.232 ppb   ← 正好抵消当前 -0.086 ppb 的漂移
 *
 * 注意：它的职责是【刹住漂移】，不是【消掉偏移】。消偏移靠频率积分项。
 *----------------------------------------------------------------------------*/
/* 2026-10-27 20:10  0.02 -> 0.005
 * 依据：19:20 那一小时的 Allan 在 tau=64 s 处有 0.606 ppb 的峰（@1s 只有 0.146）。
 * 64 s ≈ 1/K_PHASE = 50 s，说明峰就是相位反馈项的一阶时间常数造成的，
 * 而且幅度是 @1s 的 4 倍，属于【欠阻尼】。
 * 降到 0.005 后 tau_c ≈ 200 s，同时相位增益下降 4 倍，应当把峰压下去并右移。
 * 需要观察：tau=64 和 tau=200 处的 Allan 是否都下降；启动瞬态是否变长。 */
#define K_PHASE       0.005    /* ppb per ns */
/* 相位项的贡献上限。没有它的话，相位漂到 1000 ns 时相位项会贡献 2 ppb，
 * 超过整个环路的工作范围，DAC 会被顶死。钳在 2 ppb：相位在 +-1000 ns
 * 以内都不受影响，超出才开始限幅。 */
#define K_PHASE_MAX  20.0     /* 2026-10-26: 2.0 -> 20.0
                               * 实测 phs 漂到 -1.13 us 时，0.002*1130 = 2.26 ppb
                               * 早就顶到 2.0 的上限，相位 P 等于没起作用。
                               * 改成 20 ppb 后，1 us 的相位能产生 20 ppb 的修正。
                               * 稳定性：闭环增益 = 0.02*0.4785/1.914 = 0.005 << 2 ✓ */

void   control_bump_setpoint(double freq_ppb);
void   control_apply_prop(double freq_ppb);
double control_get_setpoint(void);

#endif /* CONTROL_H */