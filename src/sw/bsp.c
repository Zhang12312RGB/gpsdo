#include "bsp.h"
#include "bench.h"
#include "xil_io.h"
#include "xtimer_config.h"     // COUNTS_PER_SECOND = CPU 主频 / 2

//-----------------------------------------------------------------------------
// Cortex-A9 SCU 私有定时器寄存器（定义同 xscutimer_hw.h）
//   递减计数器: 装载 COUNTS_PER_SECOND 并自动重载, 计到 0 时 ISR 的 bit0 置 1;
//   软件用"读 ISR + 写 1 清标志"判断 1 秒到。与主循环速度/优化级别无关。
//-----------------------------------------------------------------------------
#define SCU_TIMER_BASE      XPAR_XSCUTIMER_0_BASEADDR
#define SCU_TIMER_LOAD_REG  0x00U   // 装载值
#define SCU_TIMER_CNT_REG   0x04U   // 当前计数值(递减)
#define SCU_TIMER_CTRL_REG  0x08U   // 控制
#define SCU_TIMER_ISR_REG   0x0CU   // 事件/中断状态
#define SCU_CTRL_ENABLE     0x00000001U
#define SCU_CTRL_AUTORELOAD 0x00000002U
#define SCU_ISR_EVENT_FLAG  0x00000001U

// SCU 计数频率 = CPU 主频 / 2（与装载值同一基准）
#define BSP_SCU_TICK_HZ     ((uint32_t)(COUNTS_PER_SECOND))

static volatile uint32_t system_tick = 0;   // 主循环软件计数

//=============================================================================
// 时间基准
//=============================================================================

// 初始化 1 秒硬件节拍
void bsp_timer_init(void)
{
    Xil_Out32(SCU_TIMER_BASE + SCU_TIMER_LOAD_REG, (u32)(COUNTS_PER_SECOND));
    Xil_Out32(SCU_TIMER_BASE + SCU_TIMER_CTRL_REG,
              SCU_CTRL_ENABLE | SCU_CTRL_AUTORELOAD);
}

// 硬件判断 1 秒是否到达: 到达返回 1（读 ISR 并清标志）
int one_second_elapsed(void)
{
    if ((Xil_In32(SCU_TIMER_BASE + SCU_TIMER_ISR_REG) & SCU_ISR_EVENT_FLAG) != 0U) {
        Xil_Out32(SCU_TIMER_BASE + SCU_TIMER_ISR_REG, SCU_ISR_EVENT_FLAG);
        return 1;
    }
    return 0;
}

// 毫秒级时间基准: 由 SCU 递减计数换算, 与主循环速度无关。
// 供按键去抖等使用; 必须高频调用, 否则会漏掉每秒的进位（漏一次最多偏差 1 秒）。
uint32_t bsp_millis(void)
{
    static uint32_t wraps    = 0;
    static uint32_t last_cnt = 0;
    static int      inited   = 0;
    uint32_t now, elapsed;

    now = Xil_In32(SCU_TIMER_BASE + SCU_TIMER_CNT_REG);

    if (inited == 0) {
        last_cnt = now;
        inited   = 1;
        return 0;
    }
    if (now > last_cnt) {
        wraps++;                            // 递减计数器数值变大 = 刚刚重载
    }
    last_cnt = now;

    elapsed = BSP_SCU_TICK_HZ - now;
    if (elapsed >= BSP_SCU_TICK_HZ) {
        elapsed = BSP_SCU_TICK_HZ - 1U;     // 边界钳位, 保证单调
    }
    return wraps * 1000U + elapsed / (BSP_SCU_TICK_HZ / 1000U);
}

//=============================================================================
// 相位差（PL 寄存器 0x00, 10ns/LSB, 有符号, 正 = 本地 1PPS 超前）
//=============================================================================

int32_t read_phase_error(void)
{
    uint32_t raw = Xil_In32(REG_PHASE_ERR);
    return (int32_t)raw;                    // 最高位符号位, 直接转有符号
}

double read_phase_error_ns(void)
{
#if (BENCH_ENABLE)
    return bench_phase_ns();        // 台架模式: 用模拟的相位
#else
    int32_t raw = (int32_t)Xil_In32(REG_PHASE_ERR);
    return (double)raw * PHASE_ERR_LSB_NS;
#endif
}

//=============================================================================
// STATUS (0x2C) —— 判断 0x00 里的相位差到底是不是一次新的测量
//   meas_valid=0 时 0x00 是 0 或者上一次的陈旧值, 绝不能当测量用。
//   main.c 的 TUNING / CALIB / HOLDOVER 恢复都靠它做门控。
//=============================================================================

uint32_t bsp_status(void)
{
    return Xil_In32(REG_STATUS);
}

int bsp_meas_valid(void)
{
#if (BENCH_ENABLE)
    return bench_meas_valid();      // 台架模式: 恒有效
#else
    return ((Xil_In32(REG_STATUS) & STATUS_MEAS_VALID) != 0U) ? 1 : 0;
#endif
}

/* 1PPS 是否存在：pps_lost 位为 0 才算有 */
int bsp_pps_ok(void)
{
#if (BENCH_ENABLE)
    return 1;
#else
    return ((Xil_In32(REG_STATUS) & STATUS_PPS_LOST) == 0U) ? 1 : 0;
#endif
}

//=============================================================================
// 测量样本质量检查（2026-10-30 新增）
//-----------------------------------------------------------------------------
// 背景：实测 t=547 秒出现一次 2 秒的测量间隔（OCXO 计数 4000 万、FCLK 2 亿），
//       PL 的 phase_error 把它钳到 INT32_MAX = 0x7FFFFFFF，
//       同时 STATUS_SAT_FLAG / STATUS_TDC_SAT 置位。
//       原代码【只把 SAT 位打进日志，从不用于拒绝】，
//       于是 0x7FFFFFFF × 0.1953125 ns = 419.4 ms 被累加进 main_phase_acc_ns，
//       相位项 0.005 × 4.19e8 = 2.1e6 ppb 直接顶到 K_PHASE_MAX 限幅，
//       DAC 被错误地一路推。
//
// 四道门，顺序不能乱：
//   ① 饱和：raw == INT32_MAX，或 STATUS_SAT_FLAG / STATUS_TDC_SAT 置位
//   ② 撕裂：读前后的 diff_valid 计数不同（读的过程中来了新 PPS）
//   ③ 间隔：OCXO 或 FCLK 计数偏离标称超过 ±10%
//   ④ 缺失：meas_valid / pps_ok 为 0（调用方原有的判据）
//
// 任一命中 → 返回 0，调用方【跳过本拍、不清零】。
// 这样相位参考得以保留，坏样本也不会污染累加器。
//=============================================================================

#define SAT_RAW_VALUE      ((int32_t)0x7FFFFFFF)
#define SAT_NOMINAL_OCXO   (20000000L)      /* 20 MHz × 1 s */
#define SAT_NOMINAL_CLK    (100000000L)     /* 100 MHz × 1 s */
#define SAT_COUNT_TOL_PCT  10L              /* ±10% 判为间隔异常 */

int bsp_sample_good(int32_t raw, uint32_t st, uint32_t seq0, uint32_t seq1)
{
#if (BENCH_ENABLE)
    (void)raw; (void)st; (void)seq0; (void)seq1;
    return 1;
#else
    /* ① 饱和 */
    if (raw == SAT_RAW_VALUE) { return 0; }
    if ((st & (STATUS_SAT_FLAG | STATUS_TDC_SAT)) != 0U) { return 0; }

    /* ② 撕裂 */
    if (seq0 != seq1) { return 0; }

    /* ③ 间隔异常（用 0x18 = OCXO 计数偏差、0x20 = FCLK 计数偏差） */
    {
        int32_t ocxo = (int32_t)Xil_In32(REG_OCXO_DELTA);
        int32_t clk  = (int32_t)Xil_In32(REG_CLK_DELTA);
        int32_t ocxo_cnt = (int32_t)(ocxo + SAT_NOMINAL_OCXO);
        int32_t clk_cnt  = (int32_t)(clk  + SAT_NOMINAL_CLK);
        long tol_o = SAT_NOMINAL_OCXO / 100L * SAT_COUNT_TOL_PCT;
        long tol_c = SAT_NOMINAL_CLK  / 100L * SAT_COUNT_TOL_PCT;
        long d_o = (long)ocxo_cnt - SAT_NOMINAL_OCXO;
        long d_c = (long)clk_cnt  - SAT_NOMINAL_CLK;
        if ((d_o >  tol_o) || (d_o < -tol_o)) { return 0; }
        if ((d_c >  tol_c) || (d_c < -tol_c)) { return 0; }
    }

    return 1;
#endif
}


//=============================================================================
// 统一测量快照 + 质量判定（2026-10-30）
//-----------------------------------------------------------------------------
// 背景：t=547 秒出现过一次 2 秒的测量间隔（OCXO 4000 万 / FCLK 2 亿），
//       PL 把相位钳到 INT32_MAX 并置 STATUS_SAT_FLAG；
//       原代码只把 SAT 位打进日志、从不用它拒绝，
//       于是 0x7FFFFFFF × 0.1953125 = 419.4 ms 被累加进 main_phase_acc_ns。
//
// 本模块提供：
//   bsp_snapshot_take()     一次性读全部寄存器（seq0 ... seq1 之间）
//   bsp_sample_classify()   统一判定，返回 BSP_Q_*
//   bsp_snapshot_mark_used() 记录已消费的序号（供下一拍判 DUP）
//   bsp_snapshot_reset_seq() 状态切换时复位
//=============================================================================

#define BSP_NOMINAL_OCXO   (20000000L)
#define BSP_NOMINAL_CLK    (100000000L)
/* 间隔容差 ±0.5% —— 只挡「整秒丢失 / 双拍」这类粗异常。
 * ⚠️ 0.5% = 5 ms，【挡不住 1.001 秒那种 0.1% 的偏差】。
 *    亚毫秒级的间隔误差由调用方的缺口计数 + 相位连续性补偿处理。*/
#define BSP_GAP_PERMILLE   (5L)

/* 两个基准必须分开：
 *   s_last_seq : 上次【成功消费】的序号 —— 只用于「重复」判定
 *   s_seen_seq : 上次【看到可靠发布】的序号 —— 用于「跳变」判定
 *
 * 为什么必须分开（实测会死锁）：
 *   100 正常 -> last=100
 *   101 饱和 -> 拒绝，last 仍 100
 *   102 正常 -> 若拿 last 比，差值 2 -> 判 GAP，也不更新 last
 *   103 正常 -> 判 GAP ... 永远恢复不了
 *
 * 修法：只要 seq0==seq1（序号可靠发布），就更新 s_seen_seq，
 *       不管这一拍样本是否被接受。这样下一拍能正常判「差 1」。
 *       饱和值本身仍被拒绝，只是不再阻塞序号恢复。*/
static uint32_t s_last_seq     = 0U;
static int      s_last_seq_set = 0;
static uint32_t s_seen_seq     = 0U;
static int      s_seen_seq_set = 0;

void bsp_snapshot_take(bsp_snapshot_t *s)
{
    if (s == 0) { return; }
    s->ms         = bsp_millis();
    s->seq0       = Xil_In32(REG_DBG_DIFF_CNT);
    s->raw        = (int32_t)Xil_In32(REG_PHASE_ERR);
    s->status     = Xil_In32(REG_STATUS);
    s->ocxo_delta = (int32_t)Xil_In32(REG_OCXO_DELTA);
    s->clk_delta  = (int32_t)Xil_In32(REG_CLK_DELTA);
    s->tap        = Xil_In32(REG_TDC_TAP);
    s->frac       = Xil_In32(REG_DBG_FRAC_VAL);
    s->seq1       = Xil_In32(REG_DBG_DIFF_CNT);
    s->seq        = s->seq1;
}

int bsp_sample_classify(const bsp_snapshot_t *s)
{
    if (s == 0) { return BSP_Q_NO_MEAS; }

    /* ----------------------------------------------------------------------
     * 第一步：凡是「序号可靠发布」的样本，先推进 s_seen_seq 基准。
     *   这一步【必须放在所有拒绝判定之前】。
     *   理由（实测会死锁）：饱和样本若被直接 return 而不推进基准，
     *   下一个正常样本的序号差就是 2 -> 被判 GAP -> 又不推进 -> 永远卡住。
     *
     *   把「看见一个可靠序号」与「接受这次相位测量」分成两件事：
     *     · 序号基准：只要 seq0==seq1 就推进（不管样本好坏）
     *     · 相位测量：只有全部门都通过才接受
     * -------------------------------------------------------------------- */
    /* 第一步：只要序号可靠发布，就推进 s_seen_seq。
     *   这一步【必须在所有拒绝判定之前】—— 否则饱和样本被拒后基准不推进，
     *   下一个正常样本序号差 2 会被判 GAP，然后永久卡住。
     *   但【返回值】留到后面判：先报更根本的原因（无测量/饱和），
     *   最后才报 DUP/GAP，这样日志里的 why 更有信息量。*/
    int seq_dup = 0;
    int seq_gap = 0;

    if (s->seq0 == s->seq1) {
        uint32_t prev_seen = s_seen_seq;
        int      had_seen  = s_seen_seq_set;

        s_seen_seq     = s->seq0;      /* 先推进 */
        s_seen_seq_set = 1;

        if (had_seen) {
            if (s->seq0 == prev_seen) {
                seq_dup = 1;
            } else if (s->seq0 > prev_seen && (s->seq0 - prev_seen) > 1U) {
                seq_gap = 1;
            }
        }
    }

    /* ⓿ 无测量：必须先判，否则后面读到的都是陈旧值 */
    if ((s->status & STATUS_MEAS_VALID) == 0U) { return BSP_Q_NO_MEAS; }
    if ((s->status & STATUS_PPS_LOST)   != 0U) { return BSP_Q_NO_MEAS; }

    /* ① 饱和 */
    if (s->raw == (int32_t)0x7FFFFFFF) { return BSP_Q_SAT; }
    if ((s->status & (STATUS_SAT_FLAG | STATUS_TDC_SAT)) != 0U) { return BSP_Q_SAT; }

    /* ② 撕裂 */
    if (s->seq0 != s->seq1) { return BSP_Q_TORN; }

    /* ③ 重复 / 跳变（基准已在上面推进，这里只负责报原因）*/
    if (seq_dup) { return BSP_Q_DUP; }
    if (seq_gap) { return BSP_Q_GAP; }

    /* ④ 间隔异常（粗异常）*/
    {
        long ocxo_cnt = (long)s->ocxo_delta + BSP_NOMINAL_OCXO;
        long clk_cnt  = (long)s->clk_delta  + BSP_NOMINAL_CLK;
        long tol_o = BSP_NOMINAL_OCXO / 1000L * BSP_GAP_PERMILLE;
        long tol_c = BSP_NOMINAL_CLK  / 1000L * BSP_GAP_PERMILLE;
        long d_o = ocxo_cnt - BSP_NOMINAL_OCXO;
        long d_c = clk_cnt  - BSP_NOMINAL_CLK;
        if ((d_o >  tol_o) || (d_o < -tol_o)) { return BSP_Q_GAP; }
        if ((d_c >  tol_c) || (d_c < -tol_c)) { return BSP_Q_GAP; }
    }

    return BSP_Q_OK;
}

void bsp_snapshot_mark_used(const bsp_snapshot_t *s)
{
    if (s == 0) { return; }
    s_last_seq     = s->seq;
    s_last_seq_set = 1;
    /* seen 已在 classify 里更新过，这里同步一次以防调用顺序变化 */
    s_seen_seq     = s->seq;
    s_seen_seq_set = 1;
}

void bsp_snapshot_reset_seq(void)
{
    s_last_seq     = 0U;
    s_last_seq_set = 0;
    s_seen_seq     = 0U;
    s_seen_seq_set = 0;
}

//=============================================================================
// 第二路(慢环) 寄存器 0x44 ~ 0x58
//=============================================================================

void bsp_p2_set_t(uint16_t t)
{
#if (BENCH_ENABLE)
    bench_p2_set_t(t);
#else
    Xil_Out32(REG_P2_T, (uint32_t)t);
#endif
}

uint16_t bsp_p2_get_t(void)
{
#if (BENCH_ENABLE)
    return bench_p2_get_t();
#else
    return (uint16_t)(Xil_In32(REG_P2_T) & 0xFFFFU);
#endif
}

uint16_t bsp_p2_count(void)
{
#if (BENCH_ENABLE)
    return bench_p2_count();
#else
    return (uint16_t)(Xil_In32(REG_P2_COUNT) & 0xFFFFU);
#endif
}

int32_t bsp_p2_read_ocxo(void)
{
#if (BENCH_ENABLE)
    return bench_p2_ocxo();
#else
    return (int32_t)Xil_In32(REG_P2_OCXO);
#endif
}

// 0x48 是低 32 位、0x4C 是高 32 位, 拼成有符号 64 位
int64_t bsp_p2_read_phase(void)
{
#if (BENCH_ENABLE)
    return bench_p2_phase();
#else
    uint64_t lo = (uint64_t)Xil_In32(REG_P2_PHASE_LO);
    uint64_t hi = (uint64_t)Xil_In32(REG_P2_PHASE_HI);
    return (int64_t)((hi << 32) | lo);
#endif
}

// 先读状态再清, 返回 1 表示确实拿到一个新样本
int bsp_p2_take_new(void)
{
#if (BENCH_ENABLE)
    return bench_p2_take_new();
#else
    if ((Xil_In32(REG_P2_STATUS) & P2_NEW_FLAG) == 0U) {
        return 0;
    }
    Xil_Out32(REG_P2_STATUS, P2_NEW_FLAG);   // 写 1 清 NEW
    return 1;
#endif
}

//=============================================================================
// 主循环软件计数（只给 gnss.last_update_tick 用, 不再当时间基准）
//=============================================================================

void bsp_tick(void)
{
    system_tick++;
}

uint32_t get_system_tick(void)
{
    return system_tick;
}

//=============================================================================
// UART 波特率分频 (PL 寄存器 0x40)
//   值 = PL 参考时钟 / 目标波特率。100 MHz 下: 9600 -> 10416, 115200 -> 868。
//   uart_rx 上电用编译期默认值, 只有这里写了才会变。
//=============================================================================

void bsp_set_baud_div(uint32_t div)
{
    Xil_Out32(REG_BAUD_DIV, div);
}

uint32_t bsp_get_baud_div(void)
{
    return Xil_In32(REG_BAUD_DIV);
}

/*-----------------------------------------------------------------------------
 * 空闲钩子：由 screen.c / dbg_uart.c 在逐字节发送的间隙调用。
 * 钩子内部（gnss_pack_task）无新数据时立刻返回，代价只有几次 AXI 读。
 *---------------------------------------------------------------------------*/
static void (*s_idle_hook)(void) = 0;

void bsp_set_idle_hook(void (*fn)(void))
{
    s_idle_hook = fn;
}

void bsp_idle(void)
{
    if (s_idle_hook != 0) {
        s_idle_hook();
    }
}

//=============================================================================
// TDC 标定量写入（0x0C 每周期抽头数 / 0x10 固定延迟）
//-----------------------------------------------------------------------------
// period 传 0 时写 0，RTL 会回退到编译期默认 TAP_PERIOD_REF（160），
// 等价于"不标定"，所以这里总是直接写，逻辑最简单。
// 标定方法见 hwtest.c 的 hwtest_tdc()。
//=============================================================================
void bsp_set_tdc_calib(uint16_t period, uint16_t offset)
{
    Xil_Out32(REG_TDC_TAP_PERIOD, (uint32_t)period);
    Xil_Out32(REG_TDC_TAP_OFFSET, (uint32_t)offset);
}