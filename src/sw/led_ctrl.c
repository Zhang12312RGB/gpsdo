/*=============================================================================
 * led_ctrl.c -- LED 频率差指示模块
 *-----------------------------------------------------------------------------
 * 功能:
 *   读取卡尔曼滤波器状态量 kalman.X[1] (频率差, 单位 ppb),
 *   取绝对值 |X[1]|, 按分段线性表映射成闪烁频率,
 *   |X[1]| 越大 -> 闪烁频率越高, 通过 E6 / PS_MIO0_5 / PS LED1 指示。
 *
 * 引脚与极性 (系统架构图 + AX7010 用户手册):
 *   E6 = PS_MIO0_5 = PS_MIO0_500 = MIO0_LED = PS LED1
 *   BANK500, 3.3V, "高为灭" -> 低电平点亮 (LED_ACTIVE_LOW)
 *   写 0 点亮, 写 1 熄灭
 *
 * 通道:
 *   默认用 PS GPIO(XGpioPs)直接驱动 E6。E6 是 PS 侧 MIO 引脚, PL 驱动不了,
 *   所以这是让该引脚真正按频率闪烁的最直接做法, 上板即用, 不需要改 RTL。
 *
 * 计时基准:
 *   用 Zynq 全局定时器 XTime_GetTime() 取真实时间, 不依赖主循环速度,
 *   因此闪烁频率与实际时钟一致(不像用循环计数分频那样会漂)。
 *
 * 依赖: kalman.h, xtime_l.h；PS GPIO 路径还需要 xgpiops.h 与 xparameters.h
 *===========================================================================*/

#include "led_ctrl.h"
#include "kalman.h"
#include "xiltimer.h"

#if LED_USE_PL_REG
  #include "bsp.h"          /* AXI 基地址 */
  #include "xil_io.h"
#else
  #include "xparameters.h"
  #if defined(XPAR_XGPIOPS_0_BASEADDR) || defined(XPAR_XGPIOPS_0_DEVICE_ID)
    #define LED_HAVE_GPIOPS   1
    #include "xgpiops.h"
  #else
    /* BSP 里没有 GPIO 驱动, 退化为直接操作 GPIO 寄存器(见文件末尾 */
    #define LED_HAVE_GPIOPS   0
  #endif
#endif

/*---------------------------------------------------------------------------
 * 1. 引脚极性
 *    架构图明确标注 "3.3V 为高 (高为灭)" -> 低电平点亮
 *--------------------------------------------------------------------------*/
#ifndef LED_ACTIVE_LOW
  #define LED_ACTIVE_LOW    1
#endif

#if LED_ACTIVE_LOW
  #define LED_ON_LEVEL      (0U)    /* 写 0: 点亮 */
  #define LED_OFF_LEVEL     (1U)    /* 写 1: 熄灭 */
#else
  #define LED_ON_LEVEL      (1U)
  #define LED_OFF_LEVEL     (0U)
#endif

/*---------------------------------------------------------------------------
 * 2. AXI 通道的寄存器地址 (仅 LED_USE_PL_REG = 1 时用到)
 *    架构图未给出 LED 的物理基地址, 默认复用 GNSS IP 基地址做演示,
 *    上板前请改成实际基地址(或在编译时 -DLED_BASE_ADDR=...)。
 *--------------------------------------------------------------------------*/
#if LED_USE_PL_REG
  #ifndef LED_BASE_ADDR
    #define LED_BASE_ADDR   GNSS_BASE_ADDR
  #endif
  #define LED_CTRL_REG      (LED_BASE_ADDR + LED_CTRL_OFFSET)
#endif

/*---------------------------------------------------------------------------
 * 3. 计时基准: Zynq 全局定时器计数频率
 *    xtime_l.h 的 COUNTS_PER_SECOND 由 BSP 生成, 最可靠;
 *    没有时退回 CPU 主频, 再没有才用 100MHz 兜底(此时频率会整体偏差,
 *    可在编译时 -DLED_TIMER_HZ=实际值 覆盖)。
 *--------------------------------------------------------------------------*/
#ifndef LED_TIMER_HZ
  #ifdef COUNTS_PER_SECOND
    #define LED_TIMER_HZ    ((XTime)COUNTS_PER_SECOND)
  #elif defined(XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ)
    #define LED_TIMER_HZ    ((XTime)XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ)
  #else
    #define LED_TIMER_HZ    ((XTime)100000000ULL)   /* 兜底: 100MHz */
  #endif
#endif

#define US_PER_SECOND       1000000ULL

/*---------------------------------------------------------------------------
 * 4. 频率差来源
 *    统一在 led_read_abs_freq_ppb() 里取绝对值, 只改这一个函数就能换来源。
 *
 *    注意(重要, 建议与队友确认): kalman.h 注释写的是
 *        X[0] = 相位差(ns),  X[1] = 频率差(ppb)
 *    架构图和闭环图也都要求用 X[1](X1 = 频率偏差估计)。但 kalman.c 的
 *    预测式是  X[0] = X[0] + T*X[1];  X[1] = X[1], 也就是 X[0] 会按 X[1]
 *    累积, 因此在这份滤波器里 **瞬时频率(变化率)实际落在 X[0]**, X[1] 是
 *    被积分的那个量；main.c 的 update_sample_period() 把 fabs(X[1]) 当频率
 *    用、dac_update() 把 X[1] 当频率差算 DAC, 也是同一个原因。
 *
 *    本模块按要求默认取 X[1]。若实测 X[1] 收敛后近似不变、而 X[0] 在零附近
 *    抖动, 说明滤波器的 X 顺序与注释相反, 把下面改成 kalman.X[0] 即可。
 *--------------------------------------------------------------------------*/
static double led_read_abs_freq_ppb(void)
{
    double v = kalman.X[1];        /* 架构图 / 闭环图: X[1] = 频率偏差估计 */

    if (v < 0.0) {
        v = -v;
    }
    return v;
}

/*---------------------------------------------------------------------------
 * 5. |X[1]| -> 闪烁频率 分段线性表 (ppb, Hz), 必须按 ppb 升序
 *--------------------------------------------------------------------------*/
typedef struct {
    double ppb;
    double hz;
} LedPoint;

static const LedPoint LED_BLINK_TABLE[] = {
    {   0.0, 0.50 },   /* 锁定良好: 2 秒一个周期, 慢闪          */
    {   0.1, 0.70 },
    {   0.3, 1.00 },   /* 1 Hz                                  */
    {   1.0, 1.50 },
    {   3.0, 2.20 },
    {  10.0, 3.20 },
    {  30.0, 4.50 },
    { 100.0, 6.00 },
    { 200.0, 8.00 },   /* 频率差大: 8 Hz, 闪得很快              */
};
#define LED_TABLE_LEN   (sizeof(LED_BLINK_TABLE) / sizeof(LED_BLINK_TABLE[0]))

/*---------------------------------------------------------------------------
 * 6. 模块内部状态
 *--------------------------------------------------------------------------*/
static double   s_abs_x1        = 0.0;    /* 当前 |X[1]| (ppb)       */
static double   s_freq_hz       = LED_FREQ_MIN_HZ;  /* 当前闪烁频率  */
static uint32_t s_half_us       = 1000000U;         /* 当前半周期 us */
static uint32_t s_level         = LED_OFF_LEVEL;    /* 当前 LED 电平 */
static XTime    s_last_toggle   = 0;      /* 上次翻转的定时器值      */
static int      s_timer_ready   = 0;      /* 计时基准是否可用        */
static int      s_pin_ready     = 0;      /* 引脚是否配置成功        */

#if !LED_USE_PL_REG && LED_HAVE_GPIOPS
static XGpioPs  s_gpio;                   /* PS GPIO 控制器实例      */
#endif

/*---------------------------------------------------------------------------
 * 7. 引脚底层驱动
 *--------------------------------------------------------------------------*/

#if LED_USE_PL_REG
/* ---- 通道 A: 写 AXI 寄存器, 由 PL 输出 ---- */
static void led_pin_setup(void)
{
    Xil_Out32(LED_CTRL_REG, (uint32_t)LED_OFF_LEVEL);   /* 先灭灯 */
    s_pin_ready = 1;
}

static void led_pin_write(uint32_t level)
{
    Xil_Out32(LED_CTRL_REG, (uint32_t)(level & LED_BIT_LEVEL));
}

#elif LED_HAVE_GPIOPS
/* ---- 通道 B: PS GPIO 直接驱动 E6 / PS_MIO0_5 ---- */
static void led_pin_setup(void)
{
    XGpioPs_Config *cfg;
    u32 bank;

    cfg = XGpioPs_LookupConfig((u16)LED_GPIO_DEV_ID);
    if (cfg == NULL) {
        s_pin_ready = 0;
        return;
    }
    if (XGpioPs_CfgInitialize(&s_gpio, cfg, cfg->BaseAddr) != XST_SUCCESS) {
        s_pin_ready = 0;
        return;
    }

    /* bank: MIO0~31 -> 1, MIO32~53 -> 2, MIO54~85 -> 3 */
    bank = (LED_MIO_PIN < 32U) ? 1U : ((LED_MIO_PIN < 54U) ? 2U : 3U);
    if ((bank < 1U) || (bank > 3U)) {
        s_pin_ready = 0;             /* 引脚号非法, 不配置 */
        return;
    }

    XGpioPs_SetDirectionPin(&s_gpio, LED_MIO_PIN, 1U);   /* 1 = 输出 */
    XGpioPs_SetOutputEnablePin(&s_gpio, LED_MIO_PIN, 1U);/* 使能输出 */
    XGpioPs_WritePin(&s_gpio, LED_MIO_PIN, (u32)LED_OFF_LEVEL); /* 先灭灯 */

    s_pin_ready = 1;
}

static void led_pin_write(uint32_t level)
{
    if (s_pin_ready) {
        XGpioPs_WritePin(&s_gpio, LED_MIO_PIN, (u32)(level & 1U));
    }
}

#else
/* ---- 通道 C: BSP 中没有 xgpiops 驱动 ----
 * Zynq 的 Vitis BSP 默认包含 xgpiops, 所以正常工程不会走到这里。
 * 直接照着 TRM 手搓 GPIO 寄存器很容易写错地址, 这里不做, 只给出明确提示:
 * 要么在 BSP 里勾上 ps7_gpio / xgpiops, 要么改用 LED_USE_PL_REG = 1 走 PL。
 */
#error "led_ctrl: 未找到 xgpiops 驱动。请在 Vitis BSP 中启用 PS GPIO(xgpiops), 或把 LED_USE_PL_REG 置 1 改走 PL 寄存器通道。"
#endif

/* 统一入口: 只在引脚配置成功后才真正写 */
#if defined(__GNUC__)
  /* #error 分支下这个函数没人调用, 用它压掉 unused 警告 */
  static void led_write_level(uint32_t level) __attribute__((unused));
#endif
static void led_write_level(uint32_t level)
{
    led_pin_write(level);
    s_level = level;
}

/*---------------------------------------------------------------------------
 * 8. 计算辅助
 *--------------------------------------------------------------------------*/

/* 取两次定时器读数之差, 换算成微秒. 用 64 位避免回流时下溢 */
static uint32_t led_elapsed_us(XTime now, XTime then)
{
    XTime delta = now - then;                       /* 无符号, 回流安全 */
    XTime limit = (LED_TIMER_HZ * 2ULL);            /* 2 秒对应的计数 */

    /* 限幅: 既防止 delta*1e6 溢出 64 位, 也避免长时间不调用后补偿式连翻。
     * 最大半周期为 1e6us(1 秒), 超过 2 秒的差值一律按 2 秒算, 不影响判断。 */
    if (delta > limit) {
        delta = limit;
    }
    return (uint32_t)((delta * US_PER_SECOND) / LED_TIMER_HZ);
}

/* 分段线性插值: |X[1]| -> 频率 Hz, 表外按端点饱和 */
static double led_freq_from_abs_x1(double x)
{
    unsigned int i;
    double x0, y0, x1, y1;

    if (x <= LED_BLINK_TABLE[0].ppb) {
        return LED_BLINK_TABLE[0].hz;
    }
    if (x >= LED_BLINK_TABLE[LED_TABLE_LEN - 1U].ppb) {
        return LED_BLINK_TABLE[LED_TABLE_LEN - 1U].hz;
    }

    for (i = 0; i < (LED_TABLE_LEN - 1U); i++) {
        x0 = LED_BLINK_TABLE[i].ppb;
        x1 = LED_BLINK_TABLE[i + 1U].ppb;
        if ((x >= x0) && (x <= x1)) {
            y0 = LED_BLINK_TABLE[i].hz;
            y1 = LED_BLINK_TABLE[i + 1U].hz;
            if (x1 <= x0) {
                return y0;                          /* 防除零 */
            }
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
        }
    }
    return LED_FREQ_MIN_HZ;                         /* 理论上到不了 */
}

/* 频率 Hz -> 半周期 us, 并做上下限保护 */
static uint32_t led_half_period_us(double hz)
{
    double half;

    if (hz < LED_FREQ_MIN_HZ) hz = LED_FREQ_MIN_HZ;
    if (hz > LED_FREQ_MAX_HZ) hz = LED_FREQ_MAX_HZ;

    half = (double)(US_PER_SECOND / 2ULL) / hz;     /* 500000 / hz */
    if (half > 4294967295.0) {
        return 4294967295U;
    }
    if (half < (double)LED_HALF_PERIOD_MIN_US) {
        return LED_HALF_PERIOD_MIN_US;
    }
    return (uint32_t)half;
}

/*---------------------------------------------------------------------------
 * 9. 对外接口
 *--------------------------------------------------------------------------*/

/* 初始化: 配置引脚为输出并先灭灯, 再取一次计时基准 */
void led_ctrl_init(void)
{
    XTime t0, t1;

    s_abs_x1      = 0.0;
    s_freq_hz     = LED_FREQ_MIN_HZ;
    s_half_us     = led_half_period_us(s_freq_hz);
    s_level       = LED_OFF_LEVEL;
    s_timer_ready = 0;

    led_pin_setup();
    led_write_level(LED_OFF_LEVEL);

    /* 取两次读数, 若不同说明全局定时器在跑; 相同则说明没使能,
     * 此时 tick 里保持不动, 避免 LED 乱闪 */
    XTime_GetTime(&t0);
    XTime_GetTime(&t1);
    s_last_toggle = t1;
    if (t1 != t0) {
        s_timer_ready = 1;
    }
}

/* 每秒调用一次: 由 |X[1]| 计算新的闪烁频率(带斜率限制) */
void led_update(void)
{
    double target_hz, delta;

    /* ---- 1. 取 |X[1]|, 单位 ppb ---- */
    s_abs_x1 = led_read_abs_freq_ppb();

    /* ---- 2. 映射成目标频率 ---- */
    target_hz = led_freq_from_abs_x1(s_abs_x1);

    /* ---- 3. 斜率限制: 每次最多变化 LED_FREQ_SLEW_HZ, 防抖 ---- */
    delta = target_hz - s_freq_hz;
    if (delta > LED_FREQ_SLEW_HZ) {
        delta = LED_FREQ_SLEW_HZ;
    } else if (delta < -LED_FREQ_SLEW_HZ) {
        delta = -LED_FREQ_SLEW_HZ;
    }
    s_freq_hz += delta;
    if (s_freq_hz < LED_FREQ_MIN_HZ) s_freq_hz = LED_FREQ_MIN_HZ;
    if (s_freq_hz > LED_FREQ_MAX_HZ) s_freq_hz = LED_FREQ_MAX_HZ;

    /* ---- 4. 换算半周期 ---- */
    s_half_us = led_half_period_us(s_freq_hz);
}

/* 主循环高频调用: 每到半周期翻转一次 LED */
void led_ctrl_tick(void)
{
    XTime now;

    if (!s_timer_ready || !s_pin_ready) {
        return;                      /* 基准或引脚不可用: 保持当前状态 */
    }

    XTime_GetTime(&now);

    /* 用 >= 而不是 ==, 主循环较慢时也不会漏掉翻转 */
    if (led_elapsed_us(now, s_last_toggle) >= s_half_us) {
        s_last_toggle = now;         /* 立刻复位基准, 避免补偿式连翻 */
        if (s_level == LED_ON_LEVEL) {
            led_write_level(LED_OFF_LEVEL);
        } else {
            led_write_level(LED_ON_LEVEL);
        }
    }
}

/*---------------------------------------------------------------------------
 * 10. 查询接口(调试/串口打印用)
 *--------------------------------------------------------------------------*/
int led_pin_ready(void)
{
    return s_pin_ready;
}

double led_get_abs_x1(void)
{
    return s_abs_x1;
}

double led_get_freq_hz(void)
{
    return s_freq_hz;
}

uint32_t led_get_half_period_us(void)
{
    return s_half_us;
}

/*=============================================================================
 * 附 1: 若希望 PL 自己完成闪烁(PS 只给半周期), 把 LED_USE_PL_REG 置 1, 并在
 *       RTL 中实现 0x10 写口(参考 src/rtl/led_ctrl.v), 然后把 led_pin_write()
 *       的 AXI 分支改成写半周期:
 *
 *           Xil_Out32(LED_CTRL_REG, s_half_us);
 *
 *       该寄存器在 RTL 里解释为"半周期(us)", PL 用时钟计数自己翻转,
 *       CPU 不再参与, 抖动更小。
 *
 * 附 2: 引脚说明
 *       E6 = PS_MIO0_5 属于 PS 的 MIO, 只有 PS 能驱动, PL 不能。
 *       板上 PL 侧 LED 是 M14/M15/K16/J16(BANK35, 3.3V, 低电平点亮),
 *       若要改用 PL 驱动, 请把 led_ctrl.v 的 led_pin 约束到其中一个,
 *       并让 PS 通过 0x10 寄存器下发。
 *===========================================================================*/
