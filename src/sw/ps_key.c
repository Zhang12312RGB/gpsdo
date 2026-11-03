#include "ps_key.h"
#include "bsp.h"          // bsp_millis()
#include "xparameters.h"
#include "xgpiops.h"

//-----------------------------------------------------------------------------
// MIO 引脚号（AX7010 手册 §6.9）
//-----------------------------------------------------------------------------
static const u32 key_pin[PS_KEY_NUM] = { 50U, 51U };   // KEY1, KEY2

extern XGpioPs_Config XGpioPs_ConfigTable[];

static XGpioPs key_gpio;
static int     key_ready = 0;

static int  stable_lv[PS_KEY_NUM];    // 去抖后的稳定电平: 1 = 按下
static int  cand_lv[PS_KEY_NUM];      // 候选电平
static u32  cand_at[PS_KEY_NUM];      // 候选电平出现时刻(ms)
static u32  last_ev[PS_KEY_NUM];      // 上次产生事件的时刻(ms)
static int  ev_pending[PS_KEY_NUM];   // 事件队列(深度 1)

static XGpioPs_Config *ps_key_cfg(void)
{
    return &XGpioPs_ConfigTable[0];
}

//-----------------------------------------------------------------------------
// 初始化: 两个按键引脚配置为输入（PS 侧 GPIO, 不碰 PL）
//-----------------------------------------------------------------------------
void ps_key_init(void)
{
    XGpioPs_Config *cfg;
    int i;

    key_ready = 0;

    cfg = ps_key_cfg();
    if (cfg == NULL) {
        return;
    }
    if (XGpioPs_CfgInitialize(&key_gpio, cfg, cfg->BaseAddr) != XST_SUCCESS) {
        return;
    }

    for (i = 0; i < PS_KEY_NUM; i++) {
        XGpioPs_SetDirectionPin(&key_gpio, key_pin[i], 0U);   // 0 = 输入
        stable_lv[i]  = 0;
        cand_lv[i]    = 0;
        cand_at[i]    = 0;
        last_ev[i]    = 0;
        ev_pending[i] = 0;
    }

    key_ready = 1;
}

int ps_key_ready(void)
{
    return key_ready;
}

//-----------------------------------------------------------------------------
// 原始电平: 按键低有效, 读回 0 表示按下
//-----------------------------------------------------------------------------
int ps_key_raw(int idx)
{
    if ((!key_ready) || (idx < 0) || (idx >= PS_KEY_NUM)) {
        return 0;
    }
    return (XGpioPs_ReadPin(&key_gpio, key_pin[idx]) == 0U) ? 1 : 0;
}

int ps_key_level(int idx)
{
    if ((idx < 0) || (idx >= PS_KEY_NUM)) {
        return 0;
    }
    return stable_lv[idx];
}

//-----------------------------------------------------------------------------
// 主循环高频调用: 采样 -> 去抖 -> 按下沿产生事件
//-----------------------------------------------------------------------------
void ps_key_poll(void)
{
    u32 now;
    int i, lv;

    if (!key_ready) {
        return;
    }
    now = bsp_millis();

    for (i = 0; i < PS_KEY_NUM; i++) {
        lv = ps_key_raw(i);

        if (lv != cand_lv[i]) {
            // 电平刚变: 重新计时, 不立刻采纳
            cand_lv[i] = lv;
            cand_at[i] = now;
        } else if ((cand_lv[i] != stable_lv[i]) &&
                   ((u32)(now - cand_at[i]) >= PS_KEY_DEBOUNCE_MS)) {
            // 连续 PS_KEY_DEBOUNCE_MS 保持同一电平 -> 确认
            stable_lv[i] = cand_lv[i];
            if (stable_lv[i] != 0) {
                ev_pending[i] = 1;          // 只在"按下"沿产生事件
                last_ev[i]    = now;
            }
        }

#if (PS_KEY_REPEAT_MS > 0U)
        if ((stable_lv[i] != 0) && ((u32)(now - last_ev[i]) >= PS_KEY_REPEAT_MS)) {
            ev_pending[i] = 1;              // 长按重复
            last_ev[i]    = now;
        }
#endif
    }
}

int ps_key_get_event(void)
{
    int i;

    for (i = 0; i < PS_KEY_NUM; i++) {
        if (ev_pending[i] != 0) {
            ev_pending[i] = 0;
            return (i == PS_KEY_IDX_1) ? PS_KEY_EVENT_1 : PS_KEY_EVENT_2;
        }
    }
    return PS_KEY_NONE;
}
