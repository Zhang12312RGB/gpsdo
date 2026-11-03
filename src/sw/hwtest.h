#ifndef HWTEST_H
#define HWTEST_H

//=============================================================================
// hwtest.h -- 上板硬件标定与自测（只写 UART1，不参与闭环）
//-----------------------------------------------------------------------------
// PCB 回来后一次性测三件事：
//   ① OCXO 极性：DAC 码值↑ 是让频率↑ 还是↓
//   ② 灵敏度   ：g = ppb/码，并换算成 Hz/码
//   ③ TDC 标定 ：0x0C 每周期抽头数、0x10 固定延迟
//
// 全程【开环】：DAC 用 Xil_Out32 直接钉死，不调用 dac_update()，
// 这样相位才会随频率偏差线性漂移，测出来的才是真频差。
//=============================================================================

#define HWTEST_DUMP        1   /* 只打寄存器快照        */
#define HWTEST_POLARITY    2   /* 只测极性 + 灵敏度     */
#define HWTEST_TDC         3   /* 只测 TDC 0x0C/0x10    */
#define HWTEST_ALL         9   /* 全部跑一遍            */

/* ---- 测试参数：想改就改这里 ---- */
#define HT_TDC_SAMPLES   300   // 300 s: at the lock point the phase walks a few ns per MINUTE,
                               // so 300 s covers several wraps of the 10 ns period
#define HT_POL_SAMPLES      30   /* 极性测试每点采样数         */
#define HT_POL_SETTLE       15   /* 换 DAC 后的稳定等待秒数    */
#define HT_DAC_A          1024   /* 低测点                     */
#define HT_DAC_B          3072   /* 高测点（与低点差 2048 码） */
#define HT_DAC_MID        2048   /* 中心，测完恢复到这里       */
#define HT_VERIFY_SAMPLES   20   /* 写完 0x0C 后的复核点数     */

void hwtest_run(int mode);
void hwtest_dump(void);

#endif /* HWTEST_H */
