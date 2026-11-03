#ifndef CAND_OBS_H
#define CAND_OBS_H

/*=============================================================================
 * 候选观察器（2026-10-31，补齐版）
 *-----------------------------------------------------------------------------
 * 与主 KF 保持【同等信息】，只改 Q_FREQ：
 *   · 同一 z、同一更新拍
 *   · 同步 DAC 阶跃补偿
 *   · 缺测时同步 predict_only
 *   · 只在真实更新拍打日志
 *
 * 候选不写 DAC、不改保护、不清慢环窗口。
 *===========================================================================*/

void cand_obs_init(void);
void cand_obs_update(double z, double T);
void cand_obs_predict_only(double T);
void cand_obs_log(void);

double cand_obs_get_x1(int i);
double cand_obs_get_innov(int i);
double cand_obs_get_S(int i);
double cand_obs_get_P3(int i);
double cand_obs_get_K1(int i);
double cand_obs_get_q(int i);

#endif /* CAND_OBS_H */
