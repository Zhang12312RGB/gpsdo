
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "kalman.h"
#include "control.h"
#define BSP_Q_OK 0
#define MAIN_LOG_QUIET 0
static int main_log=MAIN_LOG_QUIET;
int sys_state=STATE_HOLDOVER,sample_period=10;
double main_phase_acc_ns=0.,main_last_phase_ns=0.,main_p_sum=0.;
static int resume_flag=0,holdover_seconds=0,invalid_count=0;
static int state_before_holdover=STATE_TUNING;
static int s_prev_dac=1854,last_dac=1854;
static int g_last_why=0,g_last_acc=1,valid=1,dac_lock=1,entries=0,writes=0;
static double dac_acc=1854.,cand_time=0.;
void dbg_printf(const char *fmt,...) __attribute__((format(printf,1,2)));
void dbg_printf(const char *fmt,...) {(void)fmt;}
static int bsp_meas_valid(void){return valid;}
static double read_phase_error_ns(void){return 2.;}
int control_get_last_dac(void){return last_dac;}
int control_get_loop_sign(void){return 1;}
static void control_write_dac(int code){if(code!=last_dac)exit(90);writes++;}
static void cand_obs_predict_only(double T){if(T!=1.)exit(91);cand_time+=T;}
static const char *state_name(int s){return s==STATE_CALIB?"CALIB":"TUNING";}
static void enter_state(int s){sys_state=s;dac_lock=(s!=STATE_TUNING);entries++;}
static void check(int ok,const char *msg){if(!ok){fprintf(stderr,"FAIL %s\n",msg);exit(1);}}
static void reset(void){kalman_init(&kalman);kalman.X[0]=1000.;kalman.X[1]=2.;
 kalman.P[0]=10.;kalman.P[1]=0.;kalman.P[2]=0.;kalman.P[3]=.1;
 sample_period=10;resume_flag=0;holdover_seconds=0;sys_state=STATE_HOLDOVER;
 state_before_holdover=STATE_TUNING;s_prev_dac=last_dac=1854;g_last_why=0;g_last_acc=valid=1;
 dac_lock=1;entries=writes=0;dac_acc=1854.;cand_time=0.;main_phase_acc_ns=1002.;}

void control_holdover_tick(void)
{
    control_write_dac(last_dac);
}
void control_resume_from_holdover(void)
{
    kalman.X[0] = main_phase_acc_ns;   /* 对齐到当前绝对相位 */

    /* 守时期间 X[1] 是自由外推的。如果它已经顶到限幅（说明失锁前
     * 状态就已经不可信），恢复后继续沿用会把 DAC 推向错误方向 ——
     * 实测拔天线后 X[1] 停在 -5.000 ppb（= KALMAN_X1_MAX）。
     * 这种情况直接清零，让滤波器重新学。 */
    if ((kalman.X[1] >=  (KALMAN_X1_MAX - 0.01)) ||
        (kalman.X[1] <= -(KALMAN_X1_MAX - 0.01))) {
        kalman.X[1] = 0.0;
    }

    kalman.P[0] = 1e4;
    kalman.P[1] = 0;
    kalman.P[2] = 0;
    kalman.P[3] = 1e2;
    dac_acc = (double)last_dac;   // 增量累加器对齐当前输出
}
static void task_holdover_before(void)
{
    holdover_seconds++;

    if (resume_flag != 0) {
        kalman.R = 5000.0;
        /* 有有效测量才做平滑, 否则 z 是陈旧值/0, 会把恢复过程带偏。
         * holdover_seconds 照常累加, 不因为暂时没有测量而卡住恢复流程。 */
        if (bsp_meas_valid() != 0) {
            kalman_update(&kalman, read_phase_error_ns(), 1.0);
        }
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
        kalman.X[0] += kalman.X[1] * (double)sample_period;   // 自由外推
        control_holdover_tick();                              // DAC 保持
    }
}
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
int main(void){
 reset();task_holdover_before();check(fabs(kalman.X[0]-1020.)<1e-10,"old 10x defect reproduced");
 reset();task_holdover_second();check(fabs(kalman.X[0]-1002.)<1e-10,"one second phase advance");
 check(kalman.P[0]>10. && cand_time==1.,"missing observation uncertainty grows");
 check(dac_lock && writes==1 && last_dac==1854,"holdover command stays fixed");
 reset();last_dac=1855;task_holdover_second();
 check(fabs(kalman.X[1]-2.47852)<1e-10 && s_prev_dac==1855,"pending DAC input applied once");
 task_holdover_second();check(fabs(kalman.X[1]-2.47852)<1e-10,"DAC input not repeated");
 reset();resume_flag=1;task_holdover_second();
 check(fabs(kalman.X[0]-1002.)<1e-9 && fabs(kalman.X[1]-2.)<1e-9,"recovery uses accumulated phase");
 check(entries==0 && dac_lock,"no early release");
 reset();resume_flag=1;holdover_seconds=4;g_last_why=1;g_last_acc=0;main_phase_acc_ns=1e9;
 task_holdover_second();check(holdover_seconds==0 && entries==0,"bad recovery sample does not release");
 check(fabs(kalman.X[0]-1002.)<1e-9,"bad recovery is prediction only");
 reset();resume_flag=1;holdover_seconds=4;state_before_holdover=STATE_CALIB;
 task_holdover_second();check(sys_state==STATE_CALIB && dac_lock && entries==1,"return to CALIB keeps DAC locked");
 reset();resume_flag=1;holdover_seconds=4;state_before_holdover=STATE_TUNING;
 task_holdover_second();check(sys_state==STATE_TUNING && !dac_lock && entries==1,"return to TUNING");
 puts("PASS holdover time, covariance, pending DAC input, accumulated recovery, bad-sample delay and CALIB/TUNING return");return 0;
}
