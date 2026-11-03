
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "main.h"
#include "kalman.h"
#include "slow_loop.h"
Kalman kalman;SLOW_LOOP slow;
int sys_state=STATE_TUNING,sample_period=10;
double main_last_phase_ns=0.,main_phase_acc_ns=0.,main_p_sum=0.;
static double g_R=80.,g_fuse_w=0.,g_fuse_fsl=0.,g_phase_term=0.;
static int main_fix50_cnt=0,main_fix50_real=0,g_fuse_src=0;
static char output[4096];
static double gnss_hdop(void){return 1.999;}
static double gnss_cn0(void){return 35.99;}
static int gnss_fix(void){return 1;}
static int gnss_sat(void){return 8;}
static const char *state_name(int s){(void)s;return "TUNING";}
static int control_get_last_dac(void){return 1854;}
static double control_get_dac_acc(void){return 1853.999;}
void dbg_printf(const char *fmt,...) __attribute__((format(printf,1,2)));
void dbg_printf(const char *fmt,...){va_list ap;va_start(ap,fmt);vsnprintf(output,sizeof(output),fmt,ap);va_end(ap);}
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
int main(void){log_text();if(!strstr(output,"hdop=2.00 cn0=36.0")||!strstr(output,"acc=1854.00")){puts(output);return 1;}
 puts("PASS display carries and complete log format argument checking");return 0;}
