#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "slow_loop.h"
static void check(int ok,const char *why) {if(!ok){fprintf(stderr,"FAIL %s\n",why);exit(1);}}
int main(void) {
    SLOW_C_WEIGHT s;double w,old;int i;
    slow_c_weight_reset(&s,SLOW_CW_DISABLED);
    check(s.used==0 && !s.have_time,"reset");
    w=slow_c_weight_step(&s,1000,.8,1.,301.,SLOW_CW_OK);
    check(w==0 && fabs(s.target-.08)<1e-12,"start at zero");
    for(i=1;i<=30;i++) {
        old=w;w=slow_c_weight_step(&s,1000+10000U*i,.8,1.,301.,SLOW_CW_OK);
        check(w>=old && w<=.08+1e-12,"bounded ramp");
        check(w-old<=.1/300.*10.+1e-12,"wall time rise rate");
    }
    check(fabs(w-.08)<1e-12,"reach scaled target");
    w=slow_c_weight_step(&s,311000,.1,1.,301.,SLOW_CW_OK);
    check(fabs(w-.01)<1e-12,"lower confidence immediate");
    w=slow_c_weight_step(&s,321000,1.,331.,301.,SLOW_CW_OK);
    check(w==0 && s.reason==SLOW_CW_AGE && !s.have_time,"age exit");
    w=slow_c_weight_step(&s,331000,1.,1.,631.,SLOW_CW_OK);
    check(w==0 && s.reason==SLOW_CW_LAG,"lag exit");
    w=slow_c_weight_step(&s,341000,NAN,1.,301.,SLOW_CW_OK);
    check(w==0 && s.reason==SLOW_CW_NUMERIC,"NaN rejected");
    w=slow_c_weight_step(&s,351000,1.,1.,301.,SLOW_CW_PROTECT);
    check(w==0 && s.reason==SLOW_CW_PROTECT,"protection exit");
    w=slow_c_weight_step(&s,361000,1.,1.,301.,SLOW_CW_OK);
    check(w==0,"reentry starts fresh");
    w=slow_c_weight_step(&s,401001,1.,1.,301.,SLOW_CW_OK);
    check(w==0 && s.reason==SLOW_CW_CLOCK,"long update gap resets");
    slow_c_weight_step(&s,0xfffffff0U,1.,1.,301.,SLOW_CW_OK);
    w=slow_c_weight_step(&s,16U,1.,1.,301.,SLOW_CW_OK);
    check(fabs(w-.1/300.*.032)<1e-12,"normal timer wrap");
    w=slow_c_weight_step(&s,15U,1.,1.,301.,SLOW_CW_OK);
    check(w==0 && s.reason==SLOW_CW_CLOCK,"backward clock resets");
    slow_c_weight_step(&s,100U,2.,1.,301.,SLOW_CW_OK);
    check(s.target<=.1,"maximum target");
    slow_c_weight_reset(&s,SLOW_CW_GAP);
    check(s.used==0 && s.target==0 && s.reason==SLOW_CW_GAP,"gap exit");
    puts("weight core: ramp, cap, downweight, expiry, NaN, protection, reentry, clock and gap PASS");
    return 0;
}
