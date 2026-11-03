#ifndef ADAPTIVE_R_H
#define ADAPTIVE_R_H

/* Fixed operational R; learned quality models remain an offline extension. */
#define ADAPTIVE_R_FIXED_NS2 80.0

#define ADAPT_POINTS    40          /* ← 由 20 提到 40, 4 个未知数更稳  */
#define PHASE_WIN       20
#define HDOP_THRESH     0.5
#define SAT_THRESH      1
#define CN0_THRESH      3.0
#define JUMP_THRESH     3.0
#define R_MIN           10.0
#define R_MAX           50000.0

#define DEBOUNCE_N      3           /* 连续 N 拍越阈值才算真跳变        */
#define SETTLE_N        PHASE_WIN   /* 窗口填满才允许记录下一点         */

/* 保留定义以符合文件规范(新方案不直接用它) */
typedef struct _VarPoints {
    double x[ADAPT_POINTS];
    double R[ADAPT_POINTS];
    int    n;
} VarPoints;

/* 一个完整工作点: y = a*hdop + b*sat + c*cn0inv + d */
typedef struct _RPoint {
    double hdop;
    double sat;
    double cn0inv;
    double var;
} RPoint;

typedef struct _AdaptiveR {
    /* 系数 */
    double a, b, c, d;

    /* 完整工作点(替代原三个 VarPoints) */
    RPoint pts[ADAPT_POINTS];
    int    n;

    /* 相位窗口 */
    double phase_win[PHASE_WIN];
    int    win_cnt, win_idx;

    /* 已确认基准(只在确认跳变时更新) */
    double cur_hdop;
    int    cur_sat;
    double cur_cn0;
    int    have_cur;

    /* 消抖计数 */
    int    cand_hdop, cand_sat, cand_cn0;

    /* 稳定窗口与变化前方差快照 */
    int    stable_cnt;
    double var_pre;
    int    var_valid;

    int    ready;
} AdaptiveR;

extern AdaptiveR ar;

void   adaptive_r_init(void);
void   adaptive_r_reset(void);
void   adaptive_r_update(double hdop, int sat, double cn0, double phase);
double adaptive_r_get(double hdop, int sat, double cn0);
void   adaptive_r_fit(void);

#endif /* ADAPTIVE_R_H */