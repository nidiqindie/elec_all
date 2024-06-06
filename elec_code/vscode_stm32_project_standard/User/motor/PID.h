#ifndef __PID_H
#define __PID_H
extern int flag_runpid;
extern float curL1_value,curL2_value,curR1_value,curR2_value;  //当前各轮行进积累值
extern float pidOut_L1,pidOut_L2,pidOut_R1,pidOut_R2 ;  //PID计算输出的速度
extern float tarSp_L1,tarSp_L2,tarSp_R1,tarSp_R2;
extern int A_j,B_j,C_j,D_j;
extern float tarAngle;  //目标偏航角角度
extern int flag_pos;

void PID_timebase_init(void); //Timer6
void runPID(void);

float IncPIDCalcL(void);  //增量式PID计算
float IncPIDCalcR(void);  //增量式PID计算

float IncPIDCalc_L1(void);
float IncPIDCalc_L2(void);
float IncPIDCalc_R1(void);
float IncPIDCalc_R2(void);
#endif
