#ifndef _BSP_MOTOR_H_
#define	_BSP_MOTOR_H_

#include "stm32f10x.h"



/*PID时基(TIMER6定义*/
#define            PID_TIMEBASE_TIM               TIM6
#define            PID_TIMEBASE_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            PID_TIMEBASE_CLK               RCC_APB1Periph_TIM6
#define            PID_TIMEBASE_Period            10000
#define            PID_TIMEBASE_Prescaler         72

#define            PID_TIMEBASE_IRQn                TIM6_IRQn
#define            PID_TIMEBASE_IRQ_IRQHandler      TIM6_IRQHandler
/*end*/

extern int pos_flag;
extern int speed;

extern int curAngle ;  //当前偏航角角度

// void PID_timebase_init(void);

void move_forward(uint16_t speed,uint16_t j, float quan) ; //speed指每10ms脉冲个数

void move_backward(uint16_t speed,uint16_t j,float quan);  //speed指每10ms脉冲个数

void stop(void);

void move_left(uint16_t speed,uint16_t j,float quan); //speed指每10ms脉冲个数

void move_right(uint16_t speed,uint16_t j,float quan);  //speed指每10ms脉冲个数
void move_forward_pos(uint16_t speed,uint16_t j,float quan);
void move_backward_pos(uint16_t speed,uint16_t j,float quan);

void L_R(int speed , int j) ; //左自转

void R_R(int speed , int j); //右自转
void weitiao_2(void);

void yaw_run(int16_t target_yaw);
void weitiao(int gg);
void oo(void);
void se_huan(uint16_t num);
void Forward(int speeed);
void huan(void);
void maduo(uint16_t num);
void huan1(void);
#endif


