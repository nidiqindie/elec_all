#ifndef __MOTOR_H
#define __MOTOR_H
/*PID时基(TIMER6定义*/
#define            PID_TIMEBASE_TIM               TIM6
#define            PID_TIMEBASE_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            PID_TIMEBASE_CLK               RCC_APB1Periph_TIM6
#define            PID_TIMEBASE_Period            10000
#define            PID_TIMEBASE_Prescaler         72

#define            PID_TIMEBASE_IRQn                TIM6_IRQn
#define            PID_TIMEBASE_IRQ_IRQHandler      TIM6_IRQHandler
/*end*/

extern int flag_L,flag_R;
extern int flag_adjusting;

void Motor_Init(void);
void Set_L1Speed(int16_t LSpeed);  //满速1000
void Set_L2Speed(int16_t LSpeed);
void Set_R1Speed(int16_t RSpeed);
void Set_R2Speed(int16_t RSpeed);
void PID_timebase_init(void); //Timer6

void Forward(float Speed);
void Backward(float Speed);
void L_Translation(float Speed);
void R_Translation(float Speed);
void L_Rotation(void);
void R_Rotation(void);
void Stop(void);
void weitiao(void);

#endif
