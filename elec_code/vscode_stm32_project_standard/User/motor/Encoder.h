#ifndef __ENCODER_H
#define __ENCODER_H
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
void Encoder_Init_TIM4(void);
void Encoder_Init_TIM5(void);
int Read_L1Speed(void);
int Read_L2Speed(void);
int Read_R1Speed(void);
int Read_R2Speed(void);
#endif
