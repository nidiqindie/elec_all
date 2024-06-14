#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

//SERVO_1->PA8��SERVO_2->PA9��SERVO_3->PA10��SERVO_4->PA11

/**************************SERVO********************************/
#define            SERVO_TIM                   TIM1
#define            SERVO_TIM_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define            SERVO_TIM_CLK               RCC_APB2Periph_TIM1
#define            SERVO_TIM_Period            2000
#define            SERVO_TIM_Prescaler         720
// TIM3 ����Ƚ�ͨ��1
#define            SERVO_TIM_CH1_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            SERVO_TIM_CH1_PORT          GPIOA
#define            SERVO_TIM_CH1_PIN           GPIO_Pin_8

// TIM3 ����Ƚ�ͨ��2
#define            SERVO_TIM_CH2_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            SERVO_TIM_CH2_PORT          GPIOA
#define            SERVO_TIM_CH2_PIN           GPIO_Pin_9

// TIM3 ����Ƚ�ͨ��3
#define            SERVO_TIM_CH3_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            SERVO_TIM_CH3_PORT          GPIOA
#define            SERVO_TIM_CH3_PIN           GPIO_Pin_10

// TIM3 ����Ƚ�ͨ��4
#define            SERVO_TIM_CH4_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            SERVO_TIM_CH4_PORT          GPIOA
#define            SERVO_TIM_CH4_PIN           GPIO_Pin_11

/**************************��������********************************/
void SERVO_TIM_Init(void);
void Servo_Angle_Config(uint8_t channel_n , int16_t angle);
void Compute_Angle_and_Response(float CAR_X, float CAR_Y, float CAR_Z);
#endif	/* __BSP_PWM_H */


