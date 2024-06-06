#ifndef __KEY_H
#define	__KEY_H

#include "stm32f10x.h"

//外部中断检测  KEY1->PC0 、 KEY2->PC1
/*KEY1宏定义*/
#define    KEY1_GPIO_CLK            RCC_APB2Periph_GPIOC
#define    KEY1_GPIO_PORT           GPIOC			   
#define    KEY1_GPIO_PIN            GPIO_Pin_0
#define    KEY1_EXTI_PORTSOURCE     GPIO_PortSourceGPIOC
#define    KEY1_EXTI_PINSOURCE      GPIO_PinSource0
#define    KEY1_EXTI_LINE           EXTI_Line0

/*KEY2宏定义*/
#define    KEY2_GPIO_CLK            RCC_APB2Periph_GPIOC
#define    KEY2_GPIO_PORT           GPIOC		   
#define    KEY2_GPIO_PIN            GPIO_Pin_1
#define    KEY2_EXTI_PORTSOURCE     GPIO_PortSourceGPIOC
#define    KEY2_EXTI_PINSOURCE      GPIO_PinSource1
#define    KEY2_EXTI_LINE           EXTI_Line1

/*KEY中断源宏定义*/
#define    KEY1_EXTI_IRQ            EXTI0_IRQn
#define    KEY1_IRQHandler          EXTI0_IRQHandler

#define    KEY2_EXTI_IRQ            EXTI1_IRQn
#define    KEY2_IRQHandler          EXTI1_IRQHandler


extern uint8_t key1_val,key2_val;


void Key_GPIO_Config(void);

#endif /* __KEY_H */

