#ifndef __LED_H
#define	__LED_H


#include "stm32f10x.h"
//LED_R
#define LEDR_GPIO_PORT    	GPIOB			             
#define LEDR_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define LEDR_GPIO_PIN		GPIO_Pin_12			       

//LED_G
#define LEDG_GPIO_PORT    	GPIOB			             
#define LEDG_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define LEDG_GPIO_PIN		GPIO_Pin_13			        

//LED_B
#define LEDB_GPIO_PORT    	GPIOB			            
#define LEDB_GPIO_CLK 	    RCC_APB2Periph_GPIOB	
#define LEDB_GPIO_PIN		GPIO_Pin_14			    

//IO口输出低电平是点灯
#define ON  1
#define OFF 0

#define LEDR(a)	if (a)	\
					GPIO_ResetBits(LEDR_GPIO_PORT,LEDR_GPIO_PIN);\
					else		\
					GPIO_SetBits(LEDR_GPIO_PORT,LEDR_GPIO_PIN)

#define LEDG(a)	if (a)	\
					GPIO_ResetBits(LEDG_GPIO_PORT,LEDG_GPIO_PIN);\
					else		\
					GPIO_SetBits(LEDG_GPIO_PORT,LEDG_GPIO_PIN)

#define LEDB(a)	if (a)	\
					GPIO_ResetBits(LEDB_GPIO_PORT,LEDB_GPIO_PIN);\
					else		\
					GPIO_SetBits(LEDB_GPIO_PORT,LEDB_GPIO_PIN)



#define LED_RED  \
					LEDR(ON);\
					LEDG(OFF);\
					LEDB(OFF)


#define LED_GREEN		\
					LEDR(OFF);\
					LEDG(ON);\
					LEDB(OFF)


#define LED_BLUE	\
					LEDR(OFF);\
					LEDG(OFF);\
					LEDB(ON)

					
					
#define LED_YELLOW	\
					LEDR(ON);\
					LEDG(ON);\
					LEDB(OFF)

#define LED_PURPLE	\
					LEDR(ON);\
					LEDG(OFF);\
					LEDB(ON)


#define LED_CYAN \
					LEDR(OFF);\
					LEDG(ON);\
					LEDB(ON)
					

#define LED_WHITE	\
					LEDR(ON);\
					LEDG(ON);\
					LEDB(ON)
					
					
#define LED_RGBOFF	\
					LEDR(OFF);\
					LEDG(OFF);\
					LEDB(OFF)

void LED_GPIO_Config(void);

#endif /* __LED_H */
