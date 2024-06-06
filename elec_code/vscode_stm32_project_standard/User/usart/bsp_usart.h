#ifndef __USART_H
#define	__USART_H

#include "stm32f10x.h"
#include <stdio.h>


//串口1接数传与飞机主控通信（接发） 		小车发送帧（AA FF 命令位 校验位1 校验位2），小车接收帧（AA FF system_state flag cm0 cm1 cm2 cm3 cm4 cm5 cm6 cm7 校验位1 校验位2）
// 串口2接nano接收识别数据（接收和发送） 	小车发送帧（AA FF 命令位 校验位1 校验位2）， 小车接收帧（AA FF cm0 cm1 cm2 cm3 校验码1 校验码2）
// 串口3接串口屏（接发）					小车发送数据（无人机位置，累计航程，火情位置），小车接收数据（命令帧）
// 串口4接飞控得到偏航数据（只接收） 	 	小车接收帧（AA FF 功能吗 数据长度 cm0 cm1 cm2 cm3 cm4 cm5 cm6 cm7 校验码1 校验码2）

// #define Enable_IRQ_UART4_RX        1

extern uint8_t fc_recv_buff[];
extern uint8_t nano_recv_buff[];
extern uint8_t yaw_recv_buff[];

extern uint8_t flag_seq_screen;

#define  fc_recv_buff_size               14
#define  nano_recv_buff_size             8
#define  yaw_recv_buff_size              14

#define  fc_send_buff_size               5
#define  nano_send_buff_size             5

// USART1
#define  DEBUG_USART1                    USART1
#define  DEBUG_USART1_CLK                RCC_APB2Periph_USART1
#define  DEBUG_USART1_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  DEBUG_USART1_BAUDRATE           9600

#define  DEBUG_USART1_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DEBUG_USART1_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_USART1_TX_GPIO_PORT       GPIOA   
#define  DEBUG_USART1_TX_GPIO_PIN        GPIO_Pin_9
#define  DEBUG_USART1_RX_GPIO_PORT       GPIOA
#define  DEBUG_USART1_RX_GPIO_PIN        GPIO_Pin_10

#define  DEBUG_USART1_IRQ                USART1_IRQn
#define  DEBUG_USART1_IRQHandler         USART1_IRQHandler

// USART2
#define  DEBUG_USART2                    USART2
#define  DEBUG_USART2_CLK                RCC_APB1Periph_USART2
#define  DEBUG_USART2_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  DEBUG_USART2_BAUDRATE           9600

#define  DEBUG_USART2_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DEBUG_USART2_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
   
#define  DEBUG_USART2_TX_GPIO_PORT       GPIOA   
#define  DEBUG_USART2_TX_GPIO_PIN        GPIO_Pin_2
#define  DEBUG_USART2_RX_GPIO_PORT       GPIOA
#define  DEBUG_USART2_RX_GPIO_PIN        GPIO_Pin_3

#define  DEBUG_USART2_IRQ                USART2_IRQn
#define  DEBUG_USART2_IRQHandler         USART2_IRQHandler

// USART3
#define  DEBUG_USART3                    USART3
#define  DEBUG_USART3_CLK                RCC_APB1Periph_USART3
#define  DEBUG_USART3_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  DEBUG_USART3_BAUDRATE           115200

#define  DEBUG_USART3_GPIO_CLK           (RCC_APB2Periph_GPIOB)
#define  DEBUG_USART3_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
   
#define  DEBUG_USART3_TX_GPIO_PORT       GPIOB   
#define  DEBUG_USART3_TX_GPIO_PIN        GPIO_Pin_10
#define  DEBUG_USART3_RX_GPIO_PORT       GPIOB
#define  DEBUG_USART3_RX_GPIO_PIN        GPIO_Pin_11

#define  DEBUG_USART3_IRQ                USART3_IRQn
#define  DEBUG_USART3_IRQHandler         USART3_IRQHandler

// UART4
#define  DEBUG_UART4                    UART4
#define  DEBUG_UART4_CLK                RCC_APB1Periph_UART4
#define  DEBUG_UART4_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  DEBUG_UART4_BAUDRATE           9600 
    
#define  DEBUG_UART4_GPIO_CLK           (RCC_APB2Periph_GPIOC)
#define  DEBUG_UART4_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
   
#define  DEBUG_UART4_TX_GPIO_PORT       GPIOC   
#define  DEBUG_UART4_TX_GPIO_PIN        GPIO_Pin_10
#define  DEBUG_UART4_RX_GPIO_PORT       GPIOC
#define  DEBUG_UART4_RX_GPIO_PIN        GPIO_Pin_11

#define  DEBUG_UART4_IRQ                UART4_IRQn
#define  DEBUG_UART4_IRQHandler         UART4_IRQHandler

// UART5
#define  DEBUG_UART5                    UART5
#define  DEBUG_UART5_CLK                RCC_APB1Periph_UART5
#define  DEBUG_UART5_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  DEBUG_UART5_BAUDRATE           115200

#define  DEBUG_UART5_GPIO_CLK1           (RCC_APB2Periph_GPIOC)
#define  DEBUG_UART5_GPIO_CLK2           (RCC_APB2Periph_GPIOD)

#define  DEBUG_UART5_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
   
#define  DEBUG_UART5_TX_GPIO_PORT       GPIOC   
#define  DEBUG_UART5_TX_GPIO_PIN        GPIO_Pin_12
#define  DEBUG_UART5_RX_GPIO_PORT       GPIOD
#define  DEBUG_UART5_RX_GPIO_PIN        GPIO_Pin_2

#define  DEBUG_UART5_IRQ                UART5_IRQn
#define  DEBUG_UART5_IRQHandler         UART5_IRQHandler

void USART1_Config(void);
void USART2_Config(void);
void Usart4Init(int uibaud);
void USART3_Config(void);
void UART4_Config(void);
void Usart4Init(int baud);
void UART5_Config(void);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);
void Usart_SendArray(USART_TypeDef *pUSARTx, uint8_t *array, uint16_t num);


void Uart2Send(unsigned char *p_data, unsigned int uiSize);

#endif /* __USART_H */
