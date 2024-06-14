#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#include "Serial.h"


void USART3_init(void)
{
	USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART3��GPIOBʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��USART3��GPIOBʱ��
    USART_DeInit(USART3);  //��λ����3
    //USART3_TX   PB10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��Pb10
    //USART3_RX	  PB11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);  //��ʼ��PB11
 
    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
 
    //USART ��ʼ������
    USART_InitStructure.USART_BaudRate = 115200;//һ������Ϊ115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART3, &USART_InitStructure); //��ʼ������
 
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//����USART3�����ж�
    USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���
}


void USART3_IRQHandler(void)
{
	uint16_t i=0;
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;	

	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{	

		LED_RED;
		uint8_t RxData = USART_ReceiveData(USART3);
		
		if (RxState == 0)
		{
			if (RxData == '@' && Serial_RxFlag == 0)
			{	
//				OLED_ShowNum(1, 9,01,2);
				
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			//OLED_ShowNum(2,16,9,1);				
			if (RxData == '@'&& pRxPacket==12)
			{
				RxState = 2;
			}
			else
			{
				Serial_RxPacket[pRxPacket] = RxData;
				pRxPacket ++;//��??������???��???������pRxPacket=12
//				OLED_ShowNum(2,2,Serial_RxPacket[0],1);
				if(pRxPacket==12)
				{
//			OLED_ShowNum(1,16,9,1);				
				}
			}
		}
		else if (RxState == 2)
		{
			//OLED_ShowNum(1,13,02,2);
			
			if (RxData == '@')
			{ 
				zhimu =Serial_RxPacket[0];//��?
				shuzi=Serial_RxPacket[1];
				X1=Serial_RxPacket[2];
				X2=Serial_RxPacket[3];//?��3?
				X3=Serial_RxPacket[4];
				Y1=Serial_RxPacket[5];
				Y2=Serial_RxPacket[6];
				Y3=Serial_RxPacket[7];
//				Serial_RxPacket[pRxPacket] = '\0';
				// printf("%c",X1);
				// printf("    Y%c      ",Y1);
				DistanceX=(X1-48)*100+(X2-48)*10+(X3-48);
				DistanceY=(Y1-48)*100+(Y2-48)*10+(Y3-48);
				RxState = 0;
				//printf("%d",DistanceX);
				//printf("%d",DistanceY);

				//printf("%c",K);
//				for(i=0;i<11;i++)
//					{
//						Serial_RxPacket[i]=0x00;  
//					}
						Serial_RxFlag = 0;
			}
		}
	switch (zhimu)
	{
	case 'A':
		switch (shuzi)
			{
			case '1':
				position=2;
				break;
			case '2':
				position=3;
				break;
			case '3':
				position=4;
				break;
			case '4':
				position=5.1;
				break;
			default:
				break;
			}
	case 'B':
		switch (shuzi)
			{
			case '1':
				position=14;
				break;
			case '2':
				position=3;
				break;
			case '3':
				position=4;
				break;
			case '4':
				position=5.2;
				break;
			default:
				break;
			}
	case 'C':
		switch (shuzi)
			{
			case '1':
				position=11;
				break;
			case '2':
				position=10;
				break;
			case '3':
				position=7;
				break;
			case '4':
				position=8.1;
				break;
			default:
				break;
			}
	case 'D':
		switch (shuzi)
			{
			case '1':
				position=11.1;
				break;
			case '2':
				position=10.1;
				break;
			case '3':
				position=8;
				break;
			case '4':
				position=8.1;
				break;
			default:
				break;
			}
	default:
		break;
	}
		
		
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}

