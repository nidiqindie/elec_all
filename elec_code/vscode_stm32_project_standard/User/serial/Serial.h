#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"
#include <stdio.h>
char Serial_RxPacket[12];//"@MSG\r\n"
uint8_t Serial_RxFlag;
uint8_t zhimu,shuzi,X1, X2, X3, Y1, Y2, Y3;
float position;  
uint16_t DistanceX,DistanceY,Size;
uint16_t task[6];


void USART3_init(void);

#endif
