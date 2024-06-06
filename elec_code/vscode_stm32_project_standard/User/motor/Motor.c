/*tb6612电机驱动模块*/

#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "PID.h"
#include "bsp_usart.h"
#include "communication_protocol.h"
#include "bsp_delay.h"
#include "app.h"
#include "Serial.h"
#include "bsp_led.h"
int flag_L=0,flag_R=0;
extern uint16_t DistanceX,DistanceY,Size;
int flag_adjusting=1,flag_cx_complete,flag_cy_complete,flag_cy_start;

float bs(float a,float b)
{
	if(a==b)
return 0;

if(a>b)
return a-b;
else
return b-a;

}

extern float tarSp_L1,tarSp_L2,tarSp_R1,tarSp_R2;
void Motor_Init()
{
	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  //注意高级定时器是APB2
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //使能端口复用时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);  //失能JTAG(使PB4作普通引脚使用
	
	//tb6612方向引脚初始化
	GPIO_InitTypeDef GPIO_InitStructure1;
	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure1);
	
	GPIO_InitTypeDef GPIO_InitStructure2;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure2);
	
	GPIO_InitTypeDef GPIO_InitStructure3;
	GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure3);
	
	//tb6612PWM输出引脚初始化
	GPIO_InitTypeDef GPIO_InitStructure4;
	GPIO_InitStructure4.GPIO_Mode = GPIO_Mode_AF_PP;   //复用推挽输出
	GPIO_InitStructure4.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure4.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure4);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_Period = 1000;   //设置重装载值ARR（周期）
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);
 
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //设置计数模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM8,&TIM_OCInitStructure);
	TIM_OC2Init(TIM8,&TIM_OCInitStructure);
	TIM_OC3Init(TIM8,&TIM_OCInitStructure);
	TIM_OC4Init(TIM8,&TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
 
	TIM_Cmd(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM8,ENABLE);        //MOE 主输出使能,高级定时器必须开启这个
}

void Set_L1Speed(int16_t LSpeed)  //左轮1速度（满速1000）
{
	if(LSpeed>0)  //正转
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_5);
		GPIO_ResetBits(GPIOB,GPIO_Pin_4);
		TIM_SetCompare1(TIM8,LSpeed);   //设置速度大小（PWM1占空比 CCRx）
	}
	else if(LSpeed<0)  //反转
	{
		LSpeed = -LSpeed;
				GPIO_ResetBits(GPIOB,GPIO_Pin_5);
		GPIO_SetBits(GPIOB,GPIO_Pin_4);

		TIM_SetCompare1(TIM8,LSpeed);
	}
	else  //停止
	{
		TIM_SetCompare1(TIM8,0);
	}
}

void Set_L2Speed(int16_t LSpeed)  //左轮2速度（满速1000）
{
	if(LSpeed>0)  //正转
	{
				GPIO_SetBits(GPIOB,GPIO_Pin_8);
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);

		TIM_SetCompare2(TIM8,LSpeed);   //设置速度大小（PWM1占空比 CCRx）
	}
	else if(LSpeed<0)  //反转
	{
		LSpeed = -LSpeed;
				GPIO_ResetBits(GPIOB,GPIO_Pin_8);
		GPIO_SetBits(GPIOB,GPIO_Pin_9);

		TIM_SetCompare2(TIM8,LSpeed);
	}
	else  //停止
	{
		TIM_SetCompare2(TIM8,0);
	}
}

void Set_R1Speed(int16_t RSpeed)  //右轮1速度（满速1000）
{
	if(RSpeed>0)  //正转
	{
				GPIO_SetBits(GPIOA,GPIO_Pin_5);
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);

		TIM_SetCompare3(TIM8,RSpeed);   //设置速度大小（PWM1占空比 CCRx）
	}
	else if(RSpeed<0)  //反转
	{
		RSpeed = -RSpeed;
				GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		GPIO_SetBits(GPIOA,GPIO_Pin_4);

		TIM_SetCompare3(TIM8,RSpeed);
	}
	else  //停止
	{
		TIM_SetCompare3(TIM8,0);
	}
}

void Set_R2Speed(int16_t RSpeed)  //右轮2速度（满速1000）
{
	if(RSpeed>0)  //正转
	{
				GPIO_SetBits(GPIOC,GPIO_Pin_2);
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);

		TIM_SetCompare4(TIM8,RSpeed);   //设置速度大小（PWM1占空比 CCRx）
	}
	else if(RSpeed<0)  //反转
	{
		RSpeed = -RSpeed;
				GPIO_ResetBits(GPIOC,GPIO_Pin_2);
		GPIO_SetBits(GPIOC,GPIO_Pin_3);

		TIM_SetCompare4(TIM8,RSpeed);
	}
	else  //停止
	{
		TIM_SetCompare4(TIM8,0);
	}
}

/*基础运动函数*/
void Forward(float Speed)  //前进
{
	tarSp_L1 = Speed;
	tarSp_L2 = Speed;
	tarSp_R1 = Speed;
	tarSp_R2 = Speed;
}

void Backward(float Speed)  //后退
{
	tarSp_L1 = -Speed;
	tarSp_L2 = -Speed;
	tarSp_R1 = -Speed;
	tarSp_R2 = -Speed;
}

void L_Translation(float Speed)  //左平移
{
	tarSp_L1 = -Speed;
	tarSp_L2 = Speed;
	tarSp_R1 = Speed;
	tarSp_R2 = -Speed;
}

void R_Translation(float Speed)  //右平移
{
	tarSp_L1 = Speed;
	tarSp_L2 = -Speed;
	tarSp_R1 = -Speed;
	tarSp_R2 = Speed;
}

void L_Rotation()  //左自转
{
	if(tarAngle==0)
	{
flag_pos=0;
Set_L1Speed(-300);
Set_L2Speed(-300);
Set_R1Speed(300);
Set_R2Speed(300);
tarAngle=-90;
Delay_ms(1050);
flag_L=1;
	}
		if(tarAngle==90)
	{
flag_pos=0;
Set_L1Speed(-300);
Set_L2Speed(-300);
Set_R1Speed(300);
Set_R2Speed(300);
tarAngle=0;
Delay_ms(1050);
flag_L=1;
	}


}

void R_Rotation()  //右自转
{
	if(tarAngle==0)
{
	flag_pos=0;
Set_L1Speed(300);
Set_L2Speed(300);
Set_R1Speed(-300);
Set_R2Speed(-300);
tarAngle=90;
Delay_ms(1050);
flag_R=1;
}
if(tarAngle==-90)
{
	flag_pos=0;
Set_L1Speed(300);
Set_L2Speed(300);
Set_R1Speed(-300);
Set_R2Speed(-300);
tarAngle=0;
Delay_ms(1050);
flag_R=1;
}
}
void Stop(void)  //停止
{
	Forward(0);
		GPIO_SetBits(GPIOC,GPIO_Pin_2);
		GPIO_SetBits(GPIOC,GPIO_Pin_3);
		GPIO_SetBits(GPIOA,GPIO_Pin_5);
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
		GPIO_SetBits(GPIOB,GPIO_Pin_8);
		GPIO_SetBits(GPIOB,GPIO_Pin_9);
		GPIO_SetBits(GPIOB,GPIO_Pin_5);
		GPIO_SetBits(GPIOB,GPIO_Pin_4);

}
void weitiao(void)
{
	switch(flag_weitiao)
	{
		case 1:
	if(flag_adjusting<=2&&flag_adjusting!=0)
    {
        if(flag_cx_complete==0&&DistanceX!=0)
        {
            if(DistanceX<150) Backward(1);
            else if(DistanceX>170) Forward(1);
            else  {Stop();flag_cx_complete=1;flag_cy_complete=0;flag_cy_start=1;}
        }
        if(flag_cy_start==1&&flag_cy_complete==0&&DistanceY!=0)
        {
            if(DistanceY<110) R_Translation(1);
            else if(DistanceY>130) L_Translation(1);
            else  {Stop();flag_cy_complete=1;flag_adjusting++;flag_cx_complete=0;}
        }
    }
	
	a=A_j;
	b=B_j;
	c=C_j;
	d=D_j;

	break;


	case 2:
		if(flag_adjusting<=2&&flag_adjusting!=0)
    {
        if(flag_cx_complete==0&&DistanceX!=0)
        {
            if(DistanceX<160) L_Translation(1);
            else if(DistanceX>160) R_Translation(1);
            else  {Stop();flag_cx_complete=1;flag_cy_complete=0;flag_cy_start=1;}
        }
        if(flag_cy_start==1&&flag_cy_complete==0&&DistanceY!=0)
        {
            if(DistanceY<120) Backward(1);
            else if(DistanceY>120) Forward(1);
            else  {Stop();flag_cy_complete=1;flag_adjusting++;flag_cx_complete=0;}
        }
    }
		a=A_j;
	b=B_j;
	c=C_j;
	d=D_j;

break;


case 3:


	if(flag_adjusting<=2&&flag_adjusting!=0)
    {
        if(flag_cx_complete==0&&DistanceX!=0)
        {
            if(DistanceX<160) Forward(1);
            else if(DistanceX>160) Backward(1);
            else  {Stop();flag_cx_complete=1;flag_cy_complete=0;flag_cy_start=1;}
        }
        if(flag_cy_start==1&&flag_cy_complete==0&&DistanceY!=0)
        {
            if(DistanceY<120) L_Translation(1);
            else if(DistanceY>120) R_Translation(1);
            else  {Stop();flag_cy_complete=1;flag_adjusting++;flag_cx_complete=0;}
        }
    }
	a=A_j;
	b=B_j;
	c=C_j;
	d=D_j;

	break;


	}
}
