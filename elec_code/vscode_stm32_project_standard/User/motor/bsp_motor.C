#include "bsp_motor.h"
#include "bsp_usart.h"
#include "bsp_led.h" 
#include <stdlib.h>
#include <math.h>


#include "Encoder.h"
int mpu=0,lflag;
int C_target_velocity = 0; //设定速度控制的目标速度为40个脉冲每10ms //正值表示正转，负值表示反转
int C_encoder = 1000;        //编码器的脉冲计数1000
int C_moto = 0;           //电机PWM变量
int D_target_velocity = 0; //设定速度控制的目标速度为40个脉冲每10ms //正值表示正转，负值表示反转
int D_encoder = 1000;        //编码器的脉冲计数1000
int D_moto = 0;   
int A_target_velocity = 0; //设定速度控制的目标速度为40个脉冲每10ms //正值表示正转，负值表示反转
int A_encoder = 1000;        //编码器的脉冲计数1000
int A_moto = 0;           //电机PWM变量
int B_target_velocity = 0; //设定速度控制的目标速度为40个脉冲每10ms //正值表示正转，负值表示反转
int B_encoder = 1000;        //编码器的脉冲计数1000
int B_moto = 0; 
//pid调试参数区  
float Kp=5,Ki=0.03,kpp=0.05,kip=0.05,kdp=1;
float target_pos=0;
int out0=0,uplimit=30,down=0,up=450;
int PwmC=25,PwmA=25,PwmB=25,PwmD=25; 
#define ENCODER_TIMa			TIM2  //重定义后
#define ENCODER_TIMa_CLK	RCC_APB1Periph_TIM2
#define ENCODER_PORT1_1		GPIOA
#define ENCODER_CLK1_1		RCC_APB2Periph_GPIOA
#define ENCODER_A1				GPIO_Pin_15
#define ENCODER_PORT1_2		GPIOB
#define ENCODER_CLK1_2		RCC_APB2Periph_GPIOB
#define ENCODER_B1				GPIO_Pin_3

#define ENCODER_TIMb			TIM3
#define ENCODER_TIMb_CLK	RCC_APB1Periph_TIM3
#define ENCODER_PORT2			GPIOA
#define ENCODER_CLK2			RCC_APB2Periph_GPIOA
#define ENCODER_A2				GPIO_Pin_6
#define ENCODER_B2				GPIO_Pin_7

#define ENCODER_TIMc			TIM4
#define ENCODER_TIMc_CLK	RCC_APB1Periph_TIM4
#define ENCODER_PORT3			GPIOB
#define ENCODER_CLK3			RCC_APB2Periph_GPIOB
#define ENCODER_A3				GPIO_Pin_6
#define ENCODER_B3				GPIO_Pin_7

#define ENCODER_TIMd			TIM5
#define ENCODER_TIMd_CLK	RCC_APB1Periph_TIM5
#define ENCODER_PORT4			GPIOA
#define ENCODER_CLK4			RCC_APB2Periph_GPIOA
#define ENCODER_A4				GPIO_Pin_0
#define ENCODER_B4				GPIO_Pin_1


int pos_pidr(float yaw)
{
		int PIDOUT;
	static float Ek,Ek_1,Ek_2/*需调试的变量：*/;
	int lowlimit=0;
	Ek=yaw-target_pos;
	PIDOUT=kpp*Ek+kip*(Ek+Ek_1+Ek_2)+kdp*(Ek-Ek_1)+out0;
	Ek_2=Ek_1;
	Ek_1=Ek;
	PIDOUT=PIDOUT;
	PIDOUT=(PIDOUT<lowlimit) ? lowlimit:(PIDOUT>uplimit) ? uplimit : PIDOUT;
	return PIDOUT;
}
int pos_pidl(float yaw)
{
		int PIDOUT;
	static float ek,ek_1,ek_2;
	int lowlimit=0;
	ek=target_pos-yaw;
	PIDOUT=kpp*ek+kip*(ek+ek_1+ek_2)+kdp*(ek-ek_1)+out0;
	ek_2=ek_1;
	ek_1=ek;
	PIDOUT=(PIDOUT<lowlimit) ? lowlimit:(PIDOUT>uplimit) ? uplimit : PIDOUT;
	return PIDOUT;
}



void speed_change_b(int A,int B,int C,int D)
{
	A_moto=A;
	B_moto=B;
	C_moto=C;
	D_moto=D;
	Set_pwm(A_moto,B_moto,C_moto,D_moto);
}

void speed_change(int A,int B,int C,int D)
{
	A_target_velocity=A;
	B_target_velocity=B;
	C_target_velocity=C;
	D_target_velocity=D;
}


void move_forward(uint16_t speed)  //speed指每10ms脉冲个数
{
	A_REVERSE;
	B_REVERSE;
	C_REVERSE;
	D_REVERSE;
	 speed_change(speed, speed, speed, speed);	
}

void move_backward(uint16_t speed)  //speed指每10ms脉冲个数
{
	A_NORMAL;
	B_NORMAL;
	C_NORMAL;
	D_NORMAL;
	 speed_change(speed, speed, speed, speed);	
}

void move_left(uint16_t speed)
{
	A_NORMAL;
	B_REVERSE;
	C_REVERSE;
	D_NORMAL;
	 speed_change(speed, speed, speed, speed);	
}
void move_right(uint16_t speed)
{
	A_REVERSE;
	B_NORMAL;
	C_NORMAL;
	D_REVERSE;
	 speed_change(speed, speed, speed, speed);	

}
void stop(void)  
{	
	A_BRAKE;
	B_BRAKE;
	C_BRAKE;
	D_BRAKE;
}

void Motor_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    AIN_GPIO_CLK_CMD(AIN_GPIO_CLK,ENABLE);

  	GPIO_InitStructure.GPIO_Pin = AIN1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AIN1_GPIO_PORT, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = AIN2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AIN2_GPIO_PORT, &GPIO_InitStructure);

	GPIO_ResetBits(AIN1_GPIO_PORT,AIN1_GPIO_PIN); 
	GPIO_ResetBits(AIN2_GPIO_PORT,AIN2_GPIO_PIN); 
    
    BIN_GPIO_CLK_CMD(BIN_GPIO_CLK,ENABLE);

  	GPIO_InitStructure.GPIO_Pin = BIN1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BIN1_GPIO_PORT, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = BIN2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BIN2_GPIO_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(BIN1_GPIO_PORT,BIN1_GPIO_PIN); 
	GPIO_ResetBits(BIN2_GPIO_PORT,BIN2_GPIO_PIN); 
  
    CIN_GPIO_CLK_CMD(CIN_GPIO_CLK,ENABLE);

  	GPIO_InitStructure.GPIO_Pin = CIN1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CIN1_GPIO_PORT, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = CIN2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CIN2_GPIO_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(CIN1_GPIO_PORT,CIN1_GPIO_PIN); 
	GPIO_ResetBits(CIN2_GPIO_PORT,CIN2_GPIO_PIN); 

    DIN_GPIO_CLK_CMD(DIN_GPIO_CLK,ENABLE);

  	GPIO_InitStructure.GPIO_Pin = DIN1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DIN1_GPIO_PORT, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = DIN2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DIN2_GPIO_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(DIN1_GPIO_PORT,DIN1_GPIO_PIN); 
	GPIO_ResetBits(DIN2_GPIO_PORT,DIN2_GPIO_PIN); 
}

void Speed_pwm_config(void) //TIMER8
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(SPEED_PWM_TIM_GPIO_CLK , ENABLE);	
  GPIO_InitStructure.GPIO_Pin =  SPEED_PWM_TIM_CH1_PIN | SPEED_PWM_TIM_CH2_PIN | SPEED_PWM_TIM_CH3_PIN |SPEED_PWM_TIM_CH4_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPEED_PWM_TIM_PORT, &GPIO_InitStructure);


/* ----------------   PWM信号 周期和占空比的计算--------------- */
// ARR ：自动重装载寄存器的值
// CLK_cnt：计数器的时钟，等于 Fck_int / (psc+1) = 72M/(psc+1)
// PWM 信号的周期 T = (ARR+1) * (1/CLK_cnt) = (ARR+1)*(PSC+1) / 72M
// 占空比P=CCR/(ARR+1) 
  // 开启定时器时钟,即内部时钟CK_INT=72M
	SPEED_PWM_TIM_APBxClock_FUN(SPEED_PWM_TIM_CLK,ENABLE);

/*--------------------时基结构体初始化-------------------------*/
	// 配置周期，这里配置为50Hz
	
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Period=SPEED_PWM_TIM_Period-1;	
	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler= SPEED_PWM_TIM_Period-1;	
	// 时钟分频因子 ，配置死区时间时需要用到
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	// 计数器计数模式，设置为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	// 重复计数器的值，没用到不用管
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;			
	// 初始化定时器
	TIM_TimeBaseInit(SPEED_PWM_TIM, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	// 配置为PWM模式2
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	// 输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                          
	// 输出通道电平极性配置	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	// // 输出比较通道 1
	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OC1Init(SPEED_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(SPEED_PWM_TIM, TIM_OCPreload_Enable);

	// // 输出比较通道 2
	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OC2Init(SPEED_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(SPEED_PWM_TIM, TIM_OCPreload_Enable);

	// // 输出比较通道 3
	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OC3Init(SPEED_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(SPEED_PWM_TIM, TIM_OCPreload_Enable);

	// // 输出比较通道 4
	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OC4Init(SPEED_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(SPEED_PWM_TIM, TIM_OCPreload_Enable);

	// 使能计数器
	TIM_Cmd(SPEED_PWM_TIM, ENABLE);
	//主输出使能，当使用通用定时器时，此句不需要
	TIM_CtrlPWMOutputs(SPEED_PWM_TIM,ENABLE);
}

void Set_speed(uint16_t CCR1_Val, uint16_t CCR2_Val, uint16_t CCR3_Val, uint16_t CCR4_Val)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	// 配置为PWM模式2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	// 输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                          
	// 输出通道电平极性配置	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	// // 输出比较通道 1
	TIM_OCInitStructure.TIM_Pulse = 500-CCR1_Val;
	TIM_OC1Init(SPEED_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(SPEED_PWM_TIM, TIM_OCPreload_Enable);

	// // 输出比较通道 2
	TIM_OCInitStructure.TIM_Pulse = 500-CCR2_Val;
	TIM_OC2Init(SPEED_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(SPEED_PWM_TIM, TIM_OCPreload_Enable);

	// // 输出比较通道 3
	TIM_OCInitStructure.TIM_Pulse = 500-CCR3_Val;
	TIM_OC3Init(SPEED_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(SPEED_PWM_TIM, TIM_OCPreload_Enable);

	// // 输出比较通道 4
	TIM_OCInitStructure.TIM_Pulse = 500-CCR4_Val;
	TIM_OC4Init(SPEED_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(SPEED_PWM_TIM, TIM_OCPreload_Enable);

}
/*********************************END OF PWM**********************/

void Encoder_Init_TIM2(void)
{
	RCC_APB2PeriphClockCmd(ENCODER_CLK1_1 | ENCODER_CLK1_2,ENABLE);  //使能GPIO
	RCC_APB1PeriphClockCmd(ENCODER_TIMa_CLK,ENABLE);  //使能ENCODER_TIMb
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //失能JTAG
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);  //TIM2完全重映射
	
	//初始化IO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_InitStructure.GPIO_Pin = ENCODER_A1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_PORT1_1,&GPIO_InitStructure);
	
	GPIO_InitTypeDef GPIO_InitStructure2;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_InitStructure2.GPIO_Pin = ENCODER_B1;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_PORT1_2,&GPIO_InitStructure2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 65535; //设置自动重装载周期值
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置预分频值 不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(ENCODER_TIMa, &TIM_TimeBaseStructure); //根据指定的参数初始化ENCODER_TIMb
	
	//配置为编码器模式，计数器在TI1和TI2上升沿处均计数
	TIM_EncoderInterfaceConfig(ENCODER_TIMa, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); 
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1|TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	//边沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(ENCODER_TIMa, &TIM_ICInitStructure);
	
	//溢出中断设置
//	TIM_ITConfig(ENCODER_TIMa,TIM_IT_Update,ENABLE);  //允许ENCODER_TIMa溢出中断
		
	//中断优先级NVIC设置
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = TIM8_IRQn;  //ENCODER_TIMa中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级2级
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
	TIM_ClearFlag(ENCODER_TIMa, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_SetCounter(ENCODER_TIMa,32767);   //ENCODER_TIMb->CNT = 0x7fff（32767  65535的一半，可判断速度的正负）
	TIM_Cmd(ENCODER_TIMa, ENABLE);   //启动ENCODER_TIMb定时器
}

void Encoder_Init_TIM3(void)
{
	RCC_APB2PeriphClockCmd(ENCODER_CLK2,ENABLE);  //使能GPIO
	RCC_APB1PeriphClockCmd(ENCODER_TIMb_CLK,ENABLE);  //使能ENCODER_TIMb
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //使能AFIO
	
	//初始化IO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_InitStructure.GPIO_Pin = ENCODER_A2 | ENCODER_B2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_PORT2,&GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 65535; //设置自动重装载周期值
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置预分频值 不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(ENCODER_TIMb, &TIM_TimeBaseStructure); //根据指定的参数初始化ENCODER_TIMb
	
	//配置为编码器模式，计数器在TI1和TI2上升沿处均计数
	TIM_EncoderInterfaceConfig(ENCODER_TIMb, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); 
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1|TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	//边沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(ENCODER_TIMb, &TIM_ICInitStructure);
	
	TIM_ClearFlag(ENCODER_TIMb, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_SetCounter(ENCODER_TIMb,32767);   //ENCODER_TIMb->CNT = 0x7fff（32767  65535的一半，可判断速度的正负）
	TIM_Cmd(ENCODER_TIMb, ENABLE);   //启动ENCODER_TIMb定时器
}

void Encoder_Init_TIM4(void)
{
	RCC_APB2PeriphClockCmd(ENCODER_CLK3,ENABLE);  //使能GPIO
	RCC_APB1PeriphClockCmd(ENCODER_TIMc_CLK,ENABLE);  //使能ENCODER_TIMb
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //使能AFIO
	
	//初始化IO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_InitStructure.GPIO_Pin = ENCODER_A3 | ENCODER_B3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_PORT3,&GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 65535; //设置自动重装载周期值
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置预分频值 不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(ENCODER_TIMc, &TIM_TimeBaseStructure); //根据指定的参数初始化ENCODER_TIMb
	
	//配置为编码器模式，计数器在TI1和TI2上升沿处均计数
	TIM_EncoderInterfaceConfig(ENCODER_TIMc, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); 
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1|TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	//边沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(ENCODER_TIMc, &TIM_ICInitStructure);
	
	TIM_ClearFlag(ENCODER_TIMc, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_SetCounter(ENCODER_TIMc,32767);   //ENCODER_TIMb->CNT = 0x7fff（32767  65535的一半，可判断速度的正负）
	TIM_Cmd(ENCODER_TIMc, ENABLE);   //启动ENCODER_TIMb定时器
}

void Encoder_Init_TIM5(void)
{
	RCC_APB2PeriphClockCmd(ENCODER_CLK4,ENABLE);  //使能GPIO
	RCC_APB1PeriphClockCmd(ENCODER_TIMd_CLK,ENABLE);  //使能ENCODER_TIMb
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //使能AFIO
	
	//初始化IO口
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_InitStructure.GPIO_Pin = ENCODER_A4 | ENCODER_B4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_PORT4,&GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 65535; //设置自动重装载周期值
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置预分频值 不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(ENCODER_TIMd, &TIM_TimeBaseStructure); //根据指定的参数初始化ENCODER_TIMb
	
	//配置为编码器模式，计数器在TI1和TI2上升沿处均计数
	TIM_EncoderInterfaceConfig(ENCODER_TIMd, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); 
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1|TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	//边沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(ENCODER_TIMd, &TIM_ICInitStructure);
	
	TIM_ClearFlag(ENCODER_TIMd, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_SetCounter(ENCODER_TIMd,32767);   //ENCODER_TIMb->CNT = 0x7fff（32767  65535的一半，可判断速度的正负）
	TIM_Cmd(ENCODER_TIMd, ENABLE);   //启动ENCODER_TIMb定时器
}
//中断处理函数为空，清除中断标志位后结束中断
/*读取编码器速度*/
int Read_R2Speed()
{
	int SpV = 0;
	SpV =  ENCODER_TIMa->CNT - 32767;
	ENCODER_TIMa->CNT = 32767;
	return SpV;
}

int Read_L2Speed()
{
	int SpV = 0;
	SpV =  32767 - ENCODER_TIMb->CNT;
	ENCODER_TIMb->CNT = 32767;
	return SpV;
}

int Read_R1Speed()
{
	int SpV = 0;
	SpV =  ENCODER_TIMc->CNT - 32767;
	ENCODER_TIMc->CNT = 32767;
	return SpV;
}

int Read_L1Speed()
{
	int SpV = 0;
	SpV = 32767 - ENCODER_TIMd->CNT;
	ENCODER_TIMd->CNT = 32767;
	return SpV;
}

void ENCODER1_IRQ_IRQHandler(void)
{
 if(TIM_GetFlagStatus(ENCODER1_TIM,TIM_FLAG_Update)==SET)//溢出中断
 {
	 TIM_ClearITPendingBit(ENCODER1_TIM,TIM_IT_Update);  //清除中断标志位
 }
}

//中断处理函数为空，清除中断标志位后结束中断
void ENCODER2_IRQ_IRQHandler(void)
{
 if(TIM_GetFlagStatus(ENCODER2_TIM,TIM_FLAG_Update)==SET)//溢出中断
 {
	 TIM_ClearITPendingBit(ENCODER2_TIM,TIM_IT_Update);  //清除中断标志位
 }
}
//中断处理函数为空，清除中断标志位后结束中断
void ENCODER3_IRQ_IRQHandler(void)
{
 if(TIM_GetFlagStatus(ENCODER3_TIM,TIM_FLAG_Update)==SET)//溢出中断
 {
	 TIM_ClearITPendingBit(ENCODER3_TIM,TIM_IT_Update);  //清除中断标志位
 }
}
//中断处理函数为空，清除中断标志位后结束中断
void ENCODER4_IRQ_IRQHandler(void)
{
 if(TIM_GetFlagStatus(ENCODER4_TIM,TIM_FLAG_Update)==SET)//溢出中断
 {
	 TIM_ClearITPendingBit(ENCODER4_TIM,TIM_IT_Update);  //清除中断标志位
 }
}


//读取计数器Timer4的值
int Read_encoder(uint8_t TIMX)//读取计数器的值
{
  int Encoder_TIM;
	switch(TIMX)
	{
		case 2:Encoder_TIM=(short)(TIM2->CNT-1000); TIM2 -> CNT=1000;  break;
	  case 3:Encoder_TIM=(short)(TIM3->CNT-1000); TIM3 -> CNT=1000;  break;
	  case 4:Encoder_TIM=(short)(TIM4->CNT-1000); TIM4 -> CNT=1000;  break;
	  case 5:Encoder_TIM=(short)(TIM5->CNT-1000); TIM5 -> CNT=1000;  break;
	  default: Encoder_TIM=1000;
	  break;
	}
  return Encoder_TIM;
}

void PID_timebase_init(void) //Timer6
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
    PID_TIMEBASE_APBxClock_FUN(PID_TIMEBASE_CLK,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_Period = PID_TIMEBASE_Period;     //重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler = PID_TIMEBASE_Prescaler;  //预分频系数
	TIM_TimeBaseInitStruct.TIM_ClockDivision =0; //时钟分割
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(PID_TIMEBASE_TIM,&TIM_TimeBaseInitStruct);
	
	TIM_ITConfig(PID_TIMEBASE_TIM,TIM_IT_Update,ENABLE);  //使能定时器中断
	
	NVIC_InitStruct.NVIC_IRQChannel = PID_TIMEBASE_IRQn;   //使能外部中断通道
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;   //使能外部中断通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级1
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;    //响应优先级1
	NVIC_Init(&NVIC_InitStruct);

	TIM_Cmd(PID_TIMEBASE_TIM,ENABLE);	  //使能定时器3
}

int PID_TIMEBASE_IRQ_IRQHandler(void)
{
 if(TIM_GetFlagStatus(PID_TIMEBASE_TIM,TIM_FLAG_Update)==SET)
 {

	TIM_ClearITPendingBit(PID_TIMEBASE_TIM,TIM_IT_Update);       //===清除定时器6中断标志位
	mpu++;
	lflag++;
	C_encoder=Read_encoder(3);                         //+表示正转计编码，-表示反转计编码
	D_encoder=Read_encoder(5);
	A_encoder=Read_encoder(2);                         //+表示正转计编码，-表示反转计编码
	B_encoder=Read_encoder(4); 
	// printf("_____%d\n",A_encoder);
	// printf("#%d\n",B_encoder);	
	// printf("#%d\n",C_encoder);
	// printf("#%d\n",D_encoder);	
	if (lflag==2)
	{
		lflag=0;
		flag_runpid=1;
	}
	
    // if(mpu==20)
	// {
	// C_moto+=pos_pidr(car.yaw);    //===速度PID控制器
	// D_moto+=pos_pidr(car.yaw);    //===速度PID控制器
	// A_moto+=pos_pidl(car.yaw);    //===速度PID控制器
	// B_moto+=pos_pidl(car.yaw);    //===速度PID控制器
	// // //  printf("the_left%d,the_right%d\n",pos_pidl(car.yaw),pos_pidr(car.yaw));
	//  if(mpu==20)
	// mpu=0;
	// }
	// else
	// {// printf("the_second:%d\n",pos_pidl(Yaw));
	C_moto=Incremental_PIC(C_encoder,C_target_velocity);    //===速度PID控制器
	D_moto=Incremental_PID(D_encoder,D_target_velocity);    //===速度PID控制器
	A_moto=Incremental_PIA(A_encoder,A_target_velocity);    //===速度PID控制器
	B_moto=Incremental_PIB(B_encoder,B_target_velocity);
	// }
		
	Set_speed(A_moto, B_moto, C_moto, D_moto);
  LED_YELLOW;
 }
 return 0;
}

/**************************************************************************
函数功能：赋值给PWM
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_pwm(int A_moto,int B_moto,int C_moto, int D_moto)
{
		if(A_moto >= 0) {A_REVERSE;}
	else {
		A_NORMAL; 
		A_moto = -A_moto; 
	}
	if(A_moto > 500)     A_moto = 500;


		if(B_moto >= 0) {B_REVERSE;}
	else {
		B_NORMAL; 
		B_moto = -B_moto; 
	}
	if(B_moto > 500)     B_moto = 500;


	if(C_moto >= 0) {C_REVERSE;}
	else {
		C_NORMAL; 
		C_moto = -C_moto; 
	}
	if(C_moto > 500)     C_moto = 500;

	if(D_moto >= 0) {D_REVERSE; }
	else {
		D_NORMAL; 
		D_moto = -D_moto; 
	}
	if(D_moto > 500)     D_moto = 500;

	Set_speed(A_moto, B_moto, C_moto, D_moto);
}



/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/

int Incremental_PIB (int EncoderB,int TargetB)
{ 	
	 static int BiasB,Last_biasB;
	 BiasB=-(EncoderB-TargetB);                //计算偏差
	 PwmB+=Kp*(BiasB-Last_biasB)+Ki*BiasB;   //增量式PI控制器
	 Last_biasB=BiasB;	                 //保存上一次偏差 
	 	PwmB=(PwmB<down) ? down:(PwmB>up) ? up : PwmB;
			//  printf("   %d       \n",PwmB);        

	return PwmB;
               //增量输出
}
int Incremental_PIC (int EncoderC,int TargetC)
{ 	
	 static int BiasC,Last_biasC;
	 BiasC=-(EncoderC-TargetC);                //计算偏差
	 PwmC+=Kp*(BiasC-Last_biasC)+Ki*BiasC;   //增量式PI控制器
	 Last_biasC=BiasC;	                 //保存上一次偏差 
	 	PwmC=(PwmC<down) ? down:(PwmC>up) ? up : PwmC;
			//  printf("   %d       \n",PwmC);

	return PwmC;
}
int Incremental_PID (int EncoderD,int TargetD)
{ 	
	 static int BiasD,Last_biasD;
	 BiasD=-(EncoderD-TargetD);                //计算偏差
	 PwmD+=Kp*(BiasD-Last_biasD)+Ki*BiasD;   //增量式PI控制器
	 Last_biasD=BiasD;	                 //保存上一次偏差 
	 	PwmD=(PwmD<down) ? down:(PwmD>up) ? up : PwmD;
			//  printf("   %d       \n",PwmD);

	return PwmD;
}


int Incremental_PIA (int EncoderA,int TargetA)
{ 	
	 static int BiasA,Last_biasA;
	 BiasA=-(EncoderA-TargetA);                //计算偏差
	 PwmA+=Kp*(BiasA-Last_biasA)+Ki*BiasA;   //增量式PI控制器
	 Last_biasA=BiasA;	                 //保存上一次偏差 
	 	PwmA=(PwmA<down) ? down:(PwmA>up) ? up : PwmA;
			//  printf("   %d       \n",PwmA);

	return PwmA;
}
