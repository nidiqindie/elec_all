#include "stm32f10x.h"                  // Device header
#include "Encoder.h"

/*引脚配置*/
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


//uint16_t circle_count = 0;   //圈数

//把ENCODER_TIMc初始化为编码器接口模式
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


/*读取编码器速度*/
int Read_L1Speed()
{
	int SpV = 0;
	SpV =  32767 - ENCODER_TIMa->CNT;
	ENCODER_TIMa->CNT = 32767;
	return SpV;
}

int Read_R1Speed()
{
	int SpV = 0;
	SpV =   ENCODER_TIMb->CNT-32767;
	ENCODER_TIMb->CNT = 32767;
	return SpV;
}

int Read_L2Speed()
{
	int SpV = 0;
	SpV = 32767 - ENCODER_TIMc->CNT ;
	ENCODER_TIMc->CNT = 32767;
	return SpV;
}

int Read_R2Speed()
{
	int SpV = 0;
	SpV = ENCODER_TIMd->CNT-32767;
	ENCODER_TIMd->CNT = 32767;
	return SpV;
}
