#include "bsp_key.h"  


//按键标志位，当有按键按下时，对应标志位为1，在main函数电调用后要清零
uint8_t key1_val,key2_val;

 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* 配置中断源：按键 */
  NVIC_InitStructure.NVIC_IRQChannel = KEY1_EXTI_IRQ;
  /* 配置抢占优先级 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  /* 配置子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  /* 使能中断通道 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = KEY2_EXTI_IRQ;
  /* 配置抢占优先级 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  /* 配置子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* 使能中断通道 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//按键按下时，IO口低电平，所使用上拉输入
/**
  * @brief  配置按键用到的I/O口 
  * @param  无
  * @retval 无
  */
void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/*开启按键端口的时钟*/
	RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK|KEY2_GPIO_CLK,ENABLE);
	NVIC_Configuration();
	
/*--------------------------KEY1配置-----------------------------*/
	GPIO_InitStructure.GPIO_Pin = KEY1_GPIO_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
	
	/* 选择EXTI的信号源 */
	GPIO_EXTILineConfig(KEY1_EXTI_PORTSOURCE, KEY1_EXTI_PINSOURCE); 
	EXTI_InitStructure.EXTI_Line = KEY1_EXTI_LINE;

	/* EXTI为中断模式 */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* 上升沿中断 */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	/* 使能中断 */	
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

/*--------------------------KEY2配置-----------------------------*/
	GPIO_InitStructure.GPIO_Pin = KEY2_GPIO_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);	

	/* 选择EXTI的信号源 */
	GPIO_EXTILineConfig(KEY2_EXTI_PORTSOURCE, KEY2_EXTI_PINSOURCE); 
	EXTI_InitStructure.EXTI_Line = KEY2_EXTI_LINE;

	/* EXTI为中断模式 */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* 上升沿中断 */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	/* 使能中断 */	
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

void KEY1_IRQHandler(void)
{

  //确保是否产生了KEY1_EXTI_LINE中断
	if(EXTI_GetITStatus(KEY1_EXTI_LINE) != RESET) 
	{ 
		key1_val = 1;
         //清除中断标志位
		EXTI_ClearITPendingBit(KEY1_EXTI_LINE);     
	}  
}

void KEY2_IRQHandler(void)
{

  //确保是否产生了KEY2_EXTI_LINE中断
	if(EXTI_GetITStatus(KEY2_EXTI_LINE) != RESET) 
	{ 
		key2_val = 1;
         //清除中断标志位
		EXTI_ClearITPendingBit(KEY2_EXTI_LINE);     
	}  	
}


