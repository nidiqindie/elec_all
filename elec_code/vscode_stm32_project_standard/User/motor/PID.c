#include "stm32f10x.h"                  // Device header
#include "PID.h"

#include "Motor.h"
#include "Encoder.h"

#include "OLED.h"

//编码值对应比例136对应满速1000pwm；
/*偏航角PID参数定义*/
int mpu=0,lflag=0;
float curAngle = 0;  //当前偏航角角度
float tarAngle = 0;  //目标偏航角角度

float curErr = 0;  //角度偏差
float lastErr = 0;  //上次角度偏差
float iErr = 0;  //累计误差
float pidOut_L = 0,pidOut_R = 0;  //PID计算输出值

float Angle_Kp = 30;  //增量式微分系数
float Angle_Ki = 1.5;  //增量式比例系数
/******************/

/*速度PID参数定义*/
float curSp_L1,curSp_L2,curSp_R1,curSp_R2 = 0;  //当前速度
float tarSp_L1 = 0;  //目标速度 周期20ms 减速比30 11线电机 速度范围：40~230
float tarSp_L2 = 0;
float tarSp_R1 = 0;
float tarSp_R2 = 0;

float curErr_L1=0,curErr_L2=0,curErr_R1=0,curErr_R2 = 0;  //速度偏差
float lastErr_L1=0,lastErr_L2=0,lastErr_R1=0,lastErr_R2 = 0;  //上次速度偏差
float iErr_L1=0,iErr_L2=0,iErr_R1=0,iErr_R2 = 0;  //累计误差
float pidOut_L1=0,pidOut_L2=0,pidOut_R1=0,pidOut_R2 = 0;  //PID计算输出的速度

float Speed_Kp = 5;  //增量式微分系数
float Speed_Ki = 1.5;  //增量式比例系数
/******************/

float curL1_value,curL2_value,curR1_value,curR2_value;  //当前各轮行进积累值
 int flag_pos=0;

int flag_runpid = 0;
int A_j=0,B_j=0,C_j=0,D_j=0;

// int bs(float a,float b)
// {
// 	if(a==b)
// return 0;

// if(a>b)
// return (int)(a-b);
// else
// return (int)(b-a);

// }
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


/*偏航角增量式PID计算*/
float IncPIDCalcR()
{
	float incPidout;
	curErr =  curAngle-tarAngle;  //当前误差
	incPidout = Angle_Kp*(curErr-lastErr) + (Angle_Ki)*curErr;  //增量式PI控制器
	lastErr = curErr;
	
	return incPidout;  //返回增量值
}

float IncPIDCalcL()
{
	float incPidout;
	curErr = tarAngle - curAngle;  //当前误差
	incPidout = Angle_Kp*(curErr-lastErr) + (Angle_Ki)*curErr;  //增量式PI控制器
	lastErr = curErr;
	
	return incPidout;  //返回增量值
}

// void yaw_run(int16_t target_yaw)
// {
	
// 	if((target_yaw >= car.yaw) && (target_yaw-car.yaw) > 2)  
// 	{
// 		if((target_yaw - car.yaw) <= 180) //右转
// 		{
// Set_L1Speed(300);
// Set_L2Speed(300);
// Set_R1Speed(-300);
// Set_R2Speed(-300);
// 		}
// 		else{                //左转
// Set_L1Speed(-300);
// Set_L2Speed(-300);
// Set_R1Speed(300);
// Set_R2Speed(300);
// 		}
// 		while(bs(target_yaw,car.yaw) > 2);
// 		Stop();
// 	}
	
// 	if((car.yaw >= target_yaw) && (car.yaw-target_yaw) > 2) 
// 	{
// 		if((car.yaw - target_yaw) <= 180) //左转
// 		{
// Set_L1Speed(-300);
// Set_L2Speed(-300);
// Set_R1Speed(300);
// Set_R2Speed(300);
// 		}
// 		else{                //右转
// Set_L1Speed(300);
// Set_L2Speed(300);
// Set_R1Speed(-300);
// Set_R2Speed(-300);
// 		}
// 		while(bs(target_yaw,car.yaw) > 2);
// 		Stop();
// 		}
// }
void runPID()
{
	if(flag_runpid==1)
	{
		flag_pos++;
		curAngle = car.yaw;  //读取当前角度
	// printf("%f,%f,%f,%f,%f,%f\n",car.yaw,tarSp_R1,curSp_L1,curSp_L2,curSp_R1,curSp_R2);
		curSp_L1 = Read_L1Speed();  //读取当前速度
		curSp_L2 = Read_L2Speed();
		curSp_R1 = Read_R1Speed();
		curSp_R2 = Read_R2Speed();
		printf("%f,%f\n",tarSp_R2,curSp_R2);//vofa看pwm波形，需串口打印/*,curSp_L2,curSp_R1,curSp_R2*/
		A_j +=curSp_L1;
		B_j +=curSp_L2;
		C_j +=curSp_R1;
		D_j +=curSp_R2;
		curL1_value +=curSp_L1*0.1;
		curL2_value +=curSp_L2*0.1;
		curR1_value +=curSp_R1*0.1;
		curR2_value +=curSp_R2*0.1;

		pidOut_L1 = pidOut_L1 + IncPIDCalc_L1(); //+ IncPIDCalcL();  //增量式PID计算
		pidOut_L2 = pidOut_L2 + IncPIDCalc_L2(); //+ IncPIDCalcL();
		pidOut_R1 = pidOut_R1 + IncPIDCalc_R1(); //+ IncPIDCalcR();
		pidOut_R2 = pidOut_R2 + IncPIDCalc_R2(); //+ IncPIDCalcR();
			if(flag_L==1 ||flag_R==1)
			{
				IncPIDCalcL();
				IncPIDCalcL();
       IncPIDCalcR();
IncPIDCalcR();
flag_L=0;
flag_R=0;
			}
			if(flag_pos==10)
			{
			pidOut_L1 +=IncPIDCalcL();
			pidOut_L2 +=IncPIDCalcL();
			pidOut_R1 +=IncPIDCalcR();
			pidOut_R2 +=IncPIDCalcR();
			flag_pos=0;
			}
	

		if(pidOut_L1>1000) pidOut_L1=1000;  //输出限幅
		if(pidOut_L1<-1000) pidOut_L1=-1000;
		if(pidOut_L2>1000) pidOut_L2=1000;
		if(pidOut_L2<-1000) pidOut_L2=-1000;
		if(pidOut_R1>1000) pidOut_R1=1000;
		if(pidOut_R1<-1000) pidOut_R1=-1000;
		if(pidOut_R2>1000) pidOut_R2=1000;
		if(pidOut_R2<-1000) pidOut_R2=-1000;
		
		Set_L1Speed((int)pidOut_L1);  //设置电机速度
		Set_L2Speed((int)pidOut_L2);
		Set_R1Speed((int)pidOut_R1);
		Set_R2Speed((int)pidOut_R2);
		flag_runpid=0;
	}
}


/*速度增量式PID计算*/
float IncPIDCalc_L1()
{
	float incPidout;
	curErr_L1 = tarSp_L1 - curSp_L1;  //当前误差
	incPidout = Speed_Kp*(curErr_L1-lastErr_L1) + Speed_Ki*curErr_L1;  //增量式PI控制器
	lastErr_L1 = curErr_L1;
	
	return incPidout;  //返回增量值
}
float IncPIDCalc_L2()
{
	int incPidout;
	curErr_L2 = tarSp_L2 - curSp_L2;  //当前误差
	incPidout = Speed_Kp*(curErr_L2-lastErr_L2) + Speed_Ki*curErr_L2;  //增量式PI控制器
	lastErr_L2 = curErr_L2;
	
	return incPidout;  //返回增量值
}
float IncPIDCalc_R1()
{
	float incPidout;
	curErr_R1 = tarSp_R1 - curSp_R1;  //当前误差
	incPidout = (Speed_Kp)*(curErr_R1-lastErr_R1) + Speed_Ki*curErr_R1;  //增量式PI控制器
	lastErr_R1 = curErr_R1;
	
	return incPidout;  //返回增量值
}
float IncPIDCalc_R2()
{
	float incPidout;
	curErr_R2 = tarSp_R2 - curSp_R2;  //当前误差
	incPidout = (Speed_Kp)*(curErr_R2-lastErr_R2) + Speed_Ki*curErr_R2;  //增量式PI控制器
	lastErr_R2 = curErr_R2;
	
	return incPidout;  //返回增量值
}
int PID_TIMEBASE_IRQ_IRQHandler(void)
{
 if(TIM_GetFlagStatus(PID_TIMEBASE_TIM,TIM_FLAG_Update)==SET)
 {

	TIM_ClearITPendingBit(PID_TIMEBASE_TIM,TIM_IT_Update);       //===清除定时器6中断标志位

	static int k=0; 
	lflag++;
	k++;
	if(lflag==2)
	{
		lflag=0;
		flag_runpid=1;
	}		
	if(k==5)
	{
		// printf("%f",car.yaw);
		k=0;
	}
	

	//pid捏
 }
 return 0;
}


