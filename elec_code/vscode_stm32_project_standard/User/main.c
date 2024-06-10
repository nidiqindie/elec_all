#include "stm32f10x.h"                  // Device header
#include "bsp_usart.h"
#include "stm32f10x_it.h"
#include "bsp_i2c.h"
#include <inttypes.h>
#include <stdio.h>
#include "bsp_SysTick.h"
#include "bsp_led.h"
#include "bsp_delay.h"
#include "inv_mpu.h"
#include "Emm_V5.h"
#include "bsp_motor.h"

// 用mpu6050的时候的一些宏定义,用的时候解除注释就行
// #include "mpu6050.h"
//  float Pitch,Roll,Yaw;								//俯仰角默认跟中值一样，翻滚角，偏航角
//  int16_t ax,ay,az,gx,gy,gz;							//加速度，陀螺仪角速度
//  u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
//  u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);
//  int a=0;

// 用jy901s的一些宏定义，与上文一致
#include "wit_c_sdk.h"
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static void Delayms(uint16_t ucMs)
{
	delay_ms(ucMs);
}
#define bound 9600 

#define  use_uart_to_pc
// #define use_jy901s

int main(void)
{
    //禁用jtag接口，把这些接口开放出去。
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
   
    //滴答计时器的初始化用于延时
    delay_init();
    //调试串口初始化，这里为串口5
    UART5_Config();
    //rgb灯的gpio初始化
    LED_GPIO_Config();
    
    // 初始化四个长大头步进电机的通讯串口这里为串口1.
    USART1_Config();
    delay_ms(100);

    Emm_V5_En_Control(1, 1, 0);
    delay_ms(1);

    Emm_V5_En_Control(2, 1, 0);
    delay_ms(1);

    Emm_V5_En_Control(3, 1, 0);
    delay_ms(1);

    Emm_V5_En_Control(4, 1, 0);
    delay_ms(1);
   Delayms(1000);
   
    // 用mpu6050的时候用这些
	//  MPU6050_Init();
	//  MPU6050_DMP_Init();
	// 测试调试串口是否可以和电脑正常通讯。需要测试就定义use_uart_to_pc这个字符串
	// #ifdef use_uart_to_pc

    move_forward(100, 20, 4);
    while (1) {

    }
	// #endif // DEBUG
    // 使用jy901s，定义一个字符串use_jy901s即可使用
    // 初始化读数据的串口，这里原理图分配的是串口四
   #ifdef use_jy901s
	Usart2Init(9600);
    float fAcc[3], fGyro[3], fAngle[3];
    int i;
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
	WitDelayMsRegister(Delayms);
	printf("\r\n********************** wit-motion normal example  ************************\r\n");
    AutoScanSensor();
    
    while (1)
    {
      CmdProcess();
		if(s_cDataUpdate)
		{
			for(i = 0; i < 3; i++)
			{
				// fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				// fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
            // if(s_cDataUpdate & ACC_UPDATE)
            // {
            // 	printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
            // 	s_cDataUpdate &= ~ACC_UPDATE;
            // }
            // if(s_cDataUpdate & GYRO_UPDATE)
            // {
            // 	printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
            // 	s_cDataUpdate &= ~GYRO_UPDATE;
            // }
            Delayms(1000);
			if(s_cDataUpdate & ANGLE_UPDATE)
			{// fAngle[1], fAngle[2]
				printf("angle:%.3f\r\n", fAngle[0]);
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
	    }
    }
   #endif // DEBUG 
}


// jy901s的一些main文件的函数，必须有
void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);//
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}

}
static void ShowHelp(void)
{
	printf("\r\n************************	 WIT_SDK_DEMO	************************");
	printf("\r\n************************          HELP           ************************\r\n");
	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	printf("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
	printf("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
	printf("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
	printf("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
	printf("UART SEND:h\\r\\n   help.\r\n");
	printf("******************************************************************************\r\n");
}

static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	
			if(WitStartAccCali() != WIT_HAL_OK) 
				printf("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	
			if(WitStartMagCali() != WIT_HAL_OK) 
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	
			if(WitStopMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) 
				printf("\r\nSet Baud Error\r\n");
			else 
				Usart2Init(c_uiBaud[WIT_BAUD_115200]);											
			break;
		case 'b':	
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n");
			else 
				Usart2Init(c_uiBaud[WIT_BAUD_9600]);												
			break;
		case 'R':	
			if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) 
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'r':	
			if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) 
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'C':	
			if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) 
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'c':	
			if(WitSetContent(RSW_ACC) != WIT_HAL_OK) 
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'h':
			ShowHelp();
			break;
	}
	s_cCmd = 0xff;
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Uart2Send(p_data, uiSize);
}



static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 1; i < 10; i++)
	{
		Usart2Init(9600);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			delay_ms(100);
			if(s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

