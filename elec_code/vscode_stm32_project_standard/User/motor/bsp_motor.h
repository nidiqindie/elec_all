#ifndef _BSP_MOTOR_H_
#define	_BSP_MOTOR_H_

#include "stm32f10x.h"


/*车轮方向GPIO口定义*/
#define    AIN_GPIO_CLK     RCC_APB2Periph_GPIOB
#define    AIN_GPIO_CLK_CMD     RCC_APB2PeriphClockCmd
#define    AIN1_GPIO_PORT   GPIOB
#define    AIN1_GPIO_PIN    GPIO_Pin_5
#define    AIN2_GPIO_PORT   GPIOB
#define    AIN2_GPIO_PIN    GPIO_Pin_4

#define     AIN1_ON       GPIO_SetBits(AIN1_GPIO_PORT,AIN1_GPIO_PIN) 
#define     AIN1_OFF      GPIO_ResetBits(AIN1_GPIO_PORT,AIN1_GPIO_PIN) 
#define     AIN2_ON       GPIO_SetBits(AIN2_GPIO_PORT,AIN2_GPIO_PIN) 
#define     AIN2_OFF      GPIO_ResetBits(AIN2_GPIO_PORT,AIN2_GPIO_PIN)

#define     A_NORMAL      AIN1_ON;\
                          AIN2_OFF
#define     A_REVERSE     AIN1_OFF;\
                          AIN2_ON
#define     A_STOP      AIN1_OFF;\
                          AIN2_OFF
#define     A_BRAKE      AIN1_ON;\
                          AIN2_ON

//B
#define    BIN_GPIO_CLK     RCC_APB2Periph_GPIOB
#define    BIN_GPIO_CLK_CMD     RCC_APB2PeriphClockCmd
#define    BIN1_GPIO_PORT   GPIOB
#define    BIN1_GPIO_PIN    GPIO_Pin_8
#define    BIN2_GPIO_PORT   GPIOB
#define    BIN2_GPIO_PIN    GPIO_Pin_9

#define     BIN1_ON       GPIO_SetBits(BIN1_GPIO_PORT,BIN1_GPIO_PIN) 
#define     BIN1_OFF      GPIO_ResetBits(BIN1_GPIO_PORT,BIN1_GPIO_PIN)      
#define     BIN2_ON       GPIO_SetBits(BIN2_GPIO_PORT,BIN2_GPIO_PIN) 
#define     BIN2_OFF      GPIO_ResetBits(BIN2_GPIO_PORT,BIN2_GPIO_PIN)

#define     B_NORMAL      BIN1_ON;\
                          BIN2_OFF
#define     B_REVERSE     BIN1_OFF;\
                          BIN2_ON
#define     B_STOP      BIN1_OFF;\
                          BIN2_OFF
#define     B_BRAKE      BIN1_ON;\
                          BIN2_ON
//C
#define    CIN_GPIO_CLK     RCC_APB2Periph_GPIOA
#define    CIN_GPIO_CLK_CMD     RCC_APB2PeriphClockCmd
#define    CIN1_GPIO_PORT   GPIOA
#define    CIN1_GPIO_PIN    GPIO_Pin_5
#define    CIN2_GPIO_PORT   GPIOA
#define    CIN2_GPIO_PIN    GPIO_Pin_4

#define     CIN1_ON       GPIO_SetBits(CIN1_GPIO_PORT,CIN1_GPIO_PIN) 
#define     CIN1_OFF      GPIO_ResetBits(CIN1_GPIO_PORT,CIN1_GPIO_PIN) 
#define     CIN2_ON       GPIO_SetBits(CIN2_GPIO_PORT,CIN2_GPIO_PIN) 
#define     CIN2_OFF      GPIO_ResetBits(CIN2_GPIO_PORT,CIN2_GPIO_PIN)

#define     C_NORMAL      CIN1_ON;\
                          CIN2_OFF
#define     C_REVERSE     CIN1_OFF;\
                          CIN2_ON
#define     C_STOP      CIN1_OFF;\
                          CIN2_OFF
#define     C_BRAKE      CIN1_ON;\
                          CIN2_ON
//D
#define    DIN_GPIO_CLK     RCC_APB2Periph_GPIOC
#define    DIN_GPIO_CLK_CMD     RCC_APB2PeriphClockCmd
#define    DIN1_GPIO_PORT   GPIOC
#define    DIN1_GPIO_PIN    GPIO_Pin_2
#define    DIN2_GPIO_PORT   GPIOC
#define    DIN2_GPIO_PIN    GPIO_Pin_3 

#define     DIN1_ON       GPIO_SetBits(DIN1_GPIO_PORT,DIN1_GPIO_PIN) 
#define     DIN1_OFF      GPIO_ResetBits(DIN1_GPIO_PORT,DIN1_GPIO_PIN) 
#define     DIN2_ON       GPIO_SetBits(DIN2_GPIO_PORT,DIN2_GPIO_PIN) 
#define     DIN2_OFF      GPIO_ResetBits(DIN2_GPIO_PORT,DIN2_GPIO_PIN)

#define     D_NORMAL      DIN1_ON;\
                          DIN2_OFF
#define     D_REVERSE     DIN1_OFF;\
                          DIN2_ON
#define     D_STOP      DIN1_OFF;\
                          DIN2_OFF
#define     D_BRAKE      DIN1_ON;\
                          DIN2_ON
/*end*/

/*车速PWM(TIMER8)定义*/
#define            SPEED_PWM_TIM                   TIM8
#define            SPEED_PWM_TIM_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define            SPEED_PWM_TIM_CLK               RCC_APB2Periph_TIM8
#define            SPEED_PWM_TIM_Period            500
#define            SPEED_PWM_TIM_Prescaler         72
// TIM8 输出比较通道1
#define            SPEED_PWM_TIM_GPIO_CLK      RCC_APB2Periph_GPIOC
#define            SPEED_PWM_TIM_PORT          GPIOC
#define            SPEED_PWM_TIM_CH1_PIN           GPIO_Pin_6
#define            SPEED_PWM_TIM_CH2_PIN           GPIO_Pin_7
#define            SPEED_PWM_TIM_CH3_PIN           GPIO_Pin_8
#define            SPEED_PWM_TIM_CH4_PIN           GPIO_Pin_9
/*end*/

/*编码器b计数(TIMER4)定义TIM2*/
#define            ENCODER1_TIM                   TIM4
#define            ENCODER1_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            ENCODER1_TIM_CLK               RCC_APB1Periph_TIM4
#define            ENCODER1_TIM_Period            0xffff
#define            ENCODER1_TIM_Prescaler         0

#define            ENCODER1_TIM_GPIO_CLK          RCC_APB2Periph_GPIOB
#define            ENCODER1_TIM_PORT              GPIOB
#define            ENCODER1_TIM_CH1_PIN           GPIO_Pin_6
#define            ENCODER1_TIM_CH2_PIN           GPIO_Pin_7

#define            ENCODER1_IRQ_IRQHandler       TIM4_IRQHandler
/*end*/

/*编码器d计数(TIMER5)定义*/
#define            ENCODER2_TIM                   TIM5
#define            ENCODER2_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            ENCODER2_TIM_CLK               RCC_APB1Periph_TIM5
#define            ENCODER2_TIM_Period            0xffff
#define            ENCODER2_TIM_Prescaler         0

#define            ENCODER2_TIM_GPIO_CLK          RCC_APB2Periph_GPIOA
#define            ENCODER2_TIM_PORT              GPIOA
#define            ENCODER2_TIM_CH1_PIN           GPIO_Pin_0
#define            ENCODER2_TIM_CH2_PIN           GPIO_Pin_1

#define            ENCODER2_IRQ_IRQHandler       TIM5_IRQHandler
/*end*/

 /*编码器a计数(TIMER2)定义*/
 #define            ENCODER3_TIM                   TIM2
 #define            ENCODER3_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
 #define            ENCODER3_TIM_CLK               RCC_APB1Periph_TIM2
 #define            ENCODER3_TIM_Period            0xffff
 #define            ENCODER3_TIM_Prescaler         0
 
#define            ENCODER3_TIM_GPIO_CLK1          RCC_APB2Periph_GPIOA
#define            ENCODER3_TIM_PORT1              GPIOA
#define            ENCODER3_TIM_CH1_PIN           GPIO_Pin_15

 #define            ENCODER3_TIM_GPIO_CLK2          RCC_APB2Periph_GPIOB
 #define            ENCODER3_TIM_PORT2              GPIOB
 #define            ENCODER3_TIM_CH2_PIN           GPIO_Pin_3


 #define            ENCODER3_IRQ_IRQHandler       TIM2_IRQHandler
 /*end*/

/*编码器c计数(TIMER3)定义*/
#define            ENCODER4_TIM                   TIM3
#define            ENCODER4_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            ENCODER4_TIM_CLK               RCC_APB1Periph_TIM3
#define            ENCODER4_TIM_Period            0xffff
#define            ENCODER4_TIM_Prescaler         0

#define            ENCODER4_TIM_GPIO_CLK          RCC_APB2Periph_GPIOA
#define            ENCODER4_TIM_PORT              GPIOA
#define            ENCODER4_TIM_CH1_PIN           GPIO_Pin_6
#define            ENCODER4_TIM_CH2_PIN           GPIO_Pin_7

#define            ENCODER4_IRQ_IRQHandler       TIM3_IRQHandler
/*end*/


/*PID时基(TIMER6定义*/
#define            PID_TIMEBASE_TIM               TIM6
#define            PID_TIMEBASE_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            PID_TIMEBASE_CLK               RCC_APB1Periph_TIM6
#define            PID_TIMEBASE_Period            10000
#define            PID_TIMEBASE_Prescaler         72

#define            PID_TIMEBASE_IRQn                TIM6_IRQn
#define            PID_TIMEBASE_IRQ_IRQHandler      TIM6_IRQHandler
/*end*/

void Motor_gpio_init(void);
void Speed_pwm_config(void);
void Set_speed(uint16_t CCR1_Val, uint16_t CCR2_Val, uint16_t CCR3_Val, uint16_t CCR4_Val);

void PID_timebase_init(void);
int Read_encoder(uint8_t TIMX);
int Incremental_PIA (int Encoder1,int Target);
int Incremental_PIB(int Encoder1,int Target);
int Incremental_PIC (int Encoder1,int Target);
int Incremental_PID (int Encoder1,int Target);

void Set_pwm(int A_moto,int B_moto,int C_moto, int D_moto);

void move_forward(uint16_t speed);
void move_backward(uint16_t speed);
void move_right(uint16_t speed);
void move_left(uint16_t speed);
void stop(void);

void speed_change(int A,int B,int C,int D);
void speed_change_b(int A,int B,int C,int D);
void Set_L1Speed(int16_t LSpeed);  //满速1000
void Set_L2Speed(int16_t LSpeed);
void Set_R1Speed(int16_t RSpeed);
void Set_R2Speed(int16_t RSpeed);

void Forward(float Speed);
void Backward(float Speed);
void L_Translation(float Speed);
void R_Translation(float Speed);
void L_Rotation(float Speed);
void R_Rotation(float Speed);



//函数声明
void Motor_gpio_init(void);
#endif

