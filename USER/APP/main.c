/****************************************Copyright (c)****************************************************                                    
**                                 http://www.powermcu.com
**--------------File Info---------------------------------------------------------------------------------
** File name:               main.c
** Descriptions:            The TouchPanel application function
**--------------------------------------------------------------------------------------------------------
** Created by:              G-LINK GROP
** Created date:            2011-12-07	flash OK版本   
** Version:                 v1.0
** Descriptions:            The original version
**--------------------------------------------------------------------------------------------------------
** Modified by:			Yu Jingxiong   2011.11.10
V1.0    yujignxiong   2011.11.27
** Modified date:		重新阅读并标注加深理解。。。。keep moving！！          
** Version:                 
** Descriptions:     TIM_CtrlPWMOutputs(TIM3,ENABLE);       
**




	2012.11.02 -- 2012.11.14 更新   吴梦龙
1、完成功率自动控制
2、生产校准后台
3、AD采样改成DMA方式
4、取消触摸屏触控
5、完善电源检测

文件结构
	main.c 				主程序
	PictureData.c 		16位彩色图片资源
	gl_ui.c 			图形接口
	zimo_st9.c 			部分ascii码宋体9号字体
	key.c   			按键扫描
	flash.c  			内部flash读写
	usart.c   			外部
	
	2013.03.15 吴梦龙
1、重新划分编译器rom空间，0x08000000 - 0x0803f7ff（0x3f800大小），
	余下0x800（2K）空间用于存储校准信息，flash.h文件里开始写入开始地址是0x0803F800
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h" 
//#include "stm32_eval.h" 
#include "..\\TouchPanel\\TouchPanel.h"
#include "..\\SysTick\\systick.h"
#include "..\\GLCD\\GLCD.h"
#include "stm32f10x_adc.h" 
#include "stm32f10x_dac.h"
#include "stm32f10x_dma.h"   
#include "stm32f10x_tim.h" 
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"

#include "stm32f10x_flash.h"
#include "key.h"
#include "usart.h"

#include "lcd\\gl_ui.h"
#include "lcd\\zimo_st9.h"
#include "lcd\\zimo_st20.h"
#include <string.h>
#include <stdio.h>
#include "prj_type.h"
#include "flash.h"

#include "misc.h" 
#include <math.h>
#include <string.h>
#include <stdio.h>
//#include <PictureData.h>
#include "USER\\GLCD\\PictureData.h"
#include "project.h"
#include "gl_key.h"
#include "lib\\base.h"
#include "LCD\\gl_type.h"

/*****************************************************************************
我的定义
*****************************************************************************/
#ifndef _DEBUG_
#define debugtxt
#else 
#define debugtxt gl_text
#endif

#ifndef _DEBUG_
#define dprintf
#else 
#define dprintf printf
#endif


struct ctrl_param g_power;//功率设置
struct adj_power_flash g_adj_power;//校准参数
volatile u16 powerDownDelayCnt=0;//关机长按延时
uint32_t hackval = 0;//进入后台输入的密码
uint32_t hackflag = 0;//是否输入密码标志
volatile u8 g_red_onoff = 0;//
volatile int8_t g_red_mode = 0;//红灯显示模式，常亮、闪烁、关闭
volatile u8 g_red_delay_100ms = 0;//红灯闪烁0.5s延时
volatile u8 g_onoff_en = 0;//使能关机，防止开机后再关机
volatile u8 g_autoctrlpower_en = 1;

volatile u8 g_key_timer_100ms = 0;//按键定时器，用于判断长按下和单击，复用红灯和定时关机按键
volatile uint16_t g_batter_delay = 0;//电池刷新显示延时
volatile float g_battery_vol = 12;//电池电压，默认12V
/////////////////////
volatile uint16_t g_power_down = 0;
volatile uint16_t g_ad_ms = 0,g_adjust_ms = 0,g_lcdbug_ms = 0,g_usart_ms = 0,g_lcdlisten_ms = 0,g_debug_ms = 0,g_redbug_ms = 0;//ad采样间隔
volatile uint16_t g_adc[200];//ad采样数
volatile uint16_t ADCConvertedValue[2000];//AD采样DMA缓存

int8_t g_recvflag = 0;//串口接收标志
volatile uint8_t strout[50];

/*****************************************************************************
原来的定义
*****************************************************************************/
volatile u16 SysTickCounter=0;
volatile u16 Timer_State = 0;	    //定时器状态指示,  OFF,  5min  ，10min  ，15min  ，30min  ，60min 状态
volatile u16 Timer_Counter = 0;
volatile u16 Wavelength_Selection_state = 0;	 //0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 	4 = 红光
volatile u16 Operating_Mode = 0;	//0 = CW、 1 = PW270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
volatile u8  Batter_Lightning=0;
volatile u8  LCD_GetPoint_EN = 1;   //防止触摸屏重复操作，相当于按键去抖动,可以进行触摸屏采集。
volatile u8  LCD_GetPoint_Counter = 0; //触摸屏使能计数值。。

volatile u8 FLAG_1310 = 0;		  //定时器中断中脉冲翻转标志。
volatile u8 FLAG_1490 = 0;
volatile u8 FLAG_1550 = 0;

uint16_t  TIM_Period[3] = {132,35,17};	//计数值为133、36、18，分别对应计数值为541hz、2Khz、4Khz，对应PWM频率
//分频系数统一为1000分频,0代表关闭光源，1代表连续光源。。

volatile uint16_t  TIM_Period1310 = 132;	 //初始化计数值。。最大，对应频率最低
volatile uint16_t  TIM_Period1490 = 132;
volatile uint16_t  TIM_Period1550 = 35;

//触屏使用
uint16_t X[3]={889,960,3320};
uint16_t Y[3]={487,3352,1982};
/**************************************************************************************/

#define DAC_DHR12RD_Address   ((uint32_t)0x40007420)
#define ADC2_DR_Address ((uint32_t)0x40013C4C)

/*********************************功能控制端口*****************************************/
#define RCC_GPIO_CTRL_B 			RCC_APB2Periph_GPIOB
#define GPIO_CTRL_PORT_B 			GPIOB
#define GPIO_SYSPWR_ONOFF 				       GPIO_Pin_6    //开关机控制（CHECK2） 开机置1，关机清0 

#define RCC_GPIO_CTRL_C 			RCC_APB2Periph_GPIOC	 //多路选择我开关是能段控制口，定时器中断响应，波长选择+频率控制
#define GPIO_CTRL_PORT_C 			GPIOC
#define GPIO_PORT_POWER_CHK 			GPIOA
#define GPIO_KEY_RED_CON 					   GPIO_Pin_0
#define GPIO_KEY_1310_CON 					   GPIO_Pin_3	 //对应管脚错误，2011.11.25测试修改
#define GPIO_KEY_1490_CON 					   GPIO_Pin_2
#define GPIO_KEY_1550_CON 					   GPIO_Pin_1
#define GPIO_CHARG_CHK 					       GPIO_Pin_8
//#define Control_KEYS				GPIO_KEY_1310_CON | GPIO_KEY_1490_CON | GPIO_KEY_1550_CON | GPIO_CHARG_CHK ;

#define RCC_GPIO_CTRL_D 			RCC_APB2Periph_GPIOD 
#define GPIO_CTRL_PORT_D 			GPIOD
#define GPIO_LCD_OFF					       GPIO_Pin_7    //关LCD屏控制（ON/OFF）  LCD使能端口，置低片选 

/*********************************中断控制端口*****************************************/
#define RCC_GPIO_CTRL_A 			RCC_APB2Periph_GPIOA 
#define GPIO_CTRL_PORT_A 			GPIOA

#define GPIO_ONOFF_CHK 			               GPIO_Pin_1  //手动开关机,启停监测CHECK1,PD2,上升沿中断
#define GPIO_OperatingMode_CHK			       GPIO_Pin_5  //模式选择按键，上升沿中断
#define GPIO_TimingSet_CHK			           GPIO_Pin_3  //定时选择按键，上升沿中断
#define GPIO_PowerUp_CHK			           GPIO_Pin_2  //功率 + 选择按键，上升沿中断
#define GPIO_PowerDown_CHK			           GPIO_Pin_6  //功率 - 选择按键，上升沿中断
#define GPIO_WaveSelection_CHK			       GPIO_Pin_7  //波长选择按键，上升沿中断  



/*手动开关机监测CHECK1,上升沿中断 PA1*/
#define GPIO_ONOFF_CHK_EXTI_LINE                            EXTI_Line1
#define GPIO_ONOFF_CHK_EXTI_PORT_SOURCE                     GPIO_PortSourceGPIOA
#define GPIO_ONOFF_CHK_PIN_SOURCE                           GPIO_PinSource1
#define GPIO_ONOFF_CHK_IRQn                                 EXTI1_IRQn 

/*波长选择按键，PA7，上升沿中断*/
#define GPIO_WaveSelection_CHK_EXTI_LINE                    EXTI_Line7
#define GPIO_WaveSelection_CHK_EXTI_PORT_SOURCE             GPIO_PortSourceGPIOA
#define GPIO_WaveSelection_CHK_PIN_SOURCE                   GPIO_PinSource7
#define GPIO_WaveSelection_CHK_IRQn                         EXTI9_5_IRQn 

/*输出脉冲模式选择按键，PA5，上升沿中断*/
#define GPIO_OperatingMode_CHK_EXTI_LINE                    EXTI_Line5
#define GPIO_OperatingMode_CHK_EXTI_PORT_SOURCE             GPIO_PortSourceGPIOA
#define GPIO_OperatingMode_CHK_PIN_SOURCE                   GPIO_PinSource5
#define GPIO_OperatingMode_CHK_IRQn                         EXTI9_5_IRQn 

/*输出功率  增 加 选择按键，PA2，上升沿中断*/
#define GPIO_PowerUp_CHK_EXTI_LINE                          EXTI_Line2
#define GPIO_PowerUp_CHK_EXTI_PORT_SOURCE                   GPIO_PortSourceGPIOA
#define GPIO_PowerUp_CHK_PIN_SOURCE                         GPIO_PinSource2
#define GPIO_PowerUp_CHK_IRQn                               EXTI2_IRQn

/*输出功率  递 减 选择按键，PA6，上升沿中断*/
#define GPIO_PowerDown_CHK_EXTI_LINE                        EXTI_Line6
#define GPIO_PowerDown_CHK_EXTI_PORT_SOURCE                 GPIO_PortSourceGPIOA
#define GPIO_PowerDown_CHK_PIN_SOURCE                       GPIO_PinSource6
#define GPIO_PowerDown_CHK_IRQn                             EXTI9_5_IRQn

/*TimingSet选择按键，PA3，上升沿中断*/
#define GPIO_TimingSet_CHK_EXTI_LINE                        EXTI_Line3
#define GPIO_TimingSet_CHK_EXTI_PORT_SOURCE                 GPIO_PortSourceGPIOA
#define GPIO_TimingSet_CHK_PIN_SOURCE                       GPIO_PinSource3
#define GPIO_TimingSet_CHK_IRQn                             EXTI3_IRQn




/*****************************************************************************
工程模块配置
*****************************************************************************/
/*****************一般功能IO口初始化******************************************/

void Function_IO_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//电源检测端口CHECK1(KEY_X)、CHECK2、CHARGER
	//CHECK1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_ONOFF_CHK;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//CHECK2
	GPIO_InitStructure.GPIO_Pin = GPIO_SYSPWR_ONOFF;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_SYSPWR_ONOFF);
	//CHARGER悬浮或下拉
	
	
	
	//按键KEY_A,KEY_b,KEY_B,KEY_Y,KEY_Z
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = KEY_A | KEY_B | KEY_C | KEY_Y | KEY_Z;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	//激光器：1550、1310、1490、650
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = 
		GPIO_KEY_RED_CON | GPIO_KEY_1310_CON | 
		GPIO_KEY_1490_CON | GPIO_KEY_1550_CON;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_KEY_1310_CON);
	GPIO_ResetBits(GPIOC,GPIO_KEY_1490_CON);
	GPIO_ResetBits(GPIOC,GPIO_KEY_1550_CON);
	GPIO_ResetBits(GPIOC,GPIO_KEY_RED_CON);
	
// 	//LCD使能
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
// 	GPIO_InitStructure.GPIO_Pin = GPIO_LCD_OFF;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
// 	GPIO_Init(GPIOD, &GPIO_InitStructure);
// 	GPIO_SetBits(GPIOD,GPIO_LCD_OFF);
	
	//1963:TS
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	LCD_Configuration();
}
/*
*Function:红光端口配置
*/
void RedLightIOConfig()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
 	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz ;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void USART_Configuration()
{
	USART_InitTypeDef USART_InitStructure;
	struct com_dev comdev;
	//u8 KeyNum = 0;
	//uint32_t IntDeviceSeriNum[3];	

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	//comdev.usart = COM_B6B7;
	comdev.usart = COM_B10B11;
	comdev.dma_tch = DMA1_Channel4;
	comdev.dma_rch = DMA1_Channel5;
	comdev.dma_f_tch = DMA1_FLAG_TC4;
	comdev.dma_f_rch = DMA1_FLAG_TC5;
	CommInit(&comdev, &USART_InitStructure);
	CommDMAMode(1);	
}
void delayMs(uint16_t ms)    //20000约为3s
{ 
// 	uint16_t i,j; 
// 	for( i = 0; i < ms; i++ )
// 	{ 
// 		for( j = 0; j < 1141; j++ );
// 	}
}

/*
电池电压测量通道，显示电池电量，低电压关机
*/
void ADC_Configuration()
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	//ADC1 采样wave 等其他共5个通道，用DMA通道1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// Enable ADC1 and GPIOC clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);	

	// DMA1 channel1 configuration ----------------------------------------------
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);//ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 400;//ADC_BUF;//two adc channel,cycle sampling 11 times
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//这样可以寻址ADCConvertedValue
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);// Enable DMA1 channel1 

  
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	//ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_55Cycles5);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_55Cycles5);


	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);// Enable ADC1 reset calibration register    
	while(ADC_GetResetCalibrationStatus(ADC1));// Check the end of ADC1 reset calibration register 
	ADC_StartCalibration(ADC1);// Start ADC1 calibration 
	while(ADC_GetCalibrationStatus(ADC1));// Check the end of ADC1 calibration 	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);// Start ADC1 Software Conversion  
	
	
}

void ADC2Configuration(void)  
{  
}

/*
DAC1配置函数，DAC1用于激光器功率控制可调输出。。。
管脚PA4作为
*/
void DAC_Configuration(void)
{  
	DAC_InitTypeDef DAC_InitStructure ;
	/* GPIOA Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);

	DAC_InitStructure.DAC_Trigger=DAC_Trigger_Software ;
	//设置为软件触发
	DAC_InitStructure.DAC_WaveGeneration=DAC_WaveGeneration_None ;
	DAC_InitStructure.DAC_OutputBuffer=DAC_OutputBuffer_Enable ;

	/* DAC channel1 Configuration */
	DAC_Init(DAC_Channel_1,&DAC_InitStructure);

	/* Enable DAC Channel1: Once the DAC channel2 is enabled, PA.04 is 
	automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1,ENABLE);

	//实际对应神州IV开发板的DAC1引脚--PA.4 (PIN29)
}

/*
用于定时计数，进行定时关机。。

*/
void TIM2_Init(void)
{   
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	/* ---------------------------------------------------------------
	TIM4 Configuration: Output Compare Timing Mode:
	TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 7200;	//固定定时时长，100ms ，10Hz     注： 10ms时候72000	100Hz
	TIM_TimeBaseStructure.TIM_Prescaler = (1000 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* Clear TIM3 update pending flag[清除TIM3溢出中断标志] */
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* TIM3 enable counter */
	TIM_Cmd(TIM2, ENABLE);
	//TIM_Cmd(TIM3, DISABLE);  	//初始化时候先关闭TIM3。激光器输出CW波形
}

/*******************************************************************************
* Function Name  : NVIC_Configuration       	  对应1310通道，管脚PC1,初始频率270hz
* Description    : Configures the used IRQ Channels and sets their priority.
*******************************************************************************/
void TIM3_Init(uint16_t  TIM_Period1310)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	/* ---------------------------------------------------------------
	TIM4 Configuration: Output Compare Timing Mode:
	TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = TIM_Period1310;
	TIM_TimeBaseStructure.TIM_Prescaler = (1000 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	/* Clear TIM3 update pending flag[清除TIM3溢出中断标志] */
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM3, DISABLE);  	//初始化时候先关闭TIM3.。激光器输出CW波形
}


/*******************************************************************************
* Function Name  : NVIC_Configuration						   对应1490通道，管脚PC2,初始频率270hz
* Description    : Configures the used IRQ Channels and sets their priority.
*******************************************************************************/
void TIM4_Init(uint16_t  TIM_Period1490)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM5 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	/* ---------------------------------------------------------------
	TIM4 Configuration: Output Compare Timing Mode:
	TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = TIM_Period1490;
	TIM_TimeBaseStructure.TIM_Prescaler = (1000 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	/* Clear TIM5 update pending flag[清除TIM5溢出中断标志] */
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	/* TIM5 enable counter */
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM4, DISABLE);  	//初始化时候先关闭TIM4.。激光器输出CW波形
}


/*******************************************************************************
* Function Name  : NVIC_Configuration					   对应1550通道，管脚PC3,初始频率270hz
* Description    : Configures the used IRQ Channels and sets their priority.
*******************************************************************************/
void TIM5_Init(uint16_t  TIM_Period1550)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM5 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 

	/* ---------------------------------------------------------------
	TIM4 Configuration: Output Compare Timing Mode:
	TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = TIM_Period1550;	//自动加载的计数值。。。
	TIM_TimeBaseStructure.TIM_Prescaler = (1000 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	/* Clear TIM5 update pending flag[清除TIM5溢出中断标志] */
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	/* TIM5 enable counter */
	TIM_Cmd(TIM5, ENABLE);
	TIM_Cmd(TIM5, DISABLE);
}

void TIM6_Init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 
	//时钟72M    1ms中断  向上计数
	TIM_TimeBaseStructure.TIM_Period = 10-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE); //开定时器中断
	TIM_Cmd(TIM6,ENABLE);
}
/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the used IRQ Channels and sets their priority.
//设置NVIC优先级分组，方式。
// 注：一共16个优先级，分为抢占式和响应式。两种优先级所占的数量由此代码确定，
NVIC_PriorityGroup_x可以是0、1、2、3、4，分别代表抢占优先级有1、2、4、8、16个和响应优先级有16、8、4、2、1个。
规定两种优先级的数量后，所有的中断级别必须在其中选择，抢占级别高的会打断其他中断优先执行，
而响应级别高的会在其他中断执行完优先执行。
*******************************************************************************/
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

	/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/*
* Function: 从内部FLASH读取校准信息
* Parameters:
* Return:
	返回数组中值
* Remarks:
*/
void FLASH_Configuration()
{
	uint32_t i;
	ReadFlash(FLASH_PAGE_START,
				(uint32_t*)&(g_adj_power),
				sizeof(struct adj_power_flash));
	if(g_adj_power.flag != 0xaabbccdd) 
	{
		
		g_adj_power._1310cw = 0;
		g_adj_power._1310_270 = 0;
		g_adj_power._1310_1k = 0;
		g_adj_power._1310_2k = 0;

		g_adj_power._1550cw = 0;
		g_adj_power._1550_270 = 0;
		g_adj_power._1550_1k = 0;
		g_adj_power._1550_2k = 0;

		for(i = 0;i < 24;++i) {
			g_adj_power.sn[i] = '0';
		}
		g_adj_power.sn[24] = '\0';

		g_adj_power._650_en = 1;//650
		g_adj_power._1310_en = 1;//1310
		g_adj_power._1490_en = 0;//1490
		g_adj_power._1550_en = 1;//1550

		g_adj_power._logo_addr = 0x0000000;//logo地址
		g_adj_power._logo_backcolor = 0x22f2;//logo背景色
		g_adj_power._logo_w = 243;//logo宽度，宽高有最大限制
		g_adj_power._logo_h = 57;//logo高度	
	
	}
	printf("Load config from FLASH\r\n");
	printf("------------------------------------------------\n");
	printf("SN:%s\n",g_adj_power.sn);
	printf("Out:%4.4d\tIn:%4.4d\n",(uint32_t)g_adj_power._1310cw,(uint32_t)g_adj_power._1310_270);
	printf("Product Configuration\n");
	printf("    |650\t|%s\t",
		g_adj_power._650_en?"On":"Off");
	printf("    |1310\t|%s\n",
		g_adj_power._1310_en?"On":"Off");
	printf("    |1490\t|%s\t",
		g_adj_power._1490_en?"On":"Off");
	printf("    |1550\t|%s\n",
		g_adj_power._1550_en?"On":"Off");
	printf("------------------------------------------------\n");

	
}



/****************************************************************************
数模转换部分
*****************************************************************************/
/*
* Function: 制定AD通道获取AD数据
* Parameters:
	chx = 0 电池
	chx = 1 功率控制
* Return:
	返回数组中值
* Remarks:
*/
uint16_t GetAD(uint8_t chx)
{
	uint32_t i,k,start = 0;
	uint32_t tmp;
	float ave;
	
	if(chx == 0)
		start = 0;
	else if(chx == 1)
		start = 1;
	for(k = 0,i = start;i < 400;/*ADC_BUF;*/i+=2) {
		g_adc[k] = ADCConvertedValue[i];
		k++;
	}

	tmp = 0;
	for(i = 0;i < 200;i++) {
		tmp += g_adc[i];
	}
	ave = (float)tmp / 200;

	return (uint16_t)ave;
}



/*
* Function:                    
****************************************************************
*								该工程核心代码
****************************************************************
*
*		计算dbm所对应的比例系数，求10的x次方的幂，
*		该函数现在已经不用，属于精简版的pow函数，但是理论基础不变，详情看下面
*		Remark里的例子
* Parameters:
*	dbm 0-10.00
* Return:
*	幂，这里把它叫做比例系数
* Remarks:
* 		下表的意义在于求10的n次方，如10^0.2  =  c10_0_01[2]
*		求10^2.91=?	
* 		c10_10_0[2]  *  c10_0_1[9]   *   c10_0_01[1] = 
*		100          *    7.943282   *   1.002305     = 812.8305161
*	例子：
*		已知预输出 -10dBm光功，ADC探测得350，DAC是300，那么输出 -7dBm ADC和DAC应该得多少？
*			-10dBm - -7dBm = -3dBm					(1)
*			-3dBm = 10 * log(x) => log(x) = 0.3		(2)
*		由(1)(2)得：
*			10^0.3 = 1.995262315					(3)
*		注意：(3)里的10不是-10dbm的意思，仅仅为了求个比例系数
*
*		由于题目 -10dBm 时 ADC 为350				(4)
*		所以预输出 -7dBm ADC应该得
*			350 * 10^0.3 = 698.3418102
*		保留整数		 = 698						(5)
*		同理DAC得
*			300 * 10^0.3 = 598.5786945				(6)
*		保留整数		 = 598						(7)
*		注意：由于激光器非线性工作，所以当DAC为598时ADC未必得698，所以需要动态调整
*		DAC值，直到ADC为698
*
*/
//计算10的x次方需要的数组，x = 浮点型，小数部分(0 - 0.9999)，有效范围-99.99dBm - +99.99dBm 
//                                     10^0.0000   10^0.0001   10^0.0002   10^0.0003   10^0.0004   10^0.0005   10^0.0006   10^0.0007   10^0.0008   10^0.0009
volatile const float c10_0_0001[10] = {1.000000,   1.000230,   1.000461,   1.000691,   1.000921,   1.001152,   1.001383,   1.001613,   1.001844,   1.002074 };
//                                     10^0        10^0.001    10^0.002    10^0.003    10^0.004    10^0.005    10^0.006    10^0.007    10^0.008    10^0.009
volatile const float c10_0_001[10]  = {1.000000,   1.002305,   1.004616,   1.006932,   1.009253,   1.011579,   1.013911,   1.016249,   1.018591,   1.020939 };
//                                     10^0        10^0.01     10^0.02     10^0.03     10^0.04     10^0.05     10^0.06     10^0.07     10^0.08     10^0.09
volatile const float c10_0_01[10]   = {1.000000,   1.023293,   1.047129,   1.071519,   1.096478,   1.122018,   1.148154,   1.174898,   1.202264,   1.230269 };
//                                     10^0        10^0.1      10^0.2      10^0.3      10^0.4      10^0.5      10^0.6      10^0.7      10^0.8      10^0.9
volatile const float c10_0_1[10]    = {1.000000,   1.258925,   1.584893,   1.995262 ,  2.511886 ,  3.162278 ,  3.981072 ,  5.011872 ,  6.309573 ,  7.943282};
//                                     10^0        10^1        10^2        10^3        10^4        10^5        10^6        10^7        10^8        10^9
volatile const float c10_10_0[10]   = {1,          10,         100,        1000,       10000,      100000,     1000000,    10000000,   100000000,  1000000000};
float DbmToScale(float dbm)
{
	int a0,a1,a2;
	int b0 = 0;
	float tmp;
	int flag =0;

	if(dbm < 0) {
		dbm = -dbm;
		flag = 1;
	}
	if(dbm >= 10) {
		//dbm =10;
		b0 = (int)(dbm / 10);
		dbm = dbm - b0*10;
	}
	a0 = (int)dbm;
	dbm *=10;
	a1 = (int)dbm;
	dbm *=10;
	a2 = (int)dbm;
	dbm *=10;

	a2 = a2 - a1 * 10;
	a1 = a1 - a0 * 10;

	tmp = (float)(c10_0_1[a0]*c10_0_01[a1]*c10_0_001[a2]*c10_10_0[b0]);
	if(flag)
		tmp = 1/tmp;
	return tmp;
}

/****************************************************************************
外围器件控制部分
*****************************************************************************/
/*
* Function: 红光控制
* Parameters:
	1：点亮、0：关闭
*/
void Ctrl_RedLight(u8 v)
{
	if(v == 1) {
		GPIO_SetBits(GPIO_CTRL_PORT_C,GPIO_KEY_RED_CON);
		//debugtxt(100,0,"1",-1);
	}
	else {
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_RED_CON);
		//debugtxt(100,0,"0",-1);
	}
}
/*****************************选择波长种类端口控制********************************
* Function Name  : Ctrl_Wavelength	  BY Yu Jingxiong  2011.11.17
* Description    : 选定了波长，Ctrl_Wavelength为0、1、2、3、4，选择激光器输出波长Operating_Mode
只是对定时器3、4、5的使能与及相应IO口的屏蔽进行控制
**************************************************************************************/
#define WL_OFF  0
#define WL_1310 1
#define WL_1490 2
#define WL_1550 3
#define WL_RED  4
void Ctrl_Wavelength(u8 Wavelength_Selection_state)	  //Operating_Mode  0 = CW、 1 = 270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
{
	//Wavelength_Selection_state   0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = 红光
	switch(Wavelength_Selection_state)				
	{
	case   WL_OFF:	  // 0 = 关闭光源
		TIM_Cmd(TIM3, DISABLE);	   //1310nm
		TIM_Cmd(TIM4, DISABLE);	   //1490nm
		TIM_Cmd(TIM5, DISABLE);	   //1550nm
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);

		break;

	case	WL_1310:	 // 1 = 1310nm	PC1			
		TIM_Cmd(TIM4, DISABLE);
		TIM_Cmd(TIM5, DISABLE);
		TIM_Cmd(TIM3, ENABLE);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);

		break;		

	case	WL_1490:	 // 2 = 1490nm	PC2
		TIM_Cmd(TIM3, DISABLE);			
		TIM_Cmd(TIM5, DISABLE);
		TIM_Cmd(TIM4, ENABLE);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);

		break;

	case	WL_1550:	 // 3 = 1550nm	PC3
		TIM_Cmd(TIM3, DISABLE);
		TIM_Cmd(TIM4, DISABLE);
		TIM_Cmd(TIM5, ENABLE);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);

		break; 

	case	WL_RED:	 // 4 = 红光

		break;

	default:
		break;     		 

	}
}

/*****************************模式选择模拟开关控制端口********************************
* Function Name  : Ctrl_Operating_Mode	  BY Yu Jingxiong  2011.11.15
* Description    : 在选定了波长的前提下Wavelength_Selection_state为1、2、3，控制激光器输出模式Operating_Mode
对定时器的PWM脉冲频率进行控制 ，即对各个定时器的计数值进行设置。。
**************************************************************************************/
#define OPM_CW 0
#define OPM_270 1
#define OPM_1K 2
#define OPM_2K 3
void Ctrl_Operating_Mode(u8 Operating_Mode)	 //Operating_Mode  0 = CW、 1 = 270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
{										     //Wavelength_Selection_state   0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = 红光

	switch(Operating_Mode)				
	{
	case   0:	  // 0 = CW、
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //添加报错显示函数，关闭状态和红光 不可选模式
		else if( Wavelength_Selection_state == 1 )	  //1310波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);	
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);	
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);				
			GPIO_SetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);
		}

		else if( Wavelength_Selection_state == 2 )	  //1490波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);	
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);	
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);				
			GPIO_SetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);	
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);	
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);				
			GPIO_SetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
		}

		break;

	case	1:	 // 1 = PL 270Hz、
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //添加报错显示函数，关闭状态和红光 不可选模式
		else if( Wavelength_Selection_state == 1 )	  //1310波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			TIM_Period1310 = TIM_Period[0];
			TIM3_Init( TIM_Period1310);
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM3, ENABLE);	
		}

		else if( Wavelength_Selection_state == 2 )	  //1490波长
		{
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Period1490 = TIM_Period[0];
			TIM4_Init( TIM_Period1490);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550波长
		{
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);
			TIM_Period1550 = TIM_Period[0];
			TIM5_Init( TIM_Period1550);
			TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM5, ENABLE);
		}
		break;		

	case	2:	 // 2 = PL 1KHz、
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //添加报错显示函数，关闭状态和红光 不可选模式
		else if( Wavelength_Selection_state == 1 )	  //1310波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			TIM_Period1310 = TIM_Period[1];
			TIM3_Init( TIM_Period1310);
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM3, ENABLE);
		}

		else if( Wavelength_Selection_state == 2 )	  //1490波长
		{
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Period1490 = TIM_Period[1];
			TIM4_Init( TIM_Period1490);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550波长
		{
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);
			TIM_Period1550 = TIM_Period[1];
			TIM5_Init( TIM_Period1550);
			TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM5, ENABLE);
		}     	
		break;

	case	3:	 // 3 = PL 2KHz
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //添加报错显示函数，关闭状态和红光 不可选模式
		else if( Wavelength_Selection_state == 1 )	  //1310波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			TIM_Period1310 = TIM_Period[2];
			TIM3_Init( TIM_Period1310);
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM3, ENABLE);
		}

		else if( Wavelength_Selection_state == 2 )	  //1490波长
		{
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Period1490 = TIM_Period[2];
			TIM4_Init( TIM_Period1490);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550波长
		{
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);
			TIM_Period1550 = TIM_Period[2];
			TIM5_Init( TIM_Period1550);
			TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM5, ENABLE);
		} 
		break; 

	default:
		break;     		 

	}
}
/*
* Function: 功率控制
* Parameters:
	传递参数结构
	-1dBm = v->set = -1000;
* Remarks:
*/
void Ctrl_Power(struct ctrl_param *v)
{
	uint8_t strout[30];
	float tmpdbm;
	/*
		3.3V分成4096份，每份0.805mV
		Vad = AD * 0.805mV
		Vad = P * R * K * A/B = AD * 0.805
		P:功率大小，dbm->power,用函数DbmToScale转换dbm到power
		R:电阻大小，1000欧
		K:光响应度，(1550nm)约0.986，(1310nm)约0.895其他波长待定
		其实这个值不用太在意，它也不能说是响应度，只是个校正偏差而已，有了自动校准以后，K的值意义不大
		A/B:光环形器分光比，这里是50:50所以可以舍去
		AD = P * R * K / 0.805 = P * 1224.84472 = DbmToScale(v->set) * 1224.84472
	*/
	//scale = DbmToScale((float)(v->set/1000);
	/*if(Wavelength_Selection_state == 1)
		v->set -= 220;//后期通过响应度来修改*/
	
	if(Wavelength_Selection_state == WL_1310) {
		if(Operating_Mode == OPM_CW) {
			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1310cw;
		}
		else if(Operating_Mode == OPM_270) {
			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1310_270;
		}
		else if(Operating_Mode == OPM_1K) {
			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1310_1k;
		}
		else if(Operating_Mode == OPM_2K) {
			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1310_2k;
		}
	}
	else if(Wavelength_Selection_state == WL_1490 ) {
	}
	else if(Wavelength_Selection_state == WL_1550) {
		if(Operating_Mode == OPM_CW) {
			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1550cw;
		}
		else if(Operating_Mode == OPM_270) {
			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1550_270;
		}
		else if(Operating_Mode == OPM_1K) {
			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1550_1k;
		}
		else if(Operating_Mode == OPM_2K) {
			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1550_2k;
		}
	}
	else {
		//tmp.set = v->set;
		tmpdbm = (float)(v->set/1000.0);
	}
	tmpdbm = (float)(v->set/1000.0) + g_adj_power._1550_1k;
	//v->dac = (uint16_t)(DbmToScale((float)(v->set/1000.0)) * 2460);//2460只是个大概的数，方便快速调节
	
	v->dac = (uint16_t)((pow(10,(float)tmpdbm/10))*g_adj_power._1310_270*10);
	if(Wavelength_Selection_state == WL_1550) {
		//v->adc = (uint16_t)(DbmToScale((float)(v->set/1000.0)) * 1224.84472);
		//v->adc = (uint16_t)(DbmToScale(tmpdbm) * 1224.84472);
		v->adc = (uint16_t)((pow(10,(float)tmpdbm/10))*g_adj_power._1550cw*10);
	}
	else if(Wavelength_Selection_state == WL_1310) {
		//v->adc = (uint16_t)(DbmToScale((float)(v->set/1000.0)) * (1000*1.040/0.805));
		//v->adc = (uint16_t)(DbmToScale(tmpdbm) * (1000*1.040/0.805));
		v->adc = (uint16_t)((pow(10,(float)tmpdbm/10))*g_adj_power._1310cw*10);
	}
	else {
		//v->adc = (uint16_t)(DbmToScale(tmpdbm) * 1224.84472);
		v->adc = (uint16_t)((pow(10,(float)tmpdbm/10))*0);
	}
	//以上计算v->adc全部忽略
	v->adc = (uint16_t)((pow(10,(float)tmpdbm/10))*g_adj_power._1310cw*10);
	
	sprintf(strout,"v->dac %4.4d",v->dac);
	debugtxt(0,12,strout,-1);
	sprintf(strout,"v->adc %4.4d",v->adc);
	debugtxt(0,24,strout,-1);
	sprintf(strout,"v->set dbm %3.3f",(float)(v->set / 1000.0));
	debugtxt(0,36,strout,-1);
	
	if(v->adc > 4095) {
		v->adc = 6095;
	}
	else if(v->adc < 0){
		v->adc = 0;
	}
	if(v->dac > 4095) {
		v->dac = 6095;
	}
	else if(v->dac < 0){
		v->dac = 0;
	}
	DAC_SoftwareTriggerCmd(DAC_Channel_1,DISABLE);  
	DAC_SetChannel1Data(DAC_Align_12b_R, v->dac);
	DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);	
	LCD_Power_Control_Selection_Ex(55,90,((uint16_t)((int32_t)v->set / 1000)),White, Grey);	
	g_autoctrlpower_en = 1;
// 	g_red_mode++;
// 	if(g_red_mode >=3)
// 		g_red_mode = 0;
// 	g_red_onoff = ~g_red_onoff;
// 	LCD_RedLight_Show(9,15,g_red_onoff);
}



/*
* Function: 
****************************************************************
*								该工程核心代码
****************************************************************
*	功率自动校正，属于该产品核心部分之一，与保存在FLASH里的ADC值联合使用
*	调节方法：
*		1. Ctrl_Power()计算最终需要的ADC值，以及大概的DAC值（不在本函数里调用）
*		2. 时刻检测实际ADC值与理论ADC值的偏差
*		3. 根据偏差调节实际DAC值
*
*	该函数调节大小来源于经验值，调节太小功率平衡慢，调节太大或太快容易抖动
*
* remark:
* 如下代码经过特意劣化:
* 		1. 功率平衡时间大约为4S
* 		2. 功率最终在平衡反复跳动，影响操作人员度数
*		3. 功率在平衡点稳定性是+/-0.1dBm
* 
* 其余版本任何控制算法都比它好：
*		1. 功率平衡时间大约为2S
*		2. 功率在平衡点稳定，能够正常度数
*		3. 功率在平衡点稳定性是+/-0.02dBm（与硬件运放有关）
*/
void AutoCtrlPower()
{
#ifdef _DEBUG_
	uint8_t strout[20];
#endif
	uint16_t ad;						// 获取实际ADC值，作为调节DAC大小依据
	uint16_t fadj = 0;					// 标志位，是否需要调节DAC
	static uint16_t adjust_time  = 800;	// 下次校准时间，ms为单位
	static uint16_t balance_times = 0;	// 现在已经无用
	
	//根据偏差的多少，设定下次校正时间和校正幅度
	if(g_adjust_ms >= adjust_time ) {
		g_adjust_ms = 0;
		if(g_autoctrlpower_en == 0) {
			return ;
		}
		ad = GetAD(1);//读取ad值，200次取平均
#ifdef _DEBUG_
		sprintf(strout,"GetAD %4.4d",ad);
		debugtxt(0,48,strout,-1);
#endif
		
		if(ad - g_power.adc > 50) {
			g_power.dac -= 50;//校正幅度-50
			fadj = 1;
			adjust_time  = 800;//800ms后再次校正
			balance_times = 0;
		}
		else if(g_power.adc - ad > 50) {
			g_power.dac += 50;
			fadj = 1;
			adjust_time  = 800;
			balance_times = 0;
		}
		else if(ad - g_power.adc > 6) {
			g_power.dac -= 5;
			fadj = 1;
			adjust_time  = 500;
			balance_times = 0;
			
			
		}
		else if(g_power.adc - ad> 6) {
			g_power.dac += 5;
			fadj = 1;
			adjust_time  = 500;
			balance_times = 0;
			
			
		}
		else if(ad - g_power.adc > 2) {
			g_power.dac -= 20;
			fadj = 1;
			adjust_time  = 1000;
			balance_times = 0;
			if(Operating_Mode ==OPM_270) {
				g_autoctrlpower_en = 0;
			}
			
		}
		else if(g_power.adc - ad> 2) {
			g_power.dac += 20;
			fadj = 1;
			adjust_time  = 1000;
			balance_times = 0;
			if(Operating_Mode ==OPM_270 ) {
				g_autoctrlpower_en = 0;
			}
		}
		else {
			if(balance_times++ > 2) {
				g_autoctrlpower_en = 0;
			}
		}
		
		if(fadj) {
			if(g_power.dac > 4095) {
				g_power.dac = 4095;
			}
			DAC_SoftwareTriggerCmd(DAC_Channel_1,DISABLE);  
			DAC_SetChannel1Data(DAC_Align_12b_R, g_power.dac);
			DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
			
		}
#ifdef _DEBUG_
		sprintf(strout,"dac %4.4d",g_power.dac);
		debugtxt(0,60,strout,-1);
#endif
	}
}

/*****************************模式选择模拟开关控制端口********************************
* Function Name  : Ctrl_Timing_Device	  BY Yu Jingxiong  2011.11.25
* Description    : //Timer_State  对应定时器关机设置为5min  ，10min  ，15min  ，30min  ，60min 状态
TIM2设置为100ms周期，3000 ， 6000  ， 9000  ， 18000 ，  36000
Timer_State：0 关闭，1       ，2     ，3       ，4      ，5  
**************************************************************************************/
#define TM_OFF   0
#define TM_5MIN  1
#define TM_10MIN 2
#define TM_15MIN 3
void Ctrl_Timing_Device(u8 Timer)	  
{
	switch(Timer)				
	{
	case   0:	  // 0 = 定时器关闭
		Timer_Counter = 0;	  //调整状态之后保证每次计数从零开始
		Timer_State  = 0;
		break;
	case   1:	  // 1 = 定时器打开
		Timer_Counter = 0;
		Timer_State  = 1;
		break;
	case   2:	  // 1 = 定时器打开
		Timer_Counter = 0;
		Timer_State  = 2;
		break;
	case   3:	  // 1 = 定时器打开
		Timer_Counter = 0;
		Timer_State  = 3;
		break;
	case   4:	  // 1 = 定时器打开
		Timer_Counter = 0;
		Timer_State  = 4;
		break;
	case   5:	  // 1 = 定时器打开
		Timer_Counter = 0;
		Timer_State  = 5;			 		
		break;
	default:
		break;
	}
}
/******************************************************************************/

/*开机键、波长键、模式键 端口初始化*************************************************/
/***************************************************************************/
/*开机键、波长键、模式键端口初始化************************************************/
void External_Interrupt_Config(void)
{
}

/*CHECK1、CHECK3、CHECK4外部中断初始化**********************************/
void External_Interrupt_EXIT_Init(void)
{
	//EXTI_InitTypeDef EXTI_InitStructure;
	/* Configure the Priority Group to 2 bits */
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Connect CHECK1 EXTI Line to CHECK1 GPIO Pin */

	// 	GPIO_EXTILineConfig(GPIO_ONOFF_CHK_EXTI_PORT_SOURCE, GPIO_ONOFF_CHK_PIN_SOURCE);  
	// 	EXTI_InitStructure.EXTI_Line = GPIO_ONOFF_CHK_EXTI_LINE;
	// 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	// 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	// 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	// 	EXTI_Init(&EXTI_InitStructure);
	// 	
	// 	


	// 	/* Connect WaveSelection EXTI Line to WaveSelection GPIO Pin 	 */
	// 	GPIO_EXTILineConfig(GPIO_WaveSelection_CHK_EXTI_PORT_SOURCE, GPIO_WaveSelection_CHK_PIN_SOURCE);  
	// 	/* Configure WaveSelection EXTI line   */
	// 	EXTI_InitStructure.EXTI_Line = GPIO_WaveSelection_CHK_EXTI_LINE;
	// 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	// 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //EXTI_Trigger_Falling;  --20111017 fixed
	// 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	// 	EXTI_Init(&EXTI_InitStructure);


	// 	/* Connect OperatingMode EXTI Line to OperatingMode GPIO Pin */
	// 	GPIO_EXTILineConfig(GPIO_OperatingMode_CHK_EXTI_PORT_SOURCE, GPIO_OperatingMode_CHK_PIN_SOURCE);  
	// 	/* Configure Button  OperatingMode EXTI line */
	// 	EXTI_InitStructure.EXTI_Line = GPIO_OperatingMode_CHK_EXTI_LINE;
	// 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	// 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
	// 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	// 	EXTI_Init(&EXTI_InitStructure);

	// 	/* Connect OperatingMode EXTI Line to OperatingMode GPIO Pin */
	// 	GPIO_EXTILineConfig(GPIO_PowerUp_CHK_EXTI_PORT_SOURCE, GPIO_PowerUp_CHK_PIN_SOURCE);  
	// 	/* Configure Button  OperatingMode EXTI line */
	// 	EXTI_InitStructure.EXTI_Line = GPIO_PowerUp_CHK_EXTI_LINE;
	// 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	// 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
	// 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	// 	EXTI_Init(&EXTI_InitStructure);

	// 	/* Connect OperatingMode EXTI Line to OperatingMode GPIO Pin */
	// 	GPIO_EXTILineConfig(GPIO_PowerDown_CHK_EXTI_PORT_SOURCE, GPIO_PowerDown_CHK_PIN_SOURCE);  
	// 	/* Configure Button  OperatingMode EXTI line */
	// 	EXTI_InitStructure.EXTI_Line = GPIO_PowerDown_CHK_EXTI_LINE;
	// 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	// 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
	// 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	// 	EXTI_Init(&EXTI_InitStructure);

	// 	/* Connect OperatingMode EXTI Line to OperatingMode GPIO Pin */
	// 	GPIO_EXTILineConfig(GPIO_TimingSet_CHK_EXTI_PORT_SOURCE, GPIO_TimingSet_CHK_PIN_SOURCE);  
	// 	/* Configure Button  OperatingMode EXTI line */
	// 	EXTI_InitStructure.EXTI_Line = GPIO_TimingSet_CHK_EXTI_LINE;
	// 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	// 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
	// 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	// 	EXTI_Init(&EXTI_InitStructure);

}

/*CHECK1、CHECK3、CHECK4外部中断配置函数*********************************************************
// 注：一共16个优先级，分为抢占式和响应式。两种优先级所占的数量由此代码确定，
NVIC_PriorityGroup_x可以是0、1、2、3、4，分别代表抢占优先级有1、2、4、8、16个和响应优先级有16、8、4、2、1个。
规定两种优先级的数量后，所有的中断级别必须在其中选择，抢占级别高的会打断其他中断优先执行，
而响应级别高的会在其他中断执行完优先执行。
*/
void External_Interrupt_InterruptConfig(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
	/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//屏蔽外部中断1
	NVIC_InitStructure.NVIC_IRQChannel = GPIO_ONOFF_CHK_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = GPIO_WaveSelection_CHK_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = GPIO_OperatingMode_CHK_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = GPIO_PowerUp_CHK_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = GPIO_PowerDown_CHK_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = GPIO_TimingSet_CHK_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/*******************************************************************************
* Function Name  : DelayUS
* Description    : 延时1us
* Input          : - cnt: 延时值
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void delayUs(vu32 cnt)
{
// 	uint16_t i;
// 	for(i = 0;i<cnt;i++)
// 	{
// 		uint8_t us = 12; /* 设置值为12，大约延1微秒 */    
// 		while (us--)     /* 延1微秒	*/
// 		{
// 			;   
// 		}
// 	}
}

/*
* Function: 绘制主界面
*/
void LCD_DrawMain(void)
{
	uint16_t x,y;

	//显示LOGO
// 	for( x=0; x < 320; x++ )		//上边界蓝条宽34  下边界蓝条宽49
// 		for( y=0; y < 240; y++ )
// 			gl_setpoint(x,y,0x22f2);	//屏幕主色调gray
	LCD_Clear(Black);
			
	LCD_FLSAH_DrawPicture(38,91,38+243-1,91+57-1,(uint8_t*)gImage_logo);
	Delay_ms(2000);
// 	gl_setarea(0,0,319,239);
// 	for( x=0; x < 320; x++ )		//上边界蓝条宽34  下边界蓝条宽49
// 		for( y=0; y < 240; y++ )
// 			gl_setpoint(x,y,0x22f2);	//屏幕主色调gray
	gl_clear(0,0,320,240,0x22f2);
	
	//初始化功率
	g_power.set = (int32_t)(-10000);
	Ctrl_Power(&g_power);
	//初始化波长
	Wavelength_Selection_state = WL_1310;
	if(g_adj_power._1310_en == 0) {
		if(g_adj_power._1490_en) {
			Wavelength_Selection_state = WL_1490;
		}
		else {
			Wavelength_Selection_state = WL_1550;
		}
	}
	//Ctrl_Wavelength( Wavelength_Selection_state); //Wavelength_Selection_state   0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = 红光
	LCD_Wavelength_Selection_Ex( 170,200 ,Wavelength_Selection_state ,Yellow, Grey );
	
	//初始化输出频率
	Operating_Mode = OPM_CW;
	//Ctrl_Operating_Mode( Operating_Mode);	     //Operating_Mode  0 = CW、 1 = 270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
	LCD_OperatMode_Selection( 15, 200,Operating_Mode, Yellow,Grey );
	
	//初始化定时关机
	Timer_State = TM_10MIN;//TM_OFF;
	Ctrl_Timing_Device( Timer_State );
	LCD_Timing_Display( 120, 12 ,Timer_State);
	
	//初始化红光
	g_red_onoff = 0;
	if(g_adj_power._650_en == 1)
		LCD_RedLight_Show(9,15,g_red_onoff);
	
	//初始化电池电量
	g_batter_delay = -1;//立即检测，而不是100ms后
	ProChargerMonitor();
	
	TouchPanel_Calibrate();	//校准触摸屏
	LCD_Batter_Show(0,0,6/*LEVEL_4*/);//为了解决初始化时候电池判别次数不够，不显示电池量	
}
void Delay(uint32_t time)
{
  for (; time!= 0; time--);
}
/*
* Function: 打开电源
*/
void TurnOnPower()
{
	int i = 0,isDown = 0;
	
// 	//总电源使能，打开
 	GPIO_SetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);
	Delay_ms(10); 
	//开机按键是否按下
	i = 0;
	while(i++ < 100) {//用户长按800ms，每10ms去抖动
		Delay_ms(10);
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 1) {
			isDown = 1;
		}
		else {
			isDown = 0;
			break;
		}
	}
	
	//if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 1) 
	if(isDown)
	{ 		
	}
	else {
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);//关闭总电源
		powerDownDelayCnt = 0;
		i = 0;
		while(i++<10) {
			printf("power off\n");
			Delay_ms(1000);
		}
	}
}

/*
* Function: 关机
*/
void TurnOffPower()
{
	//GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);
	//关闭液晶屏，提示用户关机成
	if(powerDownDelayCnt >= 6) {
		g_power_down = 1;
		g_red_mode = 0;
		Ctrl_RedLight(0);
		LCD_Clear(Black);
		Delay_ms(1000);
		LCD_SetBacklight(0x03);		
		//TODO :关闭LCD背光和电源
		GPIO_SetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//关显示屏操作 
// 		delayMs(10);//延迟一小段时间，等待操作者松开按键
// 		Delay_ms(10
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //关机 CHECK2置0 清标志位
		
	}	
}


/*****************************************************************************
响应用户操作部分
6个按键，按键布局
X  Y  Z
A  B  C
*****************************************************************************/
/*
* Function: 响应波长设置
*/
void UI_ProWavelength()
{
	int tmpWave;
	
	if(KeyDown(GPIOA,KEY_B)) {
		int wave[3] = {WL_1310,WL_1490,WL_1550};
		int i,find;
		static int index = 0;

		
		if(g_adj_power._1310_en == 0) {
			wave[0] = 0;
		}
		if(g_adj_power._1490_en == 0) {
			wave[1] = 0;
		}
		if(g_adj_power._1550_en == 0) {
			wave[2] = 0;
		}
		
		find = 0;
		for(i = index+1;i < 3;++i) {
			if(wave[i] != 0 && wave[i] != Wavelength_Selection_state) {
				Wavelength_Selection_state = wave[i];
				index = i;
				find = 1;
				break;
			}
		}
		if(find == 0) {
			for(i = 0;i < index;++i) {
				if(wave[i] != 0 && wave[i] != Wavelength_Selection_state) {
					Wavelength_Selection_state = wave[i];
					index = i;
					find = 1;
					break;
				}
			}
		}

		if(find == 0 || index >= 3)
			index = -1;		
		
		Ctrl_Operating_Mode( Operating_Mode);
		Ctrl_Wavelength( Wavelength_Selection_state); //Wavelength_Selection_state   0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = 红光
		LCD_Wavelength_Selection_Ex( 170,200 ,Wavelength_Selection_state ,Yellow, Grey );	//波长显示切换
		Ctrl_Power(&g_power);
	}
}

/*
* Function: 响应功率控制
*/
void UI_ProPower()
{
	uint8_t flag = 0;
	if(KeyPress(GPIOA,KEY_C)) {
		flag = 1;
		g_power.set -= 1000;
	}
	//调整功率大小++
	else if(KeyPress(GPIOA,KEY_Z)) {
		flag = 1;
		g_power.set += 1000;
	}	
	if(flag) {
		if((int32_t)g_power.set > 0) {
			g_power.set = 0;
		}
		else if((int32_t)g_power.set < -10000) {
			g_power.set = (uint32_t)(-10000);
		}
		Ctrl_Power(&g_power);
	}
}


/*
* Function: 响应模式设置
*/
void UI_ProMode()
{
	if(KeyDown(GPIOA,KEY_Y)) {
		Operating_Mode++;
		if(Operating_Mode > 3)
			Operating_Mode = 0;
		Ctrl_Operating_Mode( Operating_Mode);	     //Operating_Mode  0 = CW、 1 = 270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
		LCD_OperatMode_Selection( 15, 200,Operating_Mode, Yellow,Grey );	//工作模式显示切换
		switch(Operating_Mode) {
		case 0:
			debugtxt(0,100-24,"cw ",-1);
			break;
		case 1:
			debugtxt(0,100-24,"270",-1);
			break;
		case 2:
			debugtxt(0,100-24,"1k ",-1);
			break;
		case 3:
			debugtxt(0,100-24,"2k ",-1);
			break;
		}
		Ctrl_Power(&g_power);
		if(Operating_Mode == 0 && Timer_State == TM_OFF) {
			hackflag = 1;
			hackval = g_power.set;
		}
	}	
}


/*
* Function: 响应红光输出和定时关机设置
*/
void UI_ProRedLight_ShutdownTimer()
{
	int i;
	i = KeyDown_Ex(GPIOA,KEY_A);
	
	
	//if(KeyDown_Ex(GPIOA,KEY_A,1000)){//g_key_timer_100ms > 10) {
	if(i >= 10)
	{
		g_key_timer_100ms = 0;
		if(Timer_State == TM_10MIN)
			Timer_State = TM_OFF;
		else
			Timer_State = TM_10MIN;
		Ctrl_Timing_Device(Timer_State);
		LCD_Timing_Display( 120, 12 ,Timer_State);
		while(GPIO_ReadInputDataBit(GPIOA,KEY_A) == 0);
	}
	else if(i >0 && i < 6 && g_adj_power._650_en == 1)
	{
		g_red_mode++;
		if(g_red_mode >= 3)
			g_red_mode = 0;
		if(g_red_mode == 0)
			g_red_onoff = 0;
		else if(g_red_mode == 1)
			g_red_onoff = 1;
		else if(g_red_mode == 2)
			g_red_onoff = 0;
		Ctrl_RedLight(g_red_onoff);
		LCD_RedLight_Show(9,15,g_red_onoff);
	}
}
// double atof_(char* s)
// {
// 	double v=0,k = 0,j = 1;
// 	int sign=1;
// 	while ( *s == ' '  ||  (unsigned int)(*s - 9) < 5u) s++;
// 	switch (*s)
// 	{
// 	case '-':
// 		sign=-1;
// 	case '+':
// 		++s;
// 	}
// 	while ((unsigned int) (*s - '0') < 10u)
// 	{
// 		v=v*10+*s-'0';
// 		++s;
// 	}
// 	if(*s == '.') {
// 		s++;
// 		while ((unsigned int) (*s - '0') < 10u)
// 		{
// 			k=k*10+*s-'0';
// 			++s;
// 			j = j * 10;
// 		}
// 		k /= j;
// 	}
// 	
// 	v = v + k;
// 	return sign==-1?-v:v;
// }
/*
* Function: 响应后门按键过程
*/

void UI_ProductionAdjust()
{
	uint16_t x,y;
	uint8_t flag = 0;
	uint8_t light = 0xff;
	int32_t strlen;
	uint32_t btnClk = 0;
	struct point loc[10] = {
		(4)*8,(56),(4+10)*8,(56),(4+10+10)*8,(56),(4+10+10+10)*8,(56),
		(4)*8,(56+24),(4+10)*8,(56+24),(4+10+10)*8,(56+24),(4+10+10+10)*8,(56+24),
		20,56+24+24,
		20,56+24+24+24,
	};
	float adjval[8];
	int8_t index = 0,last_index;
	last_index = index;
	
	

	adjval[0] = (float)g_adj_power._1310cw;
	adjval[1] = (float)g_adj_power._1310_270;
	adjval[2] = (float)g_adj_power._1310_1k;
	adjval[3] = (float)g_adj_power._1310_2k;
	
	adjval[4] = (float)g_adj_power._1550cw;
	adjval[5] = (float)g_adj_power._1550_270;
	adjval[6] = (float)g_adj_power._1550_1k;
	adjval[7] = (float)g_adj_power._1550_2k;
	
_Redraw:;
	for( x=0; x < 320; x++ )		//上边界蓝条宽34  下边界蓝条宽49
		for( y=0; y < 240; y++ )
			;//gl_setpoint(x,y,0x22f2);	//屏幕主色调gray
			
	//LCD_Clear(0x22f2);
	gl_clear(0,0,320,240,COL_White);
	gl_text((4)*8,(56-24),"ADC",-1);
	gl_text((4+10)*8,(56-24),"DAC",-1);
	gl_text((4+10+10)*8,(56-24),"--",-1);
	gl_text((4+10+10+10)*8,(56-24),"--",-1);
	
	gl_text(0,(56),"-10",-1);
	sprintf(strout,"%6.2f",(float)(adjval[0]));
	gl_text((4)*8,(56),strout,-1);
	sprintf(strout,"%6.2f",(float)adjval[1]);
	gl_text((4+10)*8,(56),strout,-1);
	sprintf(strout,"%6.2f",(float)adjval[2]);
	gl_text((4+10+10)*8,(56),strout,-1);
	sprintf(strout,"%6.2f",(float)adjval[3]);
	gl_text((4+10+10+10)*8,(56),strout,-1);
	
	gl_text(0,(56+24),"---",-1);
	sprintf(strout,"%6.2f",(float)adjval[4]);
	gl_text((4)*8,(56+24),strout,-1);
	sprintf(strout,"%6.2f",(float)adjval[5]);
	gl_text((4+10)*8,(56+24),strout,-1);
	sprintf(strout,"%6.2f",(float)adjval[6]);
	gl_text((4+10+10)*8,(56+24),strout,-1);
	sprintf(strout,"%6.2f",(float)adjval[7]);
	gl_text((4+10+10+10)*8,(56+24),strout,-1);
	
	gl_text(20,56+24+24,"save",-1);
	gl_text(20,56+24+24+24,"exit",-1);
	
	DrawFocus(loc[index].x,loc[index].y,COL_Black);//RGB16(255,255,0));
	while(1) {
		if(ProTick1963IsLive())
			;
		ProGet1963State();
		TurnOffPower();
		//if(ProTick1963IsLive()) 
		{
			;//goto _Redraw;
		}
		if(KeyPress(GPIOA,KEY_A)) {
			flag = 1;
			index--;
		}
		else if(KeyPress(GPIOA,KEY_C)) {
			flag = 1;
			index++;
		}
		else if(KeyPress(GPIOA,KEY_C)) {
			flag = 1;
			//adjval[index] -= 0.01;
		}
// 		else if(KeyPress(GPIOA,KEY_Z)) {
// 			flag = 1;
// 			btnClk = 1;
// 			//while(KeyPress(GPIOA,KEY_Z));
// 			//adjval[index] += 0.01;
// 			
// 		}
		else if(KeyPress(GPIOA,KEY_Z)) {
			flag = 1;
			btnClk = 1;
			if(index < 8) {
				//adjval[index] = 0;	
				
			}
			else if(index == 8) {
				g_adj_power.flag = 0xaabbccdd;
		
				g_adj_power._1310cw   = adjval[0];
				g_adj_power._1310_270 = adjval[1];
				g_adj_power._1310_1k  = adjval[2];
				g_adj_power._1310_2k  = adjval[3];

				g_adj_power._1550cw   = adjval[4];
				g_adj_power._1550_270 = adjval[5];
				g_adj_power._1550_1k  = adjval[6];
				g_adj_power._1550_2k  = adjval[7];
				
				WriteFlash(FLASH_PAGE_START,
						(uint32_t*)&(g_adj_power),
						sizeof(struct adj_power_flash));
				//gl_text(0,0,"save",-1);
				gl_text(20,56+24+24,"save...",-1);
				Delay_ms(1000);
				gl_text(20,56+24+24,"       ",-1);
				gl_text(20,56+24+24,"save",-1);
			}
			else if(index == 9) {
				break;
			}
				
			
		}
		if(flag) {
			//ProTick1963IsLive();
			
			if(index >= 10)
				index = 0;
			else if(index < 0)
				index = 9;
			if(btnClk && index < 8) {
				
				//sprintf(strout,"%6.2f",adjval[index]);
				*strout = '\0';
				if(InputPanel(strout,6,&strlen)) {
					adjval[index] = atof_(strout);
				}
				
				gl_text(loc[index].x,loc[index].y,strout,-1);
				btnClk = 0;
				goto _Redraw;
				
			}
			else if(btnClk && index == 9) {
				break;
			}
// 			gl_fill_rect(loc[last_index].x,loc[last_index].y - 5,5,5);
// 			gl_fill_rect(loc[index].x,loc[index].y - 5,5,5);
			DrawFocus(loc[last_index].x,loc[last_index].y,COL_White);//RGB16(38,93,150));
			DrawFocus(loc[index].x,loc[index].y,COL_Black);//RGB16(255,255,0));
			last_index = index;
			
			
			flag = 0;
			btnClk = 0;
		}
		/*
			
		}*/
		
	}
}

/*
* Function: 重新绘制用户界面
* Note：主要是为处理LCD重启后的绘制，理想状态下不会使用到它
*/
void UI_ProRedraw()
{
	LCD_Clear(Black);	
	LCD_Timing_Display( 120, 12 ,Timer_State);
	LCD_RedLight_Show(9,15,g_red_onoff);
	LCD_Batter_Show(0,0,4);
	LCD_Power_Control_Selection_Ex(55,90,((uint16_t)((int32_t)g_power.set / 1000)),White, Grey);	
	LCD_Wavelength_Selection_Ex( 170,200 ,Wavelength_Selection_state ,Yellow, Grey );	//波长显示切换
	LCD_OperatMode_Selection( 15, 200,Operating_Mode, Yellow,Grey );	//工作模式显示切换
}
int ProTick1963IsLive()
{
	if(g_lcdlisten_ms >= 100 && g_power_down == 0) {
		g_lcdlisten_ms = 0;
		//LCD_ReadReg(CMD_RD_MEMSTART,&data1);
		//if(data1 != 0x22f2 && data1 != 0xffe0) {
		if(SSD1963_IsRestart()) {
		
		
			printf("Restart LCD\n"); 
			LCD_Initializtion();	
			LCD_SetBacklight(0x80);
			//UI_ProRedraw();
			return 1;
		}
		
	}
	return 0;
}
#define RES1 0x0c
#define RES2 0x0f
#define RES3 0x3a
#define RES4 0xa8
#define RES5 0xe8
#define RES6 0xe9
#define RES7 0xff
void ProGet1963State()
{
	static uint8_t strout[70];
	uint16_t data1,data2,data3,data4;
	LCD_WriteCommand(0x00);
	if(g_usart_ms >= 1000) {
			g_usart_ms = 0;
		dprintf("\n*******************************************\n");
		LCD_WriteCommand(0xb9);
		data1 = LCD_ReadData();
		data2 = LCD_ReadData();
		dprintf("GPIO conf %x %x\n",data1,data2);
		//LCD_ReadReg(0xb9,&data1);
		//printf("GPIO conf %x\n",data1);
		
		LCD_ReadReg(RES1,&data1);
		LCD_ReadReg(RES2,&data2);
		LCD_ReadReg(RES3,&data3);
		LCD_ReadReg(RES4,&data4);
		dprintf("Reserved %x %x %x %x   ",data1,data2,data3,data4);
		LCD_ReadReg(RES5,&data1);
		LCD_ReadReg(RES6,&data2);
		LCD_ReadReg(RES7,&data3);
		dprintf("%x %x %x\n",data1,data2,data3);
		
		
		
		LCD_ReadReg(0X0A,&data1);
		LCD_ReadReg(0X0B,&data2);
		LCD_ReadReg(0X0D,&data2);
		LCD_ReadReg(0X0E,&data2);
		sprintf(strout,"Power Mode %x\tAddress Mode %x\tDisplay Mode %x\tEffect status %x\n",data1,data2,data3,data4);
		dprintf(strout);
		
		LCD_ReadReg(CMD_RD_MEMSTART,&data1);
		LCD_ReadReg(CMD_RD_MEM_AUTO,&data2);
		LCD_ReadReg(CMD_RD_DDB_START,&data3);
		LCD_ReadReg(CMD_GET_PANEL_MODE,&data4);
		sprintf(strout,"Mem start %x\tMem auto %x\tDDB start %x\tPanel mode %x\n",data1,data2,data3,data4);
		dprintf(strout);
		//gl_text(0,8,strout,-1);
		
		LCD_ReadReg(CMD_GET_HOR_PERIOD,&data1);
		LCD_ReadReg(CMD_GET_VER_PERIOD,&data2);
		sprintf(strout,"Hor %x\tVer %x\n",data1,data2);
		dprintf(strout);
		//gl_text(0,20,strout,-1);
		
			
		LCD_ReadReg(CMD_GET_GPIO_CONF,&data1);
		LCD_ReadReg(CMD_GET_GPIO_STATUS,&data2);
		sprintf(strout,"GPIO Conf %x\tStatus %x\n",data1,data2);
		dprintf(strout);
		//gl_text(0,32,strout+8,-1);
		
		LCD_ReadReg(CMD_GET_POST_PROC,&data1);
		LCD_ReadReg(CMD_GET_PWM_CONF,&data2);
		sprintf(strout,"Post %x\tPWM %x\n",data1,data2);
		dprintf(strout);
		//gl_text(0,44,strout,-1);
		
		
		LCD_ReadReg(CMD_GET_LCD_GEN0,&data1);
		LCD_ReadReg(CMD_GET_LCD_GEN1,&data2);
		LCD_ReadReg(CMD_GET_LCD_GEN2,&data3);
		LCD_ReadReg(CMD_GET_LCD_GEN3,&data4);
		sprintf(strout,"LCD GEN[0-3] %x %x %x %x\n",data1,data2,data3,data4);
		dprintf(strout);
		//gl_text(0,56,strout,-1);
		
		
		LCD_ReadReg(CMD_GET_GPIO0_ROP,&data1);
		LCD_ReadReg(CMD_GET_GPIO1_ROP,&data2);
		LCD_ReadReg(CMD_GET_GPIO2_ROP,&data3);
		LCD_ReadReg(CMD_GET_GPIO3_ROP,&data4);
		sprintf(strout,"GPIO[0-3] ROP %x %x %x %x\n",data1,data2,data3,data4);
		dprintf(strout);
		//gl_text(0,68,strout,-1);
		
		LCD_ReadReg(CMD_GET_ABC_DBC_CONF,&data1);
		LCD_ReadReg(CMD_GET_DBC_HISTO_PTR,&data2);
		LCD_ReadReg(CMD_GET_DBC_THRES,&data3);
		sprintf(strout,"DBC conf Histo ptr Thres %x %x %x\n",data1,data2,data3);
		dprintf(strout);
		//gl_text(0,80,strout,-1);
		
		LCD_ReadReg(CMD_GET_SIGNAL_MODE,&data1);
		sprintf(strout,"Tearing state %x\n",data1);
		dprintf(strout);
		//gl_text(0,92,strout,-1);
		
		dprintf("\n");
	}
}
/*
* Function: 绘制界面焦点
* Parameters:
	x,y坐标，color：RGB16(R,G,B)颜色
*/
void DrawFocus(int16_t x,int16_t y,uint32_t color)
{
	uint32_t brush,pen;
	
	brush = gl_ui_setbrushcolor(color);
	pen = gl_ui_setpencolor(color);
	//gl_fill_rect(x,y - 5,30,5);	
	gl_fill_rect(x-3,y,3,12);
	gl_ui_setbrushcolor(brush);
	gl_ui_setpencolor(pen);
}

/*
* Function: 成功进入校准界面
  后门-8,-7,-4
*/
void IsHacker()
{
	static uint8_t state = 0;
	
	//UI_ProductionAdjust();//debug;
	if(hackflag == 1) {
		hackflag = 0;
		switch(state) {
		case 0:
			if((int32_t)hackval == -9000) {
				state++;
				//UI_DebugMain();
			}
			else
				state = 0;
			break;
		case 1:
			if((int32_t)hackval == -8000)
				state++;
			else
				state = 0;
			break;
		case 2:
			if((int32_t)hackval == -7000)
				state++;
			else
				state = 0;
			break;
		case 3:
			if((int32_t)hackval == -4000) {
				state = 0;
				UI_DebugMain();
				LCD_DrawMain();
				//UI_ProductionAdjust();
				
			}
			else
				state = 0;
			break;
		default:
			state = 0;
			break;
		}
	}
}




/*****************************************************************************
检测模块
*****************************************************************************/
/*
* Function: 电源检测
* Remarks:划分电量等级，且低电量自动关机，关机前刷红色屏幕提示
//只有当前等级与上一次等级的插的绝对值>=2时才更新，电量检测不会再两个相邻等级里来回变动，
//另外也保证在LEVEL_POWER、LEVEL_FULL、LEVEL_CHARGE三个等级可以快速响应切换
//电池电量检测浮动在0.030V左右，当快没电的时候浮动可达0.100V
*/
//电池等级
#define LEVEL_POWER    0  //外部供电
#define LEVEL_FULL     2  //充满电
#define LEVEL_CHARGE   4  //正在充电
#define LEVEL_4        6  //电池4格
#define LEVEL_3        7  //电池3格
#define LEVEL_2        8  //电池2格
#define LEVEL_1        9  //电池1格
#define LEVEL_0        10  //电池0格
#define LEVEL_SHUTDOWN 11 //电池自动关机
/*int8_t disp_battery_index[12] = {
	//0  1  2  3  4  5  6  7  8  9  10 11其中1、3无用
	  4, 4, 4, 4, 4, 5, 4, 3, 2, 1, 0, 0
};*/
void ProChargerMonitor()
{
	uint8_t strout[60];
	static uint8_t times = 100;
	static uint16_t last_level = LEVEL_4,level_show = LEVEL_4;
	uint16_t level = LEVEL_4; 
	float vol,tvol;
	//static float last_vol = 100;
	int x,y;
	int ad;
	//return ;
	/*************************充电指示部分*******************************************************/
	if(g_batter_delay > 100) {
		g_batter_delay = 0;
		ad = GetAD(0);
		//vol = (float)GetAD(0)*0.00349+0.7;//Ref = 3.3
		
		vol = (float)ad*0.002643333+0.77;//Ref = 2.5,M7二极管压降0.77
		g_battery_vol = vol;
		sprintf(strout,"better:%0.3f %d %f",vol,ad,(float)ad * 0.00061);
		//debugtxt(0,12*6,strout,30);
		//gl_text(0,65,strout,30);
		
		//debug
		if((GPIO_ReadInputDataBit(GPIO_PORT_POWER_CHK, GPIO_CHARG_CHK)==0)) {//充满、只挂电池、只挂外加电源
			if(vol > 12.0) {//外加电源远远大于
				level = LEVEL_POWER;
				//debugtxt(0,100+12,"out side",20);
				//gl_text(0,100+12,"out side",20);
			}
			else if(vol > 11.4) {
				level = LEVEL_FULL;
				//debugtxt(0,100+12,"full    ",20);	
				//gl_text(0,100+12,"out side",20);
			}
			else if(vol > 7.6) {
				level = LEVEL_4;
				//debugtxt(0,100+12,"only batt",20);
				//gl_text(0,100+12,"out side",20);
			}
			//else if(7.60 < vol && vol < 7.97) {
			else if(vol > 7.4) {
				level = LEVEL_3;

				//debugtxt(0,100+12,"full    ",20);	
			}
			else if(vol > 7.1) {
				level = LEVEL_2;
			}
			//else if(6.80 <= vol && vol < 7.17) {
			else if(vol > 6.9) {
				level = LEVEL_1;
			}
			else if(vol > 6.7) {
				level = LEVEL_0;	
			}
			else if(vol < 6.7) {
				level = LEVEL_SHUTDOWN;

				LCD_Batter_Show(0,0,LEVEL_0);
				for( x=0; x < 320; x++ )		//屏幕刷红色，表示电量过低自动关机
					for( y=0; y < 240; y++ )
						gl_setpoint(x,y,RGB16(255,0,0));
				Delay_ms(2000);
				powerDownDelayCnt = 1100;//表示要关机了
				TurnOffPower();
				
			}
		}
		else if((GPIO_ReadInputDataBit(GPIO_PORT_POWER_CHK, GPIO_CHARG_CHK)==1)) {//正在充电
			level = LEVEL_CHARGE;
			//debugtxt(0,100+12,"charge...",20);	
		}
		
		
		//为了处理处理器内部AD转换不稳定（0.030V的浮动）而进行以下处理
		if(
			//只有连续多次电量检测级别相等
			//要么电量越来越低，要么是连接充电器
			(last_level == level &&
			 times++ >= 2 && 
			 (level > level_show || level - level_show <= -2 || level == level_show) )||
			//或者上一次电量检测是插入外电源的，这一次需要立即更新
			//BUG:LEVEL_POWER和LEVEL_FULL可能回来回转换，但是用户界面里这两个状态的图标是一样的，
			//所以不回被用户发现异常
			(level_show == LEVEL_CHARGE || level_show == LEVEL_POWER ||level_show == LEVEL_FULL)
			) {
			
			level_show = level;//上次显示的等级
			times = 0;
			LCD_Batter_Show(0,0,level_show);
			sprintf(strout,"better:%0.3f",vol);
			debugtxt(0,12*6,strout,30);			
		}
		if(last_level != level)
			times = 0;
		sprintf(strout,"state %d %d %d-",times,level_show,level);
		debugtxt(0,100+24,strout,-1);
		last_level = level;
	}
	return ;
}
float GetBattery()
{
	uint32_t ad;
	if(g_batter_delay > 100) {
		g_batter_delay = 0;
		ad = GetAD(0);
		//vol = (float)GetAD(0)*0.00349+0.7;//Ref = 3.3
		
		g_battery_vol = (float)ad*0.002643333+0.77;//Ref = 2.5,M7二极管压降0.77
	}
	return g_battery_vol;
}

/*
* Function: 响应定时关机
*/
void ProTimerShutdown()
{
	/***************************定时关机部分*************************************************************/
	//一般定时用，定时关机，可以设置为5min  ，10min  ，15min  ，30min  ，60min 状态
	//TIM2设置为100ms周期，3000 ， 6000  ， 9000  ， 18000 ，  36000		   

	switch(Timer_State)				
	{
	case   0:	  // 0 = 定时器关闭

		break;   
	case   1:	  // 1 = 5min定时
		if( Timer_Counter >=3000 )
		{ GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//关显示屏操作 
		//delayMs(10);//延迟一小段时间，等待操作者松开按键
		Delay_ms(10);
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //关机 CHECK2置0 清标志位
		Timer_State = 0;
		//LCD_Timing_Display( 255, 12 ,Timer_State );	  //定时器显示切换
		}
		break;
	case   2:	  // 1 = 1Omin
		if( Timer_Counter >=6000 )
		{ GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//关显示屏操作 
		//delayMs(10);//延迟一小段时间，等待操作者松开按键
		Delay_ms(10);
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //关机 CHECK2置0 清标志位
		Timer_State = 0;
		//LCD_Timing_Display( 255, 12 ,Timer_State );	  //定时器显示切换
		}
		break;
	case   3:	  // 1 = 15min
		if( Timer_Counter >=9000 )
		{ GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//关显示屏操作 
		//delayMs(10);//延迟一小段时间，等待操作者松开按键
		Delay_ms(10);
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //关机 CHECK2置0 清标志位
		Timer_State = 0;
		//LCD_Timing_Display( 255, 12 ,Timer_State );	  //定时器显示切换
		}
		break;
	case   4:	  // 1 = 30min
		if( Timer_Counter >=18000 )
		{ GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//关显示屏操作 
		//delayMs(10);//延迟一小段时间，等待操作者松开按键
		Delay_ms(10);
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //关机 CHECK2置0 清标志位
		Timer_State = 0;
		//LCD_Timing_Display( 255, 12 ,Timer_State );	  //定时器显示切换
		}
		break;
	case   5:	  // 1 = 60min
		if( Timer_Counter >=36000 )
		{ GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//关显示屏操作 
		//delayMs(10);//延迟一小段时间，等待操作者松开按键
		Delay_ms(10);
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //关机 CHECK2置0 清标志位
		Timer_State = 0;
		//LCD_Timing_Display( 255, 12 ,Timer_State );	  //定时器显示切换
		}
		break;		 		
	default:
		break;
	}	
}
int restarttimes = 100;
void Check1963()
{
	uint8_t strout[30];
	uint16_t data;
	int times = 0;	
	
	printf("Checking chip...");
	LCD_ReadReg(CMD_RD_MEMSTART,&data);
	if(data != 0x22f2) {
		while(times++ < 3) {
			LCD_ReadReg(CMD_RD_MEMSTART,&data);
			if(data != 0x22f2) {
				printf(".");
				LCD_Initializtion();	
				Delay_ms(100);
			}
		}
		g_usart_ms = 2000;
		printf("\n");
		//多次启动LCD失败，关闭系统
		LCD_ReadReg(CMD_RD_MEMSTART,&data);
		if(data != 0x22f2) {
			printf("Startup chip error!\n\n");
			ProGet1963State();
// 			powerDownDelayCnt = 1000;
// 			while(1) {
// 				printf("Shut down!!!\n");
// 				TurnOffPower();
// 				LCD_Initializtion();
// // 				LCD_ReadReg(CMD_RD_MEMSTART,&data);
// // 				if(data == 0x22f2) {	
// // 					break;
// // 				}
// 				Delay_ms(30);
// 			}
		}
	}
	restarttimes = times;
	printf("\nStartup chip success!\n");
	LCD_SetBacklight(0x80);
}
void CheckLCD()
{
	uint16_t data;
	printf("Checking LCD...\n");
	LCD_SetPoint(15,15,0xaabb);
	Delay_ms(1);
	data = LCD_GetPoint(15,15);
	if(data != 0xaabb) {
		printf("Open LCD error!Please check whether the LCD  connect!\n");
	}
	else {
		printf("Open LCD success\n");
	}
	
}
/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
//main()
int main(void)
{
	USART_Configuration();
	
	printf("\n\n----------------------------------------------------------------------------\n");
	printf("        GLink TS100 Runing\n");
#define SYSCLK_FREQ_24MHz
	SystemInit();//SetSysClock();
	//液晶屏GUI配置
	gl_ui_setlib(p_zm_ascii_st9,8,12,p_zm_step_st9);
	gl_ui_setbkmode(BACKFILL);//背景填充模式
	gl_ui_setfontcolor(RGB16(255,0,0));
	gl_ui_setbkcolor(RGB16(0,255,0));
	gl_ui_setpencolor(RGB16(235,34,209));
	
	delay_init();
	/*****************按键中断初始化部分*********************** */
	Function_IO_config(); 
	//RedLightIOConfig();
	
	//各个定时中断
	TIM2_Init();//1MS定时
	TIM6_Init();//1MS定时
	
	TIM3_Init( TIM_Period1310);
	TIM4_Init( TIM_Period1490);
	TIM5_Init( TIM_Period1550);
	NVIC_Configuration();
	//串口通信
	

	//
	ADC_Configuration();//dma mode
	DAC_Configuration();
	
	printf("Power On\n");
	TurnOnPower();
	LCD_Initializtion();	
	LCD_Clear(Black);		
	Check1963();
	CheckLCD();
	//LCD_SetBacklight(0x70);
	LCD_SetBacklight(0x80);
	
	FLASH_Configuration();
	printf("Draw UI\n");
	LCD_DrawMain();

	powerDownDelayCnt=0;
	g_batter_delay = 10001;	
	
//   	while(1)
//   		UI_DebugMain();
	//InputPanel(strout,50,0);
	sprintf(strout,"res times %d",restarttimes);
	//printf("%s\n",strout);

	while(1)
	{
 		ProGet1963State();
 		if(ProTick1963IsLive())
 			UI_ProRedraw();
		
		UI_ProRedLight_ShutdownTimer();//
		UI_ProWavelength();
		UI_ProPower();
		UI_ProMode();
		TurnOffPower();
		ProChargerMonitor();
		ProTimerShutdown();//定时关机
		//if(Operating_Mode != OPM_270) {
			AutoCtrlPower();
		
 		IsHacker();		
	}//while结尾
}

/*
* Function: 调试查看，正式版没有
*/
extern uint8_t msgindex;
void DebugLookAD()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	int16_t ad = 200;
	char strout[30];//,times = 0;
	static uint16_t flag = 0;
	float vol;//scale;
	
	gl_key_init();
	g_power.set = (uint32_t)(-10000);
	Ctrl_Power(&g_power);
	flag = 1;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
 	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz ;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	while(1) {
		TurnOffPower();
// 		if(gl_key_down(6)) {
// 			//sprintf(strout,"key %d",times++);
// 			sprintf(strout,"key %d",msgindex);
// 			gl_text(125,0,strout,-1);
// 		}
// 		continue;
// 		UI_ProMode();
// 		UI_ProWavelength();
		
		if(KeyPress(GPIOA,KEY_C)) {
			ad-= 20;
			g_power.set += 100;
			flag = 1;
		}
		else if(KeyPress(GPIOA,KEY_Z)) {
			ad+= 20;
			g_power.set -= 100;
			flag = 1;
		}
// 		else if(KeyPress(GPIOA,KEY_A)) {
// 			ad-= 100;
// 			flag = 1;
// 		}
// 		else if(KeyPress(GPIOA,KEY_X)) {
// 			ad+= 100;
// 			flag = 1;
// 		}
		if(flag) {
			flag = 0;
			if(ad > 4095)
				ad = 0;
			else if(ad < 0)
				ad = 4095;
			//DAC_SoftwareTriggerCmd(DAC_Channel_1,DISABLE);  
			DAC_SetChannel1Data(DAC_Align_12b_R, ad/*ad*/);
			DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);  			
			
			//DAC_SoftwareTriggerCmd(DAC_Channel_2,DISABLE);  
			DAC_SetChannel1Data(DAC_Align_12b_R, ad);
			DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);  			
			sprintf(strout,"debug set adc:%4.4d",ad);
			//Ctrl_Power(&g_power);
			
			gl_text(0,100-12,strout,20);
		}
		continue;
		//AutoCtrlPower();
		UI_ProRedLight_ShutdownTimer();
		if(g_ad_ms > 2000) {
			
	/* Enable GPIOD clock */

// 			if(flag) {
// 				flag = 0;
// 				GPIO_SetBits(GPIOC,GPIO_Pin_0);
// 			}
// 			else {
// 				flag = 1;
// 				GPIO_ResetBits(GPIOC,GPIO_Pin_0);
// 			}
				
			g_ad_ms = 0;
			//sprintf(strout,"adc %4.4d",GetAD(1));
			
			vol = (float)GetAD(0)*0.00349+0.7;
			sprintf(strout,"ad0 %0.3f",vol);
			gl_text(0,200,(uint8_t*)strout,20);
			
			vol = (float)GetAD(1)*0.000805;
			//vol = (float)GetAD(1)*0.00349+0.7;
			sprintf(strout,"ad1 %3.3f",vol);
			gl_text(0,200+12,(uint8_t*)strout,20);
			ProChargerMonitor();
		}
		
	}
}
