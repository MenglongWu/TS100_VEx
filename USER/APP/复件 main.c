/****************************************Copyright (c)****************************************************                                    
**                                 http://www.powermcu.com
**--------------File Info---------------------------------------------------------------------------------
** File name:               main.c
** Descriptions:            The TouchPanel application function
**--------------------------------------------------------------------------------------------------------
** Created by:              G-LINK GROP
** Created date:            2011-12-07	flash OK�汾   
** Version:                 v1.0
** Descriptions:            The original version
**--------------------------------------------------------------------------------------------------------
** Modified by:			Yu Jingxiong   2011.11.10
V1.0    yujignxiong   2011.11.27
** Modified date:		�����Ķ�����ע������⡣������keep moving����          
** Version:                 
** Descriptions:     TIM_CtrlPWMOutputs(TIM3,ENABLE);       
**




	2012.11.02 -- 2012.11.14 ����   ������
1����ɹ����Զ�����
2������У׼��̨
3��AD�����ĳ�DMA��ʽ
4��ȡ������������
5�����Ƶ�Դ���

�ļ��ṹ
	main.c 				������
	PictureData.c 		16λ��ɫͼƬ��Դ
	gl_ui.c 			ͼ�νӿ�
	zimo_st9.c 			����ascii������9������
	key.c   			����ɨ��
	flash.c  			�ڲ�flash��д
	usart.c   			�ⲿ
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h" 
#include "stm32_eval.h" 
#include "TouchPanel.h"
#include "systick.h"
#include "GLCD.h"
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
#include <PictureData.h>

/*****************************************************************************
�ҵĶ���
*****************************************************************************/
#ifndef _DEBUG_
#define debugtxt
#else 
#define debugtxt gl_text
#endif

void USART_Configuration();
void delayMs(uint16_t ms);
void ADC_Configuration();
uint16_t GetAD(uint8_t chx);
void DAC_Configuration(void);
void TIM2_Init(void);
void TIM3_Init(uint16_t);
void TIM4_Init(uint16_t);
void TIM5_Init(uint16_t);
void TIM6_Init(void);
void NVIC_Configuration(void);
void FLASH_Configuration();
float DbmToScale(float dbm);
void Ctrl_RedLight(u8 v);
void Ctrl_Wavelength(u8 Wavelength_Selection_state);
void Ctrl_Operating_Mode(u8 Operating_Mode);
void Ctrl_Power(struct ctrl_param *v);
void AutoCtrlPower();
void Ctrl_Timing_Device(u8 Timer);
void Function_IO_config(void);
void RedLightIOConfig();
void External_Interrupt_Config(void);
void External_Interrupt_EXIT_Init(void);
void External_Interrupt_InterruptConfig(void);
void delayUs(vu32 cnt);
void LCD_DrawMain(void);
void TurnOnPower();
void TurnOffPower();
void UI_ProWavelength();
void UI_ProPower();
void UI_ProMode();
void UI_ProRedLight_ShutdownTimer();
void ProChargerMonitor();
void ProTimerShutdown();
int main(void);
void DebugLookAD();
void UI_ProductionAdjust();
void DrawFocus(int16_t x,int16_t y,uint32_t color);
void IsHacker();



struct ctrl_param g_power;//��������
struct adj_power_flash g_adj_power;//У׼����
volatile u16 powerDownDelayCnt=0;//�ػ�������ʱ
uint32_t hackval = 0;//�����̨���������
uint32_t hackflag = 0;//�Ƿ����������־
volatile u8 g_red_onoff = 0;//
volatile int8_t g_red_mode = 0;//�����ʾģʽ����������˸���ر�
volatile u8 g_red_delay_100ms = 0;//�����˸0.5s��ʱ
volatile u8 g_onoff_en = 0;//ʹ�ܹػ�����ֹ�������ٹػ�

volatile u8 g_key_timer_100ms = 0;//������ʱ���������жϳ����º͵��������ú�ƺͶ�ʱ�ػ�����
volatile uint16_t g_batter_delay = 0;//���ˢ����ʾ��ʱ
/////////////////////
volatile uint16_t g_ad_ms = 0,g_adjust_ms = 0,g_lcdbug_ms = 0;//ad�������
volatile uint16_t g_adc[200];//ad������
volatile uint16_t ADCConvertedValue[2000];//AD����DMA����

int8_t g_recvflag = 0;//���ڽ��ձ�־


/*****************************************************************************
ԭ���Ķ���
*****************************************************************************/
volatile u16 SysTickCounter=0;
volatile u16 Timer_State = 0;	    //��ʱ��״ָ̬ʾ,  OFF,  5min  ��10min  ��15min  ��30min  ��60min ״̬
volatile u16 Timer_Counter = 0;
volatile u16 Wavelength_Selection_state = 0;	 //0 = �ر�״̬ 1 = 1310nm 2 = 1495nm 3 = 1550nm 	4 = ���
volatile u16 Operating_Mode = 0;	//0 = CW�� 1 = PW270Hz��2 = 1KHz��3 = 2KHz  ������/�����ѡ��
volatile u8  Batter_Lightning=0;
volatile u8  LCD_GetPoint_EN = 1;   //��ֹ�������ظ��������൱�ڰ���ȥ����,���Խ��д������ɼ���
volatile u8  LCD_GetPoint_Counter = 0; //������ʹ�ܼ���ֵ����

volatile u8 FLAG_1310 = 0;		  //��ʱ���ж������巭ת��־��
volatile u8 FLAG_1490 = 0;
volatile u8 FLAG_1550 = 0;

uint16_t  TIM_Period[3] = {132,35,17};	//����ֵΪ133��36��18���ֱ��Ӧ����ֵΪ541hz��2Khz��4Khz����ӦPWMƵ��
//��Ƶϵ��ͳһΪ1000��Ƶ,0����رչ�Դ��1����������Դ����

volatile uint16_t  TIM_Period1310 = 132;	 //��ʼ������ֵ������󣬶�ӦƵ�����
volatile uint16_t  TIM_Period1490 = 132;
volatile uint16_t  TIM_Period1550 = 35;

//����ʹ��
uint16_t X[3]={889,960,3320};
uint16_t Y[3]={487,3352,1982};
/**************************************************************************************/

#define DAC_DHR12RD_Address   ((uint32_t)0x40007420)
#define ADC2_DR_Address ((uint32_t)0x40013C4C)

/*********************************���ܿ��ƶ˿�*****************************************/
#define RCC_GPIO_CTRL_B 			RCC_APB2Periph_GPIOB
#define GPIO_CTRL_PORT_B 			GPIOB
#define GPIO_SYSPWR_ONOFF 				       GPIO_Pin_6    //���ػ����ƣ�CHECK2�� ������1���ػ���0 

#define RCC_GPIO_CTRL_C 			RCC_APB2Periph_GPIOC	 //��·ѡ���ҿ������ܶο��ƿڣ���ʱ���ж���Ӧ������ѡ��+Ƶ�ʿ���
#define GPIO_CTRL_PORT_C 			GPIOC
#define GPIO_PORT_POWER_CHK 			GPIOA
#define GPIO_KEY_RED_CON 					   GPIO_Pin_0
#define GPIO_KEY_1310_CON 					   GPIO_Pin_3	 //��Ӧ�ܽŴ���2011.11.25�����޸�
#define GPIO_KEY_1490_CON 					   GPIO_Pin_2
#define GPIO_KEY_1550_CON 					   GPIO_Pin_1
#define GPIO_CHARG_CHK 					       GPIO_Pin_8
#define Control_KEYS				GPIO_KEY_1310_CON | GPIO_KEY_1490_CON | GPIO_KEY_1550_CON | GPIO_CHARG_CHK ;

#define RCC_GPIO_CTRL_D 			RCC_APB2Periph_GPIOD 
#define GPIO_CTRL_PORT_D 			GPIOD
#define GPIO_LCD_OFF					       GPIO_Pin_7    //��LCD�����ƣ�ON/OFF��  LCDʹ�ܶ˿ڣ��õ�Ƭѡ 

/*********************************�жϿ��ƶ˿�*****************************************/
#define RCC_GPIO_CTRL_A 			RCC_APB2Periph_GPIOA 
#define GPIO_CTRL_PORT_A 			GPIOA

#define GPIO_ONOFF_CHK 			               GPIO_Pin_1  //�ֶ����ػ�,��ͣ���CHECK1,PD2,�������ж�
#define GPIO_OperatingMode_CHK			       GPIO_Pin_5  //ģʽѡ�񰴼����������ж�
#define GPIO_TimingSet_CHK			           GPIO_Pin_3  //��ʱѡ�񰴼����������ж�
#define GPIO_PowerUp_CHK			           GPIO_Pin_2  //���� + ѡ�񰴼����������ж�
#define GPIO_PowerDown_CHK			           GPIO_Pin_6  //���� - ѡ�񰴼����������ж�
#define GPIO_WaveSelection_CHK			       GPIO_Pin_7  //����ѡ�񰴼����������ж�  



/*�ֶ����ػ����CHECK1,�������ж� PA1*/
#define GPIO_ONOFF_CHK_EXTI_LINE                            EXTI_Line1
#define GPIO_ONOFF_CHK_EXTI_PORT_SOURCE                     GPIO_PortSourceGPIOA
#define GPIO_ONOFF_CHK_PIN_SOURCE                           GPIO_PinSource1
#define GPIO_ONOFF_CHK_IRQn                                 EXTI1_IRQn 

/*����ѡ�񰴼���PA7���������ж�*/
#define GPIO_WaveSelection_CHK_EXTI_LINE                    EXTI_Line7
#define GPIO_WaveSelection_CHK_EXTI_PORT_SOURCE             GPIO_PortSourceGPIOA
#define GPIO_WaveSelection_CHK_PIN_SOURCE                   GPIO_PinSource7
#define GPIO_WaveSelection_CHK_IRQn                         EXTI9_5_IRQn 

/*�������ģʽѡ�񰴼���PA5���������ж�*/
#define GPIO_OperatingMode_CHK_EXTI_LINE                    EXTI_Line5
#define GPIO_OperatingMode_CHK_EXTI_PORT_SOURCE             GPIO_PortSourceGPIOA
#define GPIO_OperatingMode_CHK_PIN_SOURCE                   GPIO_PinSource5
#define GPIO_OperatingMode_CHK_IRQn                         EXTI9_5_IRQn 

/*�������  �� �� ѡ�񰴼���PA2���������ж�*/
#define GPIO_PowerUp_CHK_EXTI_LINE                          EXTI_Line2
#define GPIO_PowerUp_CHK_EXTI_PORT_SOURCE                   GPIO_PortSourceGPIOA
#define GPIO_PowerUp_CHK_PIN_SOURCE                         GPIO_PinSource2
#define GPIO_PowerUp_CHK_IRQn                               EXTI2_IRQn

/*�������  �� �� ѡ�񰴼���PA6���������ж�*/
#define GPIO_PowerDown_CHK_EXTI_LINE                        EXTI_Line6
#define GPIO_PowerDown_CHK_EXTI_PORT_SOURCE                 GPIO_PortSourceGPIOA
#define GPIO_PowerDown_CHK_PIN_SOURCE                       GPIO_PinSource6
#define GPIO_PowerDown_CHK_IRQn                             EXTI9_5_IRQn

/*TimingSetѡ�񰴼���PA3���������ж�*/
#define GPIO_TimingSet_CHK_EXTI_LINE                        EXTI_Line3
#define GPIO_TimingSet_CHK_EXTI_PORT_SOURCE                 GPIO_PortSourceGPIOA
#define GPIO_TimingSet_CHK_PIN_SOURCE                       GPIO_PinSource3
#define GPIO_TimingSet_CHK_IRQn                             EXTI3_IRQn

/*����*/
#define KEY_A GPIO_Pin_5
#define KEY_B GPIO_Pin_3
#define KEY_C GPIO_Pin_2
#define KEY_X GPIO_Pin_1
#define KEY_Y GPIO_Pin_7
#define KEY_Z GPIO_Pin_6


/*****************************************************************************
����ģ������
*****************************************************************************/
/*****************һ�㹦��IO�ڳ�ʼ��******************************************/
void Function_IO_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//��Դ���˿�CHECK1(KEY_X)��CHECK2
	
	//����KEY_A,KEY_b,KEY_B,KEY_X,KEY_Y,KEY_Z
	
	
	
	
	
	
	
	/* Enable GPIOD clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); 
	
	
	RCC_APB2PeriphClockCmd(RCC_GPIO_CTRL_B | RCC_GPIO_CTRL_C | RCC_GPIO_CTRL_D |RCC_APB2Periph_AFIO , ENABLE);    
	/* Control pins ���ػ� CHECK2 configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_SYSPWR_ONOFF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_CTRL_PORT_B, &GPIO_InitStructure);

	/* Control pinsѡ�񿪹ؿ��ƣ����ָʾ configuration */
	GPIO_InitStructure.GPIO_Pin = Control_KEYS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_CTRL_PORT_C, &GPIO_InitStructure);

	/* Control pinsҺ����Ƭѡ���� configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_LCD_OFF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_CTRL_PORT_D, &GPIO_InitStructure);
	
	
	////////////////
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Configure CHECK1 Button �������ػ� */
	RCC_APB2PeriphClockCmd(RCC_GPIO_CTRL_A, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = GPIO_ONOFF_CHK;
	GPIO_Init(GPIO_CTRL_PORT_A, &GPIO_InitStructure);

	/* Configure CHECK3  ����ѡ�񰴼��ж� */
	RCC_APB2PeriphClockCmd(RCC_GPIO_CTRL_A, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_WaveSelection_CHK;
	GPIO_Init(GPIO_CTRL_PORT_A, &GPIO_InitStructure);

	/* Configure CHECK4  ģʽѡ�񰴼��ж� */
	RCC_APB2PeriphClockCmd(RCC_GPIO_CTRL_A, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_OperatingMode_CHK;
	GPIO_Init(GPIO_CTRL_PORT_A, &GPIO_InitStructure);  

	/* Configure CHECK4  ���� ���� ѡ�񰴼��ж� */
	RCC_APB2PeriphClockCmd(RCC_GPIO_CTRL_A, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_PowerUp_CHK;
	GPIO_Init(GPIO_CTRL_PORT_A, &GPIO_InitStructure);

	/* Configure CHECK4  ���� �ݼ� ѡ�񰴼��ж� */
	RCC_APB2PeriphClockCmd(RCC_GPIO_CTRL_A, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_PowerDown_CHK;
	GPIO_Init(GPIO_CTRL_PORT_A, &GPIO_InitStructure);

	/* Configure CHECK4  TimingSetѡ�񰴼��ж� */
	RCC_APB2PeriphClockCmd(RCC_GPIO_CTRL_A, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_TimingSet_CHK;
	GPIO_Init(GPIO_CTRL_PORT_A, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}
/*
*Function:���˿�����
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
	uint32_t IntDeviceSeriNum[3];	

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
void delayMs(uint16_t ms)    //20000ԼΪ3s
{ 
	uint16_t i,j; 
	for( i = 0; i < ms; i++ )
	{ 
		for( j = 0; j < 1141; j++ );
	}
}

/*
��ص�ѹ����ͨ������ʾ��ص������͵�ѹ�ػ�
*/
void ADC_Configuration()
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	//ADC1 ����wave ��������5��ͨ������DMAͨ��1
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
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//��������ѰַADCConvertedValue
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
DAC1���ú�����DAC1���ڼ��������ʿ��ƿɵ����������
�ܽ�PA4��Ϊ
*/
void DAC_Configuration(void)
{  
	DAC_InitTypeDef DAC_InitStructure ;
	/* GPIOA Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);

	DAC_InitStructure.DAC_Trigger=DAC_Trigger_Software ;
	//����Ϊ�������
	DAC_InitStructure.DAC_WaveGeneration=DAC_WaveGeneration_None ;
	DAC_InitStructure.DAC_OutputBuffer=DAC_OutputBuffer_Enable ;

	/* DAC channel1 Configuration */
	DAC_Init(DAC_Channel_1,&DAC_InitStructure);

	/* Enable DAC Channel1: Once the DAC channel2 is enabled, PA.04 is 
	automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1,ENABLE);

	//ʵ�ʶ�Ӧ����IV�������DAC1����--PA.4 (PIN29)
}

/*
���ڶ�ʱ���������ж�ʱ�ػ�����

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
	TIM_TimeBaseStructure.TIM_Period = 7200;	//�̶���ʱʱ����100ms ��10Hz     ע�� 10msʱ��72000	100Hz
	TIM_TimeBaseStructure.TIM_Prescaler = (1000 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* Clear TIM3 update pending flag[���TIM3����жϱ�־] */
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* TIM3 enable counter */
	TIM_Cmd(TIM2, ENABLE);
	//TIM_Cmd(TIM3, DISABLE);  	//��ʼ��ʱ���ȹر�TIM3�����������CW����
}

/*******************************************************************************
* Function Name  : NVIC_Configuration       	  ��Ӧ1310ͨ�����ܽ�PC1,��ʼƵ��270hz
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
	/* Clear TIM3 update pending flag[���TIM3����жϱ�־] */
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM3, DISABLE);  	//��ʼ��ʱ���ȹر�TIM3.�����������CW����
}


/*******************************************************************************
* Function Name  : NVIC_Configuration						   ��Ӧ1490ͨ�����ܽ�PC2,��ʼƵ��270hz
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
	/* Clear TIM5 update pending flag[���TIM5����жϱ�־] */
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	/* TIM5 enable counter */
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM4, DISABLE);  	//��ʼ��ʱ���ȹر�TIM4.�����������CW����
}


/*******************************************************************************
* Function Name  : NVIC_Configuration					   ��Ӧ1550ͨ�����ܽ�PC3,��ʼƵ��270hz
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
	TIM_TimeBaseStructure.TIM_Period = TIM_Period1550;	//�Զ����صļ���ֵ������
	TIM_TimeBaseStructure.TIM_Prescaler = (1000 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	/* Clear TIM5 update pending flag[���TIM5����жϱ�־] */
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
	//ʱ��72M    1ms�ж�  ���ϼ���
	TIM_TimeBaseStructure.TIM_Period = 10-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE); //����ʱ���ж�
	TIM_Cmd(TIM6,ENABLE);
}
/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the used IRQ Channels and sets their priority.
//����NVIC���ȼ����飬��ʽ��
// ע��һ��16�����ȼ�����Ϊ��ռʽ����Ӧʽ���������ȼ���ռ�������ɴ˴���ȷ����
NVIC_PriorityGroup_x������0��1��2��3��4���ֱ������ռ���ȼ���1��2��4��8��16������Ӧ���ȼ���16��8��4��2��1����
�涨�������ȼ������������е��жϼ������������ѡ����ռ����ߵĻ��������ж�����ִ�У�
����Ӧ����ߵĻ��������ж�ִ��������ִ�С�
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 //��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 //��Ӧ���ȼ�
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
* Function: ���ڲ�FLASH��ȡУ׼��Ϣ
* Parameters:
* Return:
	����������ֵ
* Remarks:
*/
void FLASH_Configuration()
{
	int8_t strout[50];
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
	}
}



/****************************************************************************
��ģת������
*****************************************************************************/
/*
* Function: �ƶ�ADͨ����ȡAD����
* Parameters:
	chx = 0 ���
	chx = 1 ���ʿ���
* Return:
	����������ֵ
* Remarks:
*/
uint16_t GetAD(uint8_t chx)
{
	uint32_t i,j,k,start = 0;
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
* Function: ����dbm����Ӧ�ı���ϵ��
* Parameters:
	dbm 0-10.00
* Return:
	����ϵ��
* Remarks:
*/
//����10��x�η���Ҫ�����飬x = �����ͣ�С������(0 - 0.9999)����Ч��Χ-99.99dBm - +99.99dBm
volatile const float c10_10_0[10] = {1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000};
volatile const float c10_0_0001[10] = {1.000000 ,1.000230 ,1.000461 ,1.000691 ,1.000921 ,1.001152 ,1.001383 ,1.001613 ,1.001844 ,1.002074 };
volatile const float c10_0_001[10]  = {1.000000 ,1.002305 ,1.004616 ,1.006932 ,1.009253 ,1.011579 ,1.013911 ,1.016249 ,1.018591 ,1.020939 };
volatile const float c10_0_01[10]   = {1.000000 ,1.023293 ,1.047129 ,1.071519 ,1.096478 ,1.122018 ,1.148154 ,1.174898 ,1.202264 ,1.230269 };
volatile const float c10_0_1[10]    = {1.000000 ,1.258925 ,1.584893 ,1.995262 ,2.511886 ,3.162278 ,3.981072 ,5.011872 ,6.309573 ,7.943282};
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
��Χ�������Ʋ���
*****************************************************************************/
/*
* Function: ������
* Parameters:
	1��������0���ر�
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
/*****************************ѡ�񲨳�����˿ڿ���********************************
* Function Name  : Ctrl_Wavelength	  BY Yu Jingxiong  2011.11.17
* Description    : ѡ���˲�����Ctrl_WavelengthΪ0��1��2��3��4��ѡ�񼤹����������Operating_Mode
ֻ�ǶԶ�ʱ��3��4��5��ʹ���뼰��ӦIO�ڵ����ν��п���
**************************************************************************************/
#define WL_OFF  0
#define WL_1310 1
#define WL_1490 2
#define WL_1550 3
#define WL_RED  4
void Ctrl_Wavelength(u8 Wavelength_Selection_state)	  //Operating_Mode  0 = CW�� 1 = 270Hz��2 = 1KHz��3 = 2KHz  ������/�����ѡ��
{											//Wavelength_Selection_state   0 = �ر�״̬ 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = ���

	switch(Wavelength_Selection_state)				
	{
	case   0:	  // 0 = �رչ�Դ
		TIM_Cmd(TIM3, DISABLE);	   //1310nm
		TIM_Cmd(TIM4, DISABLE);	   //1490nm
		TIM_Cmd(TIM5, DISABLE);	   //1550nm
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);

		break;

	case	1:	 // 1 = 1310nm	PC1			
		TIM_Cmd(TIM4, DISABLE);
		TIM_Cmd(TIM5, DISABLE);
		TIM_Cmd(TIM3, ENABLE);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);

		break;		

	case	2:	 // 2 = 1490nm	PC2
		TIM_Cmd(TIM3, DISABLE);			
		TIM_Cmd(TIM5, DISABLE);
		TIM_Cmd(TIM4, ENABLE);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);

		break;

	case	3:	 // 3 = 1550nm	PC3
		TIM_Cmd(TIM3, DISABLE);
		TIM_Cmd(TIM4, DISABLE);
		TIM_Cmd(TIM5, ENABLE);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);

		break; 

	case	4:	 // 4 = ���

		break;

	default:
		break;     		 

	}
}

/*****************************ģʽѡ��ģ�⿪�ؿ��ƶ˿�********************************
* Function Name  : Ctrl_Operating_Mode	  BY Yu Jingxiong  2011.11.15
* Description    : ��ѡ���˲�����ǰ����Wavelength_Selection_stateΪ1��2��3�����Ƽ��������ģʽOperating_Mode
�Զ�ʱ����PWM����Ƶ�ʽ��п��� �����Ը�����ʱ���ļ���ֵ�������á���
**************************************************************************************/
#define OPM_CW 0
#define OPM_270 1
#define OPM_1K 2
#define OPM_2K 3
void Ctrl_Operating_Mode(u8 Operating_Mode)	 //Operating_Mode  0 = CW�� 1 = 270Hz��2 = 1KHz��3 = 2KHz  ������/�����ѡ��
{										     //Wavelength_Selection_state   0 = �ر�״̬ 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = ���

	switch(Operating_Mode)				
	{
	case   0:	  // 0 = CW��
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //��ӱ�����ʾ�������ر�״̬�ͺ�� ����ѡģʽ
		else if( Wavelength_Selection_state == 1 )	  //1310����
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

		else if( Wavelength_Selection_state == 2 )	  //1490����
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

		else if( Wavelength_Selection_state == 3 )	  //1550����
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

	case	1:	 // 1 = PL 270Hz��
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //��ӱ�����ʾ�������ر�״̬�ͺ�� ����ѡģʽ
		else if( Wavelength_Selection_state == 1 )	  //1310����
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			TIM_Period1310 = TIM_Period[0];
			TIM3_Init( TIM_Period1310);
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM3, ENABLE);	
		}

		else if( Wavelength_Selection_state == 2 )	  //1490����
		{
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Period1490 = TIM_Period[0];
			TIM4_Init( TIM_Period1490);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550����
		{
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);
			TIM_Period1550 = TIM_Period[0];
			TIM5_Init( TIM_Period1550);
			TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM5, ENABLE);
		}
		break;		

	case	2:	 // 2 = PL 1KHz��
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //��ӱ�����ʾ�������ر�״̬�ͺ�� ����ѡģʽ
		else if( Wavelength_Selection_state == 1 )	  //1310����
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			TIM_Period1310 = TIM_Period[1];
			TIM3_Init( TIM_Period1310);
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM3, ENABLE);
		}

		else if( Wavelength_Selection_state == 2 )	  //1490����
		{
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Period1490 = TIM_Period[1];
			TIM4_Init( TIM_Period1490);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550����
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
			;   //��ӱ�����ʾ�������ر�״̬�ͺ�� ����ѡģʽ
		else if( Wavelength_Selection_state == 1 )	  //1310����
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			TIM_Period1310 = TIM_Period[2];
			TIM3_Init( TIM_Period1310);
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM3, ENABLE);
		}

		else if( Wavelength_Selection_state == 2 )	  //1490����
		{
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Period1490 = TIM_Period[2];
			TIM4_Init( TIM_Period1490);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550����
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
* Function: ���ʿ���
* Parameters:
	���ݲ����ṹ
	-1dBm = v->set = -1000;
* Remarks:
*/
void Ctrl_Power(struct ctrl_param *v)
{
	int8_t strout[30];
	float tmpdbm;
	/*3.3V�ֳ�4096�ݣ�ÿ��0.805mV
	Vad = AD * 0.805mV
	Vad = P * R * K * A/B = AD * 0.805
	P:���ʴ�С��dbm->power,�ú���DbmToScaleת��dbm��power
	R:�����С��1000ŷ
	K:����Ӧ�ȣ�(1550nm)Լ0.986��(1310nm)Լ0.895������������
	��ʵ���ֵ����̫���⣬��Ҳ����˵����Ӧ�ȣ�ֻ�Ǹ�У��ƫ����ѣ������Զ�У׼�Ժ�K��ֵ���岻��
	A/B:�⻷�����ֹ�ȣ�������50:50���Կ�����ȥ
	AD = P * R * K / 0.805 = P * 1224.84472 = DbmToScale(v->set) * 1224.84472
	*/
	//scale = DbmToScale((float)(v->set/1000);
	/*if(Wavelength_Selection_state == 1)
		v->set -= 220;//����ͨ����Ӧ�����޸�*/
	
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
	v->dac = (uint16_t)(DbmToScale((float)(v->set/1000.0)) * 2460);//2460ֻ�Ǹ���ŵ�����������ٵ���
	//v->dac = (uint16_t)(DbmToScale(tmpdbm) * 3000);//2460ֻ�Ǹ���ŵ�����������ٵ���
	if(Wavelength_Selection_state == WL_1550) {
		//v->adc = (uint16_t)(DbmToScale((float)(v->set/1000.0)) * 1224.84472);
		v->adc = (uint16_t)(DbmToScale(tmpdbm) * 1224.84472);
	}
	else if(Wavelength_Selection_state == WL_1310) {
		//v->adc = (uint16_t)(DbmToScale((float)(v->set/1000.0)) * (1000*1.040/0.805));
		v->adc = (uint16_t)(DbmToScale(tmpdbm) * (1000*1.040/0.805));
	}
	else {
		v->adc = (uint16_t)(DbmToScale(tmpdbm) * 1224.84472);
	}		
	
	sprintf(strout,"v->dac %4.4d",v->dac);
	debugtxt(0,12,strout,-1);
	sprintf(strout,"v->adc %4.4d",v->adc);
	debugtxt(0,24,strout,-1);
	sprintf(strout,"v->set dbm %3.3f",(float)(v->set / 1000.0));
	debugtxt(0,36,strout,-1);
	DAC_SoftwareTriggerCmd(DAC_Channel_1,DISABLE);  
	DAC_SetChannel1Data(DAC_Align_12b_R, v->dac);
	DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);	
	LCD_Power_Control_Selection_Ex(55,90,((uint16_t)((int32_t)v->set / 1000)),White, Grey);	
// 	g_red_mode++;
// 	if(g_red_mode >=3)
// 		g_red_mode = 0;
// 	g_red_onoff = ~g_red_onoff;
// 	LCD_RedLight_Show(9,15,g_red_onoff);
}
/*
* Function: �����Զ�У��
*/
void AutoCtrlPower()
{
	int8_t strout[20];
	uint16_t ad,flag = 0,fadj = 0;
	static uint16_t adjust_time  = 800;
	
	//����ƫ��Ķ��٣��趨�´�У��ʱ���У������
	if(g_adjust_ms >= adjust_time ) {
		g_adjust_ms = 0;
		ad = GetAD(1);//��ȡadֵ��200��ȡƽ��
		sprintf(strout,"GetAD %4.4d",ad);
		debugtxt(0,48,strout,-1);
		
		if(ad - g_power.adc > 50) {
			g_power.dac -= 50;//У������-50
			fadj = 1;
			adjust_time  = 300;//300ms���ٴ�У��
		}
		else if(g_power.adc - ad > 50) {
			g_power.dac += 50;
			fadj = 1;
			adjust_time  = 300;
		}
		else if(ad - g_power.adc > 6) {
			g_power.dac -= 5;
			fadj = 1;
			adjust_time  = 500;
		}
		else if(g_power.adc - ad> 6) {
			g_power.dac += 5;
			fadj = 1;
			adjust_time  = 500;
		}
		else if(ad - g_power.adc > 4) {
			g_power.dac -= 1;
			fadj = 1;
			adjust_time  = 1000;
		}
		else if(g_power.adc - ad> 4) {
			g_power.dac += 1;
			fadj = 1;
			adjust_time  = 1000;
		}
		
		if(fadj) {
			if(g_power.dac > 4095) {
				g_power.dac = 4095;
			}
			DAC_SoftwareTriggerCmd(DAC_Channel_1,DISABLE);  
			DAC_SetChannel1Data(DAC_Align_12b_R, g_power.dac);
			DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
			
		}
		sprintf(strout,"dac %4.4d",g_power.dac);
		debugtxt(0,60,strout,-1);
	}
}

/*****************************ģʽѡ��ģ�⿪�ؿ��ƶ˿�********************************
* Function Name  : Ctrl_Timing_Device	  BY Yu Jingxiong  2011.11.25
* Description    : //Timer_State  ��Ӧ��ʱ���ػ�����Ϊ5min  ��10min  ��15min  ��30min  ��60min ״̬
TIM2����Ϊ100ms���ڣ�3000 �� 6000  �� 9000  �� 18000 ��  36000
Timer_State��0 �رգ�1       ��2     ��3       ��4      ��5  
**************************************************************************************/
#define TM_OFF   0
#define TM_5MIN  1
#define TM_10MIN 2
#define TM_15MIN 3
void Ctrl_Timing_Device(u8 Timer)	  
{
	switch(Timer)				
	{
	case   0:	  // 0 = ��ʱ���ر�
		Timer_Counter = 0;	  //����״̬֮��֤ÿ�μ������㿪ʼ
		Timer_State  = 0;
		break;
	case   1:	  // 1 = ��ʱ����
		Timer_Counter = 0;
		Timer_State  = 1;
		break;
	case   2:	  // 1 = ��ʱ����
		Timer_Counter = 0;
		Timer_State  = 2;
		break;
	case   3:	  // 1 = ��ʱ����
		Timer_Counter = 0;
		Timer_State  = 3;
		break;
	case   4:	  // 1 = ��ʱ����
		Timer_Counter = 0;
		Timer_State  = 4;
		break;
	case   5:	  // 1 = ��ʱ����
		Timer_Counter = 0;
		Timer_State  = 5;			 		
		break;
	default:
		break;
	}
}
/******************************************************************************/

/*����������������ģʽ�� �˿ڳ�ʼ��*************************************************/
/***************************************************************************/
/*����������������ģʽ���˿ڳ�ʼ��************************************************/
void External_Interrupt_Config(void)
{
}

/*CHECK1��CHECK3��CHECK4�ⲿ�жϳ�ʼ��**********************************/
void External_Interrupt_EXIT_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

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

/*CHECK1��CHECK3��CHECK4�ⲿ�ж����ú���*********************************************************
// ע��һ��16�����ȼ�����Ϊ��ռʽ����Ӧʽ���������ȼ���ռ�������ɴ˴���ȷ����
NVIC_PriorityGroup_x������0��1��2��3��4���ֱ������ռ���ȼ���1��2��4��8��16������Ӧ���ȼ���16��8��4��2��1����
�涨�������ȼ������������е��жϼ������������ѡ����ռ����ߵĻ��������ж�����ִ�У�
����Ӧ����ߵĻ��������ж�ִ��������ִ�С�
*/
void External_Interrupt_InterruptConfig(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
	/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//�����ⲿ�ж�1
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
* Description    : ��ʱ1us
* Input          : - cnt: ��ʱֵ
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void delayUs(vu32 cnt)
{
	uint16_t i;
	for(i = 0;i<cnt;i++)
	{
		uint8_t us = 12; /* ����ֵΪ12����Լ��1΢�� */    
		while (us--)     /* ��1΢��	*/
		{
			;   
		}
	}
}

/*
* Function: ����������
*/
void LCD_DrawMain(void)
{
	uint16_t x,y;

	//��ʾLOGO
	for( x=0; x < 320; x++ )		//�ϱ߽�������34  �±߽�������49
		for( y=0; y < 240; y++ )
			gl_setpoint(x,y,0x22f2);	//��Ļ��ɫ��gray
			
	LCD_FLSAH_DrawPicture(38,91,38+243-1,91+57-1,gImage_logo);
	delayMs(6000);
	for( x=0; x < 320; x++ )		//�ϱ߽�������34  �±߽�������49
		for( y=0; y < 240; y++ )
			gl_setpoint(x,y,0x22f2);	//��Ļ��ɫ��gray
	
	//��ʼ������
	g_power.set = (uint32_t)(-10000);
	Ctrl_Power(&g_power);
	//��ʼ������
	Wavelength_Selection_state = WL_1310;
	Ctrl_Wavelength( Wavelength_Selection_state); //Wavelength_Selection_state   0 = �ر�״̬ 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = ���
	LCD_Wavelength_Selection_Ex( 170,200 ,Wavelength_Selection_state ,Yellow, Grey );
	
	//��ʼ�����Ƶ��
	Operating_Mode = OPM_CW;
	Ctrl_Operating_Mode( Operating_Mode);	     //Operating_Mode  0 = CW�� 1 = 270Hz��2 = 1KHz��3 = 2KHz  ������/�����ѡ��
	LCD_OperatMode_Selection( 15, 200,Operating_Mode, Yellow,Grey );
	
	//��ʼ����ʱ�ػ�
	Timer_State = TM_OFF;
	Ctrl_Timing_Device( Timer_State );
	LCD_Timing_Display( 120, 12 ,Timer_State);
	
	//��ʼ�����
	g_red_onoff = 0;
	LCD_RedLight_Show(9,15,g_red_onoff);
	
	//��ʼ����ص���
	g_batter_delay = -1;//������⣬������100ms��
	ProChargerMonitor();
	
	TouchPanel_Calibrate();	//У׼������
	LCD_Batter_Show(0,0,6/*LEVEL_4*/);//Ϊ�˽����ʼ��ʱ�����б��������������ʾ�����	
}

/*
* Function: �򿪵�Դ
*/
void TurnOnPower()
{
	int i = 0,isDown = 0;
	
// 	//�ܵ�Դʹ�ܣ���
 	GPIO_SetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);
	//LCD��Դʹ��
	GPIO_SetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);	  //Һ����ʹ�ܶ˿�
	Delay_ms(1000);
	
	//���������Ƿ���
	//GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);
	i = 0;
// 	while(i++ < 200) {//�û�����800ms��ÿ10msȥ����
// 		Delay_ms(10);
// 		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 1) {
// 			isDown = 1;
// 		}
// 		else {
// 			isDown = 0;
// 			break;
// 		}
// 	}
	
	//if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 1) 
	//if(isDown)
	{
		GPIO_SetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);//���ܵ�Դ
		powerDownDelayCnt = 0;
		
		
		GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);	  //Һ����ʹ�ܶ˿�
		Delay_ms(1000);
	}
// 	else {
// 		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);//�ر��ܵ�Դ
// 		powerDownDelayCnt = 0;
// 		i = 0;
// 		while(i++<10) {
// 			Delay_ms(1000);
// 		}
// 	}
}

/*
* Function: �ػ�
*/
void TurnOffPower()
{
	//GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);
	//�ر�Һ��������ʾ�û��ػ���
	if(powerDownDelayCnt >= 3) {
		g_red_mode = 0;
		Ctrl_RedLight(0);
		LCD_Clear(Black);
		Delay_ms(1000);
		LCD_SetBacklight(0x03);		
		//TODO :�ر�LCD����͵�Դ
		GPIO_SetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//����ʾ������ 
		delayMs(10);//�ӳ�һС��ʱ�䣬�ȴ��������ɿ�����
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //�ػ� CHECK2��0 ���־λ
		
	}	
}


/*****************************************************************************
��Ӧ�û���������
6����������������
X  Y  Z
A  B  C
*****************************************************************************/
/*
* Function: ��Ӧ��������
*/
void UI_ProWavelength()
{
	if(KeyDown(GPIOA,KEY_B)) {
//		keyflag = 1;
		Wavelength_Selection_state++;
		switch(Wavelength_Selection_state)				
		{
		case   0:
			Wavelength_Selection_state = 1;	 /*��Ч״̬*/ 
			break;
		case   1:
			Wavelength_Selection_state = 3;	 /*2*/ 
			break;
		case   2:
			Wavelength_Selection_state = 3;	 /*��Ч״̬*/ 
			break;
		case   3:
			Wavelength_Selection_state = 1;	 /*4*/
			break;
		case   4:
			Wavelength_Selection_state = 1;	 /*0*/ /*��Ч״̬*/
			break; 	

		default:
			break;    		 		    	
		}	
		Ctrl_Operating_Mode( Operating_Mode);
		Ctrl_Wavelength( Wavelength_Selection_state); //Wavelength_Selection_state   0 = �ر�״̬ 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = ���
		LCD_Wavelength_Selection_Ex( 170,200 ,Wavelength_Selection_state ,Yellow, Grey );	//������ʾ�л�
		Ctrl_Power(&g_power);
		if(Wavelength_Selection_state == WL_1310) {
			hackflag = 1;
			hackval = g_power.set;
		}
	}
}

/*
* Function: ��Ӧ���ʿ���
*/
void UI_ProPower()
{
	uint8_t flag = 0;
	if(KeyPress(GPIOA,KEY_C)) {
		flag = 1;
		g_power.set -= 1000;
	}
	//�������ʴ�С++
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
* Function: ��Ӧģʽ����
*/
void UI_ProMode()
{
	if(KeyDown(GPIOA,KEY_Y)) {
		Operating_Mode++;
		if(Operating_Mode > 3)
			Operating_Mode = 0;
		Ctrl_Operating_Mode( Operating_Mode);	     //Operating_Mode  0 = CW�� 1 = 270Hz��2 = 1KHz��3 = 2KHz  ������/�����ѡ��
		LCD_OperatMode_Selection( 15, 200,Operating_Mode, Yellow,Grey );	//����ģʽ��ʾ�л�
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
	}	
}


/*
* Function: ��Ӧ�������Ͷ�ʱ�ػ�����
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
	else if(i >0 && i < 6)
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

/*
* Function: ��Ӧ���Ű�������
*/
void UI_ProductionAdjust()
{
	int8_t strout[20];
	uint16_t x,y;
	uint8_t flag = 0;
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
	for( x=0; x < 320; x++ )		//�ϱ߽�������34  �±߽�������49
		for( y=0; y < 240; y++ )
			gl_setpoint(x,y,0x22f2);	//��Ļ��ɫ��gray
	
	gl_text((4)*8,(56-24),"CW",-1);
	gl_text((4+10)*8,(56-24),"270",-1);
	gl_text((4+10+10)*8,(56-24),"1K",-1);
	gl_text((4+10+10+10)*8,(56-24),"2K",-1);
	
	gl_text(0,(56),"1310",-1);
	sprintf(strout,"%6.2f",(float)(adjval[0]));
	gl_text((4)*8,(56),strout,-1);
	sprintf(strout,"%6.2f",(float)adjval[1]);
	gl_text((4+10)*8,(56),strout,-1);
	sprintf(strout,"%6.2f",(float)adjval[2]);
	gl_text((4+10+10)*8,(56),strout,-1);
	sprintf(strout,"%6.2f",(float)adjval[3]);
	gl_text((4+10+10+10)*8,(56),strout,-1);
	
	gl_text(0,(56+24),"1550",-1);
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
	
	DrawFocus(loc[index].x,loc[index].y,RGB16(255,255,0));
	while(1) {
		if(KeyPress(GPIOA,KEY_A)) {
			flag = 1;
			index--;
		}
		else if(KeyPress(GPIOA,KEY_B)) {
			flag = 1;
			index++;
		}
		else if(KeyPress(GPIOA,KEY_C)) {
			flag = 1;
			adjval[index] -= 0.01;
		}
		else if(KeyPress(GPIOA,KEY_Z)) {
			flag = 1;
			adjval[index] += 0.01;
		}
		else if(KeyPress(GPIOA,KEY_Y)) {
			flag = 1;
			if(index < 8)
				adjval[index] = 0;
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
				gl_text(0,0,"save",-1);
				Delay_ms(1000);
				gl_text(0,0,"    ",-1);
			}
			else if(index == 9) {
				break;
			}
				
			
		}
		if(flag) {
			flag = 0;
			if(index >= 10)
				index = 0;
			else if(index < 0)
				index = 9;
			if(index < 8) {
				sprintf(strout,"%6.2f",adjval[index]);
				gl_text(loc[index].x,loc[index].y,strout,-1);
			}
// 			gl_fill_rect(loc[last_index].x,loc[last_index].y - 5,5,5);
// 			gl_fill_rect(loc[index].x,loc[index].y - 5,5,5);
			DrawFocus(loc[last_index].x,loc[last_index].y,RGB16(38,93,150));
			DrawFocus(loc[index].x,loc[index].y,RGB16(255,255,0));
			last_index = index;
			
		}
		/*
			
		}*/
		
	}
	LCD_DrawMain();
	
}
/*
* Function: ���ƽ��潹��
* Parameters:
	x,y���꣬color��RGB16(R,G,B)��ɫ
*/
void DrawFocus(int16_t x,int16_t y,uint32_t color)
{
	uint32_t brush,pen;
	
	brush = gl_ui_setbrushcolor(color);
	pen = gl_ui_setpencolor(color);
	gl_fill_rect(x,y - 5,30,5);	
	gl_ui_setbrushcolor(brush);
	gl_ui_setpencolor(pen);
}

/*
* Function: �ɹ�����У׼����
  ����-8,-7,-4
*/
void IsHacker()
{
	static uint8_t state = 0;
	
	if(hackflag == 1) {
		hackflag = 0;
		switch(state) {
		case 0:
			if((int32_t)hackval == -8000)
				state++;
			else
				state = 0;
			break;
		case 1:
			if((int32_t)hackval == -7000)
				state++;
			else
				state = 0;
			break;
		case 2:
			if((int32_t)hackval == -4000) {
				state = 0;
				UI_ProductionAdjust();
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
���ģ��
*****************************************************************************/
/*
* Function: ��Դ���
* Remarks:���ֵ����ȼ����ҵ͵����Զ��ػ����ػ�ǰˢ��ɫ��Ļ��ʾ
//ֻ�е�ǰ�ȼ�����һ�εȼ��Ĳ�ľ���ֵ>=2ʱ�Ÿ��£�������ⲻ�����������ڵȼ������ر䶯��
//����Ҳ��֤��LEVEL_POWER��LEVEL_FULL��LEVEL_CHARGE�����ȼ����Կ�����Ӧ�л�
//��ص�����⸡����0.030V���ң�����û���ʱ�򸡶��ɴ�0.100V
*/
//��صȼ�
#define LEVEL_POWER    0  //�ⲿ����
#define LEVEL_FULL     2  //������
#define LEVEL_CHARGE   4  //���ڳ��
#define LEVEL_4        6  //���4��
#define LEVEL_3        7  //���3��
#define LEVEL_2        8  //���2��
#define LEVEL_1        9  //���1��
#define LEVEL_0        10  //���0��
#define LEVEL_SHUTDOWN 11 //����Զ��ػ�
/*int8_t disp_battery_index[12] = {
	//0  1  2  3  4  5  6  7  8  9  10 11����1��3����
	  4, 4, 4, 4, 4, 5, 4, 3, 2, 1, 0, 0
};*/
void ProChargerMonitor()
{
	int8_t strout[30];
	static uint8_t times = 100;
	static uint16_t last_level = LEVEL_4,level_show = LEVEL_4;
	uint16_t level = LEVEL_4; 
	float vol,tvol;
	//static float last_vol = 100;
	int x,y;
	
	/*************************���ָʾ����*******************************************************/
	if(g_batter_delay > 100) {
		g_batter_delay = 0;
		vol = (float)GetAD(0)*0.00349+0.7;
		sprintf(strout,"better:%0.3f",vol);
		debugtxt(0,12*6,strout,30);
		
		if((GPIO_ReadInputDataBit(GPIO_PORT_POWER_CHK, GPIO_CHARG_CHK)==0)) {//������ֻ�ҵ�ء�ֻ����ӵ�Դ
			if(vol > 12.0) {//��ӵ�ԴԶԶ����
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
				
				for( x=0; x < 320; x++ )		//��Ļˢ��ɫ����ʾ���������Զ��ػ�
					for( y=0; y < 240; y++ )
						gl_setpoint(x,y,RGB16(255,0,0));
				Delay_ms(2000);
				powerDownDelayCnt = 1100;//��ʾҪ�ػ���
				TurnOffPower();
				
			}
		}
		else if((GPIO_ReadInputDataBit(GPIO_PORT_POWER_CHK, GPIO_CHARG_CHK)==1)) {//���ڳ��
			level = LEVEL_CHARGE;
			//debugtxt(0,100+12,"charge...",20);	
		}
		
		
		//Ϊ�˴��������ڲ�ADת�����ȶ���0.030V�ĸ��������������´���
		if(
			//ֻ��������ε�����⼶�����
			//Ҫô����Խ��Խ�ͣ�Ҫô�����ӳ����
			(last_level == level &&
			 times++ >= 2 && 
			 (level > level_show || level - level_show <= -2 || level == level_show) )||
			//������һ�ε�������ǲ������Դ�ģ���һ����Ҫ��������
			//BUG:LEVEL_POWER��LEVEL_FULL���ܻ�����ת���������û�������������״̬��ͼ����һ���ģ�
			//���Բ��ر��û������쳣
			(level_show == LEVEL_CHARGE || level_show == LEVEL_POWER ||level_show == LEVEL_FULL)
			) {
			
			level_show = level;//�ϴ���ʾ�ĵȼ�
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


/*
* Function: ��Ӧ��ʱ�ػ�
*/
void ProTimerShutdown()
{
	/***************************��ʱ�ػ�����*************************************************************/
	//һ�㶨ʱ�ã���ʱ�ػ�����������Ϊ5min  ��10min  ��15min  ��30min  ��60min ״̬
	//TIM2����Ϊ100ms���ڣ�3000 �� 6000  �� 9000  �� 18000 ��  36000		   

	switch(Timer_State)				
	{
	case   0:	  // 0 = ��ʱ���ر�

		break;   
	case   1:	  // 1 = 5min��ʱ
		if( Timer_Counter >=3000 )
		{ GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//����ʾ������ 
		delayMs(10);//�ӳ�һС��ʱ�䣬�ȴ��������ɿ�����
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //�ػ� CHECK2��0 ���־λ
		Timer_State = 0;
		//LCD_Timing_Display( 255, 12 ,Timer_State );	  //��ʱ����ʾ�л�
		}
		break;
	case   2:	  // 1 = 1Omin
		if( Timer_Counter >=6000 )
		{ GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//����ʾ������ 
		delayMs(10);//�ӳ�һС��ʱ�䣬�ȴ��������ɿ�����
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //�ػ� CHECK2��0 ���־λ
		Timer_State = 0;
		//LCD_Timing_Display( 255, 12 ,Timer_State );	  //��ʱ����ʾ�л�
		}
		break;
	case   3:	  // 1 = 15min
		if( Timer_Counter >=9000 )
		{ GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//����ʾ������ 
		delayMs(10);//�ӳ�һС��ʱ�䣬�ȴ��������ɿ�����
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //�ػ� CHECK2��0 ���־λ
		Timer_State = 0;
		//LCD_Timing_Display( 255, 12 ,Timer_State );	  //��ʱ����ʾ�л�
		}
		break;
	case   4:	  // 1 = 30min
		if( Timer_Counter >=18000 )
		{ GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//����ʾ������ 
		delayMs(10);//�ӳ�һС��ʱ�䣬�ȴ��������ɿ�����
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //�ػ� CHECK2��0 ���־λ
		Timer_State = 0;
		//LCD_Timing_Display( 255, 12 ,Timer_State );	  //��ʱ����ʾ�л�
		}
		break;
	case   5:	  // 1 = 60min
		if( Timer_Counter >=36000 )
		{ GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);//����ʾ������ 
		delayMs(10);//�ӳ�һС��ʱ�䣬�ȴ��������ɿ�����
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //�ػ� CHECK2��0 ���־λ
		Timer_State = 0;
		//LCD_Timing_Display( 255, 12 ,Timer_State );	  //��ʱ����ʾ�л�
		}
		break;		 		
	default:
		break;
	}	
}
void Check1963()
{
	int _1963Down = 0;
	int times = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	/* TP_CS */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	while(times++ <= 4 && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 0) {
		GPIO_SetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);	  
		Delay_ms(700);
		GPIO_ResetBits(GPIO_CTRL_PORT_D, GPIO_LCD_OFF);	  
		Delay_ms(100);
		LCD_Initializtion();
		LCD_SetBacklight(0xff);
		Delay_ms(500);
		_1963Down = 1;
	}
	if(_1963Down) {
		LCD_DrawMain();
		g_red_mode = 3;
		g_red_onoff = 1;
		Ctrl_RedLight(g_red_onoff);
		LCD_RedLight_Show(9,15,g_red_onoff);
		
	}
	/* TP_CS */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int main(void)
{
	uint8_t str[20];
	uint8_t i;

	delay_init();
	/*****************�����жϳ�ʼ������*********************** */
	Function_IO_config(); 
	RedLightIOConfig();
	
	//������ʱ�ж�
	TIM6_Init();//1MS��ʱ
	
	
	TIM2_Init();//1MS��ʱ
	
	TIM3_Init( TIM_Period1310);
	TIM4_Init( TIM_Period1490);
	TIM5_Init( TIM_Period1550);
	NVIC_Configuration();

	//
	ADC_Configuration();//dma mode
	DAC_Configuration();
TurnOnPower();
	LCD_Initializtion();
	LCD_SetBacklight(0xff);

	
	//����ͨ��
	//USART_Configuration();
	//��ʼ��Һ����
	
	//GPIO_SetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);
	//TP_Init();
	
	
	//Һ����GUI����
	gl_ui_setlib(p_zm_ascii_st9,8,12,p_zm_step_st9);
	gl_ui_setbkmode(BACKFILL);//�������ģʽ
	gl_ui_setfontcolor(RGB16(255,0,0));
	gl_ui_setbkcolor(RGB16(0,255,0));
	gl_ui_setpencolor(RGB16(235,34,209));
	
	
	LCD_DrawMain();
	LCD_SetBacklight(0xff);
	LCD_SetBacklight(0xff);
	LCD_SetBacklight(0xff);
	

	
	powerDownDelayCnt=0;
	//DebugLookAD();
	g_batter_delay = 10001;	
	
	FLASH_Configuration();
	
	
	
	while(1)
	{
		
		
		UI_ProRedLight_ShutdownTimer();//
		UI_ProWavelength();
		UI_ProPower();
		UI_ProMode();
		TurnOffPower();
		ProChargerMonitor();
		ProTimerShutdown();//��ʱ�ػ�
		AutoCtrlPower();
 		IsHacker();

		
	}//while��β
}

/*
* Function: ���Բ鿴����ʽ��û��
*/
extern uint8_t msgindex;
void DebugLookAD()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	int16_t ad = 200;
	int8_t strout[30],times = 0;
	static uint16_t flag = 0;
	float vol,scale;
	
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
		if(gl_key_down(6)) {
			//sprintf(strout,"key %d",times++);
			sprintf(strout,"key %d",msgindex);
			gl_text(125,0,strout,-1);
		}
		continue;
		UI_ProMode();
		UI_ProWavelength();
		
		if(KeyPress(GPIOA,KEY_C)) {
			ad-= 100;
			g_power.set += 100;
			flag = 1;
		}
		else if(KeyPress(GPIOA,KEY_Z)) {
			ad+= 100;
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
			DAC_SoftwareTriggerCmd(DAC_Channel_1,DISABLE);  
			DAC_SetChannel1Data(DAC_Align_12b_R, ad/*ad*/);
			DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);  			
			
			DAC_SoftwareTriggerCmd(DAC_Channel_2,DISABLE);  
			DAC_SetChannel1Data(DAC_Align_12b_R, ad);
			DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);  			
			sprintf(strout,"debug set adc:%4.4d",ad);
			Ctrl_Power(&g_power);
			
			gl_text(0,100-12,strout,20);
		}
		AutoCtrlPower();
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
			gl_text(0,200,strout,20);
			
			vol = (float)GetAD(1)*0.000805;
			//vol = (float)GetAD(1)*0.00349+0.7;
			sprintf(strout,"ad1 %3.3f",vol);
			gl_text(0,200+12,strout,20);
			ProChargerMonitor();
		}
		
	}
}
