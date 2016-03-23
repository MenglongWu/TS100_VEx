/**
  ******************************************************************************
  * @file prj_Conf.h 
  * @author  cyw
  * @version V1.0.0
  * @date    10/25/2010
  * @brief   project configuration file.
  ******************************************************************************
  20111025 光纤识别仪第1版本
 
 COPYRIGHT 2011 G-LINK Corp. 
**/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRJ_CONF_H
#define __PRJ_CONF_H

/* Private define ------------------------------------------------------------*/

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
#define CR2_EXTTRIG_SWSTART_Set ((uint32_t)0x00500000)
#define CR2_EXTTRIG_SWSTART_Reset ((uint32_t)0xFFAFFFFF)
#define CR2_ADON_Set ((uint32_t)0x00000001)
#define CR2_ADON_Reset ((uint32_t)0xFFFFFFFE)


#define RCC_GPIO_CTRL 			RCC_APB2Periph_GPIOD 
#define GPIO_CTRL_PORT 			GPIOD 
/*LD功率控制，PD10~14, 置为1增加20mA功率 */
#define GPIO_LD_POWER0 						GPIO_Pin_10
#define GPIO_LD_POWER1 						GPIO_Pin_11
#define GPIO_LD_POWER2 						GPIO_Pin_12
#define GPIO_LD_POWER3 						GPIO_Pin_13
#define GPIO_LD_POWER4 						GPIO_Pin_14

/*手动开关机监测CHECK1,PD0,上升沿中断*/
#define GPIO_SYSPWR_ONOFF_CHK 				GPIO_Pin_0
/*开关机控制（CHECK2）PD1：开机置1，关机清0*/
#define GPIO_SYSPWR_ONOFF 				   GPIO_Pin_1
/*电池电量低自动关机监测（CHECK3）PD2：上升沿中断*/
#define GPIO_SYSPWR_LOW_CHK 				 GPIO_Pin_2
/*关LCD屏控制（ON/OFF）PD3：清0*/
#define GPIO_LCD_OFF							   GPIO_Pin_3
/*充电指示监测（CHECK4）PD4：上升沿中断*/
#define GPIO_CHARG_CHK				       GPIO_Pin_4
/*充电指示控制（EINT14）PD5，清0*/
#define GPIO_CHARG_STU				       GPIO_Pin_5

#define DAC_DHR12RD_Address      0x40007420
#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

#define ADC1_DR_Address ((uint32_t)0x4001244C)
#define ADC2_DR_Address    ((uint32_t)0x40013C4C)
#define SPI2_MASTER 1
#define SPI2_SLAVE 0
#define CMD_STRING_SIZE 128 
#define ADCSAMPLES_DMA  12
#define ADC2SAMPLES_DMA  200

#define BufferSize 1008  //
#define SPIFameSize 256
#define Dummy_Byte       0xFF

#define LDPOWER_MAX  18 //LD控制门数最大值

#define LDAUTOPOWER_MAX 2250  //最佳接收范围ADC2值
#define LDAUTOPOWER_MIN 2000

/*CHECK2端口、关LCD端口、充电指示端口  合并*/
#define Control_KEYS       GPIO_SYSPWR_ONOFF|GPIO_LCD_OFF|GPIO_CHARG_STU          

/*手动开关机监测CHECK1,PD0,上升沿中断*/
#define GPIO_SYSPWR_ONOFF_CHK 			            GPIO_Pin_0
#define GPIO_SYSPWR_ONOFF_CHK_EXTI_LINE             EXTI_Line0
#define GPIO_SYSPWR_ONOFF_CHK_EXTI_PORT_SOURCE      GPIO_PortSourceGPIOD
#define GPIO_SYSPWR_ONOFF_CHK_PIN_SOURCE            GPIO_PinSource0
#define GPIO_SYSPWR_ONOFF_CHK_IRQn                  EXTI0_IRQn 

/*电池电量低自动关机监测（CHECK3）PD2：上升沿中断
#define GPIO_SYSPWR_LOW_CHK 			            GPIO_Pin_2																   
#define GPIO_SYSPWR_LOW_CHK_EXTI_LINE               EXTI_Line2
#define GPIO_SYSPWR_LOW_CHK_EXTI_PORT_SOURCE        GPIO_PortSourceGPIOD
#define GPIO_SYSPWR_LOW_CHK_PIN_SOURCE              GPIO_PinSource2
#define GPIO_SYSPWR_LOW_CHK_IRQn                    EXTI2_IRQn 
*/
/*充电指示监测（CHECK4）PD4：上升沿中断*/
#define GPIO_CHARG_CHK				                GPIO_Pin_4
#define GPIO_CHARG_CHK_EXTI_LINE                    EXTI_Line4
#define GPIO_CHARG_CHK_EXTI_PORT_SOURCE             GPIO_PortSourceGPIOD
#define GPIO_CHARG_CHK_PIN_SOURCE                   GPIO_PinSource4
#define GPIO_CHARG_CHK_IRQn                         EXTI4_IRQn 


/* Private function prototypes -----------------------------------------------*/
//implementation at main.c
void ADC1Configuration(void);
void ADC2Configuration(void);
u16 getLDPower(u8 opt);
void ctrlLD(u8 power);
void cmdExplain(void);
uint8_t processCMMTest(uint8_t *pBuff, u16 len);
uint8_t cmmPkgSend(u16 cmdCode, u16 datLen, uint8_t *pData);
void SPI2_Init(uint8_t type);
u8 SPI2_SendByte(u8 byte);
u8 SPI2_ReadByte(void);
void testSPICmm(void);
static void TIM2_Init(void);
void initADCDMA(void);
void Delay_ARMJISHU(__IO uint32_t nCount);
void SysTick_Configuration(void);
void SerialPutChar(uint8_t c);
void SerialSendU16(u16 d);
void startMeasureAndSPI(void);
u8 LDPowerAutoAdj(u8 power);
void LDNoiseLevelAutoAdj(void);
void startSPI2(void);
u16 maxVal(u16*dataIn,u16 len);
u16 minVal(u16 *dataIn,u16 len);
void Control_KEYS_config(void);
u16 mean(u16 *dataIn, u16 len);
void startADC(void);
void stopADC(void);

//implementation at stm32f10x_it.c
void External_Interrupt_Config(void);
void External_Interrupt_EXIT_Init(void);
void External_Interrupt_InterruptConfig(void);


#endif /* __PRJ_CONF_H */

/******************* (C) COPYRIGHT 2011 G-LINK *****END OF FILE****/
