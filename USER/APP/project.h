#ifndef _PROJECT_H_
#define _PROJECT_H_

//屏蔽警告
#pragma diag_suppress 167
#pragma diag_suppress 174
#pragma diag_suppress 1295//函数参数是空必须声明成void
#pragma diag_suppress 68//强制转换有符号和无符号



/*按键*/
#define KEY_A GPIO_Pin_5
#define KEY_B GPIO_Pin_3
#define KEY_C GPIO_Pin_2
#define KEY_X GPIO_Pin_1
#define KEY_Y GPIO_Pin_7
#define KEY_Z GPIO_Pin_6


void USART_Configuration();
void delayMs(uint16_t ms);
void ADC_Configuration(void);
uint16_t GetAD(uint8_t chx);
void DAC_Configuration(void);
void TIM2_Init(void);
void TIM3_Init(uint16_t);
void TIM4_Init(uint16_t);
void TIM5_Init(uint16_t);
void TIM6_Init(void);
void NVIC_Configuration(void);
void FLASH_Configuration(void);
float DbmToScale(float dbm);
void Ctrl_RedLight(u8 v);
void Ctrl_Wavelength(u8 Wavelength_Selection_state);
void Ctrl_Operating_Mode(u8 Operating_Mode);
void Ctrl_Power(struct ctrl_param *v);
void AutoCtrlPower(void);
void Ctrl_Timing_Device(u8 Timer);
void Function_IO_config(void);
void RedLightIOConfig(void);
void External_Interrupt_Config(void);
void External_Interrupt_EXIT_Init(void);
void External_Interrupt_InterruptConfig(void);
void LCD_DrawMain(void);
void TurnOnPower(void);
void TurnOffPower(void);
void UI_ProWavelength(void);
void UI_ProPower(void);
void UI_ProMode(void);
void UI_ProRedLight_ShutdownTimer(void);
void ProChargerMonitor(void);
float GetBattery();
void ProTimerShutdown(void);
int main(void);
void DebugLookAD(void);
void UI_ProductionAdjust(void);
void UI_ProductConfig(void);

void IsHacker(void);
int ProTick1963IsLive(void);
void ProGet1963State(void);

void LCD_Batter_Show( uint16_t x0, uint16_t y0 ,uint16_t rank )	;		
void LCD_RedLight_Show( uint16_t x0, uint16_t y0 ,uint8_t flag);
void LCD_Wavelength_Selection( uint16_t x0, uint16_t y0 ,uint16_t wave ,uint16_t Color, uint16_t BkColor );
void LCD_Wavelength_Selection_Ex( uint16_t x0, uint16_t y0 ,uint16_t wave ,uint16_t Color, uint16_t BkColor );
void LCD_OperatMode_Selection( uint16_t x0, uint16_t y0 ,uint16_t mode ,uint16_t Color, uint16_t BkColor );

void LCD_Timing_Display( uint16_t x0, uint16_t y0 ,uint16_t on_off );
void Show_Matrix_zimo(uint16_t Xpos, uint16_t Ypos, uint8_t *Buffer, uint16_t Wide_char, uint16_t High, uint16_t charColor, uint16_t bkColor);
void LCD_Power_Control_Selection_Ex( uint16_t x0, uint16_t y0 ,int16_t Current_Power ,uint16_t CharColor, uint16_t BkColor );
void GUI_Text_Show_Number(uint16_t x, uint16_t y, uint8_t number, uint16_t Color, uint16_t bkColor );
void LCD_FLSAH_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY, uint8_t * pic);


//uint32_t InputPanel(int8_t *str,uint32_t len);
// uint32_t InputPanel(int8_t *str,uint32_t len,uint32_t *outLen);


//启动版本信息
#define TARGET_NAME		"G-LINK TS100 VEX1.4.2"    		//目标板名称
#define RELEASE_DATE	"Release Date 2016.2.3"				//修改发布时间

#endif

