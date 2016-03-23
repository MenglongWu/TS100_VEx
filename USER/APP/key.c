#include "key.h"
//Function define
//#define Key_Signal 1//下拉电阻输入
#define Key_Signal 0//上拉电阻输入
volatile uint16_t g_delay_ms = 0;

void Delay_ms(uint16_t ms)
{
	g_delay_ms = ms;
	while(g_delay_ms);
}

int8_t KeyPress(GPIO_TypeDef* GPIOx,uint16_t kpin)
{
	static GPIO_TypeDef *oldGPIOx = 0;//上次处理的端口组
	static uint16_t oldkpin = 0;//上一次处理的端口引脚
	static uint8_t secend = 0;  //第二个字符显示标志

	//if((GPIOx->IDR & kpin) == 0) {
	if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
		Delay_ms(60);
		//if((GPIOx->IDR & kpin) == 0) {
		if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
			//dprintf("Port = %x Pin = %x\n",oldGPIOx,oldkpin);
			if(oldGPIOx == GPIOx && oldkpin == kpin) {
				if(secend == 1) 
					Delay_ms(480);//重复延时，第一次keydown事件在按下410ms后
				else
					//Delay_ms(220);//重复速度，第二次开始每秒10次keydown事件
					Delay_ms(180);//重复速度，第二次开始每秒10次keydown事件
				secend = 2;
				//if((GPIOx->IDR & kpin) != 0) {
				if(GPIO_ReadInputDataBit(GPIOx,kpin) != Key_Signal) {
					oldGPIOx = 0;oldkpin = 0;
					secend = 0;
					return 0;
				}
				//dprintf("sec\n");
				return 2;
			}
			else {
				oldGPIOx = GPIOx;
				oldkpin = kpin;
				secend = 1;
				//printf("first\n");
				return 1;
			}
		}
	}
	//dprintf("byebye\n");
	//oldGPIOx = 0;oldkpin = 0;
	return 0;
}

int8_t KeyDown(GPIO_TypeDef* GPIOx,uint16_t kpin)
{
	static GPIO_TypeDef* oldGPIOx = 0;
	static uint16_t oldkpin = 0;
	
	if(kpin == 0 && GPIOx == 0) {
		oldGPIOx = 0;
		oldkpin = 0;
		return 1;
	}
	if(oldGPIOx == GPIOx && oldkpin == kpin) {
		if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
			return 0;
		}
		else {
			oldGPIOx = 0;
			oldkpin = 0;
		}
	}
	//if((GPIOx->IDR & kpin) == 0) {
	if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
		Delay_ms(40);
		//if((GPIOx->IDR & kpin) == 0) {
		if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
			oldGPIOx = GPIOx ;
			oldkpin = kpin;
			//while(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal);
			
			return 1;
		}
	}
	return 0;
}
int8_t KeyDown_Ex(GPIO_TypeDef* GPIOx,uint16_t kpin)
{
	static GPIO_TypeDef* oldGPIOx = 0;
	static uint16_t oldkpin = 0;
	uint8_t i;
	
// 	if(kpin == 0 && GPIOx == 0) {
// 		oldGPIOx = 0;
// 		oldkpin = 0;
// 		return 1;
// 	}
// 	if(oldGPIOx == GPIOx && oldkpin == kpin) {
// 		if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
// 			return 0;
// 		}
// 		else {
// 			oldGPIOx = 0;
// 			oldkpin = 0;
// 		}
// 	}
	//if((GPIOx->IDR & kpin) == 0) {

	i = 0;
	while(i < 10 && GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
		Delay_ms(100);
		i++;
	}
	return i;
}