/*
 * key.c
 *
 *  Created on: Oct 22, 2021
 *      Author: Administrator
 */

#include "key.h"

#include "../delay/delay.h"


uint8_t KEY_1(void)
{
	uint8_t a;
	a=0;//如果未进入按键处理，则返回0
	if(HAL_GPIO_ReadPin(GPIOA,KEY1_Pin)==GPIO_PIN_RESET){//读按键接口的电平
		delay_us(20000);//延时去抖动
		if(HAL_GPIO_ReadPin(GPIOA,KEY1_Pin)==GPIO_PIN_RESET){ //读按键接口的电平
			a=1;//进入按键处理，返回1
		}
	}
	while(HAL_GPIO_ReadPin(GPIOA,KEY1_Pin)==GPIO_PIN_RESET); //等待按键松开
	return a;
}

uint8_t KEY_2(void)
{
	uint8_t a;
	a=0;//如果未进入按键处理，则返回0
	if(HAL_GPIO_ReadPin(GPIOA,KEY2_Pin)==GPIO_PIN_RESET){//读按键接口的电平
		delay_us(20000);//延时去抖动
		if(HAL_GPIO_ReadPin(GPIOA,KEY2_Pin)==GPIO_PIN_RESET){ //读按键接口的电平
			a=1;//进入按键处理，返回1
		}
	}
	while(HAL_GPIO_ReadPin(GPIOA,KEY2_Pin)==GPIO_PIN_RESET); //等待按键松开
	return a;
}

