#ifndef _led_H
#define _led_H

#include "stm32f10x.h"

#define LED_Pin  GPIO_Pin_12|GPIO_Pin_15

#define LED_Blue_ON  GPIO_ResetBits(GPIOA,GPIO_Pin_15)
#define LED_Blue_OFF GPIO_SetBits(GPIOA,GPIO_Pin_15)
#define LED_Blue_STA GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_15)

#define LED_Red_ON  GPIO_ResetBits(GPIOA,GPIO_Pin_12)
#define LED_Red_OFF GPIO_SetBits(GPIOA,GPIO_Pin_12)
#define LED_Red_STA GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_12)

void Led_Init(void);
void LedStart_Show(void);
#endif
