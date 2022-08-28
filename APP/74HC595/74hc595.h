#ifndef _74hc595_H
#define _74hc595_H

#include "stm32f10x.h"
#include "systick.h"
#define SER GPIO_Pin_7
#define SRCK GPIO_Pin_6
#define RCLK GPIO_Pin_5

#define SER_H GPIO_SetBits(GPIOB,SER)
#define SER_L GPIO_ResetBits(GPIOB,SER)

#define SRCK_H GPIO_SetBits(GPIOB,SRCK)
#define SRCK_L GPIO_ResetBits(GPIOB,SRCK)

#define RCLK_H GPIO_SetBits(GPIOB,RCLK)
#define RCLK_L GPIO_ResetBits(GPIOB,RCLK)

void Init_74HC595(void);//初始化函数
void SendData_HC595(u8 dat);//595发送数据函数

#endif
