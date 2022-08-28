#ifndef _74hc138_H
#define _74hc138_H

#include "stm32f10x.h"
#include "systick.h"
#define A0 GPIO_Pin_14
#define A1 GPIO_Pin_13
#define A2 GPIO_Pin_12

#define A0_H GPIO_SetBits(GPIOB,A0)
#define A0_L GPIO_ResetBits(GPIOB,A0)

#define A1_H GPIO_SetBits(GPIOB,A1)
#define A1_L GPIO_ResetBits(GPIOB,A1)

#define A2_H GPIO_SetBits(GPIOB,A2)
#define A2_L GPIO_ResetBits(GPIOB,A2)
void Init_HC138(void);
int Control_hc138(int num);
#endif
