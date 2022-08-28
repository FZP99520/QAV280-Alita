#ifndef _pwm_H
#define _pwm_H
#include "stm32f10x.h"
#include "IncludeAll.h"
#define PWM_Start 1067u
#define PWM_Stop  1000u
#define PWM_Flying_Max   1800u
#define PWM_Flying_Min   1100u

typedef struct
{
	u16 pwm1;
	u16 pwm2;
	u16 pwm3;
	u16 pwm4;
}PWM_TypeDef;
extern PWM_TypeDef MOTOR;

void PWM_Init(void);
void PWM_Set(PWM_TypeDef* p,Fly_Status_t flysta);
#define PWM1 TIM3->CCR1
#define PWM2 TIM3->CCR2
#define PWM3 TIM3->CCR3
#define PWM4 TIM3->CCR4
#endif
