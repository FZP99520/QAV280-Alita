#ifndef _ADC_H
#define _ADC_H

#include "stm32f10x.h"
#define Channel_Num 1         //ͨ���� 4ͨ�����ݲɼ�

typedef struct
{
	u16 v_adc;
	u16 c_adc;
	float Voltage;
	float Current;
}VCBAT_TypeDef;
extern VCBAT_TypeDef VCBAT;
void Adc_Init(void);
void Get_Battery_Inf(void);


#endif
