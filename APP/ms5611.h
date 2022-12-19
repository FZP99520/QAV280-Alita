#ifndef _MS5611_H
#define _MS5611_H
#include "stm32f10x.h"
#include "stdbool.h"
#include <math.h>
#include "device.h"
typedef struct MS5611_Data_Typedef_s
{
    E_DevInit_Status_TypeDef eMS5611Init_Status;
    E_DevCali_Status_TypeDef eMS5611Cali_Status;
    u32 u32ut;
    u32 u32up;
    u16 u16prom[8]; //�ڲ���׼����
    u32 u32temperature;
    float f32RP;//���������ѹֵ
    float Reff_P;//�ο���ѹֵ
    float Diff_P;//��ѹ��ֵ
    float Altitude_P; //��ƽ��߶�
    float Altitude_Diff_P;//????
    float Altitude_Reff_P;//????
}MS5611_Data_TypeDef;

extern MS5611_Data_TypeDef gsMS5611_Data;


E_DevInit_Status_TypeDef MS5611_Init(void);

void MS5611_Data_Update(void);


#endif
