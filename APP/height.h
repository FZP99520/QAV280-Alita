#ifndef HIGH_H
#define HIGH_H

#include "stm32f10x.h"
#include "includeAll.h"



extern float exp_height_speed;//Ŀ���ٶ�mm/s
extern float exp_height_speed_Throttle;//��������Ŀ���ٶ�mm/s
extern float m_TH_Height_Max;//���߶�

extern float exp_height;//Ŀ��߶� mm
extern float exp_height_auto;//Ŀ��߶�mm
extern u8 flag_exp_height_auto;//Ŀ��߶�mm

extern float exp_height_speed_fly_compent;//��ɹ��������ŵ��ߵĲ�����
extern u16 Cnt_exp_height_speed_fly_compent;
extern float m_exp_height_speed_Limit;//�߶ȿ��Ƶ��ٶȷ�ֵ�趨

extern float real_height;
extern float real_speed_height;


void Press_To_Height(MS5611_Data_TypeDef* height,u8 flag_filter);
void Real_Height_Speed_Update(void);
void Height_Control_Update(void);
#endif


