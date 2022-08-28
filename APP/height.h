#ifndef HIGH_H
#define HIGH_H

#include "stm32f10x.h"
#include "includeAll.h"



extern float exp_height_speed;//目标速度mm/s
extern float exp_height_speed_Throttle;//油门输入目标速度mm/s
extern float m_TH_Height_Max;//最大高度

extern float exp_height;//目标高度 mm
extern float exp_height_auto;//目标高度mm
extern u8 flag_exp_height_auto;//目标高度mm

extern float exp_height_speed_fly_compent;//侧飞过程中油门掉高的补偿量
extern u16 Cnt_exp_height_speed_fly_compent;
extern float m_exp_height_speed_Limit;//高度控制的速度阀值设定

extern float real_height;
extern float real_speed_height;


void Press_To_Height(MS5611_Typedef* height,u8 flag_filter);
void Real_Height_Speed_Update(void);
void Height_Control_Update(void);
#endif


