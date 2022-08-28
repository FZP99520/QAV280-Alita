#ifndef _CONTROL_H
#define _CONTROL_H

#include "stm32f10x.h"
typedef enum
{
	FlyStaLock = 0,//锁定
	FlyStaRdy=1,//数据已准备
	FlyStaUnlock =2,//解锁，解锁后检查起飞角度，正常后才可以进入正常飞行模式
	FlyStaFlying=3,//正常模式，可以起飞
	FlyStaPIDAdj=4,//PID调整状态
	FlyStaAngleErr=5,//角度错误
}Fly_Status_t;
extern Fly_Status_t Fly_sta;
extern u8 FlyMode;
void Control(void);
void High_Handle(void);

#endif
