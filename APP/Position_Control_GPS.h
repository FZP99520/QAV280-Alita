#ifndef _Posiyion_Control_GPS_H
#define _Posiyion_Control_GPS_H

#include "stm32f10x.h"

void GPS_Control_Update(void);
void GPS_Position_PID_Para_Init(void);

extern float Target_Roll;
extern float Target_Pitch;

#endif