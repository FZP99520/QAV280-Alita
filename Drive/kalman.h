#ifndef KALMAN_H
#define KALMAN_H

#include "stm32f10x.h"
#include "arm_math.h"
void Kalman_Height(float ms_alt,float Az,float* height,float* speed);
void KalmanFilter_V(float* ResrcData);


#endif
