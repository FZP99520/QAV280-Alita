#ifndef IMU_H
#define IMU_H

#include "stm32f10x.h"
#include "mpu6050.h"
#include "qmc5883.h"
#include "control.h"



typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
  float Matr13;//2.0f * (q1*q3 - q0*q2);
  float Matr23;//2.0f * (q0*q1 + q2*q3);
  float Matr33;//1.0f - 2.0f*(q1*q1 + q2*q2);

  float exInt_Acc;
  float eyInt_Acc;
  float ezInt_Acc;

  float Kp_Yaw_Mag;
  float Ki_Yaw_Mag;
  float Kp_Acc;
  float Ki_Acc;

  float Pitch;
  float Roll;
  float Yaw;
  float Acc_Vertical;
}IMU_TypeDef;

extern IMU_TypeDef IMU;

void IMU_Update(IMU_Data_TypeDef IMU_data,float Angle_mag_yaw,u8 flag_mag_enable);

float To_180_degrees(float x);
float invSqrt(float x);
float LIMIT(float x,float min,float max);
#endif