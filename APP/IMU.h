#ifndef IMU_H_
#define IMU_H_

#include "stm32f10x.h"
#include "mpu6050.h"
#include "control.h"

typedef enum
{
    E_IMU_Update_Fail,
    E_IMU_Update_Doing,
    E_IMU_Update_Done
}IMU_Update_Status_TypeDef;

typedef struct 
{
    IMU_Update_Status_TypeDef eIMU_Update_Status;
    float f32q0;
    float f32q1;
    float f32q2;
    float f32q3;
    float f32Matr13;//2.0f * (q1*q3 - q0*q2);
    float f32Matr23;//2.0f * (q0*q1 + q2*q3);
    float f32Matr33;//1.0f - 2.0f*(q1*q1 + q2*q2);
    float f32exInt_Acc;
    float f32eyInt_Acc;
    float f32ezInt_Acc;
    float f32Kp_Yaw_Mag;
    float f32Ki_Yaw_Mag;
    float f32Kp_Acc;
    float f32Ki_Acc;
    float f32Pitch;
    float f32Roll;
    float f32Yaw;
    float f32Acc_Vertical;
}IMU_Data_TypeDef;

extern IMU_Data_TypeDef gsIMU_Data;

IMU_Update_Status_TypeDef IMU_Update(MPU_Data_TypeDef IMU_data,float Angle_mag_yaw,u8 flag_mag_enable);

float To_180_degrees(float x);
float invSqrt(float x);
float LIMIT(float x,float min,float max);
#endif
