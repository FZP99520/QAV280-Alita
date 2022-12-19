#ifndef _hmc5883_H_
#define _hmc5883_H_

#define MAG_QMC5883 
//#define MAG_HMC5883
#include "stm32f10x.h"
#include "iic.h"
#include "device.h"
#include "IMU.h"

typedef struct MAG_Data_TypeDef_s
{
    E_DevInit_Status_TypeDef eMagInit_Status;
    E_DevCali_Status_TypeDef eMagCali_Status;
    u8 bYawCalWithIMU;
    u8 u8MAG_Status;
    u8 bMAG_Err_Flag;
    u16 u16MAG_Err_Cnt;
    s16 s16MAG_X;
    s16 s16MAG_Y;
    s16 s16MAG_Z;
    float f32Mag_x;
    float f32Mag_y;
    float f32Mag_z;
    float f32Strength;
    s16 s16MAG_X_Min;
    s16 s16MAG_X_Max;
    s16 s16MAG_Y_Min;
    s16 s16MAG_Y_Max;
    s16 s16MAG_Z_Min;
    s16 s16MAG_Z_Max;
    float f32Mag_X_Gain;
    float f32Mag_Y_Gain;
    float f32Mag_Z_Gain;
    s16 s16os_x;
    s16 s16os_y;
    s16 s16os_z;
    float f32MAG_Yaw;
}MAG_Data_TypeDef;
extern MAG_Data_TypeDef gsMAG_Data;

E_DevInit_Status_TypeDef MAG_Init(void);
E_DevUpdate_Ret_TypeDef MAG_Data_Update(IMU_Data_TypeDef sIMU_Data,u8* bYawCalWithIMU);

void MAG_Error_Det(MAG_Data_TypeDef* pMAG_Data);

/*******************************/
#endif
