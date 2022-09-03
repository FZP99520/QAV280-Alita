#ifndef __MPU6050_H
#define __MPU6050_H


#include "stm32f10x.h"
#include "filter.h"
#include "device.h"

#define  AccelLSB          8192u
#define  AccelCoefficient  0.000122f // 1/8192
#define  GyroLSB           32.8f
#define  GyroCoefficient   0.030f //1/32.8
#define  Range_Acc_Setting 8192u

typedef struct MPU_Data_TypeDef_s
{
    E_DevInit_Status_TypeDef eAccelInit_Status;
    E_DevInit_Status_TypeDef eGyroInit_Status;
    E_DevCali_Status_TypeDef eAccelCali_Status;
    E_DevCali_Status_TypeDef eGyroCali_Status;
    E_DevReadWrite_Ret_TypeDef eAccelGyro_Read_Ret;
    E_DevReadWrite_Ret_TypeDef eAccelGyro_Write_Ret;
    u8  bGet_ORG_DataOK;
    u8  bGet_QuatOK;
    s16 s16ACCEL_X;
    s16 s16ACCEL_Y;
    s16 s16ACCEL_Z;
    s16 s16TEMP;
    s16 s16GYRO_X;
    s16 s16GYRO_Y;
    s16 s16GYRO_Z;
    s16 s16os_accel_x;
    s16 s16os_accel_y;
    s16 s16os_accel_z;
    s16 s16os_gyro_x;
    s16 s16os_gyro_y;
    s16 s16os_gyro_z;
    float fax;
    float fay;
    float faz;
    float fgx;
    float fgy;
    float fgz;
}MPU_Data_TypeDef;

extern MPU_Data_TypeDef gsMPU_Data;

E_DevInit_Status_TypeDef MPU6050_Init(void);//陀螺仪初始化，同时初始化IIC
E_DevUpdate_Ret_TypeDef MPU_Data_Update(void);


extern MoveAvarageFilter_TypeDef Filter_Acc_X;
extern MoveAvarageFilter_TypeDef Filter_Acc_Y;
extern MoveAvarageFilter_TypeDef Filter_Acc_Z;
extern MoveAvarageFilter_TypeDef Filter_Gyro_X;
extern MoveAvarageFilter_TypeDef Filter_Gyro_Y;
extern MoveAvarageFilter_TypeDef Filter_Gyro_Z;
//****************************************
#define	SMPLRT_DIV      0x19    //陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG          0x1A    //低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG     0x1B    //陀螺仪自检及测量范围
#define	ACCEL_CONFIG    0x1C    //加速计自检、测量范围及高通滤波频率
#define FIFO_EN         0x23
#define INT_EN          0x38
#define	ACCEL_XOUT_H    0x3B
#define	ACCEL_XOUT_L    0x3C
#define	ACCEL_YOUT_H    0x3D
#define	ACCEL_YOUT_L    0x3E
#define	ACCEL_ZOUT_H    0x3F
#define	ACCEL_ZOUT_L    0x40
#define	TEMP_OUT_H      0x41
#define	TEMP_OUT_L      0x42
#define	GYRO_XOUT_H     0x43
#define	GYRO_XOUT_L     0x44
#define	GYRO_YOUT_H     0x45
#define	GYRO_YOUT_L     0x46
#define	GYRO_ZOUT_H     0x47
#define	GYRO_ZOUT_L     0x48
#define	PWR_MGMT_1      0x6B    //电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I        0x75    //IIC地址寄存器(默认数值0x68，只读)
#define	MPU_SlaveAddress    0xD0    //IIC写入时的地址字节数据，+1为读取



#endif
