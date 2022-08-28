#ifndef __MPU6050_H
#define __MPU6050_H


#include "stm32f10x.h"
#include "filter.h"

#define  AccelLSB          8192u
#define  AccelCoefficient  0.000122f // 1/8192
#define  GyroLSB           32.8f
#define  GyroCoefficient   0.030f //1/32.8
#define  Range_Acc_Setting 8192u

typedef enum
{
    E_Offset_Min,
    E_OffsetReq,
    E_OffsetFinish,
    E_Offset_NumMax
}E_Device_Status_TypeDef;

typedef struct
{
    E_Device_Status_TypeDef eAccel_Offset_Status;
    E_Device_Status_TypeDef eGyro_Offset_Status;
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
}IMU_Data_TypeDef;

extern IMU_Data_TypeDef gsIMU_Data;

u8 MPU6050_Init(void);//�����ǳ�ʼ����ͬʱ��ʼ��IIC
void IMU_Data_Update(void);

extern MoveAvarageFilter_TypeDef Filter_Acc_X;
extern MoveAvarageFilter_TypeDef Filter_Acc_Y;
extern MoveAvarageFilter_TypeDef Filter_Acc_Z;
extern MoveAvarageFilter_TypeDef Filter_Gyro_X;
extern MoveAvarageFilter_TypeDef Filter_Gyro_Y;
extern MoveAvarageFilter_TypeDef Filter_Gyro_Z;
//****************************************
#define	SMPLRT_DIV      0x19    //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG          0x1A    //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG     0x1B    //�������Լ켰������Χ
#define	ACCEL_CONFIG    0x1C    //���ټ��Լ졢������Χ����ͨ�˲�Ƶ��
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
#define	PWR_MGMT_1      0x6B    //��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I        0x75    //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	MPU_SlaveAddress    0xD0    //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ



#endif
