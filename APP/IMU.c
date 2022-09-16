#include "IMU.h"
#include "math.h"
#include "qmc5883.h"

#define Radian_To_deg 57.29578f
#define deg_To_Radian 0.0174533f //Pi/180
#define halfT 0.00125f //0.5*0.0025

#define kp_acc_fast 2.0f
#define ki_acc_fast 0.02f
#define kp_acc_slow 0.2f
#define ki_acc_slow 0.0025f
#define kp_yaw_mag_fast 1.5f
#define ki_yaw_mag_fast 0.02f
#define kp_yaw_mag_slow 0.01f
#define ki_yaw_mag_slow 0.00f

IMU_Data_TypeDef gsIMU_Data=
{
    .f32q0 = 1.0f,
    .f32q1 = 0.0f,
    .f32q2 = 0.0f,
    .f32q3 = 0.0f
};

IMU_Update_Status_TypeDef IMU_Update(MPU_Data_TypeDef sMPU_data,float Angle_mag_yaw,u8 flag_mag_enable)
{
    float f32norm;
    float f32ex;
    float f32ey;
    float pf32temp_q[4];
    
    float f32Gyro_x,f32Gyro_y,f32Gyro_z;
    float f32Acc_x,f32Acc_y,f32Acc_z;
    float f32Err_Yaw;
    f32Acc_x = sMPU_data.s16ACCEL_X;
    f32Acc_y = sMPU_data.s16ACCEL_Y;
    f32Acc_z = sMPU_data.s16ACCEL_Z;
    f32Gyro_x = sMPU_data.f32Gyro_x*deg_To_Radian;//转换成弧度值
    f32Gyro_y = sMPU_data.f32Gyro_y*deg_To_Radian;
    f32Gyro_z = sMPU_data.f32Gyro_z*deg_To_Radian;

    if(Fly_sta == FlyStaRdy ||Fly_sta == FlyStaLock||Fly_sta == FlyStaUnlock)
    {
        gsIMU_Data.f32Kp_Acc = kp_acc_fast;
        gsIMU_Data.f32Ki_Acc = ki_acc_fast;
        gsIMU_Data.f32Kp_Yaw_Mag = kp_yaw_mag_fast;
        gsIMU_Data.f32Ki_Yaw_Mag = ki_yaw_mag_fast;
    }
    else if(Fly_sta == FlyStaFlying)
    {
        if(gsIMU_Data.f32Ki_Acc == kp_acc_fast)
        {
            gsIMU_Data.f32exInt_Acc = 0;
            gsIMU_Data.f32eyInt_Acc = 0;
            gsIMU_Data.f32ezInt_Acc = 0;
        }
        gsIMU_Data.f32Kp_Acc = kp_acc_slow;
        gsIMU_Data.f32Ki_Acc = ki_acc_slow;
        gsIMU_Data.f32Kp_Yaw_Mag = kp_yaw_mag_slow;
        gsIMU_Data.f32Ki_Yaw_Mag = ki_yaw_mag_slow;
    }
    else;
    f32norm = gsIMU_Data.f32Matr13*f32Acc_x + gsIMU_Data.f32Matr23*f32Acc_y + gsIMU_Data.f32Matr33*f32Acc_z; //获取重力方向的加速度 点乘运算
    gsIMU_Data.f32Acc_Vertical = (f32norm - Range_Acc_Setting);//减去重力加速度

    f32norm = invSqrt(f32Acc_x*f32Acc_x +  f32Acc_y*f32Acc_y + f32Acc_z*f32Acc_z);
    f32Acc_x = f32norm*f32Acc_x;
    f32Acc_y = f32norm*f32Acc_y;
    f32Acc_z = f32norm*f32Acc_z;
  
    f32ex = f32Acc_y*gsIMU_Data.f32Matr33 - f32Acc_z*gsIMU_Data.f32Matr23;
    f32ey = f32Acc_z*gsIMU_Data.f32Matr13 - f32Acc_x*gsIMU_Data.f32Matr33;
    gsIMU_Data.f32exInt_Acc+=f32ex*gsIMU_Data.f32Ki_Acc;
    gsIMU_Data.f32eyInt_Acc+=f32ey*gsIMU_Data.f32Ki_Acc;
    
    if(flag_mag_enable == TRUE)
    {
        f32norm = To_180_degrees(Angle_mag_yaw - gsIMU_Data.f32Yaw);
        if(gsIMU_Data.f32Kp_Yaw_Mag == kp_yaw_mag_slow)
        {
            if(Fly_sta != FlyStaFlying)
            {
                gsIMU_Data.f32Kp_Yaw_Mag = fabs(f32norm)*0.04f;
                gsIMU_Data.f32Kp_Yaw_Mag = LIMIT(gsIMU_Data.f32Kp_Yaw_Mag,kp_yaw_mag_slow,0.4f);
            }
            else
            {
                gsIMU_Data.f32Kp_Yaw_Mag=kp_yaw_mag_slow;
            }
        }
        f32Err_Yaw = gsIMU_Data.f32Kp_Yaw_Mag*f32norm*deg_To_Radian;
    }
    else
        f32Err_Yaw = 0;
  
    f32Gyro_x=f32Gyro_x+gsIMU_Data.f32Matr13*f32Err_Yaw+gsIMU_Data.f32Kp_Acc*f32ex+gsIMU_Data.f32exInt_Acc;
    f32Gyro_y=f32Gyro_y+gsIMU_Data.f32Matr23*f32Err_Yaw+gsIMU_Data.f32Kp_Acc*f32ey+gsIMU_Data.f32eyInt_Acc;
    f32Gyro_z=f32Gyro_z+gsIMU_Data.f32Matr33*f32Err_Yaw;

    pf32temp_q[0]=gsIMU_Data.f32q0;
    pf32temp_q[1]=gsIMU_Data.f32q1;
    pf32temp_q[2]=gsIMU_Data.f32q2;
    pf32temp_q[3]=gsIMU_Data.f32q3;

    gsIMU_Data.f32q0=pf32temp_q[0]+(-pf32temp_q[1]*f32Gyro_x-pf32temp_q[2]*f32Gyro_y-pf32temp_q[3]*f32Gyro_z)*halfT;
    gsIMU_Data.f32q1=pf32temp_q[1]+(pf32temp_q[0]*f32Gyro_x+pf32temp_q[2]*f32Gyro_z-pf32temp_q[3]*f32Gyro_y)*halfT;
    gsIMU_Data.f32q2=pf32temp_q[2]+(pf32temp_q[0]*f32Gyro_y-pf32temp_q[1]*f32Gyro_z+pf32temp_q[3]*f32Gyro_x)*halfT;
    gsIMU_Data.f32q3=pf32temp_q[3]+(pf32temp_q[0]*f32Gyro_z+pf32temp_q[1]*f32Gyro_y-pf32temp_q[2]*f32Gyro_x)*halfT;

    f32norm=invSqrt(gsIMU_Data.f32q0*gsIMU_Data.f32q0+gsIMU_Data.f32q1*gsIMU_Data.f32q1+gsIMU_Data.f32q2*gsIMU_Data.f32q2+gsIMU_Data.f32q3*gsIMU_Data.f32q3);
    gsIMU_Data.f32q0=gsIMU_Data.f32q0*f32norm;
    gsIMU_Data.f32q1=gsIMU_Data.f32q1*f32norm;
    gsIMU_Data.f32q2=gsIMU_Data.f32q2*f32norm;
    gsIMU_Data.f32q3=gsIMU_Data.f32q3*f32norm;

    gsIMU_Data.f32Matr13=2.0f*(gsIMU_Data.f32q1*gsIMU_Data.f32q3-gsIMU_Data.f32q0*gsIMU_Data.f32q2);
    gsIMU_Data.f32Matr23=2.0f*(gsIMU_Data.f32q0*gsIMU_Data.f32q1+gsIMU_Data.f32q2*gsIMU_Data.f32q3);
    gsIMU_Data.f32Matr33=1.0f-2.0f*(gsIMU_Data.f32q1*gsIMU_Data.f32q1+gsIMU_Data.f32q2*gsIMU_Data.f32q2);

    gsIMU_Data.f32Pitch=atan2f(gsIMU_Data.f32Matr23,gsIMU_Data.f32Matr33)*Radian_To_deg;
    gsIMU_Data.f32Roll=-asinf(gsIMU_Data.f32Matr13)*Radian_To_deg;   
    gsIMU_Data.f32Yaw=atan2f(2.0f*(gsIMU_Data.f32q1*gsIMU_Data.f32q2+gsIMU_Data.f32q0*gsIMU_Data.f32q3),1.0f-2.0f*(gsIMU_Data.f32q2*gsIMU_Data.f32q2+gsIMU_Data.f32q3*gsIMU_Data.f32q3))*Radian_To_deg;
    gsIMU_Data.f32Pitch=To_180_degrees(gsIMU_Data.f32Pitch);
    gsIMU_Data.f32Roll=To_180_degrees(gsIMU_Data.f32Roll);
    
    return E_IMU_Update_Done;
}

float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;

  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));

  return y;
}
float To_180_degrees(float x)
{
    if(x>360)
    {
        x=360;
    }
    else if(x<-360)
    {
        x=-360;
    }
    return ( x>180 ? (x-360) : ( x<-180 ? (x+360) : x ) );
}
float LIMIT(float x,float min,float max)
{
    if(x>max) x=max;
    else if(x<min) x=min;
    else;
    return x;
}
