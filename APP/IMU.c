#include "IMU.h"
#include "math.h"

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



IMU_TypeDef IMU={1.0f,0,0,0,0,0,1.0f,0,0,0};

void IMU_Update(IMU_Data_TypeDef IMU_data,float Angle_mag_yaw,u8 flag_mag_enable)
{
	float norm;
	float ex;
	float ey;
  float temp_q[4];
    
	float Gyro_x,Gyro_y,Gyro_z;
	float Acc_x,Acc_y,Acc_z;
	float Err_Yaw;
  Acc_x = IMU_Data.ACCEL_X;
	Acc_y = IMU_Data.ACCEL_Y;
	Acc_z = IMU_Data.ACCEL_Z;
	Gyro_x = IMU_data.gx*deg_To_Radian;//转换成弧度值
	Gyro_y = IMU_data.gy*deg_To_Radian;
	Gyro_z = IMU_data.gz*deg_To_Radian;
	
	if(Fly_sta == FlyStaRdy ||Fly_sta == FlyStaLock||Fly_sta == FlyStaUnlock)
	{
		IMU.Kp_Acc = kp_acc_fast;
		IMU.Ki_Acc = ki_acc_fast;
		IMU.Kp_Yaw_Mag = kp_yaw_mag_fast;
		IMU.Ki_Yaw_Mag = ki_yaw_mag_fast;
	}
	else if(Fly_sta == FlyStaFlying)
	{
		if(IMU.Ki_Acc == kp_acc_fast)
		{
			IMU.exInt_Acc = 0;
			IMU.eyInt_Acc = 0;
			IMU.ezInt_Acc = 0;
		}
		IMU.Kp_Acc=kp_acc_slow;
		IMU.Ki_Acc=ki_acc_slow;
    IMU.Kp_Yaw_Mag=kp_yaw_mag_slow;
    IMU.Ki_Yaw_Mag=ki_yaw_mag_slow;
	}
	else;
	norm = IMU.Matr13*Acc_x + IMU.Matr23*Acc_y + IMU.Matr33*Acc_z; //获取重力方向的加速度 点乘运算
	IMU.Acc_Vertical = (norm - Range_Acc_Setting);//减去重力加速度
	
	norm = invSqrt(Acc_x*Acc_x +  Acc_y*Acc_y + Acc_z*Acc_z);
  Acc_x = norm*Acc_x;
	Acc_y = norm*Acc_y;
	Acc_z = norm*Acc_z;
  
	ex = Acc_y*IMU.Matr33 - Acc_z*IMU.Matr23;
	ey = Acc_z*IMU.Matr13 - Acc_x*IMU.Matr33;
	IMU.exInt_Acc+=ex*IMU.Ki_Acc;
	IMU.eyInt_Acc+=ey*IMU.Ki_Acc;
	
	if(flag_mag_enable == 1)
	{
		norm = To_180_degrees(Angle_mag_yaw - IMU.Yaw);
		if(IMU.Kp_Yaw_Mag == kp_yaw_mag_slow)
		{
			if(Fly_sta != FlyStaFlying)
			{
				IMU.Kp_Yaw_Mag = fabs(norm)*0.04f;
        IMU.Kp_Yaw_Mag = LIMIT(IMU.Kp_Yaw_Mag,kp_yaw_mag_slow,0.4f);
			}
			else
			{
				IMU.Kp_Yaw_Mag=kp_yaw_mag_slow;
			}
		}
		Err_Yaw = IMU.Kp_Yaw_Mag*norm*deg_To_Radian;
	}
	else
		Err_Yaw = 0;
  
    Gyro_x=Gyro_x+IMU.Matr13*Err_Yaw+IMU.Kp_Acc*ex+IMU.exInt_Acc;
    Gyro_y=Gyro_y+IMU.Matr23*Err_Yaw+IMU.Kp_Acc*ey+IMU.eyInt_Acc;
    Gyro_z=Gyro_z+IMU.Matr33*Err_Yaw;

    temp_q[0]=IMU.q0;
    temp_q[1]=IMU.q1;
    temp_q[2]=IMU.q2;
    temp_q[3]=IMU.q3;

	  IMU.q0=temp_q[0]+(-temp_q[1]*Gyro_x-temp_q[2]*Gyro_y-temp_q[3]*Gyro_z)*halfT;
	  IMU.q1=temp_q[1]+(temp_q[0]*Gyro_x+temp_q[2]*Gyro_z-temp_q[3]*Gyro_y)*halfT;
	  IMU.q2=temp_q[2]+(temp_q[0]*Gyro_y-temp_q[1]*Gyro_z+temp_q[3]*Gyro_x)*halfT;
	  IMU.q3=temp_q[3]+(temp_q[0]*Gyro_z+temp_q[1]*Gyro_y-temp_q[2]*Gyro_x)*halfT;

	  norm=invSqrt(IMU.q0*IMU.q0+IMU.q1*IMU.q1+IMU.q2*IMU.q2+IMU.q3*IMU.q3);
	  IMU.q0=IMU.q0*norm;
	  IMU.q1=IMU.q1*norm;
	  IMU.q2=IMU.q2*norm;
	  IMU.q3=IMU.q3*norm;

	  IMU.Matr13=2.0f*(IMU.q1*IMU.q3-IMU.q0*IMU.q2);
	  IMU.Matr23=2.0f*(IMU.q0*IMU.q1+IMU.q2*IMU.q3);
	  IMU.Matr33=1.0f-2.0f*(IMU.q1*IMU.q1+IMU.q2*IMU.q2);

	  IMU.Pitch=atan2f(IMU.Matr23,IMU.Matr33)*Radian_To_deg;
    IMU.Roll=-asinf(IMU.Matr13)*Radian_To_deg;   
	  IMU.Yaw=atan2f(2.0f*(IMU.q1*IMU.q2+IMU.q0*IMU.q3),1.0f-2.0f*(IMU.q2*IMU.q2+IMU.q3*IMU.q3))*Radian_To_deg;
    IMU.Pitch=To_180_degrees(IMU.Pitch);
    IMU.Roll=To_180_degrees(IMU.Roll);
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