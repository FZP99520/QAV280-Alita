#include "control.h"
#include "math.h"
#include "pwm.h"
#include "mpu6050.h"
#include "pid.h"
#include "nrf24l01.h"
#include "ms5611.h"
#include "qmc5883.h"
#include "RC.h"
#include "led.h"
#include "IMU.h"
Fly_Status_t Fly_sta = FlyStaLock;
u8 FlyMode=0;
static u8 HighLockState=1;
u16 flag_angle_err_cnt=0;

float HighLock;

#define DT 0.005f
#define fly_thro 1000u


#define MAX_Angle   25.0f
#define Safe_Angle  10.0f

//u16 fly_thro_min=1180u;
//u16 fly_thro_min;
u16 fly_thro_min;
void Control(void)
{
	static u16 AngleLockCnt=0;
	if(Fly_sta == FlyStaLock || Fly_sta==FlyStaRdy)
	{
     if(IMU_Data.AccelOffsetFinished == 1&&IMU_Data.GyroOffsetFinished ==1 \
			   && MAG_Data.MagOffsetFinished ==1  && MS5611.OffsetFinished ==1 && \
		     PID_STA == PIDInitFinished && gps_data.gpssta==1)
			   {if(Fly_sta==FlyStaLock) Fly_sta=FlyStaRdy;}
	   else
         Fly_sta = FlyStaLock;
		 PWM_Set(&MOTOR,FlyStaLock);
		 return;
	}
	else if(Fly_sta == FlyStaUnlock) //锁定状态，检查起飞角度
	{	
		PWM_Set(&MOTOR,FlyStaUnlock);
		PID_Reset_All();
		if( fabs(IMU.Pitch) <= Safe_Angle &&fabs (IMU.Roll)<=Safe_Angle )
		{
			AngleLockCnt++;
			if(AngleLockCnt > 800u)
			{
				AngleLockCnt = 0;
				Fly_sta = FlyStaFlying;
			}
		}
		else
			AngleLockCnt = 0;
		return;
	}
	else if(Fly_sta == FlyStaAngleErr) //螺旋桨开始减速，容错
	{
		 PID_Reset_All();
         PWM_Set(&MOTOR,FlyStaAngleErr);
		 return;
	}
	/**********************/
	else if(Fly_sta == FlyStaFlying) //飞行状态，进行数据计算
	{
		 if(fabs(IMU.Pitch)>MAX_Angle || fabs(IMU.Roll) >MAX_Angle )//飞行时突然出现大角度 ，进入角度错误状态
	   {
			  flag_angle_err_cnt++;
		    if(flag_angle_err_cnt>200u)
		     { 
			     flag_angle_err_cnt=0;
			     Fly_sta = FlyStaAngleErr;
		     }
	   }
	   else 
			 flag_angle_err_cnt=0;
    fly_thro_min=RC_ctrl.height;
	  PIDOutput.pitch = Pitch_rate_PID.Output;
	  PIDOutput.roll  = Roll_rate_PID.Output;
	  PIDOutput.yaw   = Yaw_rate_PID.Output;
	  PIDOutput.height  = fly_thro_min + High_v_PID.Output;
	 
	  MOTOR.pwm1 = PIDOutput.height + PIDOutput.pitch + PIDOutput.roll - PIDOutput.yaw;
	  MOTOR.pwm2 = PIDOutput.height - PIDOutput.pitch + PIDOutput.roll + PIDOutput.yaw;
	  MOTOR.pwm3 = PIDOutput.height - PIDOutput.pitch - PIDOutput.roll - PIDOutput.yaw;
	  MOTOR.pwm4 = PIDOutput.height + PIDOutput.pitch - PIDOutput.roll + PIDOutput.yaw; 
		 
	  PWM_Set(&MOTOR,FlyStaFlying);
  }
	else;
	
}
void High_Handle(void)
{
//	if(RX_cnt.high ==0 )
//	{
//		if(HighLockState == 0)
//		{
//			HighLock = Altitude;
//			HighLockState = 1;
//		}
//	}
//	else
//	{
//		HighLockState = 0;
//		HighLock = Alt.now;
//	}
//	PID_Position_Cal(&High_dis_PID,HighLock ,Alt.now );
//	PID_Position_Cal(&High_v_PID,High_dis_PID.Output,Alt.vel);
	
}


