#include "pid.h"
#include "systick.h"
#include "flash.h"
#include "Position_Control_GPS.h"
u8 UpdatePID=0;
#define dt 0.005f

PID_STATUS PID_STA=PIDInitReq;
u8 data_buf[18];
PID_TypeDef LocateX_Pos_PID;
PID_TypeDef LocateY_Pos_PID;
PID_TypeDef LocateX_Speed_PID;
PID_TypeDef LocateY_Speed_PID;
PID_TypeDef Pitch_angle_PID;
PID_TypeDef Roll_angle_PID;
PID_TypeDef Yaw_angle_PID;
PID_TypeDef Pitch_rate_PID;
PID_TypeDef Roll_rate_PID;
PID_TypeDef Yaw_rate_PID;
PID_TypeDef High_dis_PID;
PID_TypeDef High_v_PID;

PIDOutput_TypeDef PIDOutput;
u8 YawLockState=0;
float Target_Yaw;
#define DT 0.0025f
//T=2.5ms
void PID_Cal_Update(void)
{
	//Pitch and Roll PID Calculation
	//
	if(Fly_sta!=FlyStaFlying)
  {
	  Target_Yaw = IMU.Yaw;
	  return;
	}
//	RC_ctrl.exp_roll=((float)RC_ctrl.roll-1500u)*0.03f; //去掉油门偏移，转化成° 0.05=25°/500 500是油门最大量
//	RC_ctrl.exp_pitch=((float)RC_ctrl.pitch-1500u)*0.03f; //去掉油门偏移，转化成° 0.05=25°/500 500是油门最大量
	if(fabs((float)RC_ctrl.roll-1500.0f) <20.0f) 
	{
		PID_Reset_Integ(&Roll_angle_PID);
		PID_Reset_Integ(&Roll_rate_PID);//没有油门输入清零积分
	}
	if(fabs((float)RC_ctrl.pitch-1500.0f) <20.0f)
	{
		PID_Reset_Integ(&Pitch_rate_PID);//没有油门输入清零积分
		PID_Reset_Integ(&Pitch_angle_PID);
	}
	//位置控制
	
	//角度控制计算
	PID_Position_Cal(&Roll_angle_PID,Target_Roll,IMU.Roll,100.0f,90.0f);   //最大期望速度 90度/s
	PID_Position_Cal(&Pitch_angle_PID,Target_Pitch,IMU.Pitch,100.0f,90.0f); //最大期望速度 90度/s
	//速率PID计算
	PID_Position_Cal(&Roll_rate_PID,Roll_angle_PID.Output, IMU_Data.gy,50,180.0f);
	PID_Position_Cal(&Pitch_rate_PID,Pitch_angle_PID.Output,IMU_Data.gx,50,180.0f);
			 /************Yaw PID Caculation*********/
		RC_ctrl.exp_yaw_rate=((float)RC_ctrl.yaw-1500.0f)*0.06f; //去掉油门偏移，转化成° 0.06=30°/500 500是油门最大量
	  if(fabs((float)RC_ctrl.yaw-1500.0f) < 10.0f )
	 {
		 if(YawLockState == 0)
		 {
			 Target_Yaw = IMU.Yaw;
			 YawLockState = 1;
		 }
	 }
	 else
	 {
		 YawLockState = 0;
		 Target_Yaw += RC_ctrl.exp_yaw_rate*DT; //把输入偏航角油门积分成角度
		 Target_Yaw=To_180_degrees(Target_Yaw);
	 }
	  PID_Yaw_Cal(&Yaw_angle_PID,Target_Yaw,IMU.Yaw,100.0f,30.0f);
	  PID_Yaw_Cal(&Yaw_rate_PID,Yaw_angle_PID.Output,IMU_Data.gz,10.0f,100.0f);
	
}
//位置式PID计算函数
//PID ：指向将被计算的PID结构体
//target：目标值
//measure：测量值
//Integ:  误差积分最大值
//OutputMax：输出最大值
void PID_Position_Cal(PID_TypeDef* PID,float target,float measure,float IntegMax,float OutputMax)
{
	PID->Error = target - measure; //计算误差
	PID->Integ +=PID->Error*DT; //误差积分
	PID->Integ = LIMIT(PID->Output,-IntegMax,IntegMax);   //积分限幅
	PID->Deriv = PID->Error - PID->PreError;//计算微分
	PID->Output = PID->P*PID->Error + PID->I*PID->Integ + PID->D*PID->Deriv;//输出计算
	PID->Output = LIMIT(PID->Output,-OutputMax,OutputMax);//输出限幅
	PID->PreError = PID->Error;//保存上一次误差
}
void PID_Yaw_Cal(PID_TypeDef* PID,float target,float measure,float IntegMax,float OutputMax)
{
	PID->Error = target - measure;
	PID->Error=To_180_degrees(PID->Error);
	PID->Integ +=PID->Error*DT;
  PID->Integ = LIMIT(PID->Output,-IntegMax,IntegMax); 
	PID->Deriv = PID->Error - PID->PreError;
	PID->Output = PID->P*PID->Error + PID->I*PID->Integ + PID->D*PID->Deriv;
	PID->Output = LIMIT(PID->Output,-OutputMax,OutputMax);//输出限幅
	PID->PreError = PID->Error;
}
void PID_Reset(PID_TypeDef* p)
{
	p->Error=0;
	p->PreError=0;
	p->Integ=0;
	p->Output=0;
	p->Deriv=0;
}
void PID_Reset_All(void)
{
	PID_Reset(&Roll_angle_PID);
	PID_Reset(&Pitch_angle_PID);
	PID_Reset(&Yaw_angle_PID);
	PID_Reset(&High_dis_PID);
	PID_Reset(&Roll_rate_PID);
	PID_Reset(&Pitch_rate_PID);
	PID_Reset(&Yaw_rate_PID);
	PID_Reset(&High_v_PID);
}
void PID_Reset_Integ(PID_TypeDef* p)
{
	p->Integ=0;
}
