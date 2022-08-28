#include "height.h"
#include "kalman.h"
#define standard_atmos 101325.0f
#define TH_Height_Max_Default 100u
#define DT 0.0025f //2.5ms

MoveAvarageFilter_TypeDef Filter_pressure={4,0,0,{0}}; //气压滑动平均滤波
MoveAvarageFilter_TypeDef Filter_Acc_Vertical={4,0,0,{0}};//加速度滑动平均滤波
MoveAvarageFilter_TypeDef Filter_bar_height={8,0,0,{0}};//气压计高度滑动平均滤波

float exp_height_speed=0;//目标速度mm/s
float exp_height_speed_Throttle=0;//油门输入目标速度mm/s
float m_TH_Height_Max=TH_Height_Max_Default;//最大高度

float exp_height;//目标高度 mm
float exp_height_auto;//目标高度mm
uint8_t flag_exp_height_auto=0;//目标高度mm

float exp_height_speed_fly_compent=0;//侧飞过程中油门掉高的补偿量
u16 Cnt_exp_height_speed_fly_compent=0;
float m_exp_height_speed_Limit;//高度控制的速度阀值设定

float real_height;
float last_real_height;
float real_speed_height;


float m_height_Altitude_real=0;


void Press_To_Height(MS5611_Typedef* height,u8 flag_filter)
{
	  float temp;
	  if(flag_filter)
         height->RP = MoveAvarageFilter(&Filter_pressure,height->RP);
	  height->Diff_P=(float)height->RP - (float)height->Reff_P;
	  temp=44330.0f*(1.0f-pow(((float)height->RP/101325.0f),0.1902949572f));//1/5.255=0.1902949572
    m_height_Altitude_real=temp;    
    height->Altitude_P=temp;//unit mm
    height->Altitude_Diff_P=height->Altitude_P - height->Altitude_Reff_P;
}

void Height_Control_Update(void)//T=20ms
{
	
	float kalman_speed;
	
	if(Fly_sta!=FlyStaFlying) exp_height=0;
//	if(RC_ctrl.height==1)      exp_height+=0.02f;
//	else if(RC_ctrl.height==-1) exp_height-=0.02f;
//	else;
	exp_height+=(RC_ctrl.height-1500)*0.5*0.02f;
	exp_height=LIMIT(exp_height,0,1.0f);
	
	IMU.Acc_Vertical=MoveAvarageFilter(&Filter_Acc_Vertical,IMU.Acc_Vertical);
	MS5611.Altitude_Diff_P=MoveAvarageFilter(&Filter_bar_height,MS5611.Altitude_Diff_P);
	Kalman_Height(MS5611.Altitude_Diff_P,IMU.Acc_Vertical*0.0011963f,&real_height,&kalman_speed);
  
	real_speed_height=(real_height-last_real_height)*50.0f;//50=1/0.020
	real_speed_height=0.8f*real_speed_height+0.2f*kalman_speed;
	KalmanFilter_V(&real_speed_height);
	last_real_height=real_height;
	
	PID_Position_Cal(&High_dis_PID,exp_height,real_height,100,1.0f);
	PID_Position_Cal(&High_v_PID,High_dis_PID.Output,real_speed_height,100,300.0f);
}
