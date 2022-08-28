#include "kalman.h"

#define SampleT 0.020f
#define SimpltT_2 SampleT*SampleT
#define SimpltT_3 SampleT*SampleT*SampleT
#define SimpltT_4 SampleT*SampleT*SampleT*SampleT
/***********high combination**************/
float Pk_last[3][3]={1,0,0,0,1,0,0,0,1};
static float Q[3]={0.001f,0.001f,0.001f};
static float R[2]={30.0f,100.0f};
float Xk_last[3]={0,0,0};

float K[3][2];

void Kalman_Height(float ms_alt,float Az,float* height,float* speed)
{
	float Pk_pre[3][3];
	float Xk_pre[3];
	float o1,inv_o1;
	float Err[2];
	//先验估计
	Xk_pre[0]=Xk_last[0]+SampleT*Xk_last[1]+0.5*SampleT*SampleT*Xk_last[2]; 
	Xk_pre[1]=Xk_last[1]+SampleT*Xk_last[2];
	Xk_pre[2]=Xk_last[2];
	//协方差矩阵估计
	Pk_pre[0][0]=Pk_last[0][0]+Q[0]+SampleT*(Pk_last[0][1]+Pk_last[1][0]) \
	              +SimpltT_2*(0.5*Pk_last[0][2]+0.5*Pk_last[2][0]+Pk_last[1][1]) \
	              +0.5*SimpltT_3*(Pk_last[2][1]+Pk_last[1][2]) \
	              +0.5*SimpltT_3*Pk_last[2][2];
	Pk_pre[0][1]=Pk_last[0][1]+SampleT*(Pk_last[0][2]+Pk_last[1][1]) \
	             +SimpltT_2*(0.5*Pk_last[2][1]+Pk_last[1][2]) \
	             +SimpltT_3*0.5*Pk_last[2][2];
	Pk_pre[0][2]=Pk_last[0][2]+SampleT*Pk_last[1][2]+0.5*SimpltT_2*Pk_last[2][2];
	Pk_pre[1][0]=Pk_last[1][0]+0.5*SimpltT_2*(Pk_last[1][2]+SampleT*Pk_last[2][2])+SampleT*Pk_last[2][0] \
	             +SampleT*(Pk_last[1][1]+SampleT*Pk_last[2][1]);
	Pk_pre[1][1]=Pk_last[1][1]+Q[1]+SampleT*(Pk_last[1][2]+Pk_last[2][1])+SimpltT_2*Pk_last[2][2];
	Pk_pre[1][2]=Pk_last[1][2]+SampleT*Pk_last[2][2];
	Pk_pre[2][0]=Pk_last[2][0]+SampleT*Pk_last[2][1]+0.5*SimpltT_2*Pk_last[2][2];
	Pk_pre[2][1]=Pk_last[2][1]+SampleT*Pk_last[2][2];
	Pk_pre[2][2]=Pk_last[2][2]+Q[2];
	//计算卡尔曼增益
	o1=Pk_pre[0][0]*Pk_pre[2][2]-Pk_pre[0][2]*Pk_pre[2][0]+Pk_pre[0][0]*R[1]+Pk_pre[2][2]*R[0]+R[0]*R[1];
	inv_o1=1.0f/o1;
	K[0][0]=inv_o1*(Pk_pre[0][0]*(Pk_pre[2][2]+R[1])-Pk_pre[0][2]*Pk_pre[2][0]);
	K[0][1]=inv_o1*(Pk_pre[0][2]*(Pk_pre[0][0]+R[0])-Pk_pre[0][0]*Pk_pre[0][2]);
	K[1][0]=inv_o1*(Pk_pre[1][0]*(Pk_pre[2][2]+R[1])-Pk_pre[1][2]*Pk_pre[2][0]);
	K[1][1]=inv_o1*(Pk_pre[1][2]*(Pk_pre[0][0]+R[0])-Pk_pre[0][2]*Pk_pre[1][0]);
	K[2][0]=inv_o1*(Pk_pre[2][0]*(Pk_pre[2][2]+R[1])-Pk_pre[2][0]*Pk_pre[2][2]);
	K[2][1]=inv_o1*(Pk_pre[2][2]*(Pk_pre[0][0]+R[0])-Pk_pre[0][2]*Pk_pre[2][0]);
	//计算输出
	Err[0]=ms_alt-Xk_pre[0];
	Err[1]=Az-Xk_pre[2];
	Xk_last[0]=Xk_pre[0]+K[0][0]*Err[0]+K[0][1]*Err[1];
	Xk_last[1]=Xk_pre[1]+K[1][0]*Err[0]+K[1][1]*Err[1];
	Xk_last[2]=Xk_pre[2]+K[2][0]*Err[0]+K[2][1]*Err[1];
	//更新协方差矩阵
	Pk_last[0][0]=-K[0][1]*Pk_pre[2][0]-Pk_pre[0][0]*(K[0][0]-1.0f);
	Pk_last[0][1]=-K[0][1]*Pk_pre[2][1]-Pk_pre[0][1]*(K[0][0]-1.0f);
	Pk_last[0][2]=-K[0][1]*Pk_pre[2][2]-Pk_pre[0][2]*(K[0][0]-1.0f);
	Pk_last[1][0]=Pk_pre[1][0]-K[1][0]*Pk_pre[0][0]-K[1][1]*Pk_pre[2][0];
	Pk_last[1][1]=Pk_pre[1][1]-K[1][0]*Pk_pre[0][1]-K[1][1]*Pk_pre[2][1];
	Pk_last[1][2]=Pk_pre[1][2]-K[1][0]*Pk_pre[0][2]-K[1][1]*Pk_pre[2][2];
	Pk_last[2][0]=-K[2][0]*Pk_pre[0][0]-Pk_pre[2][0]*(K[2][1]-1.0f);
	Pk_last[2][1]=-K[2][0]*Pk_pre[0][1]-Pk_pre[2][1]*(K[2][1]-1.0f);
	Pk_last[2][2]=-K[2][0]*Pk_pre[0][2]-Pk_pre[2][2]*(K[2][1]-1.0f);
	
	*height=Xk_last[0];
	*speed=Xk_last[1];
}
#define KALMAN_R 15
#define KALMAN_Q 0.03f
void KalmanFilter_V(float* ResrcData)
{
	double R = KALMAN_R;
	double Q = KALMAN_Q;
	static double x_last;
	double x_mid;
	double x_now;
	static float p_last;
	double p_mid;
	double p_now;
	double kg;
	
	x_mid = x_last;
	p_mid = p_last + Q;
	kg = p_mid/(p_mid+R);
	x_now = x_mid + kg*(*ResrcData - x_mid);
	p_now = (1-kg)*p_mid;
	p_last = p_now;
	x_last = x_now;
	*ResrcData = x_now;
}