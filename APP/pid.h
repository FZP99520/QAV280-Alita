#ifndef _PID_H
#define _PID_H
#include "stm32f10x.h"
typedef struct
{
	float P;
	float I;
	float D;
	float Error;
	float PreError;
	float Integ;
	float Deriv;
	float Output;
}PID_TypeDef;
typedef struct
{
	float pitch;
	float roll;
	float yaw;
	float height;
}PIDOutput_TypeDef;
typedef enum
{
	PIDInitError = 0,
	PIDInitReq ,
	PIDInitFinished,
}PID_STATUS;

extern u8 UpdatePID;
extern PID_STATUS PID_STA;
extern PID_TypeDef LocateX_Pos_PID;
extern PID_TypeDef LocateY_Pos_PID;
extern PID_TypeDef LocateX_Speed_PID;
extern PID_TypeDef LocateY_Speed_PID;
extern PIDOutput_TypeDef PIDOutput;
extern PID_TypeDef Pitch_angle_PID;
extern PID_TypeDef Roll_angle_PID;
extern PID_TypeDef Yaw_angle_PID;
extern PID_TypeDef Pitch_rate_PID;
extern PID_TypeDef Roll_rate_PID;
extern PID_TypeDef Yaw_rate_PID;
extern PID_TypeDef High_dis_PID;
extern PID_TypeDef High_v_PID;

void PID_Position_Cal(PID_TypeDef* PID,float target,float measure,float IntegMax,float OutputMax);
void PID_Yaw_Cal(PID_TypeDef* PID,float target,float measure,float IntegMax,float OutputMax);
void PID_Reset(PID_TypeDef* p);
void PID_Reset_All(void);
void PID_Reset_Integ(PID_TypeDef* p);
void PID_Cal_Update(void);
#endif

