#ifndef _RC_H
#define _RC_H

#include "stm32f10x.h"
#include "gps.h"
#include "pid.h"
typedef struct
{
   u16 m_bHead;//帧头
   u8  m_bCMD;
   u8  m_bLen;
   u8  m_bSum;//校验
}RC_Controller2Quat_Typedef;

typedef struct
{
		u8 send_version;
		u8 send_status;
		u8 send_sensor;
		u8 send_gps;
		u8 send_pid1;
		u8 send_pid2;
		u8 send_pid3;
		u8 send_pid4;
		u8 send_pid5;
		u8 send_pid6;
		u8 send_rcdata;
		u8 send_motopwm;
		u8 send_power;
	  
	  u8 send_check_pid1;   
	  u8 send_check_pid2;
	  u8 send_check_pid3;
		u8 send_accel_cal_result;
	  u8 send_gyro_cal_result;
	  u8 send_mag_cal_result;
	  u8 send_baro_cal_result;
}RC_FLAG_TypeDef;

typedef struct
{
	s16 height;
	s16 yaw;
	s16 roll;
	s16 pitch;
	float exp_height;//油门输入期望高度
	float exp_yaw;
	float exp_roll;
	float exp_pitch;
	float exp_height_rate;//油门输入期望高度速度
	float exp_yaw_rate;
	float exp_roll_rate;
	float exp_pitch_rate;
}RC_CTRL_TypeDef;

extern RC_CTRL_TypeDef RC_ctrl;
extern RC_FLAG_TypeDef RC_flag;
#define MSG_ACCEL   0x01
#define MSG_GYRO    0x02
#define MSG_MAG     0x03
#define MSG_BAR     0x04
#define MSG_GPS     0x30
#define MSG_SUCCEED 0x01
#define MSG_FAIL    0xE1


void RC_Send_MSG(u8 id,u8 result);
void RC_Send_STATUS(float angle_rol, float angle_pit, float angle_yaw, float alt,float speed_height,u8 fly_model, u8 armed);
void RC_Send_PID(u8 group,PID_TypeDef p1,PID_TypeDef p2,PID_TypeDef p3);
void RC_Send_Sensor(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void RC_Send_GPS(nmea_msg gps);
void RC_Send_Power(float voltage,float current);
void RC_Send_Sensor_Status(void);
void RC_Send_CHECK(u8 head,u8 check_sum); //返回校验

void RC_Payload(u8 receive_result);
u8 RC_Anl_BUFF(u8* buff,u8 len);
u8 RC_Anl_CMD1(u8* buff);
void RC_Anl_CMD2(u8* buff);
void RC_Anl_CTRL(u8* buff);
void RC_Anl_PID(u8* buff,u8 sum);

#endif

