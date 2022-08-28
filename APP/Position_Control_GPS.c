#include "Position_Control_GPS.h"
#include "pid.h"
#include "IncludeAll.h"
#define deg_To_Radian 0.0174533f //Pi/180
PID_TypeDef GPS_Lat_PID;
PID_TypeDef GPS_Lon_PID;
PID_TypeDef GPS_North_PID;
PID_TypeDef GPS_East_PID;

float Cos_Yaw;
float Sin_Yaw;

float Target_Roll;
float Target_Pitch;

int32_t Target_Lon;
int32_t Target_Lat;
int32_t Err_Lon;
int32_t Err_Lat;

float Speed_Flight;
float Speed_North;
float Speed_East;

float Target_North=0;
float Target_East=0;
float Target_North_fade=0;
float Target_East_fade=0;

int32_t my_deathzoom_int32(int32_t x,int32_t zoom);

void GPS_Position_PID_Para_Init(void)
{
	GPS_Lat_PID.P=1.0;
	GPS_Lat_PID.I=0.0;
	GPS_Lat_PID.D=0.0;
	GPS_Lon_PID.P=1.0;
	GPS_Lon_PID.I=0.0;
	GPS_Lon_PID.D=0.0;
	
	GPS_North_PID.P=1.0;
	GPS_North_PID.I=0.0;
	GPS_North_PID.D=0.0;
	GPS_East_PID.P=1.0;
	GPS_East_PID.I=0.0;
	GPS_East_PID.D=0.0;
	
}
//位置控制函数
void Position_Control_Update(void)
{
	Cos_Yaw=cosf(IMU.Yaw*deg_To_Radian);
	Sin_Yaw=sinf(IMU.Yaw*deg_To_Radian);
	if(Fly_sta!=FlyStaFlying)
	{
		Target_Lon=gps_data.longitude*10000000.0f;
		Target_Lat=gps_data.latitude *10000000.0f;
	}
	Err_Lat=Target_Lat-gps_data.latitude *10000000.0f;
	Err_Lon=Target_Lon-gps_data.longitude*10000000.0f;
	
	GPS_Lon_PID.Error=Err_Lon*cosf(gps_data.latitude*deg_To_Radian);
	GPS_Lat_PID.Error=Err_Lat;
	
	 Speed_North=my_deathzoom_int32(gps_data.North_velo,5);
   Speed_East=my_deathzoom_int32(gps_data.East_velo,5);   
	
	 PID_Position_Cal(&GPS_Lon_PID,Target_Lon,gps_data.latitude *10000000.0f*0.9206f,10,1.0f);
	 PID_Position_Cal(&GPS_Lat_PID,Target_Lat,gps_data.latitude *10000000.0f,10,1.0f);
	
	 PID_Position_Cal(&GPS_North_PID,Target_North,Speed_North,10,5.0f);
	 PID_Position_Cal(&GPS_East_PID,Target_East,Speed_East,10,5.0f);
	 
}
//GPS控制输出转化成Pitch和Roll
void  GPS_Positon_exchange_AHRS_Control()
{
	  Target_Pitch=-(GPS_North_PID.Output*Cos_Yaw+GPS_East_PID.Output*Sin_Yaw);
    Target_Roll=GPS_East_PID.Output*Cos_Yaw-GPS_North_PID.Output*Sin_Yaw;
}
void GPS_Control_Update(void)
{
  static u8 cnt=0;
	cnt++;
	
	if(cnt==100)
	{
		Position_Control_Update();
		cnt=0;
	}
	
}
int32_t my_deathzoom_int32(int32_t x,int32_t zoom)
{
    int32_t t;
    
    if(x>0)
    {
       	t = x - zoom;
       	if(t<0)
       	{
     		t = 0;
       	}
    }
    else
    {
       	t = x + zoom;
      	if(t>0)
      	{
       		t = 0;
       	}
    }
    return (t);
}