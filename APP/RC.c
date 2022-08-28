#include "IncludeAll.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	
u8 data_buff[32];
u8 sum_pid[3];

RC_CTRL_TypeDef RC_ctrl;
RC_FLAG_TypeDef RC_flag;
//********返回数据**********//
static void RC_Feedback_Send(u8* buff,u8 len)
{
	 NRF_W_ACK_PAYLOAD(buff,0,len);
}
void RC_Send_MSG(u8 id,u8 result);
/************************************/
void RC_Payload(u8 receive_result)
{
	static u8 i=0;
	static u8 cnt=0,num=4;
	static u8 status_cnt 	= 1;
	static u8 sensor_cnt 	= 2;
	static u8 gps_cnt 	  = 3;
//	static u8 motopwm_cnt	= 4;
	static u8 power_cnt		=	4;
	/*******************************/
	if(receive_result !=0x00 ||receive_result !=0xFF )
	{
		/************传感器校正结果返回*********/
		if(RC_flag.send_accel_cal_result||
			 RC_flag.send_gyro_cal_result||
		   RC_flag.send_mag_cal_result||
		   RC_flag.send_baro_cal_result)
		{
			 if(RC_flag.send_accel_cal_result)
		  {
			    if(IMU_Data.AccelOffsetFinished) 
			   {
				   RC_flag.send_accel_cal_result=0;
				   RC_Send_MSG(MSG_ACCEL,MSG_SUCCEED);//给遥控返回校准成功结果
			   }
			   else 
					 RC_Send_STATUS(IMU.Roll,IMU.Pitch,IMU.Yaw,real_height,real_speed_height,FlyMode,Fly_sta);
		  }
		 else if(RC_flag.send_gyro_cal_result)
		  {
			    if(IMU_Data.GyroOffsetFinished) 
			   {
				   RC_flag.send_gyro_cal_result=0;
				   RC_Send_MSG(MSG_GYRO,MSG_SUCCEED);
			   }
			   else 
					 RC_Send_MSG(MSG_GYRO,MSG_FAIL);
		  }
		 else if(RC_flag.send_mag_cal_result)
		  {
			   if(MAG_Data.MagOffsetFinished)
			   {
					  RC_flag.send_mag_cal_result=0;
		        RC_Send_MSG(MSG_MAG,MSG_SUCCEED);
				 }
				 else
					 RC_Send_MSG(MSG_MAG,MSG_FAIL);
		  }
		 else if(RC_flag.send_baro_cal_result)
		  {
			   if(MS5611.OffsetFinished)
			   {
					 RC_flag.send_baro_cal_result=0;
			     RC_Send_MSG(MSG_BAR,MSG_SUCCEED);
			   }
				 else
		 			 RC_Send_MSG(MSG_BAR,MSG_FAIL);
		  }
		 else;
		 return;
	  }
		/***********************/
		/************返回PID数据**********/
		if(RC_flag.send_pid1||RC_flag.send_pid2||RC_flag.send_pid3)
		{
			if(RC_flag.send_pid1)
			{
				RC_flag.send_pid1=0;
				RC_Send_PID(1,Roll_rate_PID,Pitch_rate_PID,Yaw_rate_PID);
			}
			else if(RC_flag.send_pid2)
			{
				RC_flag.send_pid2=0;
				RC_Send_PID(2,Roll_angle_PID,Pitch_angle_PID,Yaw_angle_PID);
			}
			else if(RC_flag.send_pid3)
			{
				RC_flag.send_pid3=0;
				RC_Send_PID(3,High_v_PID,High_dis_PID,High_dis_PID);
			}
			else;
			return;
		}
		/*************返回PID校验***************/
		if(RC_flag.send_check_pid1||RC_flag.send_check_pid2||RC_flag.send_check_pid3)
		{
			if(RC_flag.send_check_pid1) {RC_flag.send_check_pid1=0;RC_Send_CHECK(0x10,sum_pid[0]);}
			else if(RC_flag.send_check_pid2) {RC_flag.send_check_pid2=0;RC_Send_CHECK(0x11,sum_pid[1]);}
			else if(RC_flag.send_check_pid3) {RC_flag.send_check_pid3=0;RC_Send_CHECK(0x12,sum_pid[2]);}
			else;
			return;
		}
		 /************常规数据返回**************/
		if(cnt%num == (status_cnt-1))
		   RC_flag.send_status=1;
	  else if((cnt % num) == (sensor_cnt-1))
		   RC_flag.send_sensor = 1;	
	  else if((cnt % num) == (gps_cnt-1))
		   RC_flag.send_gps = 1;	
	  else if((cnt % num) == (power_cnt-1))
		   RC_flag.send_power = 1;	
	   cnt++;
		 if(RC_flag.send_status) 
		 {
			  RC_flag.send_status=0;
				RC_Send_STATUS(IMU.Roll,IMU.Pitch,IMU.Yaw,real_height,real_speed_height,FlyMode,Fly_sta);
		 }
		 else if(RC_flag.send_sensor)
		 {
			    RC_flag.send_sensor=0;
					RC_Send_Sensor(IMU_Data.ACCEL_X,IMU_Data.ACCEL_Y,IMU_Data.ACCEL_Z,\
		                       IMU_Data.GYRO_X,IMU_Data.GYRO_Y,IMU_Data.GYRO_Z,\
		                       MAG_Data.x,MAG_Data.y,MAG_Data.z);
		 }
		 else if(RC_flag.send_gps)
     {
			  RC_flag.send_gps=0;
		    RC_Send_GPS(gps_data);
		 }
		 else if(RC_flag.send_power)
     {
			 RC_flag.send_power=0;
		   RC_Send_Power(VCBAT.Voltage,VCBAT.Current);
		 }
		 
	}
	
}





/*************************************/
void RC_Send_CHECK(u8 head,u8 check_sum) //返回校验
{
	u8 sum = 0;
	u8 i;
	data_buff[0]=0xAA;
	data_buff[1]=0xAA;
	data_buff[2]=0xEF;
	data_buff[3]=2;
	data_buff[4]=head;
	data_buff[5]=check_sum;
	
	for(i=0;i<6;i++)
		sum += data_buff[i];
	data_buff[6]=sum;
	RC_Feedback_Send(data_buff, 7);
}	
//返回校准结果
void RC_Send_MSG(u8 id,u8 result) //收到校准命令，校准完成后把结果返回给遥控
{
	u8 sum = 0;
	u8 i;
	data_buff[0]=0xAA;
	data_buff[1]=0xAA;
	data_buff[2]=0xEE;
	data_buff[3]=2;
	data_buff[4]=id;
	data_buff[5]=result;
	for(i=0;i<6;i++)
		sum += data_buff[i];
	data_buff[6]=sum;
	RC_Feedback_Send(data_buff, 7);
}
//返回状态数据
void RC_Send_STATUS(float angle_rol, float angle_pit, float angle_yaw, float alt, float speed_height,u8 fly_model, u8 armed)
{
	u8 i;
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	float temp_f32;
	u8 sum = 0;
	data_buff[_cnt++]=0xAA;
	data_buff[_cnt++]=0xAA;
	data_buff[_cnt++]=0x01;
	data_buff[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp2 =(vs32)(alt*100.0f);
	data_buff[_cnt++]=BYTE3(_temp2);
	data_buff[_cnt++]=BYTE2(_temp2);
	data_buff[_cnt++]=BYTE1(_temp2);
	data_buff[_cnt++]=BYTE0(_temp2);
	_temp = (int)(speed_height*100);
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	data_buff[_cnt++] = fly_model;
	data_buff[_cnt++] = armed;
	data_buff[3] = _cnt-4;
	for(i=0;i<_cnt;i++)
		sum += data_buff[i];
	data_buff[_cnt++]=sum;
	
	RC_Feedback_Send(data_buff,_cnt);
}
//返回PID数据
void RC_Send_PID(u8 group,PID_TypeDef p1,PID_TypeDef p2,PID_TypeDef p3)//飞控上的PID参数返回
{
	u8 i;
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	data_buff[_cnt++]=0xAA;
	data_buff[_cnt++]=0xAA;
	data_buff[_cnt++]=0x10+group-1;
	data_buff[_cnt++]=0;
	
	_temp = p1.P * 1000;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = p1.I  * 1000;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = p1.D  * 1000;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = p2.P  * 1000;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = p2.I  * 1000;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = p2.D * 1000;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = p3.P  * 1000;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = p3.I  * 1000;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = p3.D * 1000;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	
	data_buff[3] = _cnt-4;
		
	for(i=0;i<_cnt;i++)
		 sum += data_buff[i];
	
	data_buff[_cnt++]=sum;

	RC_Feedback_Send(data_buff, _cnt);
}
//返回传感器数据
void RC_Send_Sensor(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0,sum,i;
	vs16 _temp;
	u32 temp_u32;
	data_buff[_cnt++]=0xAA;
	data_buff[_cnt++]=0xAA;
	data_buff[_cnt++]=0x02;
	data_buff[_cnt++]=0;
	
	_temp = a_x;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_buff[_cnt++]=BYTE1(_temp);
	data_buff[_cnt++]=BYTE0(_temp);
	data_buff[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_buff[i];
	data_buff[_cnt++] = sum;
	
	RC_Feedback_Send(data_buff, _cnt);
}
//返回GPS数据
void RC_Send_GPS(nmea_msg gps)
{
	u8 _cnt=0,sum,i;
	char temp_ch;
	vu8 temp_u8;
	vs16 temp_s16;
	vu32 temp_u32;
	
	data_buff[_cnt++]=0xAA;
	data_buff[_cnt++]=0xAA;
	data_buff[_cnt++]=0x04;
	data_buff[_cnt++]=0;
	
	temp_u8 = gps.gpssta;
	data_buff[_cnt++]=BYTE0(temp_u8);
	temp_u8 = gps.svnum;
	data_buff[_cnt++]=BYTE0(temp_u8);
	temp_u8 = gps.posslnum;	
	data_buff[_cnt++]=BYTE0(temp_u8);
	temp_u32 = gps.latitude*1000000.0f;	
	data_buff[_cnt++]=BYTE3(temp_u32);
	data_buff[_cnt++]=BYTE2(temp_u32);
	data_buff[_cnt++]=BYTE1(temp_u32);
	data_buff[_cnt++]=BYTE0(temp_u32);
	temp_ch = gps.nshemi;	
	data_buff[_cnt++]=BYTE0(temp_ch);
	temp_u32 = gps.longitude*1000000.0f;	
	data_buff[_cnt++]=BYTE3(temp_u32);
	data_buff[_cnt++]=BYTE2(temp_u32);
	data_buff[_cnt++]=BYTE1(temp_u32);
	data_buff[_cnt++]=BYTE0(temp_u32);
	temp_ch = gps.ewhemi;	
	data_buff[_cnt++]=BYTE0(temp_ch);
	temp_s16 = gps.altitude*10;//海拔，放大十倍
	data_buff[_cnt++]=BYTE1(temp_s16);
	data_buff[_cnt++]=BYTE0(temp_s16);
  temp_s16 = gps.speed*100.0f;
	data_buff[_cnt++]=BYTE1(temp_s16);
	data_buff[_cnt++]=BYTE0(temp_s16);
	
	temp_u8 = gps.utc.hour+8;
	data_buff[_cnt++]=BYTE0(temp_u8);
	temp_u8 = gps.utc.min;
	data_buff[_cnt++]=BYTE0(temp_u8);
	temp_u8 = gps.utc.sec;
	data_buff[_cnt++]=BYTE0(temp_u8);
	data_buff[3] = _cnt-4;
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_buff[i];
	data_buff[_cnt++] = sum;
	RC_Feedback_Send(data_buff, _cnt);
}
//电源信息
void RC_Send_Power(float voltage,float current)
{
	u8 _cnt=0,sum,i;
  u16 temp;
	data_buff[_cnt++]=0xAA;
	data_buff[_cnt++]=0xAA;
	data_buff[_cnt++]=0x05;
	data_buff[_cnt++]=0;
	temp = (u16)(voltage*100.0f);
	data_buff[_cnt++]=BYTE1(temp);
	data_buff[_cnt++]=BYTE0(temp);
  temp = (u16)(current*100.0f);
	data_buff[_cnt++]=BYTE1(temp);
	data_buff[_cnt++]=BYTE0(temp);
	data_buff[3] = _cnt-4;
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_buff[i];
	data_buff[_cnt++] = sum;
	RC_Feedback_Send(data_buff, _cnt);
}
/*********************************************/







RC_Controller2Quat_Typedef s_Ctrl2Quat_msg;


/********************************************/
u8 RC_Anl_BUFF(u8* buff,u8 len)
{
	u8 i;
	u8 sum=0;
	for(i=0;i<(len-1);i++)
		sum += *(buff+i);
	s_Ctrl2Quat_msg.m_bSum=sum;
	if(!(s_Ctrl2Quat_msg.m_bSum==*(buff+len-1)))		return 0;		//判断sum
	s_Ctrl2Quat_msg.m_bHead=(*(buff))<<8|*(buff+1);
	if(s_Ctrl2Quat_msg.m_bHead!=0xAAAF)		return 0xFF;		//判断帧头

	s_Ctrl2Quat_msg.m_bCMD=*(buff+2);
	
	if(s_Ctrl2Quat_msg.m_bCMD==0X01)      RC_Anl_CMD1(buff);
	else if(s_Ctrl2Quat_msg.m_bCMD==0X02) RC_Anl_CMD2(buff);
	else if(s_Ctrl2Quat_msg.m_bCMD==0X03) RC_Anl_CTRL(buff);
	else if(s_Ctrl2Quat_msg.m_bCMD==0x10|| \
		    s_Ctrl2Quat_msg.m_bCMD==0x11|| \
		    s_Ctrl2Quat_msg.m_bCMD==0x12) RC_Anl_PID(buff,sum);
	else;
	return 1; 
	
}
/*******************************************/
//解释来自遥控的命令，返回命令值
u8 RC_Anl_CMD1(u8* buff)
{
	if(*(buff+2)==0X01)
	{
	  if(Fly_sta ==FlyStaLock || Fly_sta ==FlyStaRdy)
    {
		   if(*(buff+4)==0X01)
		  { 
		    IMU_Data.AccelOffsetReq = 1;
		    IMU_Data.AccelOffsetFinished=0;
      }
		   else if(*(buff+4)==0X02)
		  {
		    IMU_Data.GyroOffsetReq = 1;
		    IMU_Data.GyroOffsetFinished = 0;
		  }
		  else if(*(buff+4)==0X03)
		  {
			  MAG_Data.MagOffsetReq = 1;
        MAG_Data.MagOffsetFinished = 0;			
		  }
		  else if(*(buff+4)==0X04)
		  {
			  MS5611.OffsetReq = 1;
        MS5611.OffsetFinished = 0;			
		  }
	  }
		if(*(buff+4)==0XA0)
		{
			 Fly_sta = FlyStaLock;		
		}
		else if(*(buff+4)==0XA1)
		{
			if(Fly_sta == FlyStaRdy )    //飞行器数据就绪条件下才可以解锁，就绪条件
             Fly_sta = FlyStaUnlock;//传感器数据正常，GPS定位正常
		}
		else;
  }
	  return *(buff+4);
}
void RC_Anl_CMD2(u8* buff)
{
	if(*(buff+2)==0X02)
	{
	  if(*(buff+4)==0X01)
		{
			 Fly_sta = FlyStaPIDAdj;
       RC_flag.send_pid1=1;
			 RC_flag.send_pid2=1;
			 RC_flag.send_pid3=1;
		}
		if(*(buff+4)==0X02)
		{
			
		}
		if(*(buff+4)==0XA0)		//读取版本信息
		{
			
		}
		if(*(buff+4)==0XA1)		//恢复默认参数
		{
			//Para_ResetToFactorySetup();
		}
	}
}
void RC_Anl_CTRL(u8* buff)
{
	if(*(buff+2)==0X03)     //控制数据
	{
		RC_ctrl.height =  (vs16)(*(buff+4)<<8 | *(buff+5));
		RC_ctrl.yaw =   (vs16)(*(buff+6)<<8 | *(buff+7));
		RC_ctrl.roll =  (vs16)(*(buff+8)<<8 | *(buff+9));
		RC_ctrl.pitch = (vs16)(*(buff+10)<<8 | *(buff+11));
	}
}
//接收到的PID分析
//buff:数据缓冲区
//sum:校验结果返回，由调用函数提供
void RC_Anl_PID(u8* buff,u8 sum)
{
	  u16 flash_data_buff[9];
	  if(*(buff+2)==0X10)								//PID1
    {
        Roll_rate_PID.P 	= 0.001f*( (vs16)(*(buff+4)<<8)|*(buff+5) );
        Roll_rate_PID.I 	= 0.001f*( (vs16)(*(buff+6)<<8)|*(buff+7) );
        Roll_rate_PID.D 	= 0.001f*( (vs16)(*(buff+8)<<8)|*(buff+9) );
        Pitch_rate_PID.P 	= 0.001f*( (vs16)(*(buff+10)<<8)|*(buff+11) );
        Pitch_rate_PID.I 	= 0.001f*( (vs16)(*(buff+12)<<8)|*(buff+13) );
        Pitch_rate_PID.D 	= 0.001f*( (vs16)(*(buff+14)<<8)|*(buff+15) );
        Yaw_rate_PID.P	  = 0.001f*( (vs16)(*(buff+16)<<8)|*(buff+17) );
        Yaw_rate_PID.I 	  = 0.001f*( (vs16)(*(buff+18)<<8)|*(buff+19) );
        Yaw_rate_PID.D 	  = 0.001f*( (vs16)(*(buff+20)<<8)|*(buff+21) );
			  flash_data_buff[0] = Roll_rate_PID.P*1000;
      	flash_data_buff[1] = Roll_rate_PID.I*1000;
        flash_data_buff[2] = Roll_rate_PID.D*1000;
        flash_data_buff[3] = Pitch_rate_PID.P*1000;
        flash_data_buff[4] = Pitch_rate_PID.I*1000;
        flash_data_buff[5] = Pitch_rate_PID.D*1000;
        flash_data_buff[6] = Yaw_rate_PID.P*1000;
        flash_data_buff[7] = Yaw_rate_PID.I*1000;
        flash_data_buff[8] = Yaw_rate_PID.D*1000;
			  FlashErase(PID_Data_Addr,1);
        FlashWrite(PID_Data_Addr,flash_data_buff,9);
			  RC_flag.send_check_pid1=1;
				sum_pid[0]=sum;
    }
    if(*(buff+2)==0X11)								//PID2
    {
			  Roll_angle_PID.P  = 0.001f*( (vs16)(*(buff+4)<<8)|*(buff+5) );
        Roll_angle_PID.I  = 0.001f*( (vs16)(*(buff+6)<<8)|*(buff+7) );
        Roll_angle_PID.D  = 0.001f*( (vs16)(*(buff+8)<<8)|*(buff+9) );
        Pitch_angle_PID.P = 0.001f*( (vs16)(*(buff+10)<<8)|*(buff+11) );
        Pitch_angle_PID.I = 0.001f*( (vs16)(*(buff+12)<<8)|*(buff+13) );
        Pitch_angle_PID.D = 0.001f*( (vs16)(*(buff+14)<<8)|*(buff+15) );
        Yaw_angle_PID.P 	= 0.001f*( (vs16)(*(buff+16)<<8)|*(buff+17) );
        Yaw_angle_PID.I 	= 0.001f*( (vs16)(*(buff+18)<<8)|*(buff+19) );
        Yaw_angle_PID.D 	= 0.001f*( (vs16)(*(buff+20)<<8)|*(buff+21) );
			  flash_data_buff[0] = Roll_angle_PID.P*1000;
      	flash_data_buff[1] = Roll_angle_PID.I*1000;
        flash_data_buff[2] = Roll_angle_PID.D*1000;
        flash_data_buff[3] = Pitch_angle_PID.P*1000;
        flash_data_buff[4] = Pitch_angle_PID.I*1000;
        flash_data_buff[5] = Pitch_angle_PID.D*1000;
        flash_data_buff[6] = Yaw_angle_PID.P*1000;
        flash_data_buff[7] = Yaw_angle_PID.I*1000;
        flash_data_buff[8] = Yaw_angle_PID.D*1000;
			  FlashWrite(PID_Data_Addr+2*9,flash_data_buff,9);
        RC_flag.send_check_pid2=1;
				sum_pid[1]=sum;
    }
    if(*(buff+2)==0X12)								//PID3
    {	
        High_v_PID.P  = 0.001f*( (vs16)(*(buff+4)<<8)|*(buff+5) );
        High_v_PID.I  = 0.001f*( (vs16)(*(buff+6)<<8)|*(buff+7) );
        High_v_PID.D  = 0.001f*( (vs16)(*(buff+8)<<8)|*(buff+9) );
        High_dis_PID.P    = 0.001f*( (vs16)(*(buff+10)<<8)|*(buff+11) );
        High_dis_PID.I    = 0.001f*( (vs16)(*(buff+12)<<8)|*(buff+13) );
        High_dis_PID.D    = 0.001f*( (vs16)(*(buff+14)<<8)|*(buff+15) );
        flash_data_buff[0] = High_v_PID.P*1000;
      	flash_data_buff[1] = High_v_PID.I*1000;
        flash_data_buff[2] = High_v_PID.D*1000;
        flash_data_buff[3] = High_dis_PID.P*1000;
        flash_data_buff[4] = High_dis_PID.I*1000;
        flash_data_buff[5] = High_dis_PID.D*1000;
        flash_data_buff[6] = High_dis_PID.P*1000;
        flash_data_buff[7] = High_dis_PID.I*1000;
        flash_data_buff[8] = High_dis_PID.D*1000;
			  FlashWrite(PID_Data_Addr+2*9+2*9,flash_data_buff,9);
        RC_flag.send_check_pid3=1;
				sum_pid[2]=sum;
    }
}


/****************************************************/
