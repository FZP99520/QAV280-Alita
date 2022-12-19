#include "ANO_DT.h"
#include "usart.h"
//#include "mpu6050.h"
#include "ms5611.h"
#include "control.h"
#include "nrf24l01.h"
#include "pwm.h"
#include "pid.h"
#include "RC.h"
#include "qmc5883.h"
#include "flash.h"
#include "adc.h"
#include "IMU.h"
//���ļ�ֻ���ڴ���ģʽ�����ݽ���
//ң�����ݽ�����rc.c�ļ�
/////////////////////////////////////////////////////////////////////////////////////
//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ
//����int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;					//��Ҫ�������ݵı�־
static u8 data_to_send[50];	//�������ݻ���
void ANO_DT_SendUserData(u8 *buff,u8 len,u8 channel)
{
	u8 _cnt=0;
	u8 i;
	u8 sum;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=channel+0xf0;
	data_to_send[_cnt++]=0;
	for(i=0;i<len;i++)
	{
	 data_to_send[_cnt++]=buff[i];
	}
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange��������������ݷ�������
//������ʵ��ÿ5ms����һ�δ�������������λ�������ڴ˺�����ʵ��
//�˺���Ӧ���û�ÿ5ms����һ��
void ANO_DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 senser_cnt 	= 1;
	static u8 status_cnt 	= 2;
	static u8 rcdata_cnt 	= 3;
	static u8 motopwm_cnt	= 4;
	static u8 power_cnt		=	5;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;	
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
		f.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-1))
		f.send_power = 1;		
	
	cnt++;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_version)
	{
		f.send_version = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_status)
	{
		f.send_status = 0;
		ANO_DT_Send_Status(gsIMU_Data.f32Roll,gsIMU_Data.f32Pitch,gsIMU_Data.f32Yaw,real_height,FlyMode,Fly_sta);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser)
	{
		f.send_senser = 0;
		ANO_DT_Send_Senser(gsMPU_Data.s16ACCEL_X,gsMPU_Data.s16ACCEL_Y,gsMPU_Data.s16ACCEL_Z,
												gsMPU_Data.s16GYRO_X,gsMPU_Data.s16GYRO_Y,gsMPU_Data.s16GYRO_Z,
												gsMAG_Data.s16MAG_X,gsMAG_Data.s16MAG_Y,gsMAG_Data.s16MAG_Z,gsMS5611_Data.RP);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_rcdata)
	{
		f.send_rcdata = 0;
		//ANO_DT_Send_RCData(RX_cnt.high*20,RX_cnt.yaw*20,RX_cnt.roll*20,RX_cnt.pitch*20,0,0,0,0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////	
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		ANO_DT_Send_MotoPWM(MOTOR.pwm1,MOTOR.pwm2,MOTOR.pwm3,MOTOR.pwm4,0,0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_power)
	{
		f.send_power = 0;
		ANO_DT_Send_Power(VCBAT.Voltage*100,VCBAT.Current);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
		ANO_DT_Send_PID(1,Roll_rate_PID.P,Roll_rate_PID.I,Roll_rate_PID.D,
		                  Pitch_rate_PID.P,Pitch_rate_PID.I,Pitch_rate_PID.D,
											Yaw_rate_PID.P,Yaw_rate_PID.I,Yaw_rate_PID.D);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)
	{
		f.send_pid2 = 0;
		ANO_DT_Send_PID(2,Roll_angle_PID.P,Roll_angle_PID.I,Roll_angle_PID.D,
		                  Pitch_angle_PID.P,Pitch_angle_PID.I,Pitch_angle_PID.D,         
											Yaw_angle_PID.P,Yaw_angle_PID.I,Yaw_angle_PID.D);
		
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid3)
	{
		f.send_pid3 = 0;
		ANO_DT_Send_PID(3,High_v_PID.P,High_v_PID.I,High_v_PID.D,
		                  High_dis_PID.P,High_dis_PID.I,High_dis_PID.D,		
											0,0,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	//Usb_Hid_Send();					
/////////////////////////////////////////////////////////////////////////////////////
}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���
//��ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	NRF24L01_Send_Data(data_to_send,length);//use NRF24L01 to send data
}

void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 sum = 0;
	u8 i;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	for(i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare������Э��Ԥ����������Э��ĸ�ʽ��
//���յ������ݽ���һ�θ�ʽ�Խ�������ʽ��ȷ�Ļ��ٽ������ݽ���
//��ֲʱ���˺���Ӧ���û���������ʹ�õ�ͨ�ŷ�ʽ���е��ã�
//���紮��ÿ�յ�һ�ֽ����ݣ�����ô˺���һ��
//�˺������������ϸ�ʽ������֡�󣬻����е������ݽ�������
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		//ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡���ú��������ȶ�Э�����ݽ���У��
//У��ͨ��������ݽ��н�����ʵ����Ӧ����
//�˺������Բ����û����е��ã��ɺ���Data_Receive_Prepare�Զ�����
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0,i;
	uint16_t buff[9];
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{ 
//		  IMU_Data.AccelOffsetReq = 1;
//		  IMU_Data.AccelOffsetFinished=0;
    }
		if(*(data_buf+4)==0X02)
		{
//		  IMU_Data.GyroOffsetReq = 1;
//		  IMU_Data.GyroOffsetFinished = 0;
		}
		if(*(data_buf+4)==0X04)
		{
//			MAG_Data.MagOffsetReq = 1;
//      MAG_Data.MagOffsetFinished = 0;			
		}
		if(*(data_buf+4)==0X05)
		{
//			MS5611.OffsetReq = 1;
//      MS5611.OffsetFinished = 0;			
		}
		if(*(data_buf+4)==0XA0)
		{
     Fly_sta = FlyStaLock;			
		}
		if(*(data_buf+4)==0XA1)
		{
      Fly_sta = FlyStaUnlock;		
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//��ȡ�汾��Ϣ
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//�ָ�Ĭ�ϲ���
		{
			//Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0X10)								//PID1
    {
        Roll_rate_PID.P 	= 0.001f*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        Roll_rate_PID.I 	= 0.001f*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        Roll_rate_PID.D 	= 0.001f*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Pitch_rate_PID.P 	= 0.001f*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        Pitch_rate_PID.I 	= 0.001f*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        Pitch_rate_PID.D 	= 0.001f*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        Yaw_rate_PID.P	  = 0.001f*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        Yaw_rate_PID.I 	  = 0.001f*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        Yaw_rate_PID.D 	  = 0.001f*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
			  FlashErase(PID_Data_Addr,1);
			  buff[0] = Roll_rate_PID.P*1000;
      	buff[1] = Roll_rate_PID.I*1000;
        buff[2] = Roll_rate_PID.D*1000;
        buff[3] = Pitch_rate_PID.P*1000;
        buff[4] = Pitch_rate_PID.I*1000;
        buff[5] = Pitch_rate_PID.D*1000;
        buff[6] = Yaw_rate_PID.P*1000;
        buff[7] = Yaw_rate_PID.I*1000;
        buff[8] = Yaw_rate_PID.D*1000;
			  FlashWrite(PID_Data_Addr,buff,9);
				//AT24Cxx_Write(PID_1_ADDR,data_buf+4,18);
			  UpdatePID = 1;
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
			  Roll_angle_PID.P  = 0.001f*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        Roll_angle_PID.I  = 0.001f*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        Roll_angle_PID.D  = 0.001f*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Pitch_angle_PID.P = 0.001f*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        Pitch_angle_PID.I = 0.001f*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        Pitch_angle_PID.D = 0.001f*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        Yaw_angle_PID.P 	= 0.001f*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        Yaw_angle_PID.I 	= 0.001f*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        Yaw_angle_PID.D 	= 0.001f*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
			  buff[0] = Roll_angle_PID.P*1000;
      	buff[1] = Roll_angle_PID.I*1000;
        buff[2] = Roll_angle_PID.D*1000;
        buff[3] = Pitch_angle_PID.P*1000;
        buff[4] = Pitch_angle_PID.I*1000;
        buff[5] = Pitch_angle_PID.D*1000;
        buff[6] = Yaw_angle_PID.P*1000;
        buff[7] = Yaw_angle_PID.I*1000;
        buff[8] = Yaw_angle_PID.D*1000;
			  FlashWrite(PID_Data_Addr+2*9,buff,9);
			  //AT24Cxx_Write(PID_2_ADDR,data_buf+4,18);
			  UpdatePID = 2;
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
        High_v_PID.P  = 0.001f*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        High_v_PID.I  = 0.001f*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        High_v_PID.D  = 0.001f*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        High_dis_PID.P    = 0.001f*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        High_dis_PID.I    = 0.001f*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        High_dis_PID.D    = 0.001f*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        ctrl_2.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        ctrl_2.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        ctrl_2.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
			  buff[0] = High_v_PID.P*1000;
      	buff[1] = High_v_PID.I*1000;
        buff[2] = High_v_PID.D*1000;
        buff[3] = High_dis_PID.P*1000;
        buff[4] = High_dis_PID.I*1000;
        buff[5] = High_dis_PID.D*1000;
        buff[6] = High_dis_PID.P*1000;
        buff[7] = High_dis_PID.I*1000;
        buff[8] = High_dis_PID.D*1000;
			  FlashWrite(PID_Data_Addr+2*9+2*9,buff,9);
			  PID_STA = PIDInitFinished;
				//AT24Cxx_Write(PID_3_ADDR,data_buf+4,18);
			  UpdatePID = 3;
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}

void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0,sum,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0,i,sum;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar)
{
	u8 _cnt=0,sum,i;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0,sum,i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0,sum,i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0,sum,i;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
