#include "qmc5883.h"
#include "math.h"
#include "led.h"
#include "flash.h"
#include "mpu6050.h"
#include "IMU.h"
#include "RC.h"
#include "filter.h"

#define MagLSB 12000.0f
#define MagOffsetNum 2000
#define MAG_QMC5883
MoveAvarageFilter_TypeDef Filter_mag_yaw={4,0,0,{0}};
MAG_Data_TypeDef MAG_Data;

u16 cnt_mag_err=0;

static int MAG_WriteByte(u8 addr,u8 data);
static int MAG_ReadByte(u8 addr,u8* pData);
static int MAG_RD_Buff(u8 addr,u8 size,u8* pBuff);
static void MAG_Calibration(MAG_Data_TypeDef* mag);
u8 MAG_Init(void)
{
	u8 id;
	MAG_ReadByte(Chip_id,&id);
	if(id != 0xFF)
		return 0;
	#ifdef MAG_HMC5883
	MAG_WriteByte(Reg_Config_A,0x10);//Output rate=10Hz
	MAG_WriteByte(Reg_Config_B,0xe0);//+-4.5Ga
	MAG_WriteByte(Reg_Mode,0x00);//Circulate output
	#endif
	
	#ifdef MAG_QMC5883
	MAG_WriteByte(Reg_Config2,0x80);//复位
	Delay_ms(100);
	MAG_WriteByte(Reg_Config1,0x49);//scale:2G,output rate:100Hz,Continuous mode ,over sample ratio 256 
	MAG_WriteByte(Reg_Config2,0x41);//disable interrupt ,pointer roll-over mode
	MAG_WriteByte(Reg_Period,0x01); //recommand 0x01
	#endif
	return 1;
	
}
void MAG_Data_Update(void)
{
	u8 buff[6];
	float yaw_x,yaw_y;
	float norm,mx=0,my=0,mz=0;
	float pitch,roll;
	MAG_ReadByte(Reg_Status,&(MAG_Data.sta));//读取状态
	if(MAG_Data.sta&MAG_DRY) //数据准备好，读取数据
	{
	 MAG_RD_Buff(0x00,6,buff);
		if(!(MAG_Data.sta&MAG_OVL))
		{
	      MAG_Data.x=(s16)(buff[1]<<8)+buff[0];
	      MAG_Data.y=(s16)(buff[3]<<8)+buff[2];
	      MAG_Data.z=(s16)(buff[5]<<8)+buff[4];
		}
		if(MAG_Data.MagOffsetReq ==1)
		{
			MAG_Calibration(&MAG_Data);
			return;
		}
		if(MAG_Data.MagOffsetFinished == 1)
		{
			MAG_Data.x = -MAG_Data.xgain*(MAG_Data.x - MAG_Data.xoffset);//坐标轴与机身坐标系一致
			MAG_Data.y = -MAG_Data.ygain*(MAG_Data.y - MAG_Data.yoffset);//坐标轴与机身坐标系一致
			MAG_Data.z = MAG_Data.zgain*(MAG_Data.z - MAG_Data.zoffset);
		}
		MAG_Data.mx=(float)MAG_Data.x/MagLSB;
		MAG_Data.my=(float)MAG_Data.y/MagLSB;
		MAG_Data.mz=(float)MAG_Data.z/MagLSB;
		
		norm = invSqrt(MAG_Data.x*MAG_Data.x + MAG_Data.y*MAG_Data.y + MAG_Data.z*MAG_Data.z);
		mx = norm*MAG_Data.x;
		my = norm*MAG_Data.y;
		mz = norm*MAG_Data.z;
		pitch = IMU.Pitch*PI/180.0f;
		roll = IMU.Roll*PI/180.0f;
		yaw_x = mx*cosf(roll)+my*sinf(pitch)*sinf(roll)+mz*cos(pitch)*sinf(roll);
		yaw_y = my*cosf(pitch) - mz*sinf(pitch);
		MAG_Data.yaw = atan2(yaw_x,yaw_y)*180.0f/PI;
		MAG_Data.yaw=MoveAvarageFilter(&Filter_mag_yaw,MAG_Data.yaw);
		//MAG_Data.Total_MAG = sqrtf(MAG_Data.mx*MAG_Data.mx+MAG_Data.my*MAG_Data.my+MAG_Data.mz*MAG_Data.mz);
		
//		MAG_Data.yaw = atan2(MAG_Data.y,MAG_Data.x)*180/PI;
//		if(MAG_Data.yaw<0) MAG_Data.yaw +=360.0f;
	}
	else 
		return;
}
static void MAG_Calibration(MAG_Data_TypeDef* mag)
{
	u16 buff[6];
	static u16 cnt=0;
	if(mag->MagOffsetReq ==1)
	{
		cnt++;
		if(cnt%10==1) LED_Red_ON;
		if(cnt%20==2) LED_Red_OFF; 
		mag->MagOffsetFinished = 0;
		if(cnt<MagOffsetNum)
		{
		 mag->xmin = (mag->x) < (mag->xmin)?(mag->x):(mag->xmin);
		 mag->ymin = (mag->y) < (mag->ymin)?(mag->y):(mag->ymin);
		 mag->zmin = (mag->z) < (mag->zmin)?(mag->z):(mag->zmin);
		
		 mag->xmax = (mag->x) > (mag->xmax)?(mag->x):(mag->xmax);
		 mag->ymax = (mag->y) > (mag->ymax)?(mag->y):(mag->ymax);
		 mag->zmax = (mag->z) > (mag->zmax)?(mag->z):(mag->zmax);
		}
		else
		{
			cnt=0;
			LED_Red_ON;
			mag->xoffset = (mag->xmin + mag->xmax)/2.0f;
			mag->yoffset = (mag->ymin + mag->ymax)/2.0f;
			mag->zoffset = (mag->zmin + mag->zmax)/2.0f;
			
			mag->xgain = 1.0f;
			mag->ygain = (float)(mag->xmax - mag->xmin)/(mag->ymax - mag->ymin);
			mag->zgain = (float)(mag->xmax - mag->xmin)/(mag->zmax - mag->zmin);
			
			buff[0] = (u16)mag->xoffset;
			buff[1] = (u16)mag->yoffset;
			buff[2] = (u16)mag->zoffset;
			buff[3] = (u16)(mag->xgain*1000);
			buff[4] = (u16)(mag->ygain*1000);
			buff[5] = (u16)(mag->zgain*1000);
			FlashErase(MAGOffsetAddr,1);
			FlashWrite(MAGOffsetAddr,buff,6);
			RC_flag.send_mag_cal_result=1;
			mag->xmin = 0;
			mag->xmax = 0;
			mag->ymin = 0;
			mag->ymax = 0;
			mag->zmin = 0;
			mag->zmax = 0;
			mag->MagOffsetFinished = 1;
			mag->MagOffsetReq = 0;
		}
	}
}
void MAG_Error_Det(void)
{
	float temp_f;
	temp_f = MAG_Data.mx*MAG_Data.mx + MAG_Data.my*MAG_Data.my + MAG_Data.mz*MAG_Data.mz;
	MAG_Data.Strength = sqrtf(temp_f);
	if(MAG_Data.Strength<0.33f || MAG_Data.Strength > 0.9f)
	{
		if(cnt_mag_err < 100) cnt_mag_err++;
	}
	else
	{
		if(cnt_mag_err) cnt_mag_err--;
	}
	if(cnt_mag_err>50)
		 MAG_Data.Err = 1;
	else
		 MAG_Data.Err = 0;
}
/******************************************/
static int MAG_RD_Buff(u8 addr,u8 size,u8* pBuff)
{
    int res;
	res = Api_IIC_ReadBytes(MAG_DeviceID, addr, size, pBuff);
    return res;
}
static int MAG_WriteByte(u8 addr,u8 data)
{
    int res;
    u8* pBuff;
    *pBuff = data;
	res = Api_IIC_WriteBytes(MAG_DeviceID, addr,1,pBuff);
    return res;
}
static int MAG_ReadByte(u8 addr,u8* pData)
{
    int res;
	  res = Api_IIC_ReadBytes(MAG_DeviceID, addr, 1,pData);
    return res;
}

