#include "IncludeAll.h"
#include "flash.h"
#include "string.h"
FLASH_Status FlashErase(uint32_t addr, uint8_t count)
{
  uint8_t i;
  FLASH_Status status;
  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
  for(i = 0; i < count; ++i)
  {
      status = FLASH_ErasePage(addr + i * FLASH_SECTOR_SIZE) ;
      if(status!=FLASH_COMPLETE)
			{
				 FLASH_Lock();
         return status;
			}
  }
  FLASH_Lock();
  return FLASH_COMPLETE;
}
 
uint16_t FlashWrite(uint32_t addr, uint16_t *buffer, uint16_t length)
{
  uint16_t i, data = 0;
 
  FLASH_Unlock();
 
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
 
  for(i = 0; i < length; i++)
  {
    if(FLASH_ProgramHalfWord((uint32_t)(addr + i*2), buffer[i]) != FLASH_COMPLETE)
    {
      return i;
    }
  }
  
  FLASH_Lock();
 
  return length;
}

uint16_t FLASH_ReadHalfWord(uint32_t address)
{
  return *(__IO uint16_t*)address; 
}
uint16_t FlashRead(uint32_t addr, u16* buff, uint32_t length)
{

  uint16_t i;
  for(i=0;i<length;i++)
  {
    buff[i]=FLASH_ReadHalfWord(addr+i*2);
  }
	return i;
}
void FLASH_ReadMoreData(uint32_t startAddress,uint16_t *buff,uint16_t cnt)
{
  uint16_t i;
  for(i=0;i<cnt;i++)
  {
    buff[i]=FLASH_ReadHalfWord(startAddress+i*2);
  }
}
//Read sensor offset Data from flash
/**********************/
void Flash_Read_Sensor_Data(IMU_Data_TypeDef* imu_data,MAG_Data_TypeDef* mag,MS5611_Typedef* ms)
{
	u16 BuffRead[6];
  FlashRead(MPUAccelOffsetAddr,BuffRead,3);
	if(BuffRead[0]!=0xFFFF && BuffRead[1]!= 0xFFFF &&BuffRead[2]!= 0xFFFF)
	{
		imu_data->os_accel_x = (s16)BuffRead[0];
		imu_data->os_accel_y = (s16)BuffRead[1];
		imu_data->os_accel_z = (s16)BuffRead[2];
		imu_data->AccelOffsetFinished =1;
		imu_data->AccelOffsetReq=0;
		//add test
	}
	else
	{
		imu_data->AccelOffsetFinished =0;
		imu_data->AccelOffsetReq=1;
	}
	FlashRead(MPUGyroOffsetAddr,BuffRead,3);
	if(BuffRead[0]!=0xFFFF && BuffRead[1]!= 0xFFFF && BuffRead[2]!= 0xFFFF)
	{
		imu_data->os_gyro_x = (s16)BuffRead[0];
		imu_data->os_gyro_y = (s16)BuffRead[1];
		imu_data->os_gyro_z = (s16)BuffRead[2];
		imu_data->GyroOffsetFinished =1;
		imu_data->GyroOffsetReq=0;
	}
	else
	{
		imu_data->GyroOffsetFinished =0;
		imu_data->GyroOffsetReq=1;
	}
	FlashRead(MAGOffsetAddr,BuffRead,6);
	if(BuffRead[0]!=0xFFFF && BuffRead[1]!= 0xFFFF &&BuffRead[2]!= 0xFFFF)
	{
		mag->xoffset = BuffRead[0];
		mag->yoffset = BuffRead[1];
	  mag->zoffset = BuffRead[2];
		mag->xgain   = (float)BuffRead[3]/1000.0f;
		mag->ygain   = (float)BuffRead[4]/1000.0f;
		mag->zgain   = (float)BuffRead[5]/1000.0f;
		mag->MagOffsetFinished = 1;
		mag->MagOffsetReq=0;
	}
	else
	{
		mag->MagOffsetFinished = 0;
		mag->MagOffsetReq=1;
	}
}
//read pid data from flash
/***************************/
PID_STATUS Flash_Read_PID(void)
{
	u8 i;
	uint16_t buff[9];
	i=FlashRead(PID_Data_Addr,buff,9);
	if(i == 9)
	{
	  Roll_rate_PID.P 	= 0.001f*( (s16) buff[0] );
    Roll_rate_PID.I 	= 0.001f*( (s16) buff[1] );
    Roll_rate_PID.D 	= 0.001f*( (s16) buff[2] );
    Pitch_rate_PID.P 	= 0.001f*( (s16) buff[3] );
    Pitch_rate_PID.I 	= 0.001f*( (s16) buff[4] );
    Pitch_rate_PID.D 	= 0.001f*( (s16) buff[5] );
    Yaw_rate_PID.P	  = 0.001f*( (s16) buff[6] );
    Yaw_rate_PID.I 	  = 0.001f*( (s16) buff[7] );
    Yaw_rate_PID.D 	  = 0.001f*( (s16) buff[8] );
	}
	else
		return PIDInitError;
	i=FlashRead(PID_Data_Addr+2*9,buff,9);
	if(i == 9)
	{
	  Roll_angle_PID.P 	= 0.001f*( (s16) buff[0] );
    Roll_angle_PID.I 	= 0.001f*( (s16) buff[1] );
    Roll_angle_PID.D 	= 0.001f*( (s16) buff[2] );
    Pitch_angle_PID.P 	= 0.001f*( (s16) buff[3] );
    Pitch_angle_PID.I 	= 0.001f*( (s16) buff[4] );
    Pitch_angle_PID.D 	= 0.001f*( (s16) buff[5] );
    Yaw_angle_PID.P	    = 0.001f*( (s16) buff[6] );
    Yaw_angle_PID.I 	  = 0.001f*( (s16) buff[7] );
    Yaw_angle_PID.D 	  = 0.001f*( (s16) buff[8] );
	}
	else
		return PIDInitError;
	i=FlashRead(PID_Data_Addr+2*9+2*9,buff,9);
	if(i == 9)
	{
	  High_v_PID.P 	= 0.001f*( (s16) buff[0] );
    High_v_PID.I 	= 0.001f*( (s16) buff[1] );
    High_v_PID.D 	= 0.001f*( (s16) buff[2] );
    High_dis_PID.P 	= 0.001f*( (s16) buff[3] );
    High_dis_PID.I 	= 0.001f*( (s16) buff[4] );
    High_dis_PID.D 	= 0.001f*( (s16) buff[5] );
    High_dis_PID.P	  = 0.001f*( (s16) buff[6] );
    High_dis_PID.I 	  = 0.001f*( (s16) buff[7] );
    High_dis_PID.D 	  = 0.001f*( (s16) buff[8] );
	}
	else
		return PIDInitError;
	return 
		PIDInitFinished;
}