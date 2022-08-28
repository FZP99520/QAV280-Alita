#ifndef _FLASH_H
#define _FLASH_H
#include "stm32f10x.h"
#include "IncludeAll.h"
#include "pid.h"
#define FLASH_START_ADDR    ((uint32_t)0x8000000)
#define FLASH_END_ADDR      ((uint32_t)(0x8000000 + (FLASH_SECTOR_NUM -1)* FLASH_SECTOR_SIZE))
#define FLASH_SECTOR_NUM    128  
#define FLASH_SECTOR_SIZE   1024 

#define FlashDataBase       (uint32_t)(0x8000000+1024*100)
#define MPUAccelOffsetAddr  (uint32_t)(0x8000000+1024*101)
#define MPUGyroOffsetAddr   (uint32_t)(0x8000000+1024*102) 
#define MAGOffsetAddr       (uint32_t)(0x8000000+1024*103) 
#define MS5611OffsetAddr    (uint32_t)(0x8000000+1024*104) 
#define PID_Data_Addr       (uint32_t)(0x8000000+1024*105)


FLASH_Status FlashErase(uint32_t addr, uint8_t count);
uint16_t FlashWrite(uint32_t addr, uint16_t *buffer, uint16_t length);
uint16_t FlashRead(uint32_t addr, u16* buffer, uint32_t length);

void Flash_Read_Sensor_Data(IMU_Data_TypeDef* imu_data,MAG_Data_TypeDef* mag,MS5611_Typedef* ms);
PID_STATUS Flash_Read_PID(void);
#endif
