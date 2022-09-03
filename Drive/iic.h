#ifndef __I2C_H
#define __I2C_H
#include "stm32f10x.h"
#include "systick.h"

typedef enum
{
    E_IIC_OK,
    E_IIC_FAIL,
    E_IIC_ERROR,
    E_IIC_MAX
}E_IIC_Status_TypeDef;


unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);

u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

E_IIC_Status_TypeDef Api_IIC_Write_CMD(u8 slave_addr,u8 cmd);
E_IIC_Status_TypeDef Api_IIC_Write_OneByte(u8 daddr,u8 addr,u8 data);
E_IIC_Status_TypeDef  Api_IIC_Read_OneByte(u8 daddr,u8 addr,u8* data);

E_IIC_Status_TypeDef Api_IIC_WriteBytes(u8 DeviceID,u8 RegAddr,u8 size,u8* pBuff);
E_IIC_Status_TypeDef Api_IIC_ReadBytes(u8 DeviveID,u8 RegAddr,u8 size,u8* pBuff);
void Api_IIC_Init(void);



#endif
