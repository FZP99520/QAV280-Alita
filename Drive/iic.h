#ifndef __I2C_H
#define __I2C_H
#include "stm32f10x.h"
#include "systick.h"
#define IIC_GPIO GPIOB
#define IIC_SCL GPIO_Pin_10
#define IIC_SDA GPIO_Pin_11


void SDA_OUT(void);
void SDA_IN(void);
//IO操作函数	
#define IIC_SCL_H     GPIO_SetBits(IIC_GPIO,IIC_SCL)//SCL
#define IIC_SCL_L     GPIO_ResetBits(IIC_GPIO,IIC_SCL)//SCL

#define IIC_SDA_H     GPIO_SetBits(IIC_GPIO,IIC_SDA)//SDA
#define IIC_SDA_L     GPIO_ResetBits(IIC_GPIO,IIC_SDA)//SDA

#define READ_SDA   GPIO_ReadInputDataBit(IIC_GPIO,IIC_SDA)  //输入SDA 

  //IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口



unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);

u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

void Api_IIC_Write_CMD(u8 slave_addr,u8 cmd);
int Api_IIC_Write_OneByte(u8 daddr,u8 addr,u8 data);
int  Api_IIC_Read_OneByte(u8 daddr,u8 addr,u8* data);	 

int Api_IIC_WriteBytes(u8 DeviceID,u8 RegAddr,u8 size,u8* pBuff);
int Api_IIC_ReadBytes(u8 DeviveID,u8 RegAddr,u8 size,u8* pBuff);


#endif
