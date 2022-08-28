#include "iic.h"
#include "systick.h"
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
/*******************************/
#define SWI2C_INUSE
static void Drv_IIC_Start(void);				//发送IIC开始信号
static void Drv_IIC_Stop(void);	  			//发送IIC停止信号
static void Drv_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
static u8 Drv_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
static u8 Drv_IIC_Wait_Ack(void); 				//IIC等待ACK信号
static void Drv_IIC_Ack(void);					//IIC发送ACK信号
static void Drv_IIC_NAck(void);				//IIC不发送ACK信号
static void Drv_IIC_Delay(u8 i);
static void Drv_SDA_OUT(void);
static void Drv_SDA_IN(void);



void IIC_Init(void)
{			
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;	//端口配置
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //10M
    GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB 
    IIC_SDA_H;
    IIC_SCL_H;
}
static void Drv_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	//端口配置
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //10M
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
static void Drv_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	//端口配置
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //10M
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}
static void Drv_IIC_Start(void)
{
	Drv_SDA_OUT();
	IIC_SDA_H;
	IIC_SCL_H;
	Drv_IIC_Delay(5);
	IIC_SDA_L;
	Drv_IIC_Delay(10);
	IIC_SCL_L;
}
static void Drv_IIC_Stop(void)
{
	Drv_SDA_OUT();//sda线输出
	IIC_SCL_L;
	IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	Drv_IIC_Delay(10);//Delay_us(1);
	IIC_SCL_H; 
	Drv_IIC_Delay(10);
	IIC_SDA_H;  //发送I2C总线结束信号						   	
}
static u8 Drv_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	Drv_SDA_IN();      //SDA设置为输入  
	IIC_SDA_H;
	//iic_delay(10);//Delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>200)
		{
			Drv_IIC_Stop();
			return 0;
		}
	  Drv_IIC_Delay(2);//Delay_us(1);
	}
	IIC_SCL_H;
	Drv_IIC_Delay(6);
	IIC_SCL_L;//时钟输出0 	   
	return 1;  
} 
static void Drv_IIC_Ack(void)
{
	//IIC_SCL_L;
	Drv_SDA_OUT();
	IIC_SDA_L;
	Drv_IIC_Delay(5);//Delay_us(1);
	IIC_SCL_H;
	Drv_IIC_Delay(5);//Delay_us(1);
	IIC_SCL_L;
}   
static void Drv_IIC_NAck(void)
{
	Drv_SDA_OUT();
	IIC_SDA_H;
	Drv_IIC_Delay(5);//Delay_us(1);
	IIC_SCL_H;
	Drv_IIC_Delay(5);//Delay_us(1);
	IIC_SCL_L;
} 
static void Drv_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  Drv_SDA_OUT();
    for(t=0;t<8;t++)
    {   
			if(txd&0x80)IIC_SDA_H;
			else IIC_SDA_L;
      txd<<=1;
		  Drv_IIC_Delay(5);
		  IIC_SCL_H;
		  Drv_IIC_Delay(5);
			IIC_SCL_L;	
			//iic_delay(2);	
    }	 
} 	 

static u8 Drv_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	Drv_SDA_IN();//SDA设置为输入
	IIC_SCL_L; 
  for(i=0;i<8;i++ )
	{
	   Drv_IIC_Delay(2);
		 IIC_SCL_H;
     receive<<=1;
		 Drv_IIC_Delay(1);
     if(READ_SDA)receive++; 
		 IIC_SCL_L; 
   }					 
    if (ack)
        Drv_IIC_Ack(); //发送ACK 
    else
        Drv_IIC_NAck();//发送nACK  
    return receive;
}
static void Drv_IIC_Delay(u8 i)
{
	u8 a;
	for(a=0;a<i;a++)
	{
		__nop();
	}
}


void Api_IIC_Write_CMD(u8 slave_addr,u8 cmd)
{
	Drv_IIC_Start();
	Drv_IIC_Send_Byte(slave_addr);
	Drv_IIC_Wait_Ack();
	Drv_IIC_Send_Byte(cmd);
	Drv_IIC_Wait_Ack();
	Drv_IIC_Stop();
}

//Write bytes
int Api_IIC_WriteBytes(u8 DeviceID,u8 RegAddr,u8 size,u8* pBuff)
{
    u8 i;
    int ret;
	Drv_IIC_Start();
	Drv_IIC_Send_Byte(DeviceID);
	ret = Drv_IIC_Wait_Ack();
    if(ret == 0) return -1;
	Drv_IIC_Send_Byte(RegAddr);
	ret = Drv_IIC_Wait_Ack();
    if(ret == 0) return -1;
	for(i=0;i<size;i++)
	{
	  Drv_IIC_Send_Byte(pBuff[i]);
	  ret = Drv_IIC_Wait_Ack();
      if(ret == 0) return -1;
	}
	Drv_IIC_Stop();
    ret = 1;
    return ret;
    
}

//Read bytes
int Api_IIC_ReadBytes(u8 DeviveID,u8 RegAddr,u8 size,u8* pBuff)
{
    u8 count;
    int ret;
    Drv_IIC_Start();
    Drv_IIC_Send_Byte(DeviveID);
    ret = Drv_IIC_Wait_Ack();
    if(ret == 0) return -1;
    Drv_IIC_Send_Byte(RegAddr);
    ret = Drv_IIC_Wait_Ack();
    if(0 == ret) return -1;
    Drv_IIC_Start();
    Drv_IIC_Send_Byte(RegAddr+1);
    ret = Drv_IIC_Wait_Ack();
    if(0 == ret) return -1;
    for(count=0;count<size-1;count++)
       {
            pBuff[count]=Drv_IIC_Read_Byte(1);
       }
    pBuff[count]=Drv_IIC_Read_Byte(0);
     Drv_IIC_Stop();
     return 1;
}

#if 0
int Api_IIC_Write_OneByte(u8 slave_addr,u8 addr,u8 data)
{
	Drv_IIC_Start();
	Drv_IIC_Send_Byte(slave_addr);
	Drv_IIC_Wait_Ack();
	Drv_IIC_Send_Byte(addr);
	Drv_IIC_Wait_Ack();
	Drv_IIC_Send_Byte(data);
	Drv_IIC_Wait_Ack();
	Drv_IIC_Stop();
}
int Api_IIC_Read_OneByte(u8 slave_addr,u8 addr,u8* data)
{
	u8 temp;
	Drv_IIC_Start();
	Drv_IIC_Send_Byte(slave_addr);
	Drv_IIC_Wait_Ack();
	Drv_IIC_Send_Byte(addr);
	Drv_IIC_Wait_Ack();
	Drv_IIC_Start();
	Drv_IIC_Send_Byte(slave_addr+1);
	Drv_IIC_Wait_Ack();
	*data = Drv_IIC_Read_Byte(0);
	Drv_IIC_Stop();
	return temp;
}
#endif

