#include "iic.h"
#include "systick.h"
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
/*******************************/
#define SWI2C_INUSE
static void Drv_IIC_Start(void);//����IIC��ʼ�ź�
static void Drv_IIC_Stop(void);//����IICֹͣ�ź�
static void Drv_IIC_Send_Byte(u8 txd);//IIC����һ���ֽ�
static u8 Drv_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
static E_IIC_Status_TypeDef Drv_IIC_Wait_Ack(void);//IIC�ȴ�ACK�ź�
static void Drv_IIC_Ack(void);//IIC����ACK�ź�
static void Drv_IIC_NAck(void);//IIC������ACK�ź�
static void Drv_IIC_Delay(u8 i);
static void Drv_SDA_OUT(void);
static void Drv_SDA_IN(void);



void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;//�˿�����
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //10M
    GPIO_Init(GPIOB, &GPIO_InitStructure);//�����趨������ʼ��GPIOB 
    IIC_SDA_H;
    IIC_SCL_H;
}
static void Drv_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	//�˿�����
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //10M
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
static void Drv_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	//�˿�����
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //10M
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
static void Drv_IIC_Start(void)
{
    Drv_SDA_OUT	();
    IIC_SDA_H;
    IIC_SCL_H;
    Drv_IIC_Delay(5);
    IIC_SDA_L;
    Drv_IIC_Delay(10);
    IIC_SCL_L;
}
static void Drv_IIC_Stop(void)
{
    Drv_SDA_OUT();//sda�����
    IIC_SCL_L;
    IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
    Drv_IIC_Delay(10);//Delay_us(1);
    IIC_SCL_H; 
    Drv_IIC_Delay(10);
    IIC_SDA_H;  //����I2C���߽����ź�
}
static E_IIC_Status_TypeDef Drv_IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    Drv_SDA_IN();      //SDA����Ϊ����  
    IIC_SDA_H;
    //iic_delay(10);//Delay_us(1);
    while(READ_SDA)
    {
        ucErrTime++;
        if(ucErrTime>200)
        {
            Drv_IIC_Stop();
            return E_IIC_ERROR;
        }
        Drv_IIC_Delay(2);//Delay_us(1);
    }
    IIC_SCL_H;
    Drv_IIC_Delay(6);
    IIC_SCL_L;//ʱ�����0
    return E_IIC_OK;  
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
    Drv_SDA_IN();//SDA����Ϊ����
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
        Drv_IIC_Ack(); //����ACK 
    else
        Drv_IIC_NAck();//����nACK  
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
E_IIC_Status_TypeDef Api_IIC_WriteBytes(u8 DeviceID,u8 RegAddr,u8 size,u8* pBuff)
{
    u8 i;
    E_IIC_Status_TypeDef eRet = E_IIC_FAIL;

    Drv_IIC_Start();
    Drv_IIC_Send_Byte(DeviceID);
    eRet = Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    Drv_IIC_Send_Byte(RegAddr);
    eRet = Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    for(i=0;i<size;i++)
    {
        Drv_IIC_Send_Byte(pBuff[i]);
        eRet = Drv_IIC_Wait_Ack();
      if(eRet != E_IIC_OK) return E_IIC_FAIL;
    }
    Drv_IIC_Stop();
    eRet = E_IIC_OK;
    return eRet;    
}

//Read bytes
E_IIC_Status_TypeDef Api_IIC_ReadBytes(u8 DeviveID,u8 RegAddr,u8 size,u8* pBuff)
{
    u8 count = 0;
    E_IIC_Status_TypeDef bRet = E_IIC_FAIL;

    Drv_IIC_Start();
    Drv_IIC_Send_Byte(DeviveID);
    bRet = Drv_IIC_Wait_Ack();
    if(bRet != E_IIC_OK) return E_IIC_FAIL;
    Drv_IIC_Send_Byte(RegAddr);
    bRet = Drv_IIC_Wait_Ack();
    if(bRet != E_IIC_OK) return E_IIC_FAIL;
    Drv_IIC_Start();
    Drv_IIC_Send_Byte(RegAddr+1);
    bRet = Drv_IIC_Wait_Ack();
    if(bRet != E_IIC_OK) return E_IIC_FAIL;
    for(count=0;count<size-1;count++)
    {
        pBuff[count]=Drv_IIC_Read_Byte(1);
    }
    pBuff[count]=Drv_IIC_Read_Byte(0);
    Drv_IIC_Stop();
    return E_IIC_OK;
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

