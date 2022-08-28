#include "iic.h"
#include "systick.h"
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
/*******************************/
#define SWI2C_INUSE
static void _Drv_IIC_Start(void);//����IIC��ʼ�ź�
static void _Drv_IIC_Stop(void);//����IICֹͣ�ź�
static void _Drv_IIC_Send_Byte(u8 txd);//IIC����һ���ֽ�
static u8 _Drv_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
static E_IIC_Status_TypeDef _Drv_IIC_Wait_Ack(void);//IIC�ȴ�ACK�ź�
static void _Drv_IIC_Ack(void);//IIC����ACK�ź�
static void _Drv_IIC_NAck(void);//IIC������ACK�ź�
static void _Drv_IIC_Delay(u8 i);
static void _Drv_SDA_OUT(void);
static void _Drv_SDA_IN(void);



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
static void _Drv_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	//�˿�����
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //10M
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
static void _Drv_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	//�˿�����
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //10M
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
static void _Drv_IIC_Start(void)
{
    _Drv_SDA_OUT	();
    IIC_SDA_H;
    IIC_SCL_H;
    _Drv_IIC_Delay(5);
    IIC_SDA_L;
    _Drv_IIC_Delay(10);
    IIC_SCL_L;
}
static void _Drv_IIC_Stop(void)
{
    _Drv_SDA_OUT();//sda�����
    IIC_SCL_L;
    IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
    _Drv_IIC_Delay(10);//Delay_us(1);
    IIC_SCL_H; 
    _Drv_IIC_Delay(10);
    IIC_SDA_H;  //����I2C���߽����ź�
}
static E_IIC_Status_TypeDef _Drv_IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    _Drv_SDA_IN();      //SDA����Ϊ����  
    IIC_SDA_H;
    //iic_delay(10);//Delay_us(1);
    while(READ_SDA)
    {
        ucErrTime++;
        if(ucErrTime>200)
        {
            _Drv_IIC_Stop();
            return E_IIC_ERROR;
        }
        _Drv_IIC_Delay(2);//Delay_us(1);
    }
    IIC_SCL_H;
    _Drv_IIC_Delay(6);
    IIC_SCL_L;//ʱ�����0

    return E_IIC_OK;  
} 
static void _Drv_IIC_Ack(void)
{
    //IIC_SCL_L;
    _Drv_SDA_OUT();
    IIC_SDA_L;
    _Drv_IIC_Delay(5);//Delay_us(1);
    IIC_SCL_H;
    _Drv_IIC_Delay(5);//Delay_us(1);
    IIC_SCL_L;
}   
static void _Drv_IIC_NAck(void)
{
    _Drv_SDA_OUT();
    IIC_SDA_H;
    _Drv_IIC_Delay(5);//Delay_us(1);
    IIC_SCL_H;
    _Drv_IIC_Delay(5);//Delay_us(1);
    IIC_SCL_L;
} 
static void _Drv_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
    _Drv_SDA_OUT();
    for(t=0;t<8;t++)
    {   
        if(txd&0x80)IIC_SDA_H;
        else IIC_SDA_L;
            txd<<=1;
        _Drv_IIC_Delay(5);
        IIC_SCL_H;
        _Drv_IIC_Delay(5);
        IIC_SCL_L;	
        //iic_delay(2);	
    } 
}

static u8 _Drv_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    _Drv_SDA_IN();//SDA����Ϊ����
    IIC_SCL_L; 
    for(i=0;i<8;i++ )
    {
        _Drv_IIC_Delay(2);
        IIC_SCL_H;
        receive<<=1;
        _Drv_IIC_Delay(1);
        if(READ_SDA)receive++; 
            IIC_SCL_L; 
   } 
    if (ack)
        _Drv_IIC_Ack(); //����ACK 
    else
        _Drv_IIC_NAck();//����nACK  
    return receive;
}
static void _Drv_IIC_Delay(u8 i)
{
    u8 a;
    for(a=0;a<i;a++)
    {
        __nop();
    }
}


E_IIC_Status_TypeDef Api_IIC_Write_CMD(u8 slave_addr,u8 cmd)
{
    E_IIC_Status_TypeDef eRet = E_IIC_FAIL;

    _Drv_IIC_Start();
    _Drv_IIC_Send_Byte(slave_addr);
    eRet = _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    _Drv_IIC_Send_Byte(cmd);
    eRet = _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    _Drv_IIC_Stop();

    return E_IIC_OK;
}

//Write bytes
E_IIC_Status_TypeDef Api_IIC_WriteBytes(u8 DeviceID,u8 RegAddr,u8 size,u8* pBuff)
{
    u8 i;
    E_IIC_Status_TypeDef eRet = E_IIC_FAIL;

    _Drv_IIC_Start();
    _Drv_IIC_Send_Byte(DeviceID);
    eRet = _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    _Drv_IIC_Send_Byte(RegAddr);
    eRet = _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    for(i=0;i<size;i++)
    {
        _Drv_IIC_Send_Byte(pBuff[i]);
        eRet = _Drv_IIC_Wait_Ack();
      if(eRet != E_IIC_OK) return E_IIC_FAIL;
    }
    _Drv_IIC_Stop();

    return E_IIC_OK;    
}

//Read bytes
E_IIC_Status_TypeDef Api_IIC_ReadBytes(u8 DeviveID,u8 RegAddr,u8 size,u8* pBuff)
{
    u8 count = 0;
    E_IIC_Status_TypeDef eRet = E_IIC_FAIL;

    _Drv_IIC_Start();
    _Drv_IIC_Send_Byte(DeviveID);
    eRet = _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    _Drv_IIC_Send_Byte(RegAddr);
    bRet = _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    _Drv_IIC_Start();
    _Drv_IIC_Send_Byte(RegAddr+1);
    eRet = _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    for(count=0;count<size-1;count++)
    {
        pBuff[count]=_Drv_IIC_Read_Byte(1);
    }
    pBuff[count]=_Drv_IIC_Read_Byte(0);
    _Drv_IIC_Stop();

    return E_IIC_OK;
}


E_IIC_Status_TypeDef Api_IIC_Write_OneByte(u8 slave_addr,u8 addr,u8 wdata)
{
    E_IIC_Status_TypeDef eRet = E_IIC_FAIL;

    _Drv_IIC_Start();
    _Drv_IIC_Send_Byte(slave_addr);
    eRet = _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    _Drv_IIC_Send_Byte(addr);
    eRet = _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    _Drv_IIC_Send_Byte(wdata);
    eRet = _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    _Drv_IIC_Stop();

    return E_IIC_OK;
}
E_IIC_Status_TypeDef Api_IIC_Read_OneByte(u8 slave_addr,u8 addr,u8* rdata)
{
    E_IIC_Status_TypeDef eRet;

    _Drv_IIC_Start();
    _Drv_IIC_Send_Byte(slave_addr);
    _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    _Drv_IIC_Send_Byte(addr);
    _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    _Drv_IIC_Start();
    _Drv_IIC_Send_Byte(slave_addr+1);
    _Drv_IIC_Wait_Ack();
    if(eRet != E_IIC_OK) return E_IIC_FAIL;
    *rdata = _Drv_IIC_Read_Byte(0);
    _Drv_IIC_Stop();

    return E_IIC_OK;
}


