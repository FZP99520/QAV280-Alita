#include "nrf24l01.h"
#include "spi.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "control.h"
NRF_TypeDef NRF;
const u8 TX_ADDRESS_P0[TX_ADR_WIDTH]={0xA3,0xA4,0xA5,0xA6,0xA7}; //���͵�ַ
const u8 RX_ADDRESS_P0[RX_ADR_WIDTH]={0xA3,0xA4,0xA5,0xA6,0xA7};

void NRF24L01_RX_Data_Pros(u8* data_buf)
{
//	u8 sum = 0;
//	for(u8 i=0;i<(num-1);i++)
//		sum += *(data_buf+i);
//	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
}

void NRF24L01_Init(void)
{ 	
	GPIO_InitTypeDef GPIO_InitStructure;	  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	       //CE
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);	//��ʼ��ָ��IO
	NRF_CE_LOW;       //���� �������ģʽ

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	//CSN
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);	//��ʼ��ָ��IO
 	NRF_CSN_HIGH;//����		��ֹSPIͨ��		

	NRF_CSN_HIGH;			//SPIƬѡȡ��  
}

//���24L01�Ƿ����
//����ֵ:1���ɹ�;0��ʧ��	
u8 NRF24L01_Check()
{
	u8 buf[5]={0xAA,0xBB,0xCC,0xDD,0xFF};
  u8 buf1[5];
	u8 i;  	 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf1,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)
	{
			if(buf1[i]!=buf[i]) break;	
	}			
	if(i!=5) return 0;
	return 1;		 
}
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF_CSN_LOW;                 //ʹ��SPI����
  	status =SPI2_ReadWriteByte(reg);//���ͼĴ����� 
  	SPI2_ReadWriteByte(value);      //д��Ĵ�����ֵ
  	NRF_CSN_HIGH;                 //��ֹSPI����	   
  	return(status);       			  //����״ֵ̬
}
u8 NRF24L01_Read_Reg(u8 reg)
{
	  u8 reg_val;	    
 		NRF_CSN_LOW;          //ʹ��SPI����		
  	SPI2_ReadWriteByte(reg);   //���ͼĴ�����
  	reg_val=SPI2_ReadWriteByte(0X00);//��ȡ�Ĵ�������
  	NRF_CSN_HIGH;           //��ֹSPI����		    
  	return reg_val;          //����״ֵ̬
}	
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,i;	       
  	NRF_CSN_LOW;            //ʹ��SPI����
  	status=SPI2_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	  for(i=0;i<len;i++)
	  {
				 *pBuf=SPI2_ReadWriteByte(0X00);//��������
				 pBuf++;
		}
  	NRF_CSN_HIGH;          //�ر�SPI����
  	return status;        //���ض�����״ֵ̬
}
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	  NRF_CSN_LOW;          //ʹ��SPI����
  	status = SPI2_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
	 {
	       SPI2_ReadWriteByte(*pBuf); //д������	
		     pBuf++;		 
	 }
  	NRF_CSN_HIGH;        //�ر�SPI����
  	return status;          //���ض�����״ֵ̬
}				 
void NRF24L01_RX_Mode(void)
{
	NRF_CE_LOW;	 
	Delay_ms(10);
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_AW,0x03);	//���õ�ַ���� bit1:0 01:3b,10:4b,11:5b
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS_P0,RX_ADR_WIDTH);//дRX�ڵ��ַ
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,01);	     //����RFͨ��Ƶ��		  	    
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x26);//����TX�������,4dbm����,250Kbps,���������濪��  
  NRF24L01_Write_Reg(NRF_WRITE_REG+DYNPD,0x01); //ʹ��ͨ��0�Ķ�̬���ݳ���
	NRF24L01_Write_Reg(NRF_WRITE_REG+FEATURE,0x06);   //ʹ�� Ӧ������ݰ��Ͷ�̬���ݳ���
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG_NRF24L01, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  NRF_CE_HIGH;	 //CEΪ��,�������ģʽ 
}					
void NRF24L01_TX_Mode(void)
{														 
	NRF_CE_LOW;	       
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_AW,0x03);	//���õ�ַ���� bit1:0 01:3b,10:4b,11:5b
  NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS_P0,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS_P0,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x5A);//�����Զ��ط����ʱ��:1500us;����Զ��ط�����:10��
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,01);       //����RFͨ��Ϊ40
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x26);  //����TX�������,4dbm����,250Kbps,���������濪�� 
  NRF24L01_Write_Reg(NRF_WRITE_REG+DYNPD,0x01);   //ʹ��ͨ��0�Ķ�̬���ݳ���
	NRF24L01_Write_Reg(NRF_WRITE_REG+FEATURE,0x06);   //ʹ�ܶ�̬���ݳ���
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG_NRF24L01,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
//	  NRF_CE_HIGH;//CEΪ��,10us����������
}
u8 NRF24L01_Send_Data(u8 *txBuf,u8 len) //�����32�ֽڵ�����  �����óɷ���ģʽ
{
	 u8 fifo_sta;
	 NRF_CE_LOW;
	 fifo_sta=NRF_R_FIFO_STA();
	 if( fifo_sta&TX_FULL ) NRF_Flush_TX_FIFO();
	 NRF24L01_Write_Buf(WR_TX_PLOAD,txBuf,len);
	 NRF_CE_HIGH;//���߷�������
	 return 0;
}
//add on 2019.05.12
void NRF_Flush_TX_FIFO(void)
{
	NRF_CE_LOW;
	NRF24L01_Write_Reg(FLUSH_TX,0x00);
}
void NRF_Flush_RX_FIFO(void)
{
	NRF_CE_LOW;
	NRF24L01_Write_Reg(FLUSH_RX,0x00);
}
u8 NRF_R_FIFO_STA(void)
{
	u8 temp;
	NRF_CE_LOW;
	temp=NRF24L01_Read_Reg(FIFO_STATUS);
	return temp; 
}
u8 NRF_R_STATUS(void)
{
	u8 temp;
	NRF_CE_LOW;
	temp=NRF24L01_Read_Reg(STATUS);
	NRF_CE_HIGH;
	return temp;
}
u8 NRF_W_ACK_PAYLOAD(u8 *buf,u8 Pn,u8 len) //д��������
{
	if(Pn>5) return 0xff;
	 NRF24L01_Write_Buf(W_ACK_PAYLOAD+Pn,buf,len);
	return 1;
}
u8 NRF_R_RX_PL_WID(void)  //��ȡ���յ������ݳ���
{
	return NRF24L01_Read_Reg(R_RX_PL_WID);
}
u8 NRF24L01_ReceiveData(NRF_TypeDef* p)
{
	p->RX_Len = NRF_R_RX_PL_WID();
  if(p->RX_Len > 32)
	{
		NRF_Flush_RX_FIFO();
	  return 0;
  }
	else
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,p->RX_BUFF,p->RX_Len);
	  return 1; 
	}
}
//end of adding
