#ifndef _nrf24l01_H
#define _nrf24l01_H

#include "systick.h"


typedef struct
{
	u8 tx_OK;
	u8 rx_OK;
	u8 RX_Len;
	u8 TX_Max;
	u8 TX_BUFF[32];
	u8 RX_BUFF[32];
	u16 TX_Cnt;
	u16 RX_Cnt;
	u16 MAX_cnt;
}NRF_TypeDef;
extern NRF_TypeDef NRF;
#define TX_Mode 0x0e
#define RX_Mode 0x0f
//NRF24L01�Ĵ�����������
#define NRF24L01_TXMode 0 //����ģʽ
#define NRF24L01_RXMode 1 //����ģʽ
#define NRF_READ_REG    0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define NRF_WRITE_REG   0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���	 
#define R_RX_PL_WID   0x60 //��ȡ���յ����ֽ���
#define W_ACK_PAYLOAD 0xA8 //дӦ�����ݰ���+3λ���ݷ���ͨ��
#define W_TX_PAYLOAD_NO_ACK 0xB0//д���͵����ݰ�������Ӧ��
//SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG_NRF24L01          0  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;		
#define FEATURE  0x1D
#define DYNPD    0x1C

#define RX_FULL     0x20
#define RX_EMPTY    0x01
#define TX_FULL     0x20
#define TX_EMPTY    0x10  
#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_OK   		0x20  //TX��������ж�
#define RX_OK   		0x40  //���յ������ж�

#define OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD              0x09  //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define FIFO_STATUS 0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;
											
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01������
#define NRF_CE_HIGH    GPIO_SetBits(GPIOA,GPIO_Pin_11)
#define NRF_CE_LOW     GPIO_ResetBits(GPIOA,GPIO_Pin_11)

#define NRF_CSN_HIGH   GPIO_SetBits(GPIOA,GPIO_Pin_8)
#define NRF_CSN_LOW    GPIO_ResetBits(GPIOA,GPIO_Pin_8)

#define NRF_IRQ_Status GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
//24L01���ͽ������ݿ�ȶ���
#define TX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  32 	//32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��


void NRF24L01_Init(void);					//��ʼ��
void NRF24L01_RX_Mode(void);					//����Ϊ����ģʽ
void NRF24L01_TX_Mode(void);					//����Ϊ����ģʽ
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len);//д������
u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 len);	//��������		  
u8 NRF24L01_Read_Reg(u8 reg);					//���Ĵ���
u8 NRF24L01_Write_Reg(u8 reg, u8 valdue);		//д�Ĵ���
u8 NRF24L01_Check(void);						//���24L01�Ƿ����
u8 NRF24L01_Send_Data(u8 *txBuf,u8 len);
u8 NRF24L01_ReceiveData(NRF_TypeDef* p);
void NRF24L01_RX_Data_Pros(u8* data_buf); //Added on 2019.05.10

void NRF_Flush_TX_FIFO(void);
void NRF_Flush_RX_FIFO(void);
u8 NRF_R_FIFO_STA(void);
u8 NRF_R_STATUS(void);
u8 NRF_W_ACK_PAYLOAD(u8 *buf,u8 Pn,u8 len);
u8 NRF_R_RX_PL_WID(void);
#endif

