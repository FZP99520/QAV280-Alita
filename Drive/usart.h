#ifndef _USART_H
#define _USART_H

#include "stm32f10x.h"
#define USART1_Buff_Len 128
#define USART2_Buff_Len 1024

#define USART_BaudRate_9600     9600u
#define USART_BaudRate_115200   115200u

typedef struct
{
	u8 DMA_TX_OK;
	u8 tx_OK;
	u8 rx_OK;
	u16 RX_Len;
	u8 TX_busy;
	u16 TX_Cnt;
	u16 RX_Cnt;
}_USART_TypeDef;
extern u8 USART1_TX_Buff[USART1_Buff_Len];
extern u8 USART1_RX_Buff[USART1_Buff_Len];

extern u8 USART2_TX_Buff[USART2_Buff_Len];
extern u8 USART2_RX_Buff[USART2_Buff_Len];

extern _USART_TypeDef USART1_;
extern _USART_TypeDef USART2_;

void USART1_Init(u32 baudrate);
void USART2_Init(u32 baudrate);
u8 USART1_SendData(u8 *buff,u8 len);
u8 USART2_SendData(u8 *buff,u8 len);


#endif
