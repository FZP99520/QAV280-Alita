#ifndef _spi_H
#define _spi_H
#include "stm32f10x.h"


void SPI2_Init(void);  //SPI��ʼ��
u8 SPI2_ReadWriteByte(u8 dat);		 //SPI2��дһ���ֽ�
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler);	//����SPI2���ٶ�


#endif
