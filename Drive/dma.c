#include "dma.h"
#include "string.h"
#include "usart.h"
void USART1_DMA_Init(void)
{
  DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;//中断优先级
  /* DMA1 Channel6 (triggered by USART1 Rx event) Config */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
  /* DMA1 Channel7 (triggered by USART2 Tx event) Config */
  DMA_DeInit(DMA1_Channel6);
  DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&USART1->DR;// 初始化外设地址  
  DMA_InitStructure.DMA_MemoryBaseAddr =(u32)USART1_RX_Buff;// 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设作为数据来源
  DMA_InitStructure.DMA_BufferSize = USART1_Buff_Len ;// 缓存容量
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址不递增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;// 内存递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设字节宽度
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;// 内存字节宽度
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 正常模式，即满了就不在接收了，而不是循环存储
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;// 优先级高
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; // 内存与外设通信，而非内存到内存 
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);// 参数初始化
  DMA_Cmd(DMA1_Channel5, ENABLE);//启动DMA接收
  /* DMA1 Channel2 (triggered by USART2 Tx event) Config */
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&USART1->DR;  // 外设地址，串口1， 即发件的快递
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_TX_Buff;// 发送内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;// 外设为传送数据目的地，即发送数据，即快递是发件
  DMA_InitStructure.DMA_BufferSize = 0;  //发送长度为0，未有数据需要发送
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);//初始化
	DMA_Cmd(DMA1_Channel4, DISABLE);
	
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);//Open Transmit Complete Interrrupt
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);
}
void DMA1_Channel4_IRQHandler(void)//USART1 TX DMA interrupt handler
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)
	{
		DMA_ClearFlag(DMA1_FLAG_TC4);
		DMA_Cmd(DMA1_Channel4,DISABLE);//Shut down dma1 channel4
		USART1_.DMA_TX_OK = 1;
		USART1_.TX_Cnt++;
	}
}
/*****************USART2 DMA******************/
void USART2_DMA_Init(void)
{
  DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;//中断优先级
  /* DMA1 Channel6 (triggered by USART1 Rx event) Config */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
  /* DMA1 Channel7 (triggered by USART2 Tx event) Config */
  DMA_DeInit(DMA1_Channel6);
  DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&USART2->DR;// 初始化外设地址  
  DMA_InitStructure.DMA_MemoryBaseAddr =(u32)USART2_RX_Buff;// 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设作为数据来源
  DMA_InitStructure.DMA_BufferSize = USART2_Buff_Len ;// 缓存容量
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址不递增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;// 内存递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设字节宽度
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;// 内存字节宽度
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 正常模式，即满了就不在接收了，而不是循环存储
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;// 优先级很高
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; // 内存与外设通信，而非内存到内存 
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);// 参数初始化
  DMA_Cmd(DMA1_Channel6, ENABLE);//启动DMA接收
  /* DMA1 Channel2 (triggered by USART2 Tx event) Config */
  DMA_DeInit(DMA1_Channel7);
  DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&USART2->DR;  // 外设地址，串口1， 即发件的快递
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART2_TX_Buff;// 发送内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;// 外设为传送数据目的地，即发送数据，即快递是发件
  DMA_InitStructure.DMA_BufferSize = 0;  //发送长度为0，未有数据需要发送
  DMA_Init(DMA1_Channel7, &DMA_InitStructure);//初始化
	DMA_Cmd(DMA1_Channel7, DISABLE);
	
	DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);//Open Transmit Complete Interrrupt
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);
}
void DMA1_Channel7_IRQHandler(void)//USART1 TX DMA interrupt handler
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC7)!=RESET)
	{
		DMA_ClearFlag(DMA1_FLAG_TC7);
		DMA_Cmd(DMA1_Channel7,DISABLE);//Shut down dma1 channel4
		USART2_.DMA_TX_OK = 1;
		USART2_.TX_Cnt++;
	}
}
