#include "dma.h"
#include "string.h"
#include "usart.h"
void USART1_DMA_Init(void)
{
  DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;//�ж����ȼ�
  /* DMA1 Channel6 (triggered by USART1 Rx event) Config */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
  /* DMA1 Channel7 (triggered by USART2 Tx event) Config */
  DMA_DeInit(DMA1_Channel6);
  DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&USART1->DR;// ��ʼ�������ַ  
  DMA_InitStructure.DMA_MemoryBaseAddr =(u32)USART1_RX_Buff;// �ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//������Ϊ������Դ
  DMA_InitStructure.DMA_BufferSize = USART1_Buff_Len ;// ��������
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // �����ַ������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;// �ڴ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�����ֽڿ��
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;// �ڴ��ֽڿ��
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ����ģʽ�������˾Ͳ��ڽ����ˣ�������ѭ���洢
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;// ���ȼ���
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; // �ڴ�������ͨ�ţ������ڴ浽�ڴ� 
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);// ������ʼ��
  DMA_Cmd(DMA1_Channel5, ENABLE);//����DMA����
  /* DMA1 Channel2 (triggered by USART2 Tx event) Config */
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&USART1->DR;  // �����ַ������1�� �������Ŀ��
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_TX_Buff;// �����ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;// ����Ϊ��������Ŀ�ĵأ����������ݣ�������Ƿ���
  DMA_InitStructure.DMA_BufferSize = 0;  //���ͳ���Ϊ0��δ��������Ҫ����
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);//��ʼ��
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
	NVIC_InitTypeDef  NVIC_InitStructure;//�ж����ȼ�
  /* DMA1 Channel6 (triggered by USART1 Rx event) Config */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
  /* DMA1 Channel7 (triggered by USART2 Tx event) Config */
  DMA_DeInit(DMA1_Channel6);
  DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&USART2->DR;// ��ʼ�������ַ  
  DMA_InitStructure.DMA_MemoryBaseAddr =(u32)USART2_RX_Buff;// �ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//������Ϊ������Դ
  DMA_InitStructure.DMA_BufferSize = USART2_Buff_Len ;// ��������
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // �����ַ������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;// �ڴ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�����ֽڿ��
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;// �ڴ��ֽڿ��
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ����ģʽ�������˾Ͳ��ڽ����ˣ�������ѭ���洢
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;// ���ȼ��ܸ�
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; // �ڴ�������ͨ�ţ������ڴ浽�ڴ� 
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);// ������ʼ��
  DMA_Cmd(DMA1_Channel6, ENABLE);//����DMA����
  /* DMA1 Channel2 (triggered by USART2 Tx event) Config */
  DMA_DeInit(DMA1_Channel7);
  DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&USART2->DR;  // �����ַ������1�� �������Ŀ��
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART2_TX_Buff;// �����ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;// ����Ϊ��������Ŀ�ĵأ����������ݣ�������Ƿ���
  DMA_InitStructure.DMA_BufferSize = 0;  //���ͳ���Ϊ0��δ��������Ҫ����
  DMA_Init(DMA1_Channel7, &DMA_InitStructure);//��ʼ��
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
