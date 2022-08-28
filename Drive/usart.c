#include "usart.h"
#include "string.h"
#include "gps.h"
#include "dma.h"
#include "printf.h"
#include "stdio.h"
_USART_TypeDef USART1_;
_USART_TypeDef USART2_;
u8 USART1_TX_Buff[USART1_Buff_Len];
u8 USART1_RX_Buff[USART1_Buff_Len];
u8 USART2_TX_Buff[USART2_Buff_Len];
u8 USART2_RX_Buff[USART2_Buff_Len];

#define LOG_USE_USART2

//int fputc(int ch, FILE * p)
//{
//    #ifdef LOG_USE_USART1
//    USART1_SendData(&ch,1);
//    #elif defined (LOG_USE_USART2)
//    USART2_SendData(&ch,1);
//    #endif
//    return ch;
//}

u8 USART1_SendData(u8 *buff,u8 len)
{
	u16 cnt=0;
	while(cnt<500 && USART1_.TX_busy ==1)
	{
		cnt++;
		return 0;
	}
	memcpy(USART1_TX_Buff,buff,len);//copy memory
  DMA_SetCurrDataCounter(DMA1_Channel4,len);
  DMA_Cmd(DMA1_Channel4,ENABLE);//Open DMA1
	USART1_.TX_busy = 1;
	return 1;//succeed
}
void USART1_Init(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure; 
	NVIC_InitTypeDef  NVIC_InitStructure;//中断优先级
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//打开复用功能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//串口时钟
	/*********************管脚配置***************************/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//txd
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//rxd
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/*******************************************************/
	/***************串口配置******************/
	USART_InitStructure.USART_BaudRate=baudrate;//波特率
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位
	USART_InitStructure.USART_Parity=USART_Parity_No;//校验位--无
	USART_InitStructure.USART_HardwareFlowControl = //硬件流失能
	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx |USART_Mode_Rx;//模式
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1,ENABLE); //串口使能
	/***********************************/
	 USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);//空闲中断
	 USART_ITConfig(USART1,USART_IT_TC, ENABLE);// 串口发送完成中断
	 /*******************************/
	 NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//子优先级3
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	 NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	 USART_DMACmd(USART1,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);//使能DMA串口发送和接受请求
	 USART1_DMA_Init();
}
void USART1_IRQHandler(void)
{
	u16 len;
	if(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=RESET)
	{
		USART_ClearFlag(USART1,USART_FLAG_TC);
		if(USART1_.DMA_TX_OK == 1)
		{
		  USART1_.TX_busy=0;//Relax bus
			USART1_.DMA_TX_OK = 0;
		}
		USART1_.tx_OK= 1;
		USART1_.TX_Cnt++;
	}
	if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET)
	{
		  USART_ReceiveData(USART1);
		  USART1_.RX_Cnt++;
			DMA_ClearFlag(DMA1_FLAG_TC5);
			DMA_Cmd(DMA1_Channel5,DISABLE);
			USART1_.rx_OK = 1;
			len = DMA_GetCurrDataCounter(DMA1_Channel5);
			USART1_.RX_Len = USART1_Buff_Len - len;
			DMA_SetCurrDataCounter(DMA1_Channel5,USART1_Buff_Len);
			DMA_Cmd(DMA1_Channel5,ENABLE);
		  USART_ClearITPendingBit(USART1,USART_IT_IDLE);
	}
}

/*******************************************************/
/********************USART2 Operation**********************************/
u8 USART2_SendData(u8 *buff,u8 len)
{
	u16 cnt=0;
	while(USART2_.TX_busy ==1)
	{
		cnt++;
        Delay_us(100);
        if(cnt>512)
		    return 0;
	}
	memcpy(USART2_TX_Buff,buff,len);//copy memory
  DMA_SetCurrDataCounter(DMA1_Channel7,len);
  DMA_Cmd(DMA1_Channel7,ENABLE);//Open DMA1 channel7
	USART2_.TX_busy = 1;
	return 1;//succeed
}
void USART2_Init(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure; 
	NVIC_InitTypeDef  NVIC_InitStructure;//中断优先级
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//打开复用功能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//串口时钟
	/*********************管脚配置***************************/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//txd
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;//rxd
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/*******************************************************/
	/***************串口配置******************/
	USART_InitStructure.USART_BaudRate=baudrate;//波特率
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位
	USART_InitStructure.USART_Parity=USART_Parity_No;//校验位--无
	USART_InitStructure.USART_HardwareFlowControl = //硬件流失能
	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx |USART_Mode_Rx;//模式
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStructure);
	USART_Cmd(USART2,ENABLE); //串口使能
	/***********************************/
	 USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);//空闲中断
	 USART_ITConfig(USART2,USART_IT_TC, ENABLE);// 串口发送完成中断
	 /*******************************/
	 NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//子优先级3
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	 NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	 USART_DMACmd(USART2,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);//使能DMA串口发送和接受请求
	 USART2_DMA_Init();
}
void USART2_IRQHandler(void)
{
	u16 len;
	if(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=RESET)
	{
		USART_ClearFlag(USART2,USART_FLAG_TC);
		if(USART2_.DMA_TX_OK == 1)
		{
		  USART2_.TX_busy=0;//Relax bus
			USART2_.DMA_TX_OK = 0;
			USART2_.tx_OK= 1;
		}
	}
	if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET)
	{
		  USART_ReceiveData(USART2);
		  USART2_.RX_Cnt++;
			DMA_ClearFlag(DMA1_FLAG_TC6);
			DMA_Cmd(DMA1_Channel6,DISABLE);
			USART2_.rx_OK = 1;
			len = DMA_GetCurrDataCounter(DMA1_Channel6);
			USART2_.RX_Len = USART2_Buff_Len - len;
			DMA_SetCurrDataCounter(DMA1_Channel6,USART2_Buff_Len);
			DMA_Cmd(DMA1_Channel6,ENABLE);

		  USART_ClearITPendingBit(USART2,USART_IT_IDLE);
	}
}