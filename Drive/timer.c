#include "IncludeAll.h"
void TIMER2_Init(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;//中断优先级
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//挂接时钟
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//定时器清除
	/**************配置定时器****************/
	TIM_TimeBaseInitStructure.TIM_Period=2500;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=0;
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;//分频系数
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	/***************使能*******************/
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//使能中断
	TIM_Cmd(TIM2,ENABLE);//使能外设
  /**************设置优先级****************/
  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn; //通道选择
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;   //响应优先级 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能
  NVIC_Init(&NVIC_InitStructure); //初始化
}
void TIMER4_Init(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;//中断优先级
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//挂接时钟
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);//定时器清除
	/**************配置定时器****************/
	TIM_TimeBaseInitStructure.TIM_Period=10000;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=0;
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;//分频系数
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	/***************使能*******************/
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//使能中断
	TIM_Cmd(TIM4,ENABLE);//使能外设
  /**************设置优先级****************/
  NVIC_InitStructure.NVIC_IRQChannel =TIM4_IRQn; //通道选择
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;   //响应优先级 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能
  NVIC_Init(&NVIC_InitStructure); //初始化
}
void TIM2_IRQHandler(void) //T=2.5ms
{
	static u16 cnt0=0;
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
		cnt0++;
		IMU_Data_Update();//更新加速度计和陀螺仪数据
		PID_Cal_Update(); //更新PID数据

		if(cnt0%4==0)  MAG_Data_Update();//T=4*2.5ms=10ms 磁力计数据输出速率100Hz
		if(cnt0%8==0)  MAG_Error_Det();//T=8*2.5ms=20ms 检查磁力计数据是否正常
		if(cnt0%4==1)  MS5611_Data_Update();//T=4*2.5ms=10ms更新气压计数据   错开采样时刻
		if(cnt0%2==0)  Control(); //T=2*2.5ms=5ms
		//if(cnt0%10==0) GPS_Control_Update();//T=10*2.5ms=25ms
		if(cnt0%8==1)  //T=8*2.5ms=20ms
        {if(IMU_Data.AccelOffsetFinished==1 && MS5611.OffsetFinished==1)Height_Control_Update();}
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}
void TIM4_IRQHandler(void) //T=10ms
{
	u8 result;
	u8 sta;
	static u16 NRF_delay=0;
	static u8 usart_delay=0;
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		
		/////////////////////
		Get_Battery_Inf();
		/////////////////////
		if(USART2_.rx_OK == 1)
		{
      USART2_.rx_OK = 0;
      GPS_Analysis(&gps_data,USART2_RX_Buff);	
		}
		usart_delay++;
		if(usart_delay == 10)
		{
			usart_delay = 0;
		}
/*****************************************************/
		if(NRF.rx_OK==1)
		{
			NRF.rx_OK=0;
			NRF_delay=0;
			result = RC_Anl_BUFF(NRF.RX_BUFF,NRF.RX_Len);  //分析接收到的数据
			RC_Payload(result); //根据接收到的数据，返回相应的数据
			if(NRF.RX_Cnt%10 == 4) LED_Red_ON;
			if(NRF.RX_Cnt%10 == 8) LED_Red_OFF;
			return;
		}
	  else
			NRF_delay++;
		if(NRF_delay >= 1000)
		{
			NRF_delay=0;
			NRF_CE_LOW;
			sta=NRF_R_STATUS();
			NRF_Flush_TX_FIFO();  //丢失连接，发送缓冲被封锁，冲刷发送数据缓冲区
			NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta);
			NRF_CE_HIGH;
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);		
//	 /***************/
		
	}
}


