#include "exti.h"
#include "stm32f10x.h"
#include "nrf24l01.h"
void EXTI12_Init(void)
{
	EXTI_InitTypeDef  EXTI_InitStructure;//外部中断结构体
	GPIO_InitTypeDef  GPIO_InitStructure;//GPIO结构体
	NVIC_InitTypeDef  NVIC_InitStructure;//中断优先级
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//挂接端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开启复用功能
	/***********GPIO管脚配置************/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//上拉输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	/*******映射PB2管脚到外部中断12上面*******/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);
	/*************配置中断**************/
	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//中断模式
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//下降沿出触发
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;//使能
	EXTI_Init(&EXTI_InitStructure);
	/**********配置中断优先级************/
  NVIC_InitStructure.NVIC_IRQChannel =EXTI15_10_IRQn; //通道选择
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;   //响应优先级 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能
  NVIC_Init(&NVIC_InitStructure); //初始化
}

void EXTI15_10_IRQHandler(void)
{
	u8 STA;
	if(EXTI_GetITStatus(EXTI_Line12)!=RESET)
	{
		
		NRF_CE_LOW;
		STA=NRF_R_STATUS();
		if(STA&RX_OK)
		{
			NRF.RX_Cnt++;
			NRF.rx_OK=1;
			NRF24L01_ReceiveData(&NRF);
			//************************
       
			//************************
	   if(STA&TX_OK)//发送完成
	   {
		  NRF.tx_OK=1;
		  NRF.TX_Cnt++;
	   }
    }
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,STA); //clear interrupt flag
		NRF_CE_HIGH;
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
}

