#include "exti.h"
#include "stm32f10x.h"
#include "nrf24l01.h"
void EXTI12_Init(void)
{
	EXTI_InitTypeDef  EXTI_InitStructure;//�ⲿ�жϽṹ��
	GPIO_InitTypeDef  GPIO_InitStructure;//GPIO�ṹ��
	NVIC_InitTypeDef  NVIC_InitStructure;//�ж����ȼ�
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//�ҽӶ˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�������ù���
	/***********GPIO�ܽ�����************/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	/*******ӳ��PB2�ܽŵ��ⲿ�ж�12����*******/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);
	/*************�����ж�**************/
	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//�ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//�½��س�����
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;//ʹ��
	EXTI_Init(&EXTI_InitStructure);
	/**********�����ж����ȼ�************/
  NVIC_InitStructure.NVIC_IRQChannel =EXTI15_10_IRQn; //ͨ��ѡ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;   //��Ӧ���ȼ� 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ��
  NVIC_Init(&NVIC_InitStructure); //��ʼ��
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
	   if(STA&TX_OK)//�������
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

