#include "led.h"
#include "systick.h"
void Led_Init()  //LED��ʼ������
{
	 GPIO_InitTypeDef  GPIO_InitStruct; //�ṹ���������
	 SystemInit();  //��ϵͳʱ��
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	/**********************************/
	 GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//�������ģʽ
	 GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_15;
	 GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	/**********************************/
	 GPIO_Init(GPIOA,&GPIO_InitStruct);   //GPIO��ʼ��
   GPIO_SetBits(GPIOA,LED_Pin);	
	//This is branch 2021
}
void LedStart_Show(void)
{
	Delay_ms(500);
	Delay_ms(500);
	Delay_ms(500);
	Delay_ms(500);
	LED_Blue_ON;
	LED_Red_ON;
	Delay_ms(500);
	Delay_ms(500);
	Delay_ms(500);
	Delay_ms(500);
	LED_Blue_OFF;
	LED_Blue_OFF;
}

