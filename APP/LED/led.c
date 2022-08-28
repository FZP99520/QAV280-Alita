#include "led.h"
#include "systick.h"
void Led_Init()  //LED初始化函数
{
	 GPIO_InitTypeDef  GPIO_InitStruct; //结构体变量定义
	 SystemInit();  //打开系统时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	/**********************************/
	 GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出模式
	 GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_15;
	 GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	/**********************************/
	 GPIO_Init(GPIOA,&GPIO_InitStruct);   //GPIO初始化
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

