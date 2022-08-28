#include "74hc595.h"
void Init_74HC595()
{
	 GPIO_InitTypeDef  GPIO_InitStruct; //结构体变量定义
	 SystemInit();  //打开系统时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	 GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出模式
	 GPIO_InitStruct.GPIO_Pin=SER|SRCK|RCLK;
	 GPIO_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	 GPIO_Init(GPIOB,&GPIO_InitStruct);   //GPIO初始化
}
void SendData_HC595(u8 dat)
{
	u8 a;
	SRCK_L;
	RCLK_L;
	for(a=0;a<8;a++)
	{
		if( (dat>>7) ==1) SER_H;
		else SER_L;
		dat<<=1;
		SRCK_H;
		Delay_us(1);
		SRCK_L;
	}
	RCLK_H;
	Delay_us(1);
	RCLK_L;
}
