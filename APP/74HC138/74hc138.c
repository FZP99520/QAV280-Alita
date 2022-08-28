#include "74hc138.h"
void Init_HC138()
{
	 GPIO_InitTypeDef  GPIO_InitStruct; //结构体变量定义
	 SystemInit();  //打开系统时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	 GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出模式
	 GPIO_InitStruct.GPIO_Pin=A0|A1|A2;
	 GPIO_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	 GPIO_Init(GPIOB,&GPIO_InitStruct);   //GPIO初始化
}
int Control_hc138(int num)
{
	if((num>7)||(num<0)) return 0;
	switch (num)
	{
		case 0 :A0_L;A1_L;A2_L;break;
		case 1 :A0_H;A1_L;A2_L;break;
		case 2 :A0_L;A1_H;A2_L;break;
		case 3 :A0_H;A1_H;A2_L;break;
		case 4 :A0_L;A1_L;A2_H;break;
		case 5 :A0_H;A1_L;A2_H;break;
		case 6 :A0_L;A1_H;A2_H;break;
		case 7 :A0_H;A1_H;A2_H;break;
	}
	return 1;
}
