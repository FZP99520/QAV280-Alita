#include "systick.h"
void Delay_ms(u32 x)
{
	 u32 temp;
	SysTick->CTRL=0x01;
	SysTick->LOAD=9000*x;
	SysTick->VAL=0;
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&(!(temp&(1<<16))));
	SysTick->VAL=0;
	SysTick->CTRL=0;
}
void Delay_us(u32 x)
{
	 u32 temp;
	SysTick->CTRL=0x01;
	SysTick->LOAD=9*x;
	SysTick->VAL=0;
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&(!(temp&(1<<16))));
	SysTick->VAL=0;
	SysTick->CTRL=0;
}
