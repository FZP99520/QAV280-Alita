#ifndef _systick_H
#define _systick_H

#include "stm32f10x.h"

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

void Delay_ms(u32 x);
void Delay_us(u32 x);



#endif
