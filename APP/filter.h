#ifndef filter_H
#define filter_H

#include "stm32f10x.h"
#define Max_Size_Wind   8//????????
typedef struct
{
    float Num_Wind;
    uint8_t Index;
    float Sum;
    float Wind[Max_Size_Wind];
}MoveAvarageFilter_TypeDef;


float MoveAvarageFilter(MoveAvarageFilter_TypeDef* filter,float data);

#endif

