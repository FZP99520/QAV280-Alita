#include "filter.h"

float MoveAvarageFilter(MoveAvarageFilter_TypeDef* filter,float data)
{
    if((filter->Num_Wind>Max_Size_Wind)||(0==filter->Num_Wind))
    {
        filter->Num_Wind=Max_Size_Wind;
    }
    if(filter->Index>=filter->Num_Wind)
    {
        filter->Index=0;
    }   
	filter->Sum-=filter->Wind[filter->Index];
	filter->Wind[filter->Index]=data;
	filter->Sum+=filter->Wind[filter->Index];
	filter->Index++;

    return filter->Sum/filter->Num_Wind;  
}
