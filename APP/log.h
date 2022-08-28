#ifndef _LOG_H
#define _LOG_H

#include "stm32f10x.h"

extern u8 log_buff[128];

void DebugLog(const char* format,...);


#endif
