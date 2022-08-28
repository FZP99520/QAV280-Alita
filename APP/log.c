#include "log.h"
#include "usart.h"
#include "stdarg.h"
#include "stdio.h"

#define DebugLog_USE_USART2

#define LOG_Buff_Size_MAX 128

static char Log_Buff[LOG_Buff_Size_MAX];

void DebugLog(const char* format,...)
{
    u16 len;
    va_list args;
    va_start(args,format);
    len = vsnprintf((char*)Log_Buff,sizeof(Log_Buff)+1,(char*)format,args);
    va_end(args);
    #ifdef DebugLog_USE_USART1
        USART1_SendData((u8*)Log_Buff,len);
    #elif defined (DebugLog_USE_USART2)
        USART2_SendData((u8*)Log_Buff,len);
    #else
        return;
    #endif
}





