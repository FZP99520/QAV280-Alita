#include "IncludeAll.h"
#include "stdlib.h"
#include "stdio.h"
#include "log.h"
u16 BuffWrite[10]={1,2,3,4,5,6,7,8,9,10};
u16 BuffRead[10];
int main()
{
    char str_buff[64];
	u8 flag_esc_cal=0;
	u16 del_cnt=0;
	u8 r;
	u8 dir=0;
    int times=0;
	Clock_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //·Ö×é

    #define ESC_CAL_MODE 0
    #if ESC_CAL_MODE
    while(1)
    {
        PWM1=RC_ctrl.height;
        PWM2=RC_ctrl.height;
        PWM3=RC_ctrl.height;
        PWM4=RC_ctrl.height;
        Delay_ms(20);
    }
    #endif
	while(1)
	{
//	    sprintf(str_buff,"times:%d,hello! Welcome to my STM32\n",times);
//	    USART2_SendData(str_buff,sizeof(str_buff));
//        DebugLog("times:%d 0x%x,Welcome to STM32,QAV280 Project is waiting for you~\n",times,times);
        Delay_ms(200);
        times++;
//      add test
//		FlashErase(FlashDataBase,1);
//		FlashWrite(FlashDataBase,BuffWrite,10);
//		Delay_ms(500);
//		FlashRead(FlashDataBase,BuffRead,10);
//		Delay_ms(500);
//		BuffWrite[0]=10;
//		BuffWrite[1]=11;
//		BuffWrite[2]=12;
//		BuffWrite[3]=13;
//		BuffWrite[4]=14;
//		BuffWrite[5]=15;
//		BuffWrite[6]=16;
//		BuffWrite[7]=17;
//		BuffWrite[8]=18;
//		BuffWrite[0]=19;
//		FlashErase(FlashDataBase,1);
//		FlashWrite(FlashDataBase,BuffWrite,10);
//		Delay_ms(500);
//		FlashRead(FlashDataBase,BuffRead,10);
//		Delay_ms(500);
//		Delay_ms(500);
//		Delay_ms(500);
  //  Kalman_High((float)ms5611.alt,(float)gps_data.altitude,(float)mpu.az,&Alt.now);
    //MPU6050_Update(&mpu);
	}
}
