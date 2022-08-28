#include "stm32f10x.h"
#include "iic.h"
#include "spi.h"
#include "led.h"
#include "pwm.h"
#include "adc.h"
#include "systick.h"
#include "usart.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "qmc5883.h"
#include "flash.h"
#include "rc.h"
#include "nrf24l01.h"
#include "exti.h"
#include "gps.h"
#include "pid.h"

#define TRUE    1
#define FALSE   0


void Init_Device(void)
{
    u8 ret;
    //interface init
    USART1_Init(USART_BaudRate_115200);
    USART2_Init(USART_BaudRate_115200);
    Delay_ms(100);
    IIC_Init();
    DebugLog("Driver:IIC init finished~\n");
    SPI2_Init();
    DebugLog("Driver:SPI2 init finished~\n");
    Led_Init();
    DebugLog("Driver:LED init finished~\n");
    Adc_Init();
    DebugLog("Driver:ADC init finished~\n");
    PWM_Init();  
    DebugLog("Driver:PWM init finished~\n");

    //device init
    Flash_Read_Sensor_Data(&IMU_Data,&MAG_Data,&MS5611); //read offset data from flash first
    //mpu6050 init
    ret = MPU6050_Init();
    if(ret == TRUE) 
        DebugLog("Driver:MPU6050 init finished~\n");
    else
        DebugLog("Driver:MPU6050 init fail!!!\n");
    //MS5611
    ret = MS5611_Init();
    if(ret == TRUE) 
        DebugLog("Driver:MS5611 init finished~\n");
    else
        DebugLog("Driver:MS5611 init fail!!!\n");
    //QMC5883 init
    ret = MAG_Init();
    if(ret == TRUE) 
        DebugLog("Driver:MS5611 init finished~\n");
    else
        DebugLog("Driver:MS5611 init fail!!!\n");
    //nrf24l01 init
    NRF24L01_Init();
    if(NRF24L01_Check() == TRUE)
    {
        LED_Red_ON;
        EXTI12_Init();//init exti to handle communication
        NRF24L01_RX_Mode();
        DebugLog("Driver:NRF24L01 init finished~\n");
    }
    else
        DebugLog("Driver:NRF24L01 init fail\n");
    
    if(IMU_Data.AccelOffsetFinished==0)     IMU_Data.AccelOffsetReq=1;
    if(IMU_Data.GyroOffsetFinished==0)      IMU_Data.GyroOffsetReq=1;
    if(MS5611.OffsetFinished == 0)          MS5611.OffsetReq=1;
    //send data to controller first
    RC_flag.send_accel_cal_result=1;
    RC_flag.send_gyro_cal_result=1;
    RC_flag.send_mag_cal_result=1;
    RC_flag.send_baro_cal_result=1;
    
    PID_STA = Flash_Read_PID();//need to confirm
    GPS_Position_PID_Para_Init();//need to fix
     
    TIMER4_Init();
    DebugLog("Timer 4 start up!\n");
    TIMER2_Init();//open timer2 interrupt
    DebugLog("Timer 2 start up!\n");


    
}







