#include "mpu6050.h"
#include "systick.h"
#include "printf.h"

#include "control.h"
#include "iic.h"

#include "math.h"
#include "IMU.h"
#include "qmc5883.h"
#include "flash.h"
#include "RC.h"
#include "log.h"

#ifndef TRUE
#define TRUE  (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#define Ka 0.08f
MPU_Data_TypeDef gsMPU_Data =
{
    .eAccelCali_Status = E_DevCali_Null;
};
#define  GyroOffsetNum   200
#define  AccelOffsetNum  200

//define Accelerate move average struct
MoveAvarageFilter_TypeDef Filter_Acc_X={8,0,0,{0}};
MoveAvarageFilter_TypeDef Filter_Acc_Y={8,0,0,{0}};
MoveAvarageFilter_TypeDef Filter_Acc_Z={8,0,0,{0}};
//define gyro move average struct
MoveAvarageFilter_TypeDef Filter_Gyro_X={4,0,0,{0}};
MoveAvarageFilter_TypeDef Filter_Gyro_Y={4,0,0,{0}};
MoveAvarageFilter_TypeDef Filter_Gyro_Z={4,0,0,{0}};

float Pitch,Roll,Yaw; 

static E_DevReadWrite_Ret_TypeDef _MPU_RD_Buff(u8 addr,u8 size,u8* pBuff);
static E_DevReadWrite_Ret_TypeDef _MPU_RD_ACCEL_GYRO(MPU_Data_TypeDef* pMPU);

static E_DevCali_Status_TypeDef _MPU_GET_ACCEL_OFFSET(MPU_Data_TypeDef* psIMU, u8 bSave2Flash, u8 bEcho2Controller);
static E_DevCali_Status_TypeDef _MPU_GET_GYRO_OFFSET(MPU_Data_TypeDef* psIMU, u8 bSave2Flash, u8 bEcho2Controller);

static E_DevReadWrite_Ret_TypeDef _MPU_Write_Data(u8 addr,u8 data);
static E_DevReadWrite_Ret_TypeDef _MPU_Read_Data(u8 addr,u8* pData);


E_DevInit_Status_TypeDef MPU6050_Init(void) //初始化
{
    u8 temp;
    u8 dev_id;
    _MPU_Read_Data(WHO_AM_I,&dev_id);
    if(dev_id != 0x68 )
    {
      //DebugLog("[ERROR]MPU6050 Init Fail:Device ID != 0x68\n");
      return E_DevInit_Status_Fail;
    }
    _MPU_Write_Data(PWR_MGMT_1, 0x80);//复位
    Delay_ms(100);
    _MPU_Write_Data(PWR_MGMT_1, 0x0b);//使用Z gyro轴作为参考时钟
    Delay_ms(100);
    _MPU_Write_Data(SMPLRT_DIV, 0x01);//Sample Rate=1Khz/(1+2) =500Hz T = 2ms output rate

    _MPU_Write_Data(CONFIG, 0x02); //DLPF Accel:Bandwidth 94Hz delay3ms Gyro:Bandwidth 98Hz delay 2.8ms

    _MPU_Write_Data(GYRO_CONFIG, 0x10);//设置量程+-1000deg/s

    _MPU_Write_Data(ACCEL_CONFIG, 0x08);//+-4g
    _MPU_Read_Data(ACCEL_CONFIG,&temp);

    _MPU_Write_Data(FIFO_EN,0x00);//Disable fifo
    _MPU_Write_Data(INT_EN,0x00);//Disable interrupt

    _MPU_Write_Data(INT_EN,0x00);  //设置中断,closed
    //DebugLog("[OK]MPU6050 Init OK~\n");

    return E_DevInit_Status_Done;
}
E_DevUpdate_Ret_TypeDef MPU_Data_Update(void)
{
    E_DevReadWrite_Ret_TypeDef eDevRW_Ret;

    eDevRW_Ret = _MPU_RD_ACCEL_GYRO(&gsMPU_Data); //Read data from mpu6050 first
    if(eDevRW_Ret != E_DevReadWrite_Ret_Ok)
    {
        gsMPU_Data.bGet_ORG_DataOK = FALSE;
        return E_DevUpdate_Fail;
    }
    if(gsMPU_Data.eAccelCali_Status == E_DevCali_Req || gsMPU_Data.eAccelCali_Status == E_DevCali_Doing \
        gsMPU_Data.eGyroCali_Status == E_DevCali_Req || gsMPU_Data.eGyroCali_Status == E_DevCali_Doing)
    {
        if(gsMPU_Data.eAccelCali_Status == E_DevCali_Req || gsMPU_Data.eAccelCali_Status == E_DevCali_Doing)
        {
            _MPU_GET_ACCEL_OFFSET(&gsMPU_Data,TRUE,TRUE);
        }
        if(gsMPU_Data.eGyroCali_Status == E_DevCali_Req || gsMPU_Data.eGyroCali_Status == E_DevCali_Doing)
        {
            _MPU_GET_GYRO_OFFSET(&gsMPU_Data,TRUE,TRUE);
        }
        //需要校准时不做四元数解算
        IMU.q0 = 1.0f;
        IMU.q1 = 0;
        IMU.q2 = 0;
        IMU.q3 = 0;
    }
    gsMPU_Data.s16ACCEL_X = gsMPU_Data.s16ACCEL_X - gsMPU_Data.s16os_accel_x;
    gsMPU_Data.s16ACCEL_Y = gsMPU_Data.s16ACCEL_Y - gsMPU_Data.s16os_accel_x;
    gsMPU_Data.s16ACCEL_Z = gsMPU_Data.s16ACCEL_Z - gsMPU_Data.s16os_accel_z;
    gsMPU_Data.s16GYRO_X  = gsMPU_Data.s16GYRO_X - gsMPU_Data.s16os_gyro_x;
    gsMPU_Data.s16GYRO_Y  = gsMPU_Data.s16GYRO_Y - gsMPU_Data.s16os_gyro_y;
    gsMPU_Data.s16GYRO_Z  = gsMPU_Data.s16GYRO_Z - gsMPU_Data.s16os_gyro_z;

    gsMPU_Data.s16ACCEL_X = MoveAvarageFilter(&Filter_Acc_X,gsMPU_Data.s16ACCEL_X);//滑动平均滤波
    gsMPU_Data.s16ACCEL_Y = MoveAvarageFilter(&Filter_Acc_Y,gsMPU_Data.s16ACCEL_Y);//滑动平均滤波
    gsMPU_Data.s16ACCEL_Z = MoveAvarageFilter(&Filter_Acc_Z,gsMPU_Data.s16ACCEL_Z);//滑动平均滤波
    gsMPU_Data.s16GYRO_X = MoveAvarageFilter(&Filter_Gyro_X,gsMPU_Data.s16GYRO_X);//滑动平均滤波
    gsMPU_Data.s16GYRO_Y = MoveAvarageFilter(&Filter_Gyro_Y,gsMPU_Data.s16GYRO_Y);//滑动平均滤波
    gsMPU_Data.s16GYRO_Z = MoveAvarageFilter(&Filter_Gyro_Z,gsMPU_Data.s16GYRO_Z);//滑动平均滤波

    gsMPU_Data.fgx = gsMPU_Data.s16GYRO_X*GyroCoefficient;
    gsMPU_Data.fgy = gsMPU_Data.s16GYRO_Y*GyroCoefficient;
    gsMPU_Data.fgz = gsMPU_Data.s16GYRO_Z*GyroCoefficient;

    IMU_Update(gsMPU_Data,MAG_Data.yaw,1);

    return E_DevUpdate_Ok;
}
/*
read mpu6050 data from registers
*/
static E_DevReadWrite_Ret_TypeDef _MPU_RD_ACCEL_GYRO(MPU_Data_TypeDef* pMPU)
{
    E_DevReadWrite_Ret_TypeDef eRet;
    u8 accel_temp_gyro[14];
    eRet = _MPU_RD_Buff(ACCEL_XOUT_H,14,accel_temp_gyro);
    if(eRet == E_DevReadWrite_Ret_Ok)
    {
        pMPU->s16ACCEL_X = ((s16)(accel_temp_gyro[0])<<8 |accel_temp_gyro[1]) ;
        pMPU->s16ACCEL_Y = ((s16)(accel_temp_gyro[2])<<8 |accel_temp_gyro[3]) ;
        pMPU->s16ACCEL_Z = ((s16)(accel_temp_gyro[4])<<8 |accel_temp_gyro[5]) ;
        pMPU->s16TEMP =    ((s16)(accel_temp_gyro[6])<<8 |accel_temp_gyro[7]);
        pMPU->s16GYRO_X =  ((s16)(accel_temp_gyro[8])<<8 |accel_temp_gyro[9]);
        pMPU->s16GYRO_Y =  ((s16)(accel_temp_gyro[10])<<8 |accel_temp_gyro[11]);
        pMPU->s16GYRO_Z =  ((s16)(accel_temp_gyro[12])<<8 |accel_temp_gyro[13]);
        pMPU->bGet_ORG_DataOK = TRUE;
        return E_DevReadWrite_Ret_Ok;
    }
    else
    {
        pMPU->bGet_ORG_DataOK = FALSE;
        return E_DevReadWrite_Ret_Fail;
    }
}
/*************************/
static E_DevCali_Status_TypeDef _MPU_GET_ACCEL_OFFSET(MPU_Data_TypeDef* psIMU, u8 bSave2Flash, u8 bEcho2Controller)
{
    u8 i;
    u16 buff[3];
    static s32 Accel[3]={0,0,0};
    static u16 count=0;
    if(psIMU->eAccelCali_Status == E_DevCali_Req )
    {
        if(count >= AccelOffsetNum)
        {
            psIMU->eAccelCali_Status = E_DevCali_Finished;
            psIMU->s16os_accel_x = Accel[0]/count;
            psIMU->s16os_accel_y = Accel[1]/count;
            psIMU->s16os_accel_z = Accel[2]/count;
            /*********Save Data to Flash**************/
            if(bSave2Flash)
            {
                buff[0] = (u16)psIMU->s16os_accel_x;
                buff[1] = (u16)psIMU->s16os_accel_y;
                buff[2] = (u16)psIMU->s16os_accel_z;
                FlashErase(MPUAccelOffsetAddr,1);
                i = FlashWrite(MPUAccelOffsetAddr,buff,3); //Save Offset Data
            /*************************************/
            }
            if(bEcho2Controller && (i == 3))
                RC_flag.send_accel_cal_result=1;
            Accel[0] = 0;
            Accel[1] = 0;
            Accel[2] = 0;
            count=0;
            return E_DevCali_Finished;
        }
        else
        {
            psIMU->eAccelCali_Status = E_DevCali_Doing;
            Accel[0] += psIMU->s16ACCEL_X;
            Accel[1] += psIMU->s16ACCEL_Y;
            Accel[2] += psIMU->s16ACCEL_Z-Range_Acc_Setting;
            count++;
            return E_DevCali_Doing;
        }
    }
    else 
        return E_DevCali_Null;
}
static E_DevCali_Status_TypeDef _MPU_GET_GYRO_OFFSET(MPU_Data_TypeDef* psIMU, u8 bSave2Flash, u8 bEcho2Controller)
{
    u8 i;
    u16 buff[3];
    static s32 Gyro[3]={0,0,0};
    static u16 count=0;
    if(psIMU->eGyroCali_Status == E_DevCali_Req)
    {
        if(count >= GyroOffsetNum)
        {
            psIMU->eGyroCali_Status = E_DevCali_Finished;
            psIMU->s16os_gyro_x = Gyro[0]/count;
            psIMU->s16os_gyro_y = Gyro[1]/count;
            psIMU->s16os_gyro_z = Gyro[2]/count;
            //Save Data to flash
            if(bSave2Flash)
            {
                buff[0] = (u16)psIMU->s16os_gyro_x;
                buff[1] = (u16)psIMU->s16os_gyro_y;
                buff[2] = (u16)psIMU->s16os_gyro_z;
                FlashErase(MPUGyroOffsetAddr,1);
                i = FlashWrite(MPUGyroOffsetAddr,buff,3);
            //*************************************//
            }
            if(bEcho2Controller && (i == 3)) 
                RC_flag.send_gyro_cal_result=1;
            count = 0;
            Gyro[0] = 0;
            Gyro[1] = 0;
            Gyro[2] = 0;
            return E_DevCali_Finished;
        }
        else
        {
            psIMU->eGyroCali_Status = E_DevCali_Doing;
            Gyro[0] += psIMU->s16GYRO_X;
            Gyro[1] += psIMU->s16GYRO_Y;
            Gyro[2] += psIMU->s16GYRO_Z;
            count++;
            return E_DevCali_Doing;
        }
    }
    else 
        return E_DevCali_Null;
}



/*************************/
static E_DevReadWrite_Ret_TypeDef _MPU_Write_Data(u8 addr,u8 data) //写数据
{
    E_IIC_Status_TypeDef eIIC_Status;
    u8* pBuff;
    *pBuff = data;

    eIIC_Status = Api_IIC_WriteBytes(MPU_SlaveAddress,addr,1,pBuff);

    if(eIIC_Status == E_IIC_OK) 
        return E_DevReadWrite_Ret_Ok;
    else
        return E_DevReadWrite_Ret_Fail;
}
static E_DevReadWrite_Ret_TypeDef _MPU_Read_Data(u8 addr,u8* pData) //读取数据
{
    E_IIC_Status_TypeDef eIIC_Status;

    eIIC_Status = Api_IIC_ReadBytes(MPU_SlaveAddress,addr,1,pData);
    if(eIIC_Status == E_IIC_OK) 
        return E_DevReadWrite_Ret_Ok;
    else
        return E_DevReadWrite_Ret_Fail;
}

static E_DevReadWrite_Ret_TypeDef _MPU_RD_Buff(u8 addr,u8 size,u8* pBuff)
{
    E_IIC_Status_TypeDef eIIC_Status;

    eIIC_Status = Api_IIC_ReadBytes(MPU_SlaveAddress,addr,size,pBuff);

    if(eIIC_Status == E_IIC_OK) 
        return E_DevReadWrite_Ret_Ok;
    else
        return E_DevReadWrite_Ret_Fail;
}



