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

#define TRUE  (1)
#define FALSE (0)

#define Ka 0.08f
IMU_Data_TypeDef gsIMU_Data;
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

static int MPU6050_RD_Buff(u8 addr,u8 size,u8* pBuff);
static void MPU_RD_ACCEL_GYRO(IMU_Data_TypeDef* imu_data);
static u8 MPU_GET_ACCEL_OFFSET(IMU_Data_TypeDef* psIMU, u8 bSave2Flash, u8 bEcho2Controller);
static u8 MPU_GET_GYRO_OFFSET(IMU_Data_TypeDef* psIMU, u8 bSave2Flash, u8 bEcho2Controller);

static int MPU6050_Write_Data(u8 addr,u8 data);
static int MPU6050_Read_Data(u8 addr,u8* pData);

u8 temp;
u8 MPU6050_Init(void) //初始化
{
    u8 dev_id;
    MPU6050_Read_Data(WHO_AM_I,&dev_id);
    if(dev_id != 0x68 )
    {
      DebugLog("[ERROR]MPU6050 Init Fail:Device ID != 0x68\n");
      return 0;
    }
    MPU6050_Write_Data(PWR_MGMT_1, 0x80);//复位
    Delay_ms(100);
    MPU6050_Write_Data(PWR_MGMT_1, 0x0b);//使用Z gyro轴作为参考时钟
    Delay_ms(100);
    MPU6050_Write_Data(SMPLRT_DIV, 0x01);//Sample Rate=1Khz/(1+2) =500Hz T = 2ms output rate

    MPU6050_Write_Data(CONFIG, 0x02); //DLPF Accel:Bandwidth 94Hz delay3ms Gyro:Bandwidth 98Hz delay 2.8ms

    MPU6050_Write_Data(GYRO_CONFIG, 0x10);//设置量程+-1000deg/s

    MPU6050_Write_Data(ACCEL_CONFIG, 0x08);//+-4g
    MPU6050_Read_Data(ACCEL_CONFIG,&temp);

    MPU6050_Write_Data(FIFO_EN,0x00);//Disable fifo
    MPU6050_Write_Data(INT_EN,0x00);//Disable interrupt

    MPU6050_Write_Data(INT_EN,0x00);  //设置中断,closed
    DebugLog("[OK]MPU6050 Init OK~\n");
    return TRUE;
}
void IMU_Data_Update(void)
{
    u8 bRet;
    MPU_RD_ACCEL_GYRO(&gsIMU_Data); //Read data from mpu6050 first
    if(gsIMU_Data.eAccel_Offset_Status == E_OffsetReq || gsIMU_Data.eGyro_Offset_Status == E_OffsetReq)//需要校准时不做四元数解算
    {
        if(gsIMU_Data.eAccel_Offset_Status == E_OffsetReq)
        {
            MPU_GET_ACCEL_OFFSET(&gsIMU_Data,TRUE,TRUE);
        }
        if(gsIMU_Data.eGyro_Offset_Status == E_OffsetReq)
        {
            MPU_GET_GYRO_OFFSET(&gsIMU_Data,TRUE,TRUE);
        }
        IMU.q0 = 1.0f;
        IMU.q1 = 0;
        IMU.q2 = 0;
        IMU.q3 = 0;
    }
    IMU_Data.ACCEL_X = IMU_Data.ACCEL_X - IMU_Data.os_accel_x;
    IMU_Data.ACCEL_Y = IMU_Data.ACCEL_Y - IMU_Data.os_accel_y;
    IMU_Data.ACCEL_Z = IMU_Data.ACCEL_Z - IMU_Data.os_accel_z;
    IMU_Data.GYRO_X  = IMU_Data.GYRO_X - IMU_Data.os_gyro_x;
    IMU_Data.GYRO_Y  = IMU_Data.GYRO_Y - IMU_Data.os_gyro_y;
    IMU_Data.GYRO_Z  = IMU_Data.GYRO_Z - IMU_Data.os_gyro_z;

    IMU_Data.ACCEL_X = MoveAvarageFilter(&Filter_Acc_X,IMU_Data.ACCEL_X);//滑动平均滤波
    IMU_Data.ACCEL_Y = MoveAvarageFilter(&Filter_Acc_Y,IMU_Data.ACCEL_Y);//滑动平均滤波
    IMU_Data.ACCEL_Z = MoveAvarageFilter(&Filter_Acc_Z,IMU_Data.ACCEL_Z);//滑动平均滤波
    IMU_Data.GYRO_X = MoveAvarageFilter(&Filter_Gyro_X,IMU_Data.GYRO_X);//滑动平均滤波
    IMU_Data.GYRO_Y = MoveAvarageFilter(&Filter_Gyro_Y,IMU_Data.GYRO_Y);//滑动平均滤波
    IMU_Data.GYRO_Z = MoveAvarageFilter(&Filter_Gyro_Z,IMU_Data.GYRO_Z);//滑动平均滤波

    IMU_Data.gx = IMU_Data.GYRO_X*GyroCoefficient;
    IMU_Data.gy = IMU_Data.GYRO_Y*GyroCoefficient;
    IMU_Data.gz = IMU_Data.GYRO_Z*GyroCoefficient;
    IMU_Update(IMU_Data,MAG_Data.yaw,1);
}
/*
read mpu6050 data from registers
*/
static void MPU_RD_ACCEL_GYRO(IMU_Data_TypeDef* mpu)
{
    u8 accel_temp_gyro[14];
    MPU6050_RD_Buff(ACCEL_XOUT_H,14,accel_temp_gyro);
    mpu->ACCEL_X = ((s16)(accel_temp_gyro[0])<<8 |accel_temp_gyro[1]) ;
    mpu->ACCEL_Y = ((s16)(accel_temp_gyro[2])<<8 |accel_temp_gyro[3]) ;
    mpu->ACCEL_Z = ((s16)(accel_temp_gyro[4])<<8 |accel_temp_gyro[5]) ;
    mpu->TEMP =    ((s16)(accel_temp_gyro[6])<<8 |accel_temp_gyro[7]);
    mpu->GYRO_X =  ((s16)(accel_temp_gyro[8])<<8 |accel_temp_gyro[9]);
    mpu->GYRO_Y =  ((s16)(accel_temp_gyro[10])<<8 |accel_temp_gyro[11]);
    mpu->GYRO_Z =  ((s16)(accel_temp_gyro[12])<<8 |accel_temp_gyro[13]);
}
/*************************/
static u8 MPU_GET_ACCEL_OFFSET(IMU_Data_TypeDef* psIMU, u8 bSave2Flash, u8 bEcho2Controller)
{
    u8 i;
    u16 buff[3];
    static s32 Accel[3]={0,0,0};
    static u16 count=0;
    if(psIMU->eAccel_Offset_Status == E_OffsetReq )
    {
        if(count >= AccelOffsetNum)
        {
            psIMU->eAccel_Offset_Status = E_OffsetFinish;
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
            return TRUE;
        }
        else
        {
            Accel[0] += psIMU->s16ACCEL_X;
            Accel[1] += psIMU->s16ACCEL_Y;
            Accel[2] += psIMU->s16ACCEL_Z-Range_Acc_Setting;
            count++;
            return FALSE;
        }
    }
    else 
        return FALSE;
}
static u8 MPU_GET_GYRO_OFFSET(IMU_Data_TypeDef* psIMU, u8 bSave2Flash, u8 bEcho2Controller)
{
    u8 i;
    u16 buff[3];
    static s32 Gyro[3]={0,0,0};
    static u16 count=0;
    if(psIMU->eGyro_Offset_Status == E_OffsetReq)
    {
        if(count >= GyroOffsetNum)
        {
            psIMU->eGyro_Offset_Status = E_OffsetFinish;
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
            return TRUE;
        }
        else
        {
            Gyro[0] += psIMU->s16GYRO_X;
            Gyro[1] += psIMU->s16GYRO_Y;
            Gyro[2] += psIMU->s16GYRO_Z;
            count++;
            return FALSE;
        }
    }
    else 
        return FALSE;
}



/*************************/
static int MPU6050_Write_Data(u8 addr,u8 data) //写数据
{
    int res;
    u8* pBuff;
    *pBuff = data;
    res = Api_IIC_WriteBytes(MPU_SlaveAddress,addr,1,pBuff);
    return res;
}
static int MPU6050_Read_Data(u8 addr,u8* pData) //读取数据
{
    int res;
    res = Api_IIC_ReadBytes(MPU_SlaveAddress,addr,1,pData);
    return res;
}

static int MPU6050_RD_Buff(u8 addr,u8 size,u8* pBuff)
{
    int res;
    res = Api_IIC_ReadBytes(MPU_SlaveAddress,addr,size,pBuff);
    return res;
}



