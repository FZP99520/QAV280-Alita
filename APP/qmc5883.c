#include "qmc5883.h"
#include "math.h"
#include "led.h"
#include "flash.h"
#include "mpu6050.h"
#include "IMU.h"
#include "RC.h"
#include "filter.h"

#ifndef TRUE
#define TRUE  (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif

#define MagLSB 12000.0f
#define MagOffsetNum 2000
#define MAG_QMC5883

#ifndef PI
#define PI (3.1415f)
#endif

MoveAvarageFilter_TypeDef Filter_mag_yaw={4,0,0,{0}};
MAG_Data_TypeDef gsMAG_Data;

#ifdef MAG_HMC5883
#define	 Address  0x3c
#define  Reg_Config_A 0x00  //配置寄存器A
#define  Reg_Config_B 0x01  //配置寄存器B
#define  Reg_Mode     0x02  //模式寄存器
#define  Reg_XData_H 0x03
#define  Reg_XData_L 0x04
#define  Reg_YData_H 0x05
#define  Reg_YData_L 0x06
#define  Reg_ZData_H 0x07
#define  Reg_ZData_L 0x08
#define  Reg_Status  0x09 //状态寄存器
/**********************************/
#elif defined(MAG_QMC5883)
#define	 MAG_DeviceID   0x1a
#define  Reg_XData_L    0x00
#define  Reg_XData_H    0x01
#define  Reg_YData_L    0x02
#define  Reg_YData_H    0x03
#define  Reg_ZData_L    0x04
#define  Reg_ZData_H    0x05
#define  Reg_Status     0x06  //状态寄存器 读
#define  Reg_Temp_L     0x07  //temperature 100LSB/c
#define  Reg_Temp_H     0x08
#define  Reg_Config1    0x09 //配置寄存器
#define  Reg_Config2    0x0a //interrupt reg
#define  Reg_Period     0x0b //recommanded 0x01
#define  Chip_id        0x0d
#define  MAG_DRY        0x01 
#define  MAG_OVL        0x02 //超过量程
#define  MAG_DOR        0x04 //跳过数据没有读取
#endif

static E_DevReadWrite_Ret_TypeDef _MAG_RD_Buff(u8 addr,u8 size,u8* pBuff);
static E_DevReadWrite_Ret_TypeDef _MAG_WriteOneByte(u8 addr,u8 data);
static E_DevReadWrite_Ret_TypeDef _MAG_ReadOneByte(u8 addr,u8* pData);

static E_DevCali_Status_TypeDef _MAG_Calibration(MAG_Data_TypeDef* pMAG_Data, u8 bSave2Flash, u8 bEcho2Controller);

E_DevInit_Status_TypeDef MAG_Init(void)
{
    u8 u8ID;
    _MAG_ReadOneByte(Chip_id,&u8ID);
    if(u8ID != 0xFF)
    {
        gsMAG_Data.eMagInit_Status = E_DevInit_Status_Fail;
        return E_DevInit_Status_Err;
    }

#ifdef MAG_HMC5883
    _MAG_WriteOneByte(Reg_Config_A,0x10);//Output rate=10Hz
    _MAG_WriteOneByte(Reg_Config_B,0xe0);//+-4.5Ga
    _MAG_WriteOneByte(Reg_Mode,0x00);//Circulate output

#elif defined(MAG_QMC5883)
    _MAG_WriteOneByte(Reg_Config2,0x80);//复位
    Delay_ms(100);
    _MAG_WriteOneByte(Reg_Config1,0x49);//scale:2G,output rate:100Hz,Continuous mode ,over sample ratio 256 
    _MAG_WriteOneByte(Reg_Config2,0x41);//disable interrupt ,pointer roll-over mode
    _MAG_WriteOneByte(Reg_Period,0x01); //recommand 0x01
#else
    return E_DevInit_Status_Null;
#endif
    gsMAG_Data.eMagInit_Status = E_DevInit_Status_Done;
    return E_DevInit_Status_Done;
}

E_DevUpdate_Ret_TypeDef MAG_Data_Update(IMU_Data_TypeDef sIMU_Data,u8* bYawCalWithIMU)
{
    u8 pu8buff[6];
    float f32yaw_x;
    float f32yaw_y;
    float f32Norm;
    float f32mx=0;
    float f32my=0;
    float f32mz=0;
    float f32pitch;
    float f32roll;
    _MAG_ReadOneByte(Reg_Status,&(gsMAG_Data.u8MAG_Status));//读取状态
    if(!(gsMAG_Data.u8MAG_Status&MAG_DRY))//数据准备好，读取数据
    {
        if(_MAG_RD_Buff(0x00,6,pu8buff) != E_DevReadWrite_Ret_Ok) 
        {
            return E_DevUpdate_Fail;
        }
        if(!(gsMAG_Data.u8MAG_Status&MAG_OVL))
        {
            gsMAG_Data.s16MAG_X = (s16)(pu8buff[1]<<8)+pu8buff[0];
            gsMAG_Data.s16MAG_Y = (s16)(pu8buff[3]<<8)+pu8buff[2];
            gsMAG_Data.s16MAG_Z = (s16)(pu8buff[5]<<8)+pu8buff[4];
        }
        if(gsMAG_Data.eMagCali_Status == E_DevCali_Req || gsMAG_Data.eMagCali_Status == E_DevCali_Doing)
        {
           if(_MAG_Calibration(&gsMAG_Data,FALSE,FALSE) != E_DevCali_Finished);
            return E_DevUpdate_Null;
        }
        if(gsMAG_Data.eMagCali_Status == E_DevCali_Finished)
        {
            gsMAG_Data.s16MAG_X = -gsMAG_Data.f32Mag_X_Gain*(gsMAG_Data.s16MAG_X - gsMAG_Data.s16os_x);//坐标轴与机身坐标系一致
            gsMAG_Data.s16MAG_Y = -gsMAG_Data.f32Mag_Y_Gain*(gsMAG_Data.s16MAG_Y - gsMAG_Data.s16os_y);//坐标轴与机身坐标系一致
            gsMAG_Data.s16MAG_Z =  gsMAG_Data.f32Mag_Z_Gain*(gsMAG_Data.s16MAG_Z - gsMAG_Data.s16os_z);
        }

        gsMAG_Data.f32Mag_x=(float)gsMAG_Data.s16MAG_X/MagLSB;
        gsMAG_Data.f32Mag_y=(float)gsMAG_Data.s16MAG_Y/MagLSB;
        gsMAG_Data.f32Mag_z=(float)gsMAG_Data.s16MAG_Z/MagLSB;

        f32Norm = invSqrt(gsMAG_Data.f32Mag_x*gsMAG_Data.f32Mag_x + gsMAG_Data.f32Mag_y*gsMAG_Data.f32Mag_y + gsMAG_Data.f32Mag_z*gsMAG_Data.f32Mag_z);

        f32mx = f32Norm*gsMAG_Data.f32Mag_x;
        f32my = f32Norm*gsMAG_Data.f32Mag_y;
        f32mz = f32Norm*gsMAG_Data.f32Mag_z;

        if(sIMU_Data.eIMU_Update_Status == E_IMU_Update_Done)
        {
            f32pitch = sIMU_Data.f32Pitch*PI/180.0f;
            f32roll = sIMU_Data.f32Roll*PI/180.0f;
            f32yaw_x = f32mx*cosf(f32roll) + f32my*sinf(f32pitch)*sinf(f32roll) + f32mz*cos(f32pitch)*sinf(f32roll);
            f32yaw_y = f32my*cosf(f32pitch) - f32mz*sinf(f32pitch);
            gsMAG_Data.f32MAG_Yaw = atan2(f32yaw_x,f32yaw_y)*180.0f/PI;
            gsMAG_Data.bYawCalWithIMU = TRUE;
            *bYawCalWithIMU = TRUE;
        }
        else
        {
            gsMAG_Data.f32MAG_Yaw = atan2(f32mx,f32my)*180.0f/PI;
            gsMAG_Data.bYawCalWithIMU = FALSE;
            *bYawCalWithIMU = FALSE;
        }

        gsMAG_Data.f32MAG_Yaw = MoveAvarageFilter(&Filter_mag_yaw,gsMAG_Data.f32MAG_Yaw);
        //gsMAG_Data.f32Strength = sqrtf(gsMAG_Data.f32Mag_x*gsMAG_Data.f32Mag_x + gsMAG_Data.f32Mag_y*gsMAG_Data.f32Mag_y + gsMAG_Data.f32Mag_z*gsMAG_Data.f32Mag_z);

        //if(MAG_Data.yaw<0) MAG_Data.yaw +=360.0f;
        return E_DevUpdate_Ok;
    }
    else 
        return E_DevUpdate_Fail;
}
static E_DevCali_Status_TypeDef _MAG_Calibration(MAG_Data_TypeDef* pMAG_Data, u8 bSave2Flash, u8 bEcho2Controller)
{
    u16 pu16buff[6];
    static u16 _u16Cnt=0;
    if(pMAG_Data->eMagCali_Status == E_DevCali_Req || pMAG_Data->eMagCali_Status == E_DevCali_Doing)
    {
        _u16Cnt++;
        if(_u16Cnt%10==1) LED_Red_ON;
        if(_u16Cnt%20==2) LED_Red_OFF; 
        if(_u16Cnt < MagOffsetNum)
        {
            pMAG_Data->s16MAG_X_Min = (pMAG_Data->s16MAG_X < pMAG_Data->s16MAG_X_Min)?pMAG_Data->s16MAG_X:pMAG_Data->s16MAG_X_Min;
            pMAG_Data->s16MAG_Y_Min = (pMAG_Data->s16MAG_Y < pMAG_Data->s16MAG_Y_Min)?pMAG_Data->s16MAG_Y:pMAG_Data->s16MAG_Y_Min;
            pMAG_Data->s16MAG_Z_Min = (pMAG_Data->s16MAG_Z < pMAG_Data->s16MAG_Z_Min)?pMAG_Data->s16MAG_Z:pMAG_Data->s16MAG_Z_Min;

            pMAG_Data->s16MAG_X_Max = (pMAG_Data->s16MAG_X > pMAG_Data->s16MAG_X_Max)?pMAG_Data->s16MAG_X_Max:pMAG_Data->s16MAG_X;
            pMAG_Data->s16MAG_Y_Max = (pMAG_Data->s16MAG_Y > pMAG_Data->s16MAG_Y_Max)?pMAG_Data->s16MAG_Y_Max:pMAG_Data->s16MAG_Y;
            pMAG_Data->s16MAG_Z_Max = (pMAG_Data->s16MAG_Z > pMAG_Data->s16MAG_Z_Max)?pMAG_Data->s16MAG_Z_Max:pMAG_Data->s16MAG_Z;
            return E_DevCali_Doing;
        }
        else
        {
            _u16Cnt=0;
            LED_Red_ON;
            pMAG_Data->s16os_x = (pMAG_Data->s16MAG_X_Min + pMAG_Data->s16MAG_X_Max)/2.0f;
            pMAG_Data->s16os_y = (pMAG_Data->s16MAG_Y_Min + pMAG_Data->s16MAG_Y_Max)/2.0f;
            pMAG_Data->s16os_z = (pMAG_Data->s16MAG_Z_Min + pMAG_Data->s16MAG_Z_Max)/2.0f;

            pMAG_Data->f32Mag_X_Gain = 1.0f;
            pMAG_Data->f32Mag_Y_Gain = (float)(pMAG_Data->s16MAG_X_Max - pMAG_Data->s16MAG_X_Min)/(pMAG_Data->s16MAG_Y_Max - pMAG_Data->s16MAG_Y_Min);
            pMAG_Data->f32Mag_Z_Gain = (float)(pMAG_Data->s16MAG_X_Max - pMAG_Data->s16MAG_X_Min)/(pMAG_Data->s16MAG_Z_Max - pMAG_Data->s16MAG_Z_Min);

            if(bSave2Flash)
            {
                pu16buff[0] = (u16)pMAG_Data->s16os_x;
                pu16buff[1] = (u16)pMAG_Data->s16os_y;
                pu16buff[2] = (u16)pMAG_Data->s16os_z;
                pu16buff[3] = (u16)(pMAG_Data->f32Mag_X_Gain*1000);
                pu16buff[4] = (u16)(pMAG_Data->f32Mag_Y_Gain*1000);
                pu16buff[5] = (u16)(pMAG_Data->f32Mag_Z_Gain*1000);
                FlashErase(MAGOffsetAddr,1);
                FlashWrite(MAGOffsetAddr,pu16buff,6);
            }
            if(bEcho2Controller)
            {
                RC_flag.send_mag_cal_result=1;
            }

            pMAG_Data->s16MAG_X_Min = 0;
            pMAG_Data->s16MAG_X_Max = 0;
            pMAG_Data->s16MAG_Y_Min = 0;
            pMAG_Data->s16MAG_Y_Max = 0;
            pMAG_Data->s16MAG_Z_Min = 0;
            pMAG_Data->s16MAG_Z_Max = 0;
            pMAG_Data->eMagCali_Status = E_DevCali_Finished;
            return E_DevCali_Finished;
        }
    }
    else
        return E_DevCali_Null;
}

void MAG_Error_Det(MAG_Data_TypeDef* pMAG_Data)
{
    float f32temp;
    f32temp = pMAG_Data->f32Mag_x*pMAG_Data->f32Mag_x + pMAG_Data->f32Mag_y*pMAG_Data->f32Mag_y + pMAG_Data->f32Mag_z*pMAG_Data->f32Mag_z;
    pMAG_Data->f32Strength = sqrtf(f32temp);

    if(pMAG_Data->f32Strength < 0.33f || pMAG_Data->f32Strength > 0.9f)
    {
        if(pMAG_Data->u16MAG_Err_Cnt < 100) pMAG_Data->u16MAG_Err_Cnt++;
    }
    else
    {
        if(pMAG_Data->u16MAG_Err_Cnt) pMAG_Data->u16MAG_Err_Cnt--;
    }
    if(pMAG_Data->u16MAG_Err_Cnt > 50)
        pMAG_Data->bMAG_Err_Flag = TRUE;
    else
        pMAG_Data->bMAG_Err_Flag = FALSE;
}
/******************************************/

static E_DevReadWrite_Ret_TypeDef _MAG_RD_Buff(u8 addr,u8 size,u8* pBuff)
{
    E_IIC_Status_TypeDef eIIC_Status;
    eIIC_Status = Api_IIC_ReadBytes(MAG_DeviceID, addr, size, pBuff);
    if(eIIC_Status == E_IIC_OK) 
        return E_DevReadWrite_Ret_Ok;
    else
        return E_DevReadWrite_Ret_Fail;
}
static E_DevReadWrite_Ret_TypeDef _MAG_WriteOneByte(u8 addr,u8 data)
{
    E_IIC_Status_TypeDef eIIC_Status;
    u8* pBuff;
    *pBuff = data;
    eIIC_Status = Api_IIC_WriteBytes(MAG_DeviceID, addr,1,pBuff);
    if(eIIC_Status == E_IIC_OK) 
        return E_DevReadWrite_Ret_Ok;
    else
        return E_DevReadWrite_Ret_Fail;
}
static E_DevReadWrite_Ret_TypeDef _MAG_ReadOneByte(u8 addr,u8* pData)
{
    E_IIC_Status_TypeDef eIIC_Status;
    eIIC_Status = Api_IIC_ReadBytes(MAG_DeviceID, addr, 1,pData);
    if(eIIC_Status == E_IIC_OK) 
        return E_DevReadWrite_Ret_Ok;
    else
        return E_DevReadWrite_Ret_Fail;
}

