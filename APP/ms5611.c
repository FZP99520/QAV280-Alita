#include "IncludeAll.h"

MS5611_Data_TypeDef gsMS5611_Data;

//ALT_Typedef Alt;
#define dt 0.005f
//static uint8_t ms5611_osr = CMD_ADC_4096;

#define PA_OFFSET_INIT_NUM 300
// MS5611, Standard address 0x77
#define MS5611_ADDR             0xEE

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command

#define PROM_NB                 8

#define MS5611_OSR_4096_CONV_DELAY       8220   // 8.22ms
#define MS5611_OSR_2018_CONV_DELAY       4130   // 4.13ms
#define MS5611_OSR_1024_CONV_DELAY       2080   
#define MS5611_OSR_512_CONV_DELAY        1060   
#define MS5611_OSR_256_CONV_DELAY        540    


static void _ms5611_prom_read_start(u8* u8sig);
static u16 _ms5611_prom(u8 coef_num);  
static void _ms5611_reset(void);
static u32 _ms5611_read_adc(void);
static u8 _ms5611_crc(u16* pu16prom);
static void _MS5611_start_ut(void);
static void _MS5611_Get_Reff_P(MS5611_Data_TypeDef* ms,u8 bSave2Flash, u8 bEcho2Controller);


E_DevInit_Status_TypeDef MS5611_Init(void)
{
    u8 u8sig;
    u8 u8i;
    Delay_ms(10); // No idea how long the chip takes to power-up, but let's make it 10ms
    _ms5611_prom_read_start(&u8sig);
   
    gsMS5611_Data.eMS5611Cali_Status = E_DevCali_Req;

    _ms5611_reset();
    Delay_ms(100);
    // read all coefficients
    for (u8i = 0; u8i < PROM_NB; u8i++)
        gsMS5611_Data.u16prom[u8i] = _ms5611_prom(u8i);

    if(_ms5611_crc(gsMS5611_Data.u16prom) != 0)
    {
        return E_DevInit_Status_Err;
    }

    _MS5611_start_ut();
    return E_DevInit_Status_Done;
}

static void _ms5611_prom_read_start(u8* u8sig)
{
    Api_IIC_ReadBytes(MS5611_ADDR, CMD_PROM_RD,1, u8sig);
}

static void _ms5611_reset(void)
{
    Api_IIC_Write_CMD(MS5611_ADDR,CMD_RESET);
    Delay_ms(10);
}
static u16 _ms5611_prom(u8 coef_num)
{
   u8 rxbuf[2] = { 0, 0 };
   Api_IIC_ReadBytes(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2 ,rxbuf); // send PROM READ command
   return rxbuf[0] << 8 | rxbuf[1];

}
static u32 _ms5611_read_adc(void)
{
   u8 u8rxbuf[3];
   Api_IIC_ReadBytes(MS5611_ADDR, CMD_ADC_READ, 3 ,u8rxbuf); // read ADC
   return (u8rxbuf[0] << 16) | (u8rxbuf[1] << 8) | u8rxbuf[2];
}
 static void _MS5611_start_ut(void)
{
    Api_IIC_Write_CMD(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + CMD_ADC_4096); // D2 (temperature) conversion start!
}

 static void _MS5611_start_up(void)
{
    Api_IIC_Write_CMD(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + CMD_ADC_4096); // D1 (pressure) conversion start!
}
////////////////////////
void MS5611_Data_Pros(MS5611_Data_TypeDef* ms)
{
    uint32_t u32press;
    int64_t s64temp;
    int64_t s64delt;

    int32_t s32dT = (int64_t)ms->u32ut - ((uint64_t)ms->u16prom[5] * 256);
    int64_t s64off = ((int64_t)ms->u16prom[2] << 16) + (((int64_t)ms->u16prom[4] * s32dT) >> 7);
    int64_t s64sens = ((int64_t)ms->u16prom[1] << 15) + (((int64_t)ms->u16prom[3] * s32dT) >> 8);
    s64temp = 2000 + ((s32dT * (int64_t)ms->u16prom[6]) >> 23);
    if (s64temp < 2000) 
   { // temperature lower than 20degC
        s64delt = s64temp - 2000;
        s64delt = 5 * s64delt * s64delt;
        s64off -= s64delt >> 1;
        s64sens -= s64delt >> 2;

        if (s64temp < -1500) { // temperature lower than -15degC
            s64delt = s64temp + 1500;
            s64delt = s64delt * s64delt;
            s64off -= 7 * s64delt;
            s64sens -= (11 * s64delt) >> 1;
        }
    }
    u32press = ((((int64_t)ms->u32up * s64sens) >> 21) - s64off) >> 15;
    ms->f32RP = u32press;
    ms->u32temperature = s64temp;
}

static u8 _ms5611_crc(u16 *pu16prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t zero = 1;
    uint8_t crc = pu16prom[7] & 0xF;
    pu16prom[7] &= 0xFF00;

    // if eeprom is all zeros, we're probably fucked - BUT this will return valid CRC lol
    for (i = 0; i < 8; i++) {
        if (pu16prom[i] != 0)
            zero = 0;
    }

    if (zero)
        return TRUE;

    for (i = 0; i < 16; i++) {
        if (i & 1)
            res ^= ((pu16prom[i >> 1]) & 0x00FF);
        else
            res ^= (pu16prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    pu16prom[7] |= crc;

    if (crc == ((res >> 12) & 0xF))
        return FALSE;
    return TRUE;
}

static void _MS5611_Get_Reff_P(MS5611_Data_TypeDef* ms,u8 bSave2Flash, u8 bEcho2Controller)
{
    u16 pu16buff[2];
    static u16 cnt=0;
    static u8 cnt0=0;
    static float pa=0;
    if(ms->eMS5611Cali_Status == E_DevCali_Req || ms->eMS5611Cali_Status == E_DevCali_Doing)
    { 
        if(cnt%80== 0) LED_Blue_ON;
        if(cnt%80 == 40) LED_Blue_OFF;	
        if( cnt < 200) //丢弃前200次数据
        {
            cnt++;
            return;
        }
        else
        {
            if( cnt0>9)
            {
                ms->Reff_P = pa/cnt0;
                ms->eMS5611Cali_Status = E_DevCali_Finished;
                ms->f32RP=ms->Reff_P;
                ms->Diff_P=0;
                Press_To_Height(ms,0);
                ms->Altitude_Reff_P = ms->Altitude_P;
                ms->Altitude_Diff_P = 0;
                real_height=0;
                real_speed_height=0;
                exp_height=0;
                real_speed_height=0;
                cnt=0;
                cnt0=0;
                pa = 0;
                RC_flag.send_baro_cal_result=1;
                pu16buff[0] = (u16)(((int)ms->f32RP>>16)&0xFFFF);
                pu16buff[1] = (u16)((int)(ms->f32RP)&0xFFFF);
                FlashErase(MS5611OffsetAddr,1);
                FlashWrite(MS5611OffsetAddr,pu16buff,2);
            }
            else
            {
                cnt0++;
                pa += ms->f32RP; 
                return;
            }	
        }				
    }
}

void MS5611_Data_Update(void) //调用周期T=10ms  计算出气压周期为10ms
{
    static u16 _u16count=0;
    if(_u16count%2==0) 
    {
        gsMS5611_Data.u32ut = _ms5611_read_adc();
        _MS5611_start_up();
    }
    else if(_u16count%2==1) 
    {
        gsMS5611_Data.u32up = _ms5611_read_adc();
        MS5611_Data_Pros(&gsMS5611_Data); //利用补偿计算温度和气压
        Press_To_Height(&gsMS5611_Data,1); //计算出高度值，参考标准大气压

        if(gsMS5611_Data.eMS5611Cali_Status == E_DevCali_Req || gsMS5611_Data.eMS5611Cali_Status == E_DevCali_Doing)
        {
            _MS5611_Get_Reff_P(&gsMS5611_Data,FALSE,FALSE);
        }
        _MS5611_start_ut();
    }
    else;
    _u16count++;
}

#if 0

#define KALMAN_Q 0.03f
#define KALMAN_R 15.0f
static void KalmanFilter_Alt(double* ResrcData)
{
    double R = KALMAN_R;
    double Q = KALMAN_Q;
    static double x_last;
    double x_mid;
    double x_now;
    static float p_last;
    double p_mid;
    double p_now;
    double kg;

    x_mid = x_last;
    p_mid = p_last + Q;
    kg = p_mid/(p_mid+R);
    x_now = x_mid + kg*(*ResrcData - x_mid);
    p_now = (1-kg)*p_mid;
    p_last = p_now;
    x_last = x_now;
    *ResrcData=x_now;
}
#endif
