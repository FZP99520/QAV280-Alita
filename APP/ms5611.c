#include "IncludeAll.h"

MS5611_Typedef MS5611;

//ALT_Typedef Alt;
#define dt 0.005f
//static uint8_t ms5611_osr = CMD_ADC_4096;

#define PA_OFFSET_INIT_NUM 300


static u16 ms5611_prom(int8_t coef_num);  
static void ms5611_reset(void);
static u32 ms5611_read_adc(void);
static u8 ms5611_crc(uint16_t *prom);
static void MS5611_start_ut(void);
u8 MS5611_Init(void)
{
    u8 sig;
    u8 i;
    Delay_ms(10); // No idea how long the chip takes to power-up, but let's make it 10ms
    Api_IIC_ReadBytes(MS5611_ADDR, CMD_PROM_RD,1, &sig);
   
	MS5611.OffsetFinished=0;
    MS5611.OffsetReq=1;
    ms5611_reset();
	  Delay_ms(100);
    // read all coefficients
    for (i = 0; i < PROM_NB; i++)
        MS5611.prom[i] = ms5611_prom(i);

    if(ms5611_crc(MS5611.prom) != 0)
        return 0;
    MS5611_start_ut();
    return 1;
}
void ms5611_reset(void)
{
    Api_IIC_Write_CMD(MS5611_ADDR,CMD_RESET);
    Delay_ms(10);
}
static u16 ms5611_prom(int8_t coef_num)
{
   u8 rxbuf[2] = { 0, 0 };
   Api_IIC_ReadBytes(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2 ,rxbuf); // send PROM READ command
   return rxbuf[0] << 8 | rxbuf[1];

}
u32 ms5611_read_adc(void)
{
   uint8_t rxbuf[3];
   Api_IIC_ReadBytes(MS5611_ADDR, CMD_ADC_READ, 3 ,rxbuf); // read ADC
   return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}
 void MS5611_start_ut(void)
{
    Api_IIC_Write_CMD(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + CMD_ADC_4096); // D2 (temperature) conversion start!
}

 void MS5611_start_up(void)
{
    Api_IIC_Write_CMD(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + CMD_ADC_4096); // D1 (pressure) conversion start!
}
////////////////////////
void MS5611_Data_Pros(MS5611_Typedef* ms)
{
    uint32_t press;
    int64_t temp;
    int64_t delt;

    int32_t dT = (int64_t)ms->ut - ((uint64_t)ms->prom[5] * 256);
    int64_t off = ((int64_t)ms->prom[2] << 16) + (((int64_t)ms->prom[4] * dT) >> 7);
    int64_t sens = ((int64_t)ms->prom[1] << 15) + (((int64_t)ms->prom[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)ms->prom[6]) >> 23);
    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;

        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    }
    press = ((((int64_t)ms->up * sens) >> 21) - off) >> 15;
    ms->RP = press;
		ms->temperature = temp;
}

u8 ms5611_crc(u16 *prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t zero = 1;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    // if eeprom is all zeros, we're probably fucked - BUT this will return valid CRC lol
    for (i = 0; i < 8; i++) {
        if (prom[i] != 0)
            zero = 0;
    }

    if (zero)
        return 1;

    for (i = 0; i < 16; i++) {
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;

    if (crc == ((res >> 12) & 0xF))
        return 0;
    return 1;
}
void MS5611_Get_Reff_P(MS5611_Typedef* ms)
{
	 u16 buff[2];
	 static u16 cnt=0;
	 static u8 cnt0=0;
	 static float pa=0;
   if(ms->OffsetReq == 1 )
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
            ms->OffsetFinished = 1;
					  ms->OffsetReq = 0;
					
					 ms->RP=ms->Reff_P;
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
					buff[0] = (u16)(((int)ms->RP>>16)&0xFFFF);
					buff[1] = (u16)((int)(ms->RP)&0xFFFF);
					FlashErase(MS5611OffsetAddr,1);
					FlashWrite(MS5611OffsetAddr,buff,2);
         }
				else
				{
					cnt0++;
           pa += ms->RP; 
					return;
				}	
			}				
    }
}
void MS5611_Data_Update(void) //调用周期T=10ms  计算出气压周期为10ms
{
	 static u16 count=0;
		if(count%2==0) {MS5611.ut = ms5611_read_adc();MS5611_start_up();}
		else if(count%2==1) 
	  {
			MS5611.up = ms5611_read_adc();
		  MS5611_Data_Pros(&MS5611); //利用补偿计算温度和气压
			Press_To_Height(&MS5611,1); //计算出高度值，参考标准大气压
		  if(MS5611.OffsetReq == 1)
			   MS5611_Get_Reff_P(&MS5611);
			MS5611_start_ut();
	  }
	  else;
    count++;
}

//#define KALMAN_Q 0.03f
//#define KALMAN_R 15.0f
//static void KalmanFilter_Alt(double* ResrcData)
//{
//	double R = KALMAN_R;
//	double Q = KALMAN_Q;
//	static double x_last;
//	double x_mid;
//	double x_now;
//	static float p_last;
//	double p_mid;
//	double p_now;
//	double kg;
//	
//	x_mid = x_last;
//	p_mid = p_last + Q;
//	kg = p_mid/(p_mid+R);
//	x_now = x_mid + kg*(*ResrcData - x_mid);
//	p_now = (1-kg)*p_mid;
//	p_last = p_now;
//	x_last = x_now;
//	*ResrcData=x_now;
//}

