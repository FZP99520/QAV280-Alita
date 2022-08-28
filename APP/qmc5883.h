#ifndef _hmc5883_H_
#define _hmc5883_H_

#define MAG_QMC5883 
//#define MAG_HMC5883
#include "stm32f10x.h"
#include "iic.h"
typedef struct
{
	u8 MagOffsetReq;
	u8 MagOffsetFinished;
	u8 sta;
	u8 Err;
	s16 x;
	s16 y;
	s16 z;
	float mx;
	float my;
	float mz;
	float Strength;
	s16 xmin;
	s16 xmax;
	s16 ymin;
	s16 ymax;
	s16 zmin;
	s16 zmax;
	float xgain;
	float ygain;
	float zgain;
	s16 xoffset;
	s16 yoffset;
	s16 zoffset;
	float yaw;
}MAG_Data_TypeDef;
extern MAG_Data_TypeDef MAG_Data;


u8 MAG_Init(void);
void MAG_Data_Update(void);
void MAG_Error_Det(void);

#ifdef MAG_HMC5883
#define	 Address  0x3c
#define  Reg_Config_A 0x00  //ÅäÖÃ¼Ä´æÆ÷A
#define  Reg_Config_B 0x01  //ÅäÖÃ¼Ä´æÆ÷B
#define  Reg_Mode     0x02  //Ä£Ê½¼Ä´æÆ÷
#define  Reg_XData_H 0x03
#define  Reg_XData_L 0x04
#define  Reg_YData_H 0x05
#define  Reg_YData_L 0x06
#define  Reg_ZData_H 0x07
#define  Reg_ZData_L 0x08
#define  Reg_Status  0x09 //×´Ì¬¼Ä´æÆ÷
#endif
/**********************************/
#ifdef   MAG_QMC5883
#define	 MAG_DeviceID  0x1a
#define  Reg_XData_L 0x00
#define  Reg_XData_H 0x01
#define  Reg_YData_L 0x02
#define  Reg_YData_H 0x03
#define  Reg_ZData_L 0x04
#define  Reg_ZData_H 0x05
#define  Reg_Status  0x06  //×´Ì¬¼Ä´æÆ÷ ¶Á
#define  Reg_Temp_L  0x07  //temperature 100LSB/c
#define  Reg_Temp_H  0x08
#define  Reg_Config1  0x09 //ÅäÖÃ¼Ä´æÆ÷
#define  Reg_Config2 0x0a //interrupt reg
#define  Reg_Period  0x0b //recommanded 0x01
#define  Chip_id     0x0d
#define MAG_DRY 0x01 
#define MAG_OVL 0x02 //³¬¹ıÁ¿³Ì
#define MAG_DOR 0x04 //Ìø¹ıÊı¾İÃ»ÓĞ¶ÁÈ¡
#endif
/*******************************/
#endif
