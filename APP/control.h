#ifndef _CONTROL_H
#define _CONTROL_H

#include "stm32f10x.h"
typedef enum
{
	FlyStaLock = 0,//����
	FlyStaRdy=1,//������׼��
	FlyStaUnlock =2,//����������������ɽǶȣ�������ſ��Խ�����������ģʽ
	FlyStaFlying=3,//����ģʽ���������
	FlyStaPIDAdj=4,//PID����״̬
	FlyStaAngleErr=5,//�Ƕȴ���
}Fly_Status_t;
extern Fly_Status_t Fly_sta;
extern u8 FlyMode;
void Control(void);
void High_Handle(void);

#endif
