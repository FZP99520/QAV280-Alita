#include "pwm.h"
#include "IncludeAll.h"


PWM_TypeDef MOTOR;
void PWM_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period=2500-1;//ARR
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=0;
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;//分频系数
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	TIM_Cmd(TIM3,ENABLE);//使能外设
	
	/*************PWM配置***************/
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//输出模式
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//使能
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//输出极性为高
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	/***********************************/
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	MOTOR.pwm1 = PWM_Stop;
	MOTOR.pwm2 = PWM_Stop;
	MOTOR.pwm3 = PWM_Stop;
	MOTOR.pwm4 = PWM_Stop;
	PWM_Set(&MOTOR,FlyStaLock);
}
void PWM_Set(PWM_TypeDef* p,Fly_Status_t flysta)
{
	u16 pwm_min;
  if(flysta == FlyStaAngleErr)
  {
    pwm_min=PWM_Stop;
	   p->pwm1-=4;
		 p->pwm2-=4;
		 p->pwm3-=4;
		 p->pwm4-=4;
	   p->pwm1 = LIMIT(p->pwm1,pwm_min,PWM_Flying_Max);
	   p->pwm2 = LIMIT(p->pwm2,pwm_min,PWM_Flying_Max);
	   p->pwm3 = LIMIT(p->pwm3,pwm_min,PWM_Flying_Max);
	   p->pwm4 = LIMIT(p->pwm4,pwm_min,PWM_Flying_Max);
		
		PWM1=p->pwm1;
		PWM2=p->pwm2;
		PWM3=p->pwm3;
		PWM4=p->pwm4;
  }
 else if(flysta == FlyStaFlying)
 {
	 pwm_min = PWM_Start;
	 PWM1 = LIMIT(p->pwm1,pwm_min,PWM_Flying_Max);
	 PWM2 = LIMIT(p->pwm2,pwm_min,PWM_Flying_Max);
	 PWM3 = LIMIT(p->pwm3,pwm_min,PWM_Flying_Max);
	 PWM4 = LIMIT(p->pwm4,pwm_min,PWM_Flying_Max);
 }
 else if(flysta==FlyStaUnlock)
 {
	 PWM1 = PWM_Start;
	 PWM2 = PWM_Start;
	 PWM3 = PWM_Start;
	 PWM4 = PWM_Start;
 }
 else if(flysta == FlyStaLock||flysta == FlyStaRdy || flysta == FlyStaPIDAdj)
 {
	 PWM1=PWM_Stop;
	 PWM2=PWM_Stop;
	 PWM3=PWM_Stop;
	 PWM4=PWM_Stop;
 }
 else;
}
