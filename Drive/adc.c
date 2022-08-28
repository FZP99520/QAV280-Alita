#include "adc.h"
#include "filter.h"
MoveAvarageFilter_TypeDef Filter_VBAT={20,0,0,{0}};
VCBAT_TypeDef VCBAT;
void Adc_Init()//ADC初始化以及DMA配置
{
	  GPIO_InitTypeDef GPIO_InitStructure; 
    ADC_InitTypeDef  ADC_InitStructure; 
	  DMA_InitTypeDef  DMA_InitStructure;
		//NVIC_InitTypeDef NVIC_InitStructure; 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE); 
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  最大 14M 设置 ADC 时钟（ADCCLK） 
	  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//挂接DMA时钟
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;//ADC 
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //模拟输入 
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 
		GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //独立工作模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;  
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel =1; //1通道转换 
    ADC_Init(ADC1, &ADC_InitStructure); 
    //设置指定 ADC 的规则组通道，设置它们的转化顺序和采样时间 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );
    ADC_Cmd(ADC1,ENABLE);  
		
	  ADC_ResetCalibration(ADC1);//重置指定的ADC的校准寄存器
	  while(ADC_GetResetCalibrationStatus(ADC1));//获取ADC重置校准寄存器的状态	
	  ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
	  while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC的校准程序
	  ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能或者失能指定的ADC的软件转换启动功能
}
void Get_Battery_Inf(void)
{
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){}
	VCBAT.v_adc = ADC_GetConversionValue(ADC1);
  VCBAT.v_adc =MoveAvarageFilter(&Filter_VBAT,VCBAT.v_adc );
	VCBAT.Voltage = VCBAT.v_adc*3.3f/4096*11.0f;
}

