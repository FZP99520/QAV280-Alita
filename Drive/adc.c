#include "adc.h"
#include "filter.h"
MoveAvarageFilter_TypeDef Filter_VBAT={20,0,0,{0}};
VCBAT_TypeDef VCBAT;
void Adc_Init()//ADC��ʼ���Լ�DMA����
{
	  GPIO_InitTypeDef GPIO_InitStructure; 
    ADC_InitTypeDef  ADC_InitStructure; 
	  DMA_InitTypeDef  DMA_InitStructure;
		//NVIC_InitTypeDef NVIC_InitStructure; 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE); 
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ��� 14M ���� ADC ʱ�ӣ�ADCCLK�� 
	  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//�ҽ�DMAʱ��
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;//ADC 
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //ģ������ 
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 
		GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //��������ģʽ
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;  
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //�����Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel =1; //1ͨ��ת�� 
    ADC_Init(ADC1, &ADC_InitStructure); 
    //����ָ�� ADC �Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ�� 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );
    ADC_Cmd(ADC1,ENABLE);  
		
	  ADC_ResetCalibration(ADC1);//����ָ����ADC��У׼�Ĵ���
	  while(ADC_GetResetCalibrationStatus(ADC1));//��ȡADC����У׼�Ĵ�����״̬	
	  ADC_StartCalibration(ADC1);//��ʼָ��ADC��У׼״̬
	  while(ADC_GetCalibrationStatus(ADC1));//��ȡָ��ADC��У׼����
	  ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������
}
void Get_Battery_Inf(void)
{
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){}
	VCBAT.v_adc = ADC_GetConversionValue(ADC1);
  VCBAT.v_adc =MoveAvarageFilter(&Filter_VBAT,VCBAT.v_adc );
	VCBAT.Voltage = VCBAT.v_adc*3.3f/4096*11.0f;
}

