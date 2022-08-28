#include "clock.h"
#include "stm32f10x.h"
void Clock_Init(void)
{
	 RCC_DeInit();  
    RCC_HSEConfig(RCC_HSE_ON);
    while(SUCCESS != RCC_WaitForHSEStartUp()){} 
    FLASH_SetLatency(FLASH_Latency_2);    

    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);     
    /* Fcpu = (PLL_src * PLL_MUL) = (8 Mhz / 1) * (9) = 72Mhz   */ 
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);            
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);  
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}
    /* Set system clock dividers */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK2Config(RCC_HCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);   
    /* Embedded Flash Configuration */
    FLASH_SetLatency(FLASH_Latency_2);                           
    FLASH_HalfCycleAccessCmd(FLASH_HalfCycleAccess_Disable);
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /*SYSCLK configuration*/
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

}
