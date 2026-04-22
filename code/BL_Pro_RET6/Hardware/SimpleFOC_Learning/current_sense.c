#include "stm32g4xx_hal.h"
#include "current_sense.h"



void CurrentSense_Disable(CurrentSense_t *cs)
{
    if (!cs) return;

    // 关闭ADC DMA和定时器PWM输出
    HAL_ADC_Stop_DMA((ADC_HandleTypeDef*)cs->adc);
    HAL_TIM_PWM_Stop((TIM_HandleTypeDef*)cs->tim, cs->TIM_Channel);
    cs->enabled = 0;
}


void CurrentSense_Enable(CurrentSense_t *cs)
{
    if (!cs) return;

    // 启用ADC DMA和定时器PWM输出
    HAL_ADC_Start_DMA((ADC_HandleTypeDef*)cs->adc, cs->adc_buf, 2);
    HAL_TIM_PWM_Start((TIM_HandleTypeDef*)cs->tim, cs->TIM_Channel);
    cs->enabled = 1;

}

uint8_t CurrentSense_Init(CurrentSense_t *cs)
{
    if (!cs) return 0;

    cs->enabled = 0;
    return 1;
}

void CurrentSense_Config(CurrentSense_t *cs,
    CurrentSense_ADCHandle adc, CurrentSense_TIMHandle tim, uint32_t TIM_Channel)
{
    if (!cs) return;

    cs->tim = tim;
    cs->adc = adc;
    cs->TIM_Channel = TIM_Channel;
}