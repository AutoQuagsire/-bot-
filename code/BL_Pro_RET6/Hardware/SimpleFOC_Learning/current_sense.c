#include "stm32g4xx_hal.h"
#include "current_sense.h"
#include "usb_debug.h"

#define CS_DEFAULT_SHUNT_OHM      (0.02f)
#define CS_DEFAULT_AMP_GAIN       (20.0f)
#define CS_CAL_SAMPLE_COUNT       (1000U)
#define CS_CAL_PREP_DELAY_MS      (100U)
#define CS_CAL_SAMPLE_DELAY_MS    (1U)

static float cs_calc_v_to_i_gain(float shunt_resistor, float amp_gain)
{
    if ((shunt_resistor <= 0.0f) || (amp_gain <= 0.0f)) {
        shunt_resistor = CS_DEFAULT_SHUNT_OHM;
        amp_gain = CS_DEFAULT_AMP_GAIN;
    }
    return 1.0f / (shunt_resistor * amp_gain);
}

void CurrentSense_Disable(CurrentSense_t *cs)
{
    if (cs == NULL) {
        return;
    }
    if (cs->enabled == 0) {
        return;
    }

    if (cs->adc != NULL) {
        HAL_ADC_Stop_DMA((ADC_HandleTypeDef *)cs->adc);
    }

    if ((cs->tim != NULL) && (cs->TIM_Channel != 0U)) {
        HAL_TIM_PWM_Stop((TIM_HandleTypeDef *)cs->tim, cs->TIM_Channel);
    }

    cs->enabled = 0;
}

void CurrentSense_Enable(CurrentSense_t *cs)
{
    if ((cs == NULL) || (cs->adc == NULL)) {
        return;
    }
    if (cs->enabled != 0) {
        return;
    }

    if ((cs->tim != NULL) && (cs->TIM_Channel != 0U)) {
        HAL_TIM_PWM_Start((TIM_HandleTypeDef *)cs->tim, cs->TIM_Channel);
    }

    HAL_ADC_Start_DMA((ADC_HandleTypeDef *)cs->adc, (uint32_t *)cs->adc_buf, 2);
    cs->enabled = 1;
}

uint8_t CurrentSense_Init(CurrentSense_t *cs)
{
    if (cs == NULL) {
        return 0U;
    }

    cs->enabled = 0;
    cs->tim = NULL;
    cs->adc = NULL;
    cs->TIM_Channel = 0U;
    cs->adc_buf[0] = 0U;
    cs->adc_buf[1] = 0U;
    cs->current.ia = 0.0f;
    cs->current.ib = 0.0f;

    cs->CsParam.A_SIGN = 1;
    cs->CsParam.B_SIGN = 1;
    cs->CsParam.offset_ia = 0.0f;
    cs->CsParam.offset_ib = 0.0f;
    cs->CsParam._shunt_resistor = CS_DEFAULT_SHUNT_OHM;
    cs->CsParam.amp_gain = CS_DEFAULT_AMP_GAIN;
    cs->CsParam.gain = cs_calc_v_to_i_gain(cs->CsParam._shunt_resistor, cs->CsParam.amp_gain);

    return 1U;
}

void CurrentSense_Config(CurrentSense_t *cs,
                         CurrentSense_ADCHandle adc,
                         CurrentSense_TIMHandle tim,
                         uint32_t TIM_Channel)
{
    if (cs == NULL) {
        return;
    }

    cs->tim = tim;
    cs->adc = adc;
    cs->TIM_Channel = TIM_Channel;
}

void CurrentSenseParam_Init(CurrentSense_t *cs,
                            float shunt_resistor,
                            float amp_gain,
                            int8_t A_SIGN,
                            int8_t B_SIGN)
{
    if (cs == NULL) {
        return;
    }

    cs->CsParam._shunt_resistor = (shunt_resistor > 0.0f) ? shunt_resistor : CS_DEFAULT_SHUNT_OHM;
    cs->CsParam.amp_gain = (amp_gain > 0.0f) ? amp_gain : CS_DEFAULT_AMP_GAIN;
    cs->CsParam.gain = cs_calc_v_to_i_gain(cs->CsParam._shunt_resistor, cs->CsParam.amp_gain);

    cs->CsParam.A_SIGN = (A_SIGN >= 0) ? 1 : -1;
    cs->CsParam.B_SIGN = (B_SIGN >= 0) ? 1 : -1;
}

void CurrentSense_CalibrateOffsets(CurrentSense_t *cs)
{
    uint8_t was_enabled;
    float offset_ia = 0.0f;
    float offset_ib = 0.0f;
    uint32_t i;
    const unsigned long cs_tag = (unsigned long)(uintptr_t)cs;
    const unsigned long adc_tag = (unsigned long)(uintptr_t)((cs != NULL) ? cs->adc : NULL);
    const unsigned long tim_tag = (unsigned long)(uintptr_t)((cs != NULL) ? cs->tim : NULL);

    if ((cs == NULL) || (cs->adc == NULL)) {
        USB_Debug_Printf("[CS 0x%08lX] calibrate skip (null cs/adc)\r\n", cs_tag);
        return;
    }

    was_enabled = (uint8_t)cs->enabled;
    if (!was_enabled) {
        CurrentSense_Enable(cs);
    }

    HAL_Delay(CS_CAL_PREP_DELAY_MS);

    USB_Debug_Printf("[CS 0x%08lX adc=0x%08lX tim=0x%08lX ch=%lu] offset calibration start (%lu samples)\r\n",
                     cs_tag,
                     adc_tag,
                     tim_tag,
                     (unsigned long)cs->TIM_Channel,
                     (unsigned long)CS_CAL_SAMPLE_COUNT);

    for (i = 0U; i < CS_CAL_SAMPLE_COUNT; i++) {
        HAL_Delay(CS_CAL_SAMPLE_DELAY_MS);
        offset_ia += (float)cs->adc_buf[0] * _ADC_CONV;
        offset_ib += (float)cs->adc_buf[1] * _ADC_CONV;
    }

    offset_ia /= (float)CS_CAL_SAMPLE_COUNT;
    offset_ib /= (float)CS_CAL_SAMPLE_COUNT;

    cs->CsParam.offset_ia = offset_ia;
    cs->CsParam.offset_ib = offset_ib;
    cs->CsParam.gain = cs_calc_v_to_i_gain(cs->CsParam._shunt_resistor, cs->CsParam.amp_gain);
    cs->current.ia = 0.0f;
    cs->current.ib = 0.0f;

    USB_Debug_Printf("[CS 0x%08lX adc=0x%08lX tim=0x%08lX ch=%lu] offset ia=%.5fV ib=%.5fV gain=%.5f\r\n",
                     cs_tag,
                     adc_tag,
                     tim_tag,
                     (unsigned long)cs->TIM_Channel,
                     cs->CsParam.offset_ia,
                     cs->CsParam.offset_ib,
                     cs->CsParam.gain);

    if (!was_enabled) {
        CurrentSense_Disable(cs);
    }
}

PhaseCurrent_t CurrentSense_GetPhaseCurrent(CurrentSense_t *cs)
{
    float tran_vol_a;
    float tran_vol_b;
    float sign_a;
    float sign_b;
    PhaseCurrent_t zero = {0.0f, 0.0f};

    if (cs == NULL) {
        return zero;
    }

    tran_vol_a = (float)cs->adc_buf[0] * _ADC_CONV;
    tran_vol_b = (float)cs->adc_buf[1] * _ADC_CONV;

    sign_a = (cs->CsParam.A_SIGN >= 0) ? 1.0f : -1.0f;
    sign_b = (cs->CsParam.B_SIGN >= 0) ? 1.0f : -1.0f;

    cs->current.ia = sign_a * (tran_vol_a - cs->CsParam.offset_ia) * cs->CsParam.gain;
    cs->current.ib = sign_b * (tran_vol_b - cs->CsParam.offset_ib) * cs->CsParam.gain;

    return cs->current;
}

float CurrentSense_CalcIq(const CurrentSense_t *cs, float sint, float cost)
{
    float i_alpha;
    float i_beta;

    if (cs == NULL) {
        return 0.0f;
    }

    i_alpha = cs->current.ia;
    i_beta = _1_SQRT3 * cs->current.ia + _2_SQRT3 * cs->current.ib;

    return i_beta * cost - i_alpha * sint;
}
