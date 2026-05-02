#include "stm32g4xx_hal.h"
#include "current_sense.h"
#include "usb_debug.h"



/* 零偏校准参数：
 * 上电/初始化后采样 1000 次，计算 ADC 静态偏置电压。
 */
#define CS_DEFAULT_SHUNT_OHM      (0.02f)
#define CS_DEFAULT_AMP_GAIN       (20.0f)
#define CS_CAL_SAMPLE_COUNT       (1000U)
#define CS_CAL_PREP_DELAY_MS      (100U)
#define CS_CAL_SAMPLE_DELAY_MS    (1U)
#define CS_DEFAULT_SHUNT_OHM      (0.02f)
#define CS_DEFAULT_AMP_GAIN       (20.0f)




/* 电压 -> 电流换算增益
 *
 * 采样关系：
 *     V_sample = I_phase * R_shunt * amp_gain
 *
 * 所以：
 *     I_phase = V_sample / (R_shunt * amp_gain)
 */
static float cs_calc_v_to_i_gain(float shunt_resistor, float amp_gain)
{
    if ((shunt_resistor <= 0.0f) || (amp_gain <= 0.0f)) {
        shunt_resistor = CS_DEFAULT_SHUNT_OHM;
        amp_gain = CS_DEFAULT_AMP_GAIN;
    }

    return 1.0f / (shunt_resistor * amp_gain);
}


/* 关闭电流采样：
 * 停止 ADC DMA，并关闭用于触发 ADC 的定时器 PWM 通道。
 */
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


/* 启用电流采样：
 * 先启动定时器触发源，再启动 ADC DMA。
 *
 * adc_buf[0] / adc_buf[1] 会被 DMA 持续更新，
 * 后续 CurrentSense_GetPhaseCurrent() 直接读取该缓存。
 */
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


/* 初始化 CurrentSense 对象。
 *
 * 只负责清状态和填默认参数；
 * 不绑定 ADC/TIM，也不启动采样。
 */
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
    cs->CsParam.gain = cs_calc_v_to_i_gain(cs->CsParam._shunt_resistor,
                                           cs->CsParam.amp_gain);

    return 1U;
}


/* 绑定 ADC 和触发用 TIM。
 *
 * 注意：
 * 这里只保存句柄，不启动 ADC/DMA。
 * 真正启动在 CurrentSense_Enable()。
 */
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


/* 设置电流采样硬件参数和符号方向。
 *
 * A_SIGN / B_SIGN 用于修正采样方向：
 * 如果某一路电流方向和 Clarke/Park 约定相反，就设为 -1。
 */
void CurrentSenseParam_Init(CurrentSense_t *cs,
                            float shunt_resistor,
                            float amp_gain,
                            int8_t A_SIGN,
                            int8_t B_SIGN)
{
    if (cs == NULL) {
        return;
    }

    cs->CsParam._shunt_resistor =
        (shunt_resistor > 0.0f) ? shunt_resistor : CS_DEFAULT_SHUNT_OHM;

    cs->CsParam.amp_gain =
        (amp_gain > 0.0f) ? amp_gain : CS_DEFAULT_AMP_GAIN;

    cs->CsParam.gain = cs_calc_v_to_i_gain(cs->CsParam._shunt_resistor,
                                           cs->CsParam.amp_gain);

    cs->CsParam.A_SIGN = (A_SIGN >= 0) ? 1 : -1;
    cs->CsParam.B_SIGN = (B_SIGN >= 0) ? 1 : -1;
}


/* 电流采样零偏校准。
 *
 * 要求：
 * 校准期间电机不能输出电流，PWM 应处于安全状态。
 *
 * 作用：
 * 统计 ADC 静态电压，作为 offset_ia / offset_ib。
 * 后续真实电流计算时会先减去该偏置。
 */
void CurrentSense_CalibrateOffsets(CurrentSense_t *cs)
{
    uint8_t was_enabled;
    float offset_ia = 0.0f;
    float offset_ib = 0.0f;
    uint32_t i;

    const unsigned long cs_tag = (unsigned long)(uintptr_t)cs;
    const unsigned long adc_tag =
        (unsigned long)(uintptr_t)((cs != NULL) ? cs->adc : NULL);
    const unsigned long tim_tag =
        (unsigned long)(uintptr_t)((cs != NULL) ? cs->tim : NULL);

    if ((cs == NULL) || (cs->adc == NULL)) {
        USB_Debug_Printf("[CS 0x%08lX] calibrate skip (null cs/adc)\r\n", cs_tag);
        return;
    }

    /* 如果当前采样没有开启，则临时开启；校准结束后恢复原状态 */
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

    /* 累加 ADC 转换后的电压值 */
    for (i = 0U; i < CS_CAL_SAMPLE_COUNT; i++) {
        HAL_Delay(CS_CAL_SAMPLE_DELAY_MS);

        offset_ia += (float)cs->adc_buf[0] * _ADC_CONV;
        offset_ib += (float)cs->adc_buf[1] * _ADC_CONV;
    }

    offset_ia /= (float)CS_CAL_SAMPLE_COUNT;
    offset_ib /= (float)CS_CAL_SAMPLE_COUNT;

    cs->CsParam.offset_ia = offset_ia;
    cs->CsParam.offset_ib = offset_ib;
    cs->CsParam.gain = cs_calc_v_to_i_gain(cs->CsParam._shunt_resistor,
                                           cs->CsParam.amp_gain);

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


/* 读取两相电流。
 *
 * 当前是两电阻采样：
 * - adc_buf[0] 对应 ia
 * - adc_buf[1] 对应 ib
 *
 * 计算流程：
 * ADC 原始值 -> 电压 -> 减零偏 -> 乘电压转电流增益 -> 乘符号修正。
 */
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

    cs->current.ia =
        sign_a * (tran_vol_a - cs->CsParam.offset_ia) * cs->CsParam.gain;

    cs->current.ib =
        sign_b * (tran_vol_b - cs->CsParam.offset_ib) * cs->CsParam.gain;

    return cs->current;
}


/* 根据当前 ia/ib 和电角度 sin/cos 计算 q 轴电流。
 *
 * 两相采样下：
 *     i_alpha = ia
 *     i_beta  = (ia + 2ib) / sqrt(3)
 *
 * Park 变换 q 轴：
 *     iq = i_beta * cos(theta) - i_alpha * sin(theta)
 */
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