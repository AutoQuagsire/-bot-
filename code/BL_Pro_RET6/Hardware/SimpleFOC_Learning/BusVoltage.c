#include "stm32g4xx_hal.h"
#include "BusVoltage.h"

#ifndef BUS_VOLTAGE_SAMPLE_TIMEOUT_MS
#define BUS_VOLTAGE_SAMPLE_TIMEOUT_MS 2U
#endif


/* Initialize the bus-voltage module with safe defaults.
 *
 * The module starts disabled and unbound from any ADC/TIM handles.
 * Default parameters are loaded from the macros in BusVoltage.h.
 */
uint8_t BusVoltage_Init(BusVoltage_t *bv)
{
    if (bv == NULL) {
        return 0U;
    }

    bv->initialized = 1U;
    bv->enabled = 0U;
    bv->trigger_mode = BUS_VOLTAGE_TRIGGER_SOFTWARE;
    bv->adc = NULL;
    bv->tim = NULL;
    bv->tim_channel = 0U;

    bv->data.raw_adc = 0U;
    bv->data.adc_pin_voltage = 0.0f;
    bv->data.bus_voltage = 0.0f;

    BusVoltageParam_Init(bv,
                         BUS_VOLTAGE_ADC_REF_VOLTAGE,
                         BUS_VOLTAGE_ADC_MAX_COUNT,
                         BUS_VOLTAGE_DIVIDER_RATIO);

    return 1U;
}


/* One-call setup helper for the common ADC3 software-trigger path.
 *
 * This prepares the object with default parameters and binds the ADC handle,
 * but still leaves sampling disabled until BusVoltage_Enable() is called.
 */
uint8_t BusVoltage_Setup(BusVoltage_t *bv, BusVoltage_ADCHandle adc)
{
    if ((bv == NULL) || (adc == NULL)) {
        return 0U;
    }

    if (!BusVoltage_Init(bv)) {
        return 0U;
    }

    BusVoltage_Config(bv,
                      adc,
                      NULL,
                      0U,
                      BUS_VOLTAGE_TRIGGER_SOFTWARE);

    BusVoltageParam_Init(bv,
                         BUS_VOLTAGE_ADC_REF_VOLTAGE,
                         BUS_VOLTAGE_ADC_MAX_COUNT,
                         BUS_VOLTAGE_DIVIDER_RATIO);

    return 1U;
}


/* Bind ADC/TIM handles and select the trigger mode.
 *
 * This function only stores handles and mode selection. It does not start
 * ADC conversions or enable the trigger source; that is left to
 * BusVoltage_Enable().
 */
void BusVoltage_Config(BusVoltage_t *bv,
                       BusVoltage_ADCHandle adc,
                       BusVoltage_TIMHandle tim,
                       uint32_t tim_channel,
                       BusVoltageTriggerMode_t trigger_mode)
{
    if (bv == NULL) {
        return;
    }

    bv->adc = adc;
    bv->tim = tim;
    bv->tim_channel = tim_channel;
    bv->trigger_mode = trigger_mode;
}


/* Initialize bus-voltage conversion parameters.
 *
 * The default path is:
 *   ADC raw count -> ADC pin voltage -> bus voltage
 *
 *   Vadc = raw / adc_max_count * adc_ref_voltage
 *   Vbus = Vadc * divider_ratio
 */
void BusVoltageParam_Init(BusVoltage_t *bv,
                          float adc_ref_voltage,
                          uint16_t adc_max_count,
                          float divider_ratio)
{
    if (bv == NULL) {
        return;
    }

    bv->param.adc_ref_voltage =
        (adc_ref_voltage > 0.0f) ? adc_ref_voltage : BUS_VOLTAGE_ADC_REF_VOLTAGE;

    bv->param.adc_max_count =
        (adc_max_count > 0U) ? adc_max_count : BUS_VOLTAGE_ADC_MAX_COUNT;

    bv->param.divider_ratio =
        (divider_ratio > 0.0f) ? divider_ratio : BUS_VOLTAGE_DIVIDER_RATIO;
}


/* Enable bus-voltage sampling.
 *
 * For the current recommended software-trigger mode, this mainly marks the
 * module enabled so BusVoltage_SampleOnce() may run from a low-rate task.
 *
 * For a future timer-trigger mode, we also arm the ADC and start the trigger
 * PWM channel when valid handles are present.
 */
void BusVoltage_Enable(BusVoltage_t *bv)
{
    if ((bv == NULL) || (bv->adc == NULL)) {
        return;
    }

    if (bv->enabled != 0U) {
        return;
    }

    if (bv->trigger_mode == BUS_VOLTAGE_TRIGGER_TIMER) {
        if ((bv->tim != NULL) && (bv->tim_channel != 0U)) {
            HAL_TIM_PWM_Start((TIM_HandleTypeDef *)bv->tim, bv->tim_channel);
        }

        /* Arm the ADC so external triggers can launch conversions. */
        (void)HAL_ADC_Start((ADC_HandleTypeDef *)bv->adc);
    }

    bv->enabled = 1U;
}


/* Disable bus-voltage sampling.
 *
 * Timer-trigger mode stops both the ADC and its trigger source.
 * Software-trigger mode simply clears the enable state; HAL_ADC_Stop() is
 * still called as a harmless safety net in case a conversion was left armed.
 */
void BusVoltage_Disable(BusVoltage_t *bv)
{
    if (bv == NULL) {
        return;
    }

    if (bv->enabled == 0U) {
        return;
    }

    if (bv->adc != NULL) {
        (void)HAL_ADC_Stop((ADC_HandleTypeDef *)bv->adc);
    }

    if ((bv->trigger_mode == BUS_VOLTAGE_TRIGGER_TIMER) &&
        (bv->tim != NULL) &&
        (bv->tim_channel != 0U)) {
        HAL_TIM_PWM_Stop((TIM_HandleTypeDef *)bv->tim, bv->tim_channel);
    }

    bv->enabled = 0U;
}


/* Sample the bus voltage once using software-triggered ADC conversion.
 *
 * This API is intended for low-rate tasks such as 1kHz bus-voltage updates.
 * It is not meant to run inside the 10kHz loopFOC ISR.
 */
uint8_t BusVoltage_SampleOnce(BusVoltage_t *bv)
{
    ADC_HandleTypeDef *hadc;
    uint32_t raw_adc;
    float adc_pin_voltage;

    if ((bv == NULL) ||
        (bv->initialized == 0U) ||
        (bv->enabled == 0U) ||
        (bv->adc == NULL)) {
        return 0U;
    }

    if (bv->trigger_mode != BUS_VOLTAGE_TRIGGER_SOFTWARE) {
        return 0U;
    }

    if ((bv->param.adc_ref_voltage <= 0.0f) ||
        (bv->param.adc_max_count == 0U) ||
        (bv->param.divider_ratio <= 0.0f)) {
        return 0U;
    }

    hadc = (ADC_HandleTypeDef *)bv->adc;

    if (HAL_ADC_Start(hadc) != HAL_OK) {
        return 0U;
    }

    if (HAL_ADC_PollForConversion(hadc, BUS_VOLTAGE_SAMPLE_TIMEOUT_MS) != HAL_OK) {
        (void)HAL_ADC_Stop(hadc);
        return 0U;
    }

    raw_adc = HAL_ADC_GetValue(hadc);
    (void)HAL_ADC_Stop(hadc);

    if (raw_adc > (uint32_t)bv->param.adc_max_count) {
        raw_adc = (uint32_t)bv->param.adc_max_count;
    }

    adc_pin_voltage =
        ((float)raw_adc * bv->param.adc_ref_voltage) / (float)bv->param.adc_max_count;

    bv->data.raw_adc = (uint16_t)raw_adc;
    bv->data.adc_pin_voltage = adc_pin_voltage;
    bv->data.bus_voltage = adc_pin_voltage * bv->param.divider_ratio;

    return 1U;
}


uint16_t BusVoltage_GetRawAdc(const BusVoltage_t *bv)
{
    if (bv == NULL) {
        return 0U;
    }

    return bv->data.raw_adc;
}


float BusVoltage_GetAdcPinVoltage(const BusVoltage_t *bv)
{
    if (bv == NULL) {
        return 0.0f;
    }

    return bv->data.adc_pin_voltage;
}


float BusVoltage_GetBusVoltage(const BusVoltage_t *bv)
{
    if (bv == NULL) {
        return 0.0f;
    }

    return bv->data.bus_voltage;
}
