
#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>

/* Use opaque pointers for HAL handles to avoid including HAL headers here */
typedef void *CurrentSense_TIMHandle;
typedef void *CurrentSense_ADCHandle;

#define _ADC_CONV 0.00080586f  // ADC转电压系数: 3.3V / 4096
#define _1_SQRT3  0.57735026919f
#define _2_SQRT3  1.15470053838f




typedef struct {
    int8_t A_SIGN;
    int8_t B_SIGN;
    float offset_ia;
    float offset_ib;
    float amp_gain;
    float gain;
    float _shunt_resistor;
} CurrentSenseParam_t;




typedef struct {
    float ia;
    float ib;
} PhaseCurrent_t;



typedef struct {
    int enabled;
    CurrentSense_TIMHandle tim;
    CurrentSense_ADCHandle adc;
    volatile uint16_t adc_buf[2];
    uint32_t TIM_Channel;
    PhaseCurrent_t current;
    CurrentSenseParam_t CsParam;
} CurrentSense_t;

/* Public API */
uint8_t CurrentSense_Init(CurrentSense_t *cs);
void CurrentSense_Enable(CurrentSense_t *cs);
void CurrentSense_Disable(CurrentSense_t *cs);
void CurrentSense_Config(CurrentSense_t *cs,
                         CurrentSense_ADCHandle adc,
                         CurrentSense_TIMHandle tim,
                         uint32_t TIM_Channel);
void CurrentSenseParam_Init(CurrentSense_t *cs,
                            float shunt_resistor,
                            float amp_gain,
                            int8_t A_SIGN,
                            int8_t B_SIGN);
void CurrentSense_CalibrateOffsets(CurrentSense_t *cs);
PhaseCurrent_t CurrentSense_GetPhaseCurrent(CurrentSense_t *cs);
float CurrentSense_CalcIq(const CurrentSense_t *cs, float sint, float cost);
#endif /* CURRENT_SENSE_H */

