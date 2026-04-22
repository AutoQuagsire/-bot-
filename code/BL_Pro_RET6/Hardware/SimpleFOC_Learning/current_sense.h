
#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>

/* Use opaque pointers for HAL handles to avoid including HAL headers here */
typedef void *CurrentSense_TIMHandle;
typedef void *CurrentSense_ADCHandle;

typedef struct {
    int enabled;
    CurrentSense_TIMHandle tim;
    CurrentSense_ADCHandle adc;
    uint32_t adc_buf[2];
    uint32_t TIM_Channel;
} CurrentSense_t;

/* Public API */
void CurrentSense_Disable(CurrentSense_t *cs);
void CurrentSense_Enable(CurrentSense_t *cs);

#endif /* CURRENT_SENSE_H */

