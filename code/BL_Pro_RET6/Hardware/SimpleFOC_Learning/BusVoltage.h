#ifndef BUS_VOLTAGE_H
#define BUS_VOLTAGE_H

#include <stdint.h>

/* Use opaque pointers for HAL handles to avoid including HAL headers here. */
typedef void *BusVoltage_ADCHandle;
typedef void *BusVoltage_TIMHandle;

#ifndef BUS_VOLTAGE_ADC_REF_VOLTAGE
#define BUS_VOLTAGE_ADC_REF_VOLTAGE 3.3f
#endif

#ifndef BUS_VOLTAGE_ADC_MAX_COUNT
#define BUS_VOLTAGE_ADC_MAX_COUNT 4095U
#endif

/*
 * Divider ratio from ADC pin voltage back to real bus voltage.
 * Example:
 *   Vbus = Vadc * divider_ratio
 *
 * Keep the default neutral and override it after the real resistor
 * network is confirmed.
 */
#ifndef BUS_VOLTAGE_DIVIDER_RATIO
#define BUS_VOLTAGE_DIVIDER_RATIO 8.8883f
#endif

typedef enum {
    BUS_VOLTAGE_TRIGGER_SOFTWARE = 0,
    BUS_VOLTAGE_TRIGGER_TIMER    = 1
} BusVoltageTriggerMode_t;

typedef struct {
    float adc_ref_voltage;
    uint16_t adc_max_count;
    float divider_ratio;
} BusVoltageParam_t;

typedef struct {
    uint16_t raw_adc;
    float adc_pin_voltage;
    float bus_voltage;
} BusVoltageData_t;

typedef struct {
    uint8_t initialized;
    uint8_t enabled;
    BusVoltageTriggerMode_t trigger_mode;
    BusVoltage_ADCHandle adc;
    BusVoltage_TIMHandle tim;
    uint32_t tim_channel;
    BusVoltageParam_t param;
    BusVoltageData_t data;
} BusVoltage_t;


typedef struct{
    uint16_t raw_adc;
    float adc_pin_voltage;
    float bus_voltage;
} BusVoltageDebug_t;

/* Public API */
uint8_t BusVoltage_Init(BusVoltage_t *bv);
uint8_t BusVoltage_Setup(BusVoltage_t *bv, BusVoltage_ADCHandle adc);
void BusVoltage_Enable(BusVoltage_t *bv);
void BusVoltage_Disable(BusVoltage_t *bv);

void BusVoltage_Config(BusVoltage_t *bv,
                       BusVoltage_ADCHandle adc,
                       BusVoltage_TIMHandle tim,
                       uint32_t tim_channel,
                       BusVoltageTriggerMode_t trigger_mode);

void BusVoltageParam_Init(BusVoltage_t *bv,
                          float adc_ref_voltage,
                          uint16_t adc_max_count,
                          float divider_ratio);


/*
 * Intended for low-rate sampling such as 1kHz software-triggered bus
 * voltage updates. Avoid calling this from the 10kHz loopFOC ISR.
 */
uint8_t BusVoltage_SampleOnce(BusVoltage_t *bv);
uint16_t BusVoltage_GetRawAdc(const BusVoltage_t *bv);
float BusVoltage_GetAdcPinVoltage(const BusVoltage_t *bv);
float BusVoltage_GetBusVoltage(const BusVoltage_t *bv);

#endif /* BUS_VOLTAGE_H */
