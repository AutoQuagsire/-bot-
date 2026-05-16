#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include <stdint.h>
#include "attitude_estimator.h"

#ifndef APP_ATTITUDE_USB_DEBUG_ENABLE
#define APP_ATTITUDE_USB_DEBUG_ENABLE 0
#endif

typedef struct
{
    float pitch_target_rad;
    float pitch_meas_rad;
    float pitch_rate_meas_radps;
    float speed_target_radps;
    float speed_meas_radps;
    float iq_cmd_a;
    float iq_cmd_clamped_a;
} App_AttitudeTelemetry_t;

uint8_t App_Attitude_Init(void);
void App_Attitude_Loop(void);
uint8_t App_Attitude_SetControlEnabled(uint8_t enable);
void App_Attitude_OnDrdyExtiISR(void);
void App_Attitude_OnSpi2DmaCpltISR(void);
void App_Attitude_OnSpi2DmaErrorISR(void);
float App_Attitude_GetPitch(void);
float App_Attitude_GetPitchRate(void);
void App_Attitude_GetTelemetry(App_AttitudeTelemetry_t *telemetry);
uint8_t App_Attitude_IsReady(void);
uint8_t App_Attitude_IsControlEnabled(void);

#endif
