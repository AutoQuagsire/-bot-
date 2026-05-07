#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include <stdint.h>
#include "attitude_estimator.h"


typedef struct {
    float ax_mps2, ay_mps2, az_mps2;
    float gx_radps, gy_radps, gz_radps;
} IMU_PhysData_t;





uint8_t App_Attitude_Init(void);
void App_Attitude_Loop(void);
void App_Attitude_OnDrdyExtiISR(void);
void App_Attitude_OnSpi2DmaCpltISR(void);
void App_Attitude_OnSpi2DmaErrorISR(void);
#endif
