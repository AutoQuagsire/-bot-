#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "sys.h"

typedef enum {
    DRIVER_LEFT = 0,
    DRIVER_RIGHT = 1,
} DriverSide_t;

typedef struct {
    bool initialized;
    uint8_t enabled;
    TIM_HandleTypeDef *htim;
    uint32_t chA;
    uint32_t chB;
    uint32_t chC;       
    float voltage_limit;
} Driver_t;

/* NOTE: remove any top-level function calls from headers. */

/* Driver enable/disable macros. Ensure Pin_L/Pin_H and Motor_EN_* are defined elsewhere. */
#define DRIVER_ENABLE() Pin_L(Motor_EN_GPIO_Port, Motor_EN_Pin)
#define DRIVER_DISABLE() Pin_H(Motor_EN_GPIO_Port, Motor_EN_Pin)

/* Public driver API */
void Driver_SetPwm(Driver_t *driver, float ua, float ub, float uc);
void Driver_Disable(Driver_t *driver);

/* Driver initialization and accessor */
uint8_t Driver_Init(Driver_t *driver);
void Driver_LinkHardware(Driver_t *driver, TIM_HandleTypeDef *htim,
                         uint32_t chA, uint32_t chB, uint32_t chC,
                         float voltage_limit);
Driver_t* Driver_GetInstance(DriverSide_t side);

#endif /* DRIVER_H */