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
    uint8_t initialized;
    uint8_t enabled;

    TIM_HandleTypeDef *htim;
    uint32_t chA;
    uint32_t chB;
    uint32_t chC;

    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    uint8_t enable_active_low;   // 1=低电平使能，0=高电平使能

    float voltage_limit;
} Driver_t;


/* NOTE: remove any top-level function calls from headers. */


/* Public driver API */
void Driver_SetPwm(Driver_t *driver, float ua, float ub, float uc);
void Driver_Disable(Driver_t *driver);
void Driver_Enable(Driver_t *driver);
/* Driver initialization and accessor */
uint8_t Driver_Init(Driver_t *driver);
void Driver_LinkHardware(Driver_t *driver, TIM_HandleTypeDef *htim,
                         uint32_t chA, uint32_t chB, uint32_t chC,
                         GPIO_TypeDef *en_port, uint16_t en_pin,
                         uint8_t enable_active_low,
                         float voltage_limit);
Driver_t* Driver_GetInstance(DriverSide_t side);


#endif /* DRIVER_H */