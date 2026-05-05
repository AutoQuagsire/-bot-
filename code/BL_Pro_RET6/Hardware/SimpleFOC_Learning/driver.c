#include "driver.h"
#include "BLDCMotor.h"
#include "stm32g4xx_hal_tim.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


/* Global driver instances */
static Driver_t g_drivers[2] = {0};  /* [0]=LEFT, [1]=RIGHT */
Driver_t *driver = NULL;  /* Legacy pointer, use Driver_GetInstance instead */





static void DriverTIM_WriteCompare(Driver_t *m, uint32_t phA, uint32_t phB, uint32_t phC)
{
    if (m == NULL || m->htim == NULL) return;

    __HAL_TIM_SET_COMPARE(m->htim, m->chA, phA);
    __HAL_TIM_SET_COMPARE(m->htim, m->chB, phB);
    __HAL_TIM_SET_COMPARE(m->htim, m->chC, phC);
}


void Driver_SetPwm(Driver_t *driver, float ua, float ub, float uc)
{
    if (!driver) return;
    (void)ua; (void)ub; (void)uc;

    // TODO: 在此实现三相 PWM 输出。
    // 例如：
    // SVPWM_SetPhaseVoltage(ua, ub, uc);
    // 或者更新 TIM CCR 寄存器
    DriverTIM_WriteCompare(driver, (uint32_t)(ua), (uint32_t)(ub), (uint32_t)(uc));

}


void Driver_Enable(Driver_t *driver)
{
    if (!driver) return;

    if (driver->en_port != NULL) {
        GPIO_PinState state =
            driver->enable_active_low ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(driver->en_port, driver->en_pin, state);
    }

    driver->enabled = 1;
}



void Driver_Disable(Driver_t *driver)
{
    if (!driver) return;

    Driver_SetPwm(driver, 0.0f, 0.0f, 0.0f);

    if (driver->en_port != NULL) {
        GPIO_PinState state =
            driver->enable_active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(driver->en_port, driver->en_pin, state);
    }

    driver->enabled = 0;
}


void Driver_LinkHardware(Driver_t *driver, TIM_HandleTypeDef *htim,
                         uint32_t chA, uint32_t chB, uint32_t chC,
                         GPIO_TypeDef *en_port, uint16_t en_pin,
                         uint8_t enable_active_low,
                         float voltage_limit)
{
    if (!driver || !htim) return;

    driver->htim = htim;
    driver->chA = chA;
    driver->chB = chB;
    driver->chC = chC;
    driver->en_port = en_port;
    driver->en_pin = en_pin;
    driver->enable_active_low = enable_active_low;
    driver->voltage_limit = voltage_limit;
    driver->supply_voltage = voltage_limit;     //母线电压
    driver->enabled = 0;
    driver->initialized = 0;
}



uint8_t Driver_Init(Driver_t *driver, TIM_HandleTypeDef *htim,
                 uint32_t chA, uint32_t chB, uint32_t chC,
                 float voltage_limit)
{
    if (!driver || !htim) return 0;

    driver->initialized = 0;
    driver->enabled = 0;

    driver->htim  = htim;
    driver->chA   = chA;
    driver->chB   = chB;
    driver->chC   = chC;
    driver->voltage_limit = voltage_limit;
    driver->supply_voltage = voltage_limit;

    if (HAL_TIM_PWM_Start(driver->htim, driver->chA) != HAL_OK) return 0;
    if (HAL_TIM_PWM_Start(driver->htim, driver->chB) != HAL_OK) return 0;
    if (HAL_TIM_PWM_Start(driver->htim, driver->chC) != HAL_OK) return 0;

    __HAL_TIM_SET_COMPARE(driver->htim, driver->chA, 0);
    __HAL_TIM_SET_COMPARE(driver->htim, driver->chB, 0);
    __HAL_TIM_SET_COMPARE(driver->htim, driver->chC, 0);

    driver->initialized =true;
    return 1;
}


Driver_t* Driver_GetInstance(DriverSide_t side)
{
    if (side >= 2) return NULL;
    return &g_drivers[side];
}
