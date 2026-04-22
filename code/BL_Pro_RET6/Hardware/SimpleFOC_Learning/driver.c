#include "driver.h"
#include "BLDCMotor.h"
#include "stm32g4xx_hal_tim.h"
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


void Driver_Disable(Driver_t *driver)
{
    if (!driver) return;

    // 先终止能量：把比较值置零
    DriverTIM_WriteCompare(driver, 0, 0, 0);
    // 再关闭驱动
    DRIVER_DISABLE();

    // 改变状态
    driver->enabled = 0;
}


void Driver_Init(Driver_t *driver, TIM_HandleTypeDef *htim,
                 uint32_t chA, uint32_t chB, uint32_t chC,
                 float voltage_limit)
{
    if (!driver || !htim) return;

    driver->htim = htim;
    driver->chA = chA;
    driver->chB = chB;
    driver->chC = chC;
    driver->voltage_limit = voltage_limit;
    driver->enabled = 0;
    driver->initialized = 1;
}


Driver_t* Driver_GetInstance(DriverSide_t side)
{
    if (side >= 2) return NULL;
    return &g_drivers[side];
}