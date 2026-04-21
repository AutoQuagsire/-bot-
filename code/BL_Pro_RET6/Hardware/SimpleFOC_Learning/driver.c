#include "driver.h"
#include "BLDCMotor.h"


Driver_t *driver;





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

    // 这里先写你真正的三相PWM输出代码
    // 比如：
    // SVPWM_SetPhaseVoltage(ua, ub, uc);
    // 或者 TIM CCR 更新


}


void Driver_Disable(Driver_t *driver, float ua, float ub, float uc)
{
    if (!driver) return;

    //先终止能量
    DriverTIM_WriteCompare(driver, (uint32_t)(ua), (uint32_t)(ub), (uint32_t)(uc));
    //再关闭驱动
    Driver_Disable();

    //改变状态
    driver->enabled = 0;
}