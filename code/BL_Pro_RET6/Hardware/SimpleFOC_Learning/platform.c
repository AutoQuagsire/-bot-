#include "platform.h"
#include "stm32g4xx_hal.h"

void Platform_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}