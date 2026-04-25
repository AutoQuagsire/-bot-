#include "as5047p.h"
#include "main.h"
#include "stm32g4xx_hal_gpio.h"
#include "sys.h"
#include "tim.h"
#include <stdint.h>
#include "app_foc.h"

pid_csv_data_t pid_csv_data = {0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim ==&htim5)
    {
        App_LoopForIT();

    }
}
