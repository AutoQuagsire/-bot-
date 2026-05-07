#include "main.h"
#include "stm32g4xx_hal_gpio.h"
#include "sys.h"
#include "tim.h"
#include <stdint.h>
#include "app_foc.h"

pid_csv_data_t pid_csv_data = {0};
volatile uint8_t IMU_DRDY_Flag = 0U;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim ==&htim5)
    {
        App_LoopForIT();

    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint8_t imu_int_count = 0U;
    if (GPIO_Pin == IMU_INT_Pin) {
        imu_int_count++;
        if (imu_int_count >= 2) {
            IMU_DRDY_Flag = 1U;
            imu_int_count = 0U;
        }
    }
}
