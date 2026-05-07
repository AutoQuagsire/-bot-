#include "main.h"
#include "stm32g4xx_hal_gpio.h"
#include "sys.h"
#include "tim.h"
#include "spi.h"
#include <stdint.h>
#include "app_foc.h"
#include "app_attitude.h"

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
    if (GPIO_Pin == IMU_INT_Pin) {
        App_Attitude_OnDrdyExtiISR();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2) {
        App_Attitude_OnSpi2DmaCpltISR();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2) {
        App_Attitude_OnSpi2DmaErrorISR();
    }
}
