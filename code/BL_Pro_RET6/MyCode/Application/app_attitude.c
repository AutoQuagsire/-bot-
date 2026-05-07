#include "app_attitude.h"

#include <string.h>
#include "INT.h"
#include "usb_debug.h"
#include "stm32g4xx_hal.h"
#include "main.h"

#define ICM42688_BURST_DATA_BYTES   12U
#define ICM42688_BURST_XFER_BYTES   (1U + ICM42688_BURST_DATA_BYTES)
#define APP_ATTITUDE_PRINT_DIV      20U
#define APP_G_CONST                 9.80665f
#define APP_ACCEL_LSB_TO_MPS2       (APP_G_CONST / 8192.0f) /* +/-4g */
#define APP_GYRO_LSB_TO_RADPS       (0.017453292519943295f / 32.8f) /* +/-1000dps */

static ICM42688_Handle_t g_icm42688;
static Attitude_Estimator_t g_estimator;
static IMU_PhysData_t g_phys_data;
static uint8_t g_imu_spi2_tx[ICM42688_BURST_XFER_BYTES];
static uint8_t g_imu_spi2_rx[ICM42688_BURST_XFER_BYTES];
static volatile uint8_t g_imu_spi2_dma_busy = 0U;
static volatile uint8_t g_attitude_init_ready = 0U;

static void App_Attitude_ParseRawFrame(const uint8_t *frame, IMU_RawData_t *raw_data)
{
    raw_data->accel[0] = (int16_t)(((uint16_t)frame[0]  << 8) | frame[1]);
    raw_data->accel[1] = (int16_t)(((uint16_t)frame[2]  << 8) | frame[3]);
    raw_data->accel[2] = (int16_t)(((uint16_t)frame[4]  << 8) | frame[5]);

    raw_data->gyro[0]  = (int16_t)(((uint16_t)frame[6]  << 8) | frame[7]);
    raw_data->gyro[1]  = (int16_t)(((uint16_t)frame[8]  << 8) | frame[9]);
    raw_data->gyro[2]  = (int16_t)(((uint16_t)frame[10] << 8) | frame[11]);
}

static void App_Attitude_ConvertToPhysical(const IMU_RawData_t *raw_data, IMU_PhysData_t *phys_data)
{
    if (raw_data == NULL || phys_data == NULL) {
        return;
    }

    phys_data->ax_mps2 = (float)raw_data->accel[0] * APP_ACCEL_LSB_TO_MPS2;
    phys_data->ay_mps2 = (float)raw_data->accel[1] * APP_ACCEL_LSB_TO_MPS2;
    phys_data->az_mps2 = (float)raw_data->accel[2] * APP_ACCEL_LSB_TO_MPS2;

    phys_data->gx_radps = (float)raw_data->gyro[0] * APP_GYRO_LSB_TO_RADPS;
    phys_data->gy_radps = (float)raw_data->gyro[1] * APP_GYRO_LSB_TO_RADPS;
    phys_data->gz_radps = (float)raw_data->gyro[2] * APP_GYRO_LSB_TO_RADPS;
}


uint8_t App_Attitude_Init(void)
{
    memset(&g_icm42688, 0, sizeof(g_icm42688));
    memset(&g_estimator, 0, sizeof(g_estimator));
    memset(&g_phys_data, 0, sizeof(g_phys_data));
    memset(g_imu_spi2_tx, 0, sizeof(g_imu_spi2_tx));
    memset(g_imu_spi2_rx, 0, sizeof(g_imu_spi2_rx));
    g_imu_spi2_dma_busy = 0U;
    g_attitude_init_ready = 0U;

    Estimator_LinkICM42688P(&g_icm42688, &g_estimator);
    if (IMU_Init(&g_estimator) != IMU_OK) {
        return IMU_ERROR;
    }

    g_imu_spi2_tx[0] = ICM42688_REG_ACCEL_DATA_X1 | ICM42688_READ_FLAG;
    g_attitude_init_ready = 1U;
    return IMU_OK;
}



void App_Attitude_OnDrdyExtiISR(void)
{
    if (g_attitude_init_ready == 0U) {
        return;
    }

    if (g_imu_spi2_dma_busy != 0U) {
        return;
    }

    g_imu_spi2_dma_busy = 1U;
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive_DMA(&hspi2,
                                    g_imu_spi2_tx,
                                    g_imu_spi2_rx,
                                    ICM42688_BURST_XFER_BYTES) != HAL_OK) {
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        g_imu_spi2_dma_busy = 0U;
    }
}

void App_Attitude_OnSpi2DmaCpltISR(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    g_imu_spi2_dma_busy = 0U;
    App_Attitude_ParseRawFrame(&g_imu_spi2_rx[1], &g_estimator.raw_data);
    App_Attitude_ConvertToPhysical(&g_estimator.raw_data, &g_phys_data);
    IMU_DRDY_Flag = 1U;
}

void App_Attitude_OnSpi2DmaErrorISR(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    g_imu_spi2_dma_busy = 0U;
}

void App_Attitude_Loop(void)
{
    static uint8_t print_div = 0U;
    IMU_RawData_t raw_data;

    if (IMU_DRDY_Flag == 0U) {
        return;
    }
    IMU_DRDY_Flag = 0U;

    /* USB CDC bandwidth is limited: downsample print rate for stable FireWater streaming. */
    print_div++;
    if (print_div < APP_ATTITUDE_PRINT_DIV) {
        return;
    }
    print_div = 0U;

    __disable_irq();
    raw_data = g_estimator.raw_data;
    __enable_irq();

    /* VOFA+ FireWater: one frame per line, comma separated numeric channels. */
    USB_Debug_Printf("%lu,%f,%f,%f,%f,%f,%f\r\n",
                     (unsigned long)HAL_GetTick(),
                     (float)g_phys_data.ax_mps2,
                     (float)g_phys_data.ay_mps2,
                     (float)g_phys_data.az_mps2,
                     (float)g_phys_data.gx_radps,
                     (float)g_phys_data.gy_radps,
                     (float)g_phys_data.gz_radps);
}
