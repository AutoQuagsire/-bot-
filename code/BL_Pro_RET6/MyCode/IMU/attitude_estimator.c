#include "attitude_estimator.h"

#include "main.h"
#include "usb_debug.h"

#include <string.h>

static ICM42688_Handle_t g_icm42688;
static Attitude_Estimator_t g_estimator;

void Estimator_LinkICM42688P(ICM42688_Handle_t *dev, Attitude_Estimator_t *estimator)
{
    if (dev == NULL || estimator == NULL) {
        return;
    }

    estimator->imu_dev = dev;
    estimator->imu_type = dev->module_id;
}

uint8_t IMU_Init(Attitude_Estimator_t *estimator)
{
    uint8_t state = ICM42688_ERR;

    if (estimator == NULL || estimator->imu_dev == NULL) {
        return ICM42688_ERR;
    }

    switch (estimator->imu_type) {
    case IMU_MODULE_ID_ICM42688:
        state = ICM42688_Init((ICM42688_Handle_t *)estimator->imu_dev,
                              &hspi2,
                              IMU_CS_GPIO_Port,
                              IMU_CS_Pin);
        break;

    default:
        return ICM42688_ERR;
    }

    USB_Debug_Printf("IMU_State:%u\r\n", state);
    return state;
}

uint8_t Estimator_Update(Attitude_Estimator_t *estimator, float dt)
{
    (void)dt;

    if (estimator == NULL || estimator->imu_dev == NULL) {
        return ICM42688_ERR;
    }

    switch (estimator->imu_type) {
    case IMU_MODULE_ID_ICM42688:
        return ICM42688_ReadRawData((ICM42688_Handle_t *)estimator->imu_dev,
                                    estimator->raw_data);

    default:
        return ICM42688_ERR;
    }
}

const uint16_t *Estimator_GetRawData(const Attitude_Estimator_t *estimator)
{
    if (estimator == NULL) {
        return NULL;
    }

    return estimator->raw_data;
}

void App_Estimator_Init(void)
{
    memset(&g_icm42688, 0, sizeof(g_icm42688));
    g_icm42688.module_id = IMU_MODULE_ID_ICM42688;
    memset(&g_estimator, 0, sizeof(g_estimator));
    Estimator_LinkICM42688P(&g_icm42688, &g_estimator);
    (void)IMU_Init(&g_estimator);
}
