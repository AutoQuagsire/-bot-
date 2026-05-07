#include "attitude_estimator.h"

#include "main.h"
#include "usb_debug.h"

#include <string.h>



void Estimator_LinkICM42688P(ICM42688_Handle_t *dev, Attitude_Estimator_t *estimator)
{
    if (dev == NULL || estimator == NULL) {
        return;
    }

    estimator->imu_dev = dev;
    estimator->imu_type = IMU_MODULE_ID_ICM42688;
}

uint8_t IMU_Init(Attitude_Estimator_t *estimator)
{
    uint8_t state = IMU_ERROR;

    if (estimator == NULL || estimator->imu_dev == NULL) {
        return IMU_ERROR;
    }

    switch (estimator->imu_type) {
    case IMU_MODULE_ID_ICM42688:
        state = (ICM42688_Init((ICM42688_Handle_t *)estimator->imu_dev,
                               &hspi2,
                               IMU_CS_GPIO_Port,
                               IMU_CS_Pin) == ICM42688_OK) ? IMU_OK : IMU_ERROR;
        break;

    default:
        return IMU_ERROR;
    }

    USB_Debug_Printf("IMU_State:%u\r\n", state);
    return state;
}

uint8_t IMU_Update(Attitude_Estimator_t *estimator)
{
    if (estimator == NULL || estimator->imu_dev == NULL) {
        return IMU_ERROR;
    }

    switch (estimator->imu_type) {
    case IMU_MODULE_ID_ICM42688:
        return (ICM42688_ReadRawData((ICM42688_Handle_t *)estimator->imu_dev,
                                     &estimator->raw_data) == ICM42688_OK) ? IMU_OK : IMU_ERROR;

    default:
        return IMU_ERROR;
    }
}

uint8_t Estimator_Update(Attitude_Estimator_t *estimator, float dt)
{
    (void)dt;

    if (estimator == NULL) {
        return ESTIMATOR_ERROR;
    }

    if (IMU_Update(estimator) != IMU_OK) {
        return ESTIMATOR_ERROR;
    }

    /* 姿态解算逻辑后续放在这里。 */
    return ESTIMATOR_OK;
}

const IMU_RawData_t *Estimator_GetRawData(const Attitude_Estimator_t *estimator)
{
    if (estimator == NULL) {
        return NULL;
    }

    return &estimator->raw_data;
}
