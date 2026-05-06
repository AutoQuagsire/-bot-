#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <stdint.h>
#include "icm42688p.h"

typedef struct
{
    void *imu_dev;
    uint8_t imu_type;
    uint16_t raw_data[ICM42688_RAW_DATA_WORDS];
} Attitude_Estimator_t;

void Estimator_LinkICM42688P(ICM42688_Handle_t *dev, Attitude_Estimator_t *estimator);
uint8_t IMU_Init(Attitude_Estimator_t *estimator);
uint8_t Estimator_Update(Attitude_Estimator_t *estimator, float dt);
const uint16_t *Estimator_GetRawData(const Attitude_Estimator_t *estimator);
void App_Estimator_Init(void);

#endif
