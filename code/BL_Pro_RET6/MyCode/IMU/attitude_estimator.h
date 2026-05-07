#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <stdint.h>
#include "imu_types.h"
#include "icm42688p.h"

#define IMU_OK              1u
#define IMU_ERROR           0u

#define ESTIMATOR_OK        1u
#define ESTIMATOR_ERROR     0u

typedef struct
{
    void *imu_dev;
    uint8_t imu_type;
    IMU_RawData_t raw_data;
} Attitude_Estimator_t;

void Estimator_LinkICM42688P(ICM42688_Handle_t *dev, Attitude_Estimator_t *estimator);
uint8_t IMU_Init(Attitude_Estimator_t *estimator);
uint8_t IMU_Update(Attitude_Estimator_t *estimator);
uint8_t Estimator_Update(Attitude_Estimator_t *estimator, float dt);
const IMU_RawData_t *Estimator_GetRawData(const Attitude_Estimator_t *estimator);


#endif
