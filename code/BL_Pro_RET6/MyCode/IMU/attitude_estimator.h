#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <stdint.h>
#include "imu_types.h"
#include "icm42688p.h"
#include "kalman_1d.h"



#define IMU_OK              1u
#define IMU_ERROR           0u

#define ESTIMATOR_OK        1u
#define ESTIMATOR_ERROR     0u

#define USE_KALMAN_ESTIMATE 1


typedef struct
{
    void *imu_dev;
    uint8_t imu_type;
    IMU_RawData_t raw_data;
#ifdef USE_KALMAN_ESTIMATE
    Kalman1D_t kalman_pitch;
#endif
} Attitude_Estimator_t;

void Estimator_LinkICM42688P(ICM42688_Handle_t *dev, Attitude_Estimator_t *estimator);
uint8_t IMU_Init(Attitude_Estimator_t *estimator);
uint8_t Estimator_Init(Attitude_Estimator_t *estimator);
uint8_t IMU_Update(Attitude_Estimator_t *estimator);
uint8_t IMU_OnSpiTxRxCpltISR(Attitude_Estimator_t *estimator);
void IMU_OnSpiErrorISR(Attitude_Estimator_t *estimator);
uint8_t IMU_ConvertRawToPhysical(Attitude_Estimator_t *estimator, IMU_PhysData_t *phys);
void Estimator_MapPhysicalToBodyFrame(const IMU_PhysData_t *imu_phys, IMU_PhysData_t *body_phys);
uint8_t Estimator_Update(Attitude_Estimator_t *estimator, const IMU_PhysData_t *phys_data, float dt);
float Estimator_GetPitch(const Attitude_Estimator_t *estimator);
float Estimator_GetPitchRate(const Attitude_Estimator_t *estimator);
const IMU_RawData_t *Estimator_GetRawData(const Attitude_Estimator_t *estimator);


#endif
