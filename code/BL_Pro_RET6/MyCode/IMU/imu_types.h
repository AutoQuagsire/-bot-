#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>

typedef struct
{
    int16_t accel[3];
    int16_t gyro[3];
} IMU_RawData_t;

typedef struct
{
    float ax_mps2;
    float ay_mps2;
    float az_mps2;
    float gx_radps;
    float gy_radps;
    float gz_radps;
} IMU_PhysData_t;

#endif
