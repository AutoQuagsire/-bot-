#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>

typedef struct
{
    int16_t accel[3];
    int16_t gyro[3];
} IMU_RawData_t;

#endif
