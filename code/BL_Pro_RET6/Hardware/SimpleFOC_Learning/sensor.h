#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include "AS5047P_RW.h"



typedef struct {
    float shaft_angle;
    float shaft_velocity;
} SensorData_t;

typedef struct {
    uint8_t initialized;
    AS5047P_Handle_t *as5047p_dev;
    SensorData_t data;
    
} Sensor_t;


uint8_t Sensor_Init(Sensor_t *sensor);
void Sensor_Update(Sensor_t *sensor);
float Sensor_GetAngle(Sensor_t *sensor);
float Sensor_GetVelocity(Sensor_t *sensor);

#endif