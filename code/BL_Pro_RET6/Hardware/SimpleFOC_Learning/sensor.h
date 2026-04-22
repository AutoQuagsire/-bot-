#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

typedef struct {
    float shaft_angle;
    float shaft_velocity;
} SensorData_t;

typedef struct {
    uint8_t initialized;
    SensorData_t data;
} Sensor_t;


uint8_t Sensor_Init(Sensor_t *sensor);
void Sensor_Update(Sensor_t *sensor);
float Sensor_GetAngle(Sensor_t *sensor);
float Sensor_GetVelocity(Sensor_t *sensor);

#endif