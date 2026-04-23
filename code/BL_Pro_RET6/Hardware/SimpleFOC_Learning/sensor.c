#include "sensor.h"
#include "as5047p_rw.h"



AS5047P_Handle_t *as5047p_left;


Sensor_LinkAS5047P( AS5047P_Handle_t *dev,Sensor_t *sensor)
{

    if (!dev || !sensor) return;

    sensor->as5047p_dev = dev;
   
}

uint8_t Sensor_Init(Sensor_t *sensor)
{
    if (!sensor) return 0;
    sensor->initialized = 1;
    sensor->data.shaft_angle = 0.0f;
    sensor->data.shaft_velocity = 0.0f;
    return 1;
}

void Sensor_Update(Sensor_t *sensor)
{
    if (!sensor || !sensor->initialized) return;

    // 这里以后再接 AS5047P 实际读取
}

float Sensor_GetAngle(Sensor_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->data.shaft_angle;
}

float Sensor_GetVelocity(Sensor_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->data.shaft_velocity;
}