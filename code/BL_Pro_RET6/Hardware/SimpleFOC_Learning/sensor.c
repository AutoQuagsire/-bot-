#include "sensor.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


/* 角度归一化到 [0, 2pi) */
static float normalize_angle(float angle)
{
    while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
    while (angle < 0.0f)         angle += 2.0f * M_PI;
    return angle;
}

/* 计算最短角度差，范围压到 (-pi, pi] */
static float angle_diff(float now, float last)
{
    float diff = now - last;
    while (diff >  M_PI) diff -= 2.0f * M_PI;
    while (diff <= -M_PI) diff += 2.0f * M_PI;
    return diff;
}


AS5047P_Handle_t *as5047p_left;


void Sensor_LinkAS5047P( AS5047P_Handle_t *dev,Sensor_t *sensor)
{

    if (!dev || !sensor) return;

    sensor->type = SENSOR_AS5047P;
    sensor->dev = (void *)dev;
    sensor->initialized = 0U;

    sensor->data.shaft_angle = 0.0f;
    sensor->data.shaft_angle_track = 0.0f;
    sensor->data.shaft_velocity = 0.0f;

    sensor->last_angle = 0.0f;
    sensor->last_angle_track = 0.0f;
    sensor->velocity_ready = 0U;
}

uint8_t Sensor_Init(Sensor_t *sensor)
{
    if (!sensor || !sensor->dev) return 0;

    sensor->velocity_ready = 0U;
    sensor->data.shaft_velocity = 0.0f;
    sensor->data.shaft_angle_track = 0.0f;

    switch (sensor->type) {
    case SENSOR_AS5047P:
    {
        AS5047P_Handle_t *dev = (AS5047P_Handle_t *)sensor->dev;
        float angle = 0.0f;

        if (!dev->initialized) return 0;
        if (AS5047P_ReadAngleRad(dev, &angle) != AS5047P_OK) return 0;

        sensor->init_angle = angle;
        sensor->data.shaft_angle = angle;
        sensor->last_angle = angle;
        sensor->last_angle_track = angle;
        sensor->initialized = 1U;
        return 1;
    }

    default:
        return 0;
    }
}

void Sensor_Update(Sensor_t *sensor, float dt)
{
    if (!sensor || !sensor->initialized || !sensor->dev) return;
    if (dt <= 0.0f) return;

    switch (sensor->type) {
    case SENSOR_AS5047P:
    {
        AS5047P_Handle_t *dev = (AS5047P_Handle_t *)sensor->dev;
        float angle_now = 0.0f;

        if (AS5047P_ReadAngleRad(dev, &angle_now) != AS5047P_OK) {
            return;
        }

        angle_now = normalize_angle(angle_now);

    if (!sensor->velocity_ready) {
        sensor->data.shaft_angle = angle_now;
        sensor->data.shaft_angle_track = angle_now;
        sensor->data.shaft_velocity = 0.0f;

        sensor->last_angle = angle_now;
        sensor->last_angle_track = angle_now;
        sensor->velocity_ready = 1U;
        return;
    }

        float dtheta = angle_diff(angle_now, sensor->last_angle);
        float angle_track_now = sensor->last_angle_track + dtheta;
        float velocity = dtheta / dt;

        sensor->data.shaft_angle = angle_now;
        sensor->data.shaft_angle_track = angle_track_now;
        sensor->data.shaft_velocity = velocity;

        sensor->last_angle = angle_now;
        sensor->last_angle_track = angle_track_now;
        break;
    }

    default:
        break;
    }
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