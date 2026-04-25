#include "sensor.h"
#include "stm32g4xx_hal_gpio.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define PI                  3.14159265359f
#define TWO_PI              (2.0f * PI)
#define INV_VEL_WIN_NUMBER  (1.0f / (float)SENSOR_VEL_WIN_NUMBER)
#define SENSOR_INV_UPDATE_PERIOD_S (1.0f / SENSOR_UPDATE_PERIOD_S)

#if defined(__GNUC__) && !defined(__clang__)
#define SENSOR_OPTIMIZE __attribute__((optimize("O2,fast-math")))
#else
#define SENSOR_OPTIMIZE
#endif

/* Wrap to shortest delta in (-pi, pi] */
static SENSOR_OPTIMIZE float angle_diff(float now, float last)
{
    float diff = now - last;
    if (diff > PI) {
        diff -= TWO_PI;
    } else if (diff <= -PI) {
        diff += TWO_PI;
    }
    return diff;
}

AS5047P_Handle_t *as5047p_left;

void Sensor_LinkAS5047P(AS5047P_Handle_t *dev, Sensor_t *sensor)
{
    if (!dev || !sensor) return;

    sensor->type = SENSOR_AS5047P;
    sensor->dev = (void *)dev;
    sensor->initialized = 0U;

    sensor->data.shaft_angle = 0.0f;
    sensor->data.shaft_angle_track = 0.0f;
    sensor->data.shaft_velocity = 0.0f;
    sensor->data.shaft_velocity_windowed = 0.0f;
    sensor->data.rotations = 0;

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
        uint16_t raw = 0U;

        if (!dev->initialized) return 0;
        if (AS5047P_ReadRawAngle(dev, &raw) != AS5047P_OK) return 0;
        (void)raw;

        float angle = dev->angle_rad;
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

SENSOR_OPTIMIZE
void Sensor_Update(Sensor_t *sensor, float dt)
{
    if (!sensor || !sensor->initialized || !sensor->dev) return;
    if (dt <= 0.0f) return;
    if (sensor->type != SENSOR_AS5047P) return;

    AS5047P_Handle_t *dev = (AS5047P_Handle_t *)sensor->dev;
    if (AS5047P_ReadRawAngle(dev, &dev->raw_angle) != AS5047P_OK) {
        return;
    }

    float angle_now = dev->angle_rad;
    sensor->data.shaft_angle = angle_now;

    float d_rad = angle_diff(angle_now, sensor->last_angle);
    float angle_track = sensor->last_angle_track + d_rad;
    sensor->data.shaft_angle_track = angle_track;
    int rotations = sensor->data.rotations;
    float rotation_base = (float)rotations * TWO_PI;
    if (angle_track >= (rotation_base + TWO_PI)) {
        rotations++;
    } else if (angle_track <= (rotation_base - TWO_PI)) {
        rotations--;
    }
    sensor->data.rotations = rotations;

    if (!sensor->velocity_ready) {
        sensor->data.shaft_velocity = 0.0f;
        sensor->last_angle = angle_now;
        sensor->last_angle_track = angle_track;
        sensor->velocity_ready = 1U;

        for (uint16_t i = 0U; i < SENSOR_VEL_WIN_MAX; i++) {
            sensor->velocity_windowed.vel_angle_track_buf[i] = angle_track;
        }

        sensor->velocity_windowed.vel_win_head = 0U;
        sensor->velocity_windowed.vel_win_count = 1U;
        sensor->velocity_windowed.vel_win_sum_dt = 0.0f;
        return;
    }

    velocity_windowed_t *vw = &sensor->velocity_windowed;
    uint16_t head = (uint16_t)vw->vel_win_head + 1U;
    if (head >= SENSOR_VEL_WIN_MAX) {
        head = 0U;
    }
    vw->vel_win_head = (uint8_t)head;
    vw->vel_angle_track_buf[head] = angle_track;

    if (vw->vel_win_count < SENSOR_VEL_WIN_MAX) {
        vw->vel_win_count++;
        sensor->data.shaft_velocity_windowed = 0.0f;
        sensor->last_angle = angle_now;
        sensor->last_angle_track = angle_track;
        return;
    }

    uint16_t old_idx = head + 1U;
    if (old_idx >= SENSOR_VEL_WIN_MAX) {
        old_idx = 0U;
    }

    float theta_old = vw->vel_angle_track_buf[old_idx];
    float inv_dt = (dt == SENSOR_UPDATE_PERIOD_S) ? SENSOR_INV_UPDATE_PERIOD_S : (1.0f / dt);

    sensor->data.shaft_velocity_windowed = (angle_track - theta_old) * (INV_VEL_WIN_NUMBER * inv_dt);
    sensor->data.shaft_velocity = (angle_track - sensor->last_angle_track) * inv_dt;
    sensor->last_angle = angle_now;
    sensor->last_angle_track = angle_track;
}

float Sensor_GetAngle(Sensor_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->data.shaft_angle;
}

float Sensor_GetVelocityRaw(Sensor_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->data.shaft_velocity;
}

float Sensor_GetVelocityWindowed(Sensor_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->data.shaft_velocity_windowed;
}
