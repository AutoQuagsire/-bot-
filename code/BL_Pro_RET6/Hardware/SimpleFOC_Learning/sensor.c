#include "sensor.h"
#include "stm32g4xx_hal_gpio.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define PI                  3.14159265359f  // 圆周率

/* 角度归一化到 [0, 2pi) */
static float normalize_angle(float angle)
{
   float a = angle - (float)(int32_t)(angle * (1.0f / (2.0f * PI))) * (2.0f * PI);
    return (a >= 0.0f) ? a : (a + 2.0f * PI);
}

/* 计算最短角度差，范围压到 (-pi, pi] */
static float angle_diff(float now, float last)
{
    float diff = now - last;
    while (diff >  PI) diff -= 2.0f * PI;
    while (diff <= -PI) diff += 2.0f * PI;
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
    sensor->data.shaft_velocity_windowed = 0.0f;
    sensor->data.rotations = 0 ;   

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
        sensor->data.shaft_angle = angle_now;       //装载机械角(单圈)


        float d_rad = angle_diff(angle_now, sensor->last_angle);
        sensor->data.shaft_angle_track = sensor->last_angle_track + d_rad;
        sensor->data.rotations = (int)(sensor->data.shaft_angle_track / (2.0f * PI));
        //装载机械角(带圈数)



    if (!sensor->velocity_ready) {
        sensor->data.shaft_velocity = 0.0f;

        sensor->last_angle = angle_now;//更新last_angle
        sensor->last_angle_track = sensor->data.shaft_angle_track;//更新last_angle_track
        sensor->velocity_ready = 1U;


        for (uint16_t i = 0; i < SENSOR_VEL_WIN_MAX; i++)
        {
            sensor->velocity_windowed.vel_angle_track_buf[i] = sensor->data.shaft_angle_track;
        }

        sensor->velocity_windowed.vel_win_head   = 0U;
        sensor->velocity_windowed.vel_win_count  = 1U;
        sensor->velocity_windowed.vel_win_sum_dt = 0.0f;


        return;
    }


        /* 写入当前角度到环形缓冲区 */
        sensor->velocity_windowed.vel_win_head++;
        if (sensor->velocity_windowed.vel_win_head >= SENSOR_VEL_WIN_MAX)
        {
            sensor->velocity_windowed.vel_win_head = 0U;
        }
        sensor->velocity_windowed.vel_angle_track_buf[sensor->velocity_windowed.vel_win_head] = sensor->data.shaft_angle_track;

        if (sensor->velocity_windowed.vel_win_count < SENSOR_VEL_WIN_MAX)
        {
            sensor->velocity_windowed.vel_win_count++;
            sensor->data.shaft_velocity_windowed = 0.0f;   /* 灌满窗口前先输出 0，最简单 */

            /* 预填充阶段也要持续更新差分基准，避免后续速度计算引用陈旧值 */
            sensor->last_angle = angle_now;
            sensor->last_angle_track = sensor->data.shaft_angle_track;
            return ;
        }

        /* 满窗口后，“head 的下一个位置”就是 N 个周期前的样本 */
        uint16_t old_idx = sensor->velocity_windowed.vel_win_head + 1U;
        if (old_idx >= SENSOR_VEL_WIN_MAX)
        {
            old_idx = 0U;
        }

        float theta_old = sensor->velocity_windowed.vel_angle_track_buf[old_idx];


        sensor->data.shaft_velocity_windowed = (sensor->data.shaft_angle_track - theta_old) / ((float)SENSOR_VEL_WIN_NUMBER * dt);
        sensor->data.shaft_velocity = (sensor->data.shaft_angle_track - sensor->last_angle_track) / dt;

        /* 正常更新路径下同步刷新基准，供下一周期差分使用 */
        sensor->last_angle = angle_now;
        sensor->last_angle_track = sensor->data.shaft_angle_track;


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