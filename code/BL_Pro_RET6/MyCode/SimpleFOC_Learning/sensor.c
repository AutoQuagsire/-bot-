#include "sensor.h"

#define PI                  3.14159265359f
#define TWO_PI              (2.0f * PI)

/* 窗口测速系数：窗口内总角度增量 / 窗口总时间 */
#define INV_VEL_WIN_NUMBER  (1.0f / (float)SENSOR_VEL_WIN_NUMBER)

/* 固定更新周期下的倒数，避免每次都做除法 */
#define SENSOR_INV_UPDATE_PERIOD_S (1.0f / SENSOR_UPDATE_PERIOD_S)

#if defined(__GNUC__) && !defined(__clang__)
#define SENSOR_OPTIMIZE __attribute__((optimize("O2,fast-math")))
#else
#define SENSOR_OPTIMIZE
#endif


/* 计算两个 [0, 2π) 角度之间的最短有符号差值。
 * 输出范围约为 (-π, π]，用于处理编码器跨 0/2π 边界。
 */
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


/* 将 AS5047P 底层驱动对象绑定到 Sensor 层。
 *
 * Sensor 层只保存 dev 指针，不直接关心 SPI 细节；
 * 后续 Motor 层也只通过 Sensor_* 接口拿角度/速度。
 */
void Sensor_LinkAS5047P(AS5047P_Handle_t *dev, Sensor_t *sensor)
{
    if (!dev || !sensor) return;

    sensor->type = SENSOR_AS5047P;
    sensor->dev = (void *)dev;
    sensor->initialized = 0U;
    sensor->velocity_ready = 0U;

    sensor->data.shaft_angle = 0.0f;
    sensor->data.shaft_velocity = 0.0f;
    sensor->data.shaft_velocity_windowed = 0.0f;

    /* last_angle 用于单步速度差分；
     * angle_track 是连续角度，用于跨圈和窗口测速。
     */
    sensor->last_angle = 0.0f;
    sensor->angle_track = 0.0f;
    sensor->rotations = 0;

    /* 初始化窗口测速环形缓冲区 */
    sensor->velocity_windowed.vel_win_head = 0U;
    sensor->velocity_windowed.vel_win_count = 0U;

    for (uint16_t i = 0U; i < SENSOR_VEL_WIN_MAX; i++) {
        sensor->velocity_windowed.vel_angle_track_buf[i] = 0.0f;
    }
}


/* Sensor 初始化。
 *
 * 职责：
 * 1. 检查底层编码器链路是否可用；
 * 2. 读取一次初始机械角；
 * 3. 建立初始角度参考。
 *
 * 注意：
 * Init 不计算速度。真正的速度参考从第一次 Sensor_Update() 开始建立。
 */
uint8_t Sensor_Init(Sensor_t *sensor)
{
    if (!sensor || !sensor->dev) return 0U;

    sensor->velocity_ready = 0U;
    sensor->data.shaft_velocity = 0.0f;
    sensor->data.shaft_velocity_windowed = 0.0f;

    switch (sensor->type) {
    case SENSOR_AS5047P:
    {
        AS5047P_Handle_t *dev = (AS5047P_Handle_t *)sensor->dev;
        uint16_t raw = 0U;

        if (!dev->initialized) return 0U;
        if (AS5047P_ReadRawAngle(dev, &raw) != AS5047P_OK) return 0U;

        /* 记录初始机械角，只作为角度参考，不作为速度计算起点 */
        sensor->data.shaft_angle = dev->angle_rad;
        sensor->last_angle = dev->angle_rad;
        sensor->angle_track = dev->angle_rad;
        sensor->rotations = 0;

        sensor->initialized = 1U;
        return 1U;
    }

    default:
        return 0U;
    }
}


/* 更新 Sensor 数据。
 *
 * 输入：
 * dt：本次更新周期，单位 s。
 *
 * 更新内容：
 * 1. 读取 AS5047P 当前机械角；
 * 2. 计算单步角度增量；
 * 3. 更新连续角度 angle_track；
 * 4. 计算原始速度 shaft_velocity；
 * 5. 通过窗口差分计算低噪声速度 shaft_velocity_windowed。
 */
SENSOR_OPTIMIZE
void Sensor_Update(Sensor_t *sensor, float dt)
{
    if (!sensor || !sensor->initialized || !sensor->dev) return;
    if (dt <= 0.0f) return;
    if (sensor->type != SENSOR_AS5047P) return;

    AS5047P_Handle_t *dev = (AS5047P_Handle_t *)sensor->dev;

    /* AS5047P_ReadRawAngle() 会同步更新 dev->raw_angle 和 dev->angle_rad */
    if (AS5047P_ReadRawAngle(dev, &dev->raw_angle) != AS5047P_OK) {
        return;
    }

    float angle_now = dev->angle_rad;
    sensor->data.shaft_angle = angle_now;

    /* 处理跨 0/2π 边界后的单步角度差 */
    float d_rad = angle_diff(angle_now, sensor->last_angle);

    /* 连续机械角，不限制在 [0, 2π)，用于速度和跨圈统计 */
    sensor->angle_track += d_rad;

    /* 根据连续角度更新整圈计数 */
    float rotation_base = (float)sensor->rotations * TWO_PI;
    if (sensor->angle_track >= (rotation_base + TWO_PI)) {
        sensor->rotations++;
    } else if (sensor->angle_track <= (rotation_base - TWO_PI)) {
        sensor->rotations--;
    }

    /* 第一次 Update 只建立测速参考，不输出有效速度 */
    if (!sensor->velocity_ready) {
        sensor->data.shaft_velocity = 0.0f;
        sensor->last_angle = angle_now;
        sensor->velocity_ready = 1U;

        /* 用当前连续角填满缓冲区，避免窗口速度启动时出现随机值 */
        for (uint16_t i = 0U; i < SENSOR_VEL_WIN_MAX; i++) {
            sensor->velocity_windowed.vel_angle_track_buf[i] = sensor->angle_track;
        }

        sensor->velocity_windowed.vel_win_head = 0U;
        sensor->velocity_windowed.vel_win_count = 1U;
        return;
    }

    velocity_windowed_t *vw = &sensor->velocity_windowed;

    /* 环形缓冲区写入最新连续角度 */
    uint16_t head = (uint16_t)vw->vel_win_head + 1U;
    if (head >= SENSOR_VEL_WIN_MAX) {
        head = 0U;
    }

    vw->vel_win_head = (uint8_t)head;
    vw->vel_angle_track_buf[head] = sensor->angle_track;

    /* 固定周期时走快速倒数；非固定周期时用实际 dt */
    float inv_dt = (dt == SENSOR_UPDATE_PERIOD_S) ? SENSOR_INV_UPDATE_PERIOD_S : (1.0f / dt);

    /* 原始速度：单个采样周期角度差分，响应快但噪声较大 */
    float vel_raw = d_rad * inv_dt;
    sensor->data.shaft_velocity = vel_raw;

    /* 窗口未填满前，不输出窗口速度 */
    if (vw->vel_win_count < SENSOR_VEL_WIN_MAX) {
        vw->vel_win_count++;
        sensor->data.shaft_velocity_windowed = 0.0f;
        sensor->last_angle = angle_now;
        return;
    }

    /* 取窗口中最旧的连续角，计算 N 个周期内的平均速度 */
    uint16_t old_idx = head + 1U;
    if (old_idx >= SENSOR_VEL_WIN_MAX) {
        old_idx = 0U;
    }

    float theta_old = vw->vel_angle_track_buf[old_idx];

    sensor->data.shaft_velocity_windowed =
        (sensor->angle_track - theta_old) * (INV_VEL_WIN_NUMBER * inv_dt);

    sensor->last_angle = angle_now;
}


/* 获取当前机械角，范围通常为 [0, 2π) */
float Sensor_GetAngle(Sensor_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->data.shaft_angle;
}


/* 获取原始速度：单周期差分速度，响应快但噪声较大 */
float Sensor_GetVelocityRaw(Sensor_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->data.shaft_velocity;
}


/* 获取窗口速度：多周期平均速度，噪声更低但相位滞后更大 */
float Sensor_GetVelocityWindowed(Sensor_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->data.shaft_velocity_windowed;
}
