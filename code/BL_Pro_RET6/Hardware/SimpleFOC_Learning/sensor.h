#ifndef SENSOR_H
#define SENSOR_H


#include "main.h"
#include "AS5047P_RW.h"

#define SENSOR_UPDATE_PERIOD_S  0.0001f   // 例如 10 kHz


/* 传感器类型 */
typedef enum {
    SENSOR_NONE = 0,
    SENSOR_AS5047P,
    SENSOR_AS5600,     // 预留
} SensorType_t;




#define SENSOR_VEL_WIN_NUMBER 5U
#define SENSOR_VEL_WIN_MAX (SENSOR_VEL_WIN_NUMBER+1U)

typedef struct {
 uint8_t  vel_win_size;                 // N
    uint8_t  vel_win_head;                 // ring head
    uint8_t  vel_win_count;                // valid count
    float    vel_win_sum_dt;               // sum(dt) in window
    float    vel_angle_track_buf[SENSOR_VEL_WIN_MAX];
    float    vel_dt_buf[SENSOR_VEL_WIN_MAX];
} velocity_windowed_t;



/* 传感器公共输出数据 */
typedef struct {
    float shaft_angle;         // 单圈机械角 [0, 2pi)
    float shaft_angle_track;   // 连续机械角
    float shaft_velocity;      // 机械角速度 [rad/s]
    float shaft_velocity_windowed; // 窗口平均机械角速度 [rad/s]
    int rotations;             // 完整转数，正数表示正向，负数表示反向
} SensorData_t;




/* 统一传感器对象 */
typedef struct {
    uint8_t initialized;      // sensor模块是否初始化完成
    uint8_t velocity_ready;   // 是否已经建立速度估计参考
    SensorType_t type;
    void *dev;
    uint8_t test;
    SensorData_t data;
    velocity_windowed_t velocity_windowed;   // 窗口测速相关数据

    float init_angle;         // init阶段读到的初始机械角
    float last_angle;         // 上一次update用于速度计算的角度
    float last_angle_track;   // 上一次连续角
} Sensor_t;


/* 绑定具体传感器 */
void Sensor_LinkAS5047P( AS5047P_Handle_t *dev,Sensor_t *sensor);

/* 初始化 */
uint8_t Sensor_Init(Sensor_t *sensor);

/* 周期更新，dt单位：秒 */
void Sensor_Update(Sensor_t *sensor, float dt);

/* 获取公共数据 */
float Sensor_GetAngle(Sensor_t *sensor);
float Sensor_GetAngleTrack(Sensor_t *sensor);
float Sensor_GetVelocityRaw(Sensor_t *sensor);
float Sensor_GetVelocityWindowed(Sensor_t *sensor);

#endif