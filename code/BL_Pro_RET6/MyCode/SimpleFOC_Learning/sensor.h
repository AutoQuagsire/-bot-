#ifndef SENSOR_H
#define SENSOR_H

#include "main.h"
#include "AS5047P_RW.h"

#define SENSOR_UPDATE_PERIOD_S  0.0001f

typedef enum {
    SENSOR_NONE = 0,
    SENSOR_AS5047P,
    SENSOR_AS5600,
} SensorType_t;

#define SENSOR_VEL_WIN_NUMBER 10U
#define SENSOR_VEL_WIN_MAX (SENSOR_VEL_WIN_NUMBER + 1U)

typedef struct {
    uint8_t vel_win_head;
    uint8_t vel_win_count;
    float vel_angle_track_buf[SENSOR_VEL_WIN_MAX];
} velocity_windowed_t;

typedef struct {
    float shaft_angle;
    float shaft_velocity;
    float shaft_velocity_windowed;
} SensorData_t;

typedef struct {
    uint8_t initialized;
    uint8_t velocity_ready;
    SensorType_t type;
    void *dev;

    SensorData_t data;
    velocity_windowed_t velocity_windowed;

    float last_angle;
    float angle_track;
    int rotations;
} Sensor_t;

void Sensor_LinkAS5047P(AS5047P_Handle_t *dev, Sensor_t *sensor);
uint8_t Sensor_Init(Sensor_t *sensor);
void Sensor_Update(Sensor_t *sensor, float dt);

float Sensor_GetAngle(Sensor_t *sensor);
float Sensor_GetVelocityRaw(Sensor_t *sensor);
float Sensor_GetVelocityWindowed(Sensor_t *sensor);

#endif
