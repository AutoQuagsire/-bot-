//无刷电机基础参数配置
#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#include <stdint.h>
#include "driver.h"
#include "current_sense.h"
#include "sensor.h"

#ifndef CURRENT_LOOP_USE_FEEDFORWARD
#define CURRENT_LOOP_USE_FEEDFORWARD 1
#endif

#ifndef CURRENT_LOOP_I_SEP_RATIO
#define CURRENT_LOOP_I_SEP_RATIO 0.15f
#endif

#ifndef CURRENT_LOOP_PURE_PI_I_SEP_RATIO
#define CURRENT_LOOP_PURE_PI_I_SEP_RATIO 0.75f
#endif

/*
 * CurrentLoop_FFPI_V1
 *
 * The current loop uses a feedforward PI structure:
 *   Uq = PI(Iq_ref - Iq_meas) + Iq_ref * R_phase * ff_coef
 *
 * ff_coef and integral_limit are scheduled by |Iq_ref|.
 * Feedforward provides the basic voltage estimate, while PI only
 * corrects dynamic error and residual steady-state error.
 *
 * Pure PI is kept only for comparison/debugging.
 */
typedef struct {
    float iq_abs;
    float ff_coef;
    float integral_limit;
} CurrentLoopSchedulePoint_t;

typedef struct {
    float target_iq;
    float filtered_iq;
    float raw_iq;
    float error;
    float pi_out;
    float ff_term;
    float uq_final;
    float ff_coef;
    float integral_limit;
    float pid_integral;
} CurrentLoopDebugSnapshot_t;



//==================== 电机物理参数 ====================
// 这部分存放“电机本体参数”
// 一般由电机规格、测量结果或辨识得到，运行过程中通常不会频繁修改
typedef struct {
    float pole_pairs;         // 极对数
    float phase_resistance;   // 相电阻（单位：欧姆）
    float kv;                 // 电机 KV 值（通常单位：rpm/V）
    float Lq;                 // q轴电感（单位：H）
    float Ld;                 // d轴电感（单位：H）
} MotorParam_t;


//==================== 电机控制模式 ====================
// 描述当前电机按什么目标量进行控制
typedef enum {
    motor_control_torque = 0,        // 力矩控制（底层可对应电流控制或电压控制）
    motor_control_velocity,          // 闭环速度控制
    motor_control_angle,             // 闭环位置/角度控制
    motor_control_openloop_velocity, // 开环速度控制
    motor_control_openloop_angle     // 开环角度控制
} MotorControlMode_t;


//==================== 电机控制配置 ====================
// 这部分存放“控制策略配置”
// 属于用户设定，不是电机物理属性
typedef struct {
    MotorControlMode_t control_mode; // 当前控制模式
    float voltage_limit;             // 允许输出的最大电压限制
    float voltage_sensor_align;      // 传感器对齐/零电角校准时使用的电压
} MotorConfig_t;


//==================== 三相电压输出 ====================
// 存放当前三相目标输出电压
typedef struct
{
    float Ua;   // A相电压
    float Ub;   // B相电压
    float Uc;   // C相电压
} PhaseVoltage_t;


//==================== 电机状态枚举 ====================
// 描述当前电机处于什么阶段
typedef enum {
    motor_init_failed = 0, // 初始化失败
    motor_initializing,    // 正在初始化
    motor_uncalibrated,    // 已初始化，但还未完成校准
    motor_ready,           // 已完成初始化/校准，可正常运行
    motor_error            // 运行过程中出现错误
} MotorStatus_t;


//==================== 传感器方向枚举 ====================
// 描述传感器读数方向与电机定义正方向之间的关系
typedef enum {
    sensor_direction_unknown = 0, // 方向未知，尚未校准
    sensor_direction_cw,          // 顺时针为正方向
    sensor_direction_ccw          // 逆时针为正方向
} SensorDirection_t;


//==================== 电机运行状态 ====================
// 这部分存放“运行中会变化的状态量”
typedef struct {
    uint8_t enabled;                    // 电机是否已使能：1=已使能，0=未使能
    uint8_t has_sensor;                 // 是否带传感器：1=有，0=无
    MotorStatus_t motor_status;         // 当前电机状态
    SensorDirection_t sensor_direction; // 传感器方向
    PhaseVoltage_t phase_voltage;       // 当前三相输出电压状态
} MotorState_t;




//==================== 电机总对象 ====================
// 这是电机控制对象的总结构体
// 包含：参数、配置、状态，以及与外部模块的连接关系
typedef struct {
    MotorParam_t param;             // 电机物理参数
    MotorConfig_t config;           // 控制配置参数
    MotorState_t state;             // 运行状态信息

    Driver_t *driver;               // 指向驱动模块
    CurrentSense_t *current_sense;  // 指向电流采样模块
    Sensor_t *sensor;               // 指向磁传感器模块

    float electrical_angle;         // 当前电机电角度（单位：弧度）
    float zero_electrical_angle;    // 零位电角度（单位：弧度）

} Motor_t;


void linkDriver(Driver_t *driver, Motor_t *motor);
void linkSensor(Sensor_t *sensor, Motor_t *motor);
void linkCurrentSense(CurrentSense_t *Current_Sense, Motor_t *motor);

uint8_t FOCMotor_init(Motor_t *FOC_Motor);
void MotorParam_Init(Motor_t *motor, float pole_pairs, float phase_resistance, 
                    float kv, float Ld, float Lq);
void FOCMotor_disable(Motor_t *FOC_Motor);
void FOCMotor_enable(Motor_t *FOC_Motor);

float Motor_GetMechanicalAngle(Motor_t *motor);
float Motor_GetElectricalAngle(Motor_t *motor);
uint8_t Motor_UpdateSensor(Motor_t *motor, float dt);
void Motor_SetPhaseVoltageQ(Motor_t *motor, float uq, float elec_angle);
void Motor_SetPhaseVoltageQBySinCos(Motor_t *motor, float uq, float sin_el, float cos_el);
void CurrentLoop_GetScheduledParams(float target_iq,
                                    float *ff_coef,
                                    float *integral_limit);

uint8_t Motor_CalibrateZeroElectricalAngle(Motor_t *motor,
                                           float align_voltage,
                                           float align_angle,
                                           uint16_t settle_ms);

#endif
