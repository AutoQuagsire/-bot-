#ifndef APP_FOC_H
#define APP_FOC_H

#include <stdint.h>

#define APP_FOC_STATUS_FLAG_SPEED_FAULT_L        (1U << 0)
#define APP_FOC_STATUS_FLAG_SPEED_FAULT_R        (1U << 1)
#define APP_FOC_STATUS_FLAG_STACK_READY          (1U << 8)
#define APP_FOC_STATUS_FLAG_CONTROL_IT_ENABLED   (1U << 9)
#define APP_FOC_STATUS_FLAG_BUS_VALID            (1U << 10)
#define APP_FOC_STATUS_FLAG_CURRENT_LOOP_ACTIVE  (1U << 11)
#define APP_FOC_STATUS_FLAG_SPEED_LOOP_ENABLED   (1U << 12)
#define APP_FOC_STATUS_FLAG_CURRENT_LOOP_ENABLED (1U << 13)
#define APP_FOC_STATUS_FLAG_POWER_STAGE_OFF      (1U << 14)
#define APP_FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON  (1U << 15)

/* FastLog：高速采样缓存（共享给 debug_link.c） */
#define APP_FASTLOG_SIZE (512U)

typedef struct {
    float target_iq;
    float iq_ref;
    float filtered_iq;
    float raw_iq;
    float pi_out;
    float ff_term;
    float uq_final;
    float ff_coef;
    float integral_limit;
    float pid_integral;
    float shaft_angle;
    float shaft_velocity;
    float electrical_angle;
    float bus_raw_adc;
    float bus_pin_voltage;
    float bus_voltage;
} FastLogSample_t;

typedef struct
{
    float wheel_vel_left_radps;
    float wheel_vel_right_radps;
    float filtered_iq_left_a;
    float filtered_iq_right_a;
    float uq_left_v;
    float uq_right_v;
    float bus_voltage_v;
    uint16_t status_flags;
} App_FOCTelemetry_t;

uint8_t App_FOCStack_Init(void);
uint8_t App_StartupCalibrate(void);
void App_Loop(void);
uint8_t App_FOC_BusTelemetryInit(void);
void App_FOC_BusTelemetryService(void);
void App_CurrentSenseSignTest(void);
void App_SensorDirectionTest(void);
void App_FOCControlIT_Enable(void);
void App_LoopForIT(void);
void DebuginWhile(void);
void App_ArmFastLog(void);
uint8_t App_TryArmFastLog(void);
void App_StopFastLog(void);
#define FASTLOG_SOURCE_LEFT  0U
#define FASTLOG_SOURCE_RIGHT 1U
uint8_t App_SetFastLogSource(uint8_t source);
void App_GetFastLogStatus(uint16_t *count,
                          uint8_t *armed,
                          uint8_t *done,
                          uint32_t *capture_id,
                          uint8_t *blocked,
                          uint8_t *source);
uint8_t App_CopyFastLogChunk(uint16_t start_idx,
                             uint8_t max_samples,
                             FastLogSample_t *out);
void App_ResetSpeedPIDs(void);
void App_ResetCurrentPIDs(void);
void App_FOC_SetIqTarget(float left_iq, float right_iq);
float App_FOC_GetAverageWheelSpeedRadps(void);
void App_FOC_GetTelemetry(App_FOCTelemetry_t *telemetry);
uint8_t App_FOC_SetPowerStageEnabled(uint8_t enable);
uint8_t App_FOC_IsPowerStageEnabled(void);
void App_CurrentPID_SetSame(float kp, float ki, float kd, float integral_limit);
void App_CurrentPID_GetSame(float *kp, float *ki, float *kd, float *integral_limit);

extern volatile uint8_t g_current_pid_mode; /* 0=CurrentLoop_FFPI_V1, 1=Pure PI compare */

/* DebugLink 用的状态变量 */
extern float vel_windowed_f1;
extern float vel_windowed_f2;
extern float Left_FilteredIq;
extern float Right_FilteredIq;
extern float uq_cmd1;
extern float uq_cmd2;
extern float g_speed_fault1;
extern float g_speed_fault2;

float App_FOC_GetBusVoltageFiltered(void);

#endif
