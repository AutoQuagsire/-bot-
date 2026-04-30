#ifndef APP_FOC_H
#define APP_FOC_H

#include <stdint.h>

uint8_t App_FOCStack_Init(void);
uint8_t App_StartupCalibrate(void);
void App_Loop(void);
void App_CurrentSenseSignTest(void);
void App_SensorDirectionTest(void);
void App_FOCControlIT_Enable(void);
void App_LoopForIT(void);
void DebuginWhile(void);
void App_ArmFastLog(void);
uint8_t App_TryArmFastLog(void);
void App_StopFastLog(void);
void App_GetFastLogStatus(uint16_t *count,
                          uint8_t *armed,
                          uint8_t *done,
                          uint32_t *capture_id,
                          uint8_t *blocked);
void App_ResetSpeedPIDs(void);
void App_ResetCurrentPIDs(void);
void App_CurrentPID_SetSame(float kp, float ki, float kd, float integral_limit);
void App_CurrentPID_GetSame(float *kp, float *ki, float *kd, float *integral_limit);

extern volatile uint8_t g_current_pid_mode; /* 0=CurrentLoop_FFPI_V1, 1=Pure PI compare */

#endif
