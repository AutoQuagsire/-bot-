#ifndef __FOC_H
#define __FOC_H

#include <stdint.h>
#include "tim.h"

/* Legacy FOC module local macros (still used only by legacy FOC.c path). */
#define PWM_TIMER_PERIOD htim2.Init.Period
#define MAX_PWM htim1.Init.Period




#define OVERCURRENT_THRESHOLD 2.8f
#define OVERCURRENT_DURATION_MS 100

typedef enum {
    MOTOR_SIDE_LEFT = 0,
    MOTOR_SIDE_RIGHT = 1,
} MotorSide_t;

typedef struct {
    MotorSide_t side;
    TIM_HandleTypeDef *htim;
    uint32_t chA;
    uint32_t chB;
    uint32_t chC;
    float Ua;
    float Ub;
    float Uc;
    float Ualpha;
    float Ubeta;
} FOC_Motor_t;

typedef struct {
    MotorSide_t side;
    int8_t A_SIGN;
    int8_t B_SIGN;
    volatile uint16_t *adc_buf;
    float offset_ia;
    float offset_ib;
    float gain_a;
    float gain_b;
} CurrentConfig_t;

typedef struct {
    float I_a;
    float I_b;
    float U_a;
    float U_b;
} CurrentDetect_t;

extern volatile uint16_t adc1_buf[2];
extern volatile uint16_t adc2_buf[2];
extern FOC_Motor_t R_Motor;
extern FOC_Motor_t L_Motor;
extern CurrentConfig_t R_Motor_CurrentCfg;
extern CurrentConfig_t L_Motor_CurrentCfg;

extern float L_uq_final;
extern float left_elec_angle;

void FOC_SetSVPWM(FOC_Motor_t *motor, float Uq, float sint, float cost, float Ud);
void FOC_SetSVPWM_ByAngle(float Uq, float angle_el);
void FOC_ADC_Start(void);
void FOC_ADC_Stop(void);
void CurrentSenser_Init(void);
void Init_CunrrentCfg(CurrentConfig_t *cfg, float offsetA, float offsetB, float gain, volatile uint16_t *adc_buf);
CurrentDetect_t GetPhaseCurrent(CurrentConfig_t *cfg);
float cal_Iq_Id(float current_a, float current_b, float sint, float cost);
void OvercurrentProtection_Check(void);
uint8_t OvercurrentProtection_GetStatus(void);
void OvercurrentProtection_Reset(void);

#endif /* __FOC_H */
