#ifndef __FOC_H
#define __FOC_H

#include <stdint.h>

// ========== FOC控制频率配置（统一管理） ==========
#define FOC_FREQUENCY       10000.0f        // FOC控制频率 (Hz)
#define FOC_PERIOD_S        0.0001f         // FOC控制周期 (秒) = 1/FOC_FREQUENCY
#define FOC_PERIOD_US       100             // FOC控制周期 (微秒)
#define PWM_TIMER_PERIOD    htim2.Init.Period        
// ================================================

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define MAX_PWM             htim1.Init.Period            // PWM最大占空比计数值
#define DIR                 (1.0f)               // 电机旋转方向
#define PolePair            14               // 极对数
#define PI                  3.14159265359f  // 圆周率

#define RIGHT_DIR           -1               // 右路电机方向（与编码器相位一致时为1，反向改为-1）
#define RIGHT_PolePair      14              // 右路极对数
#define RIGHT_zero_elec_angle 0.0f          // 右路零电角偏移 (rad)
#define V_SUPPLY             19.0f           // 供电电压 (V)
#define Uq_max              (V_SUPPLY * 0.577) // 最大 q 轴电压（SVPWM 理论最大值）
#define _SQRT3 1.73205080757f



#define FOC_SPEED_TARGET 2.0f



// ============ 电流采样相关变量 ============
#define _ADC_CONV 0.00080586f  // ADC转电压系数: 3.3V / 4096
#define _1_SQRT3  0.57735026919f
#define _2_SQRT3  1.15470053838f



#define _shunt_resistor 0.02f  // 电流采样分流电阻 (Ω)
#define amp_gain        20     // INA240 运放放大倍数
#define RIGHT_ADC_SWAP_AB 0    // 右路ADC通道A/B互换：0=不换，1=互换
#define RIGHT_IA_SIGN    1.0f  // 右路Ia符号：接反时改为-1.0f
#define RIGHT_IB_SIGN    1.0f  // 右路Ib符号：接反时改为-1.0f

// ========== 过流保护配置 ==========
#define OVERCURRENT_THRESHOLD    2.8f          // 过流阈值 (A)
#define OVERCURRENT_DURATION_MS  100            // 过流持续时间阈值 (ms) —— 持续超过此时间则触发保护

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

    /* debug/runtime values */
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
    volatile uint16_t *adc_buf;  /* 指向 DMA buffer，必须保留 volatile */
    float offset_ia;
    float offset_ib;
    float gain_a;
    float gain_b;
} CurrentConfig_t;




// 电流检测结构体
typedef struct {
    float I_a;
    float I_b;
    float U_a;  // 可选:保存电压值用于调试
    float U_b;
} CurrentDetect_t;



extern volatile uint16_t adc1_buf[2];
extern volatile uint16_t adc2_buf[2];
extern FOC_Motor_t R_Motor;
extern FOC_Motor_t L_Motor;
extern CurrentConfig_t R_Motor_CurrentCfg;
extern CurrentConfig_t L_Motor_CurrentCfg;


extern      float     L_uq_final;
extern   float   left_elec_angle;

void Get_SinCos(float angle_el, float *sint, float *cost);
void FOC_SetSVPWM(FOC_Motor_t *motor, float Uq, float sint, float cost, float Ud);
void FOC_SetSVPWM_ByAngle(float Uq, float angle_el);
void FOC_ADC_Start(void);
void FOC_ADC_Stop(void);
void CurrentSense_Init(void);
void Init_CunrrentCfg(CurrentConfig_t *cfg, float offsetA, float offsetB, float gain, volatile uint16_t *adc_buf);
CurrentDetect_t GetPhaseCurrent(CurrentConfig_t *cfg);
float cal_Iq_Id(float current_a, float current_b, float sint, float cost);

// ========== 过流保护函数 ==========
void OvercurrentProtection_Check(void);     // 检测过流并在需要时关闭电机
uint8_t OvercurrentProtection_GetStatus(void);  // 获取过流保护状态（0=正常，1=过流保护激活）
void OvercurrentProtection_Reset(void);     // 重置保护状态并重新启用电机驱动

#endif