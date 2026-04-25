#include "sys.h"
#include "FOC.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>
#include <stdint.h>

/* ADC buffer for ADC1 (DMA writes here) */
volatile uint16_t adc1_buf[2];
volatile uint16_t adc2_buf[2];
static volatile uint8_t adc1_ready = 0;
/* Precomputed scale: compare = voltage * pwm_scale  (pwm_scale = MAX_PWM / V_SUPPLY) */
static float pwm_scale = 0.0f;

// * 自动计算 sin(angle_el) 和 cos(angle_el)
__attribute__((optimize("O2,fast-math")))
void Get_SinCos(float angle_el, float *sint, float *cost)
{
    if (!sint || !cost) {
        return;
    }
#if defined(__GNUC__)
    __builtin_sincosf(angle_el, sint, cost);
#else
    *sint = sinf(angle_el);
    *cost = cosf(angle_el);
#endif
}



FOC_Motor_t R_Motor = {
    .htim = &htim4,
    .chA = TIM_CHANNEL_4,
    .chB = TIM_CHANNEL_3,
    .chC = TIM_CHANNEL_2,
    .Ua = 0.0f,
    .Ub = 0.0f,
    .Uc = 0.0f,
    .Ualpha = 0.0f,
    .Ubeta = 0.0f
};


FOC_Motor_t L_Motor = {
    .htim = &htim1,
    .chA = TIM_CHANNEL_1,
    .chB = TIM_CHANNEL_3,
    .chC = TIM_CHANNEL_4,
    .Ua = 0.0f,
    .Ub = 0.0f,
    .Uc = 0.0f,
    .Ualpha = 0.0f,
    .Ubeta = 0.0f
};


static void Motor_WriteCompare(const FOC_Motor_t *m, uint32_t phA, uint32_t phB, uint32_t phC)
{
    if (m == NULL || m->htim == NULL) return;

    __HAL_TIM_SET_COMPARE(m->htim, m->chA, phA);
    __HAL_TIM_SET_COMPARE(m->htim, m->chB, phB);
    __HAL_TIM_SET_COMPARE(m->htim, m->chC, phC);
}




__attribute__((optimize("O2,fast-math")))
void FOC_SetSVPWM(FOC_Motor_t *motor, float Uq, float sint, float cost, float Ud)
{
    (void)Ud;  /* 调用方恒传 0；需要磁场弱化时恢复通用形式 */
    if (motor == NULL) return;
    if (pwm_scale <= 0.0f) return;

    /* Park(Ud=0) + Clarke 合并
     * 通用形式需 4 VMUL + 2 VADD；Ud=0 退化为 2 VMUL，节省约 4 条 FPU 指令：
     *   Ualpha = -Uq*sint,  Ubeta = Uq*cost
     *   Ua = -Uq*sint
     *   Ub = (-Ualpha + √3·Ubeta)·0.5 = Uq·(sint + √3·cost)·0.5
     *   Uc = (-Ualpha - √3·Ubeta)·0.5 = Uq·(sint - √3·cost)·0.5  */
    float Ualpha = -Uq * sint;
    float Ubeta  =  Uq * cost;
    float t      = _SQRT3 * Ubeta;
    float Ua     = Ualpha;
    float Ub     = (-Ualpha + t) * 0.5f;
    float Uc     = (-Ualpha - t) * 0.5f;

    /* 调试字段：仅写一次，不在计算链中（如不需要可删除以节省 5 次 VSTR） */
    motor->Ualpha = Ualpha;
    motor->Ubeta  = Ubeta;
    motor->Ua = Ua;
    motor->Ub = Ub;
    motor->Uc = Uc;

    /* SVPWM zero-sequence injection */
    float Umax  = fmaxf(Ua, fmaxf(Ub, Uc));
    float Umin  = fminf(Ua, fminf(Ub, Uc));
    float Uzero = (V_SUPPLY * 0.5f) - (Umax + Umin) * 0.5f;

    Ua = fmaxf(0.0f, fminf(V_SUPPLY, Ua + Uzero));
    Ub = fmaxf(0.0f, fminf(V_SUPPLY, Ub + Uzero));
    Uc = fmaxf(0.0f, fminf(V_SUPPLY, Uc + Uzero));

    /* 直接写 CCR：TIM_CHANNEL_N=(N-1)*4，与 CCR1~4 偏移一一对应，仅 CH1~4 有效 */
    volatile uint32_t *ccr = &motor->htim->Instance->CCR1;
    float s = pwm_scale;
    *(ccr + (motor->chA >> 2)) = (uint32_t)(Ua * s + 0.5f);
    *(ccr + (motor->chB >> 2)) = (uint32_t)(Ub * s + 0.5f);
    *(ccr + (motor->chC >> 2)) = (uint32_t)(Uc * s + 0.5f);
}








/* Start ADC sampling synchronized to TIM2 CC2 and enable DMA transfer */
void FOC_ADC_Start(void)
{
    /* Ensure TIM2 CH2 comparison running (generates CC2 events) */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);    
    /* Start ADC1/ADC2 in DMA mode for dual-motor current sampling */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, 2);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, 2);
}

/* Stop ADC sampling and TIM2 compare */
void FOC_ADC_Stop(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);       
}




CurrentConfig_t L_Motor_CurrentCfg = {0};
CurrentConfig_t R_Motor_CurrentCfg = {0};


void Init_CunrrentCfg(CurrentConfig_t *CurrentCfg, float OFFSETA, float OFFSETB, float GAIN, volatile uint16_t *ADC_BUF)
{
    CurrentCfg->A_SIGN = 1;
    CurrentCfg->B_SIGN = 1;      

    CurrentCfg->adc_buf = &ADC_BUF[0];

    CurrentCfg->offset_ia = OFFSETA;
    CurrentCfg->offset_ib = OFFSETB;
    CurrentCfg->gain_a = GAIN*CurrentCfg->A_SIGN;
    CurrentCfg->gain_b = GAIN*CurrentCfg->B_SIGN;
}

/**
 * @brief  Calibrate ADC offset and calculate current conversion gain
 *         (ADC DMA must be already started by FOC_ADC_Start())
 */
void CurrentSenser_Init(void)
{
    USB_Debug_Printf("=== START CurrentSense_Init ===\r\n");
    /* Ensure ADC DMA / TIM2 trigger are stopped before calibration */
    //FOC_ADC_Stop();
    // ============ 电流采样相关变量 ============
    float L_offset_ia = 0.0f, L_offset_ib = 0.0f;   // 左路零点漂移校准值
    float R_offset_ia = 0.0f, R_offset_ib = 0.0f;   // 右路零点漂移校准值
    float vlots_to_amps = 0.0f;                     // 电压到电流转换系数
    float gain = 0.0f;      

    HAL_Delay(50);
    /* Restart ADC DMA and TIM2 compare after calibration */
    USB_Debug_Printf("Restarting ADC DMA and TIM2 after calibration...\r\n");
    FOC_ADC_Start();
    HAL_Delay(50);
    /* Wait for ADC DMA to stabilize */
    USB_Debug_Printf("Waiting 100ms for ADC stabilization...\r\n");
    HAL_Delay(100);
       
    /* Collect 1000 samples for zero-point drift offset calibration */
    USB_Debug_Printf("Starting offset calibration (1000 samples)...\r\n");
    for(int i = 0; i < 1000; i++) 
    {
        HAL_Delay(1);  /* Wait for fresh ADC DMA sample */
        L_offset_ia += adc1_buf[0] * _ADC_CONV;
        L_offset_ib += adc1_buf[1] * _ADC_CONV;
        R_offset_ia += adc2_buf[0] * _ADC_CONV;
        R_offset_ib += adc2_buf[1] * _ADC_CONV;
    }
    L_offset_ia /= 1000.0f;
    L_offset_ib /= 1000.0f;
    R_offset_ia /= 1000.0f;
    R_offset_ib /= 1000.0f;
    USB_Debug_Printf("L offset: ia=%.4fV, ib=%.4fV\r\n", L_offset_ia, L_offset_ib);
    USB_Debug_Printf("R offset: ia=%.4fV, ib=%.4fV\r\n", R_offset_ia, R_offset_ib);
    
    /* Calculate voltage-to-current conversion factor and per-channel gain */
    vlots_to_amps = 1.0f / _shunt_resistor / amp_gain;
    gain = vlots_to_amps;
    USB_Debug_Printf("v2i_factor=%.2f, gain=%.2f\r\n", vlots_to_amps, gain);
 
   
    Init_CunrrentCfg(&L_Motor_CurrentCfg, L_offset_ia, L_offset_ib, gain,adc1_buf);
    Init_CunrrentCfg(&R_Motor_CurrentCfg, R_offset_ia, R_offset_ib, gain,adc2_buf);
    R_Motor_CurrentCfg.gain_a *= -1;
    R_Motor_CurrentCfg.gain_b *= -1;  /* 右路A相反向，调整增益符号 */ 


    L_Motor_CurrentCfg.gain_a *= 1;
    L_Motor_CurrentCfg.gain_b *= 1;  /* 左路A相反向，调整增益符号 */ 

    USB_Debug_Printf("=== END CurrentSense_Init ===\r\n");

    /* precompute PWM scale coefficient (MAX_PWM / V_SUPPLY) */
    pwm_scale = (float)MAX_PWM / V_SUPPLY;
}





CurrentDetect_t GetPhaseCurrent(CurrentConfig_t *CurrentCfg)
{
    CurrentDetect_t current = {0};
    float tran_vol_a = 0.0f, tran_vol_b = 0.0f;
    tran_vol_a = (float)CurrentCfg->adc_buf[0] * _ADC_CONV;
    tran_vol_b = (float)CurrentCfg->adc_buf[1] * _ADC_CONV;
    /* Voltage → current: I = (V - offset) × gain */
    current.I_a = (tran_vol_a - CurrentCfg->offset_ia) * CurrentCfg->gain_a;
    current.I_b = (tran_vol_b - CurrentCfg->offset_ib) * CurrentCfg->gain_b;
    /* Store voltage for debugging */
    current.U_a = (tran_vol_a - CurrentCfg->offset_ia);
    current.U_b = (tran_vol_b - CurrentCfg->offset_ib);
    return current;
}



// 计算电流空间矢量的 q 轴分量（Park变换）
float cal_Iq_Id(float current_a, float current_b, float sint, float cost) 
{
    // ABC → αβ (Clarke变换)
    float I_alpha = current_a;
    float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;
    // αβ → dq (Park变换)，使用预计算的 sin/cos
    float I_q = I_beta * cost - I_alpha * sint;
    return I_q;
}


// ========== 过流保护实现 ==========

/* 过流保护状态结构体 */
static struct {
    uint32_t overcurrent_start_time;    // 首次过流的时间戳 (ms)
    uint8_t protection_active;          // 保护激活标志 (0=未激活, 1=已激活)
} OvercurrentProtection = {0};

/**
 * @brief  过流保护检查函数
 *         检测电流是否超过阈值，如果持续超过设定时长则关闭电机驱动
 *         基于系统时钟（HAL_GetTick）,可在main的while循环中调用
 */
void OvercurrentProtection_Check(void)
{
    /* 引用INT.c中的滤波后Iq电流 */
    extern float Right_Filtered_Iq;
    
    /* 取绝对值作为电流幅值 */
    float current_magnitude = (Right_Filtered_Iq < 0) ? -Right_Filtered_Iq : Right_Filtered_Iq;
    uint32_t now = HAL_GetTick();
    
    /* 检测是否超过阈值 */
    if (current_magnitude > OVERCURRENT_THRESHOLD)
    {
        /* 首次过流，记录时间戳 */
        if (OvercurrentProtection.overcurrent_start_time == 0)
        {
            OvercurrentProtection.overcurrent_start_time = now;
        }
        
        /* 检查持续过流时间是否达到阈值 */
        if ((now - OvercurrentProtection.overcurrent_start_time) >= OVERCURRENT_DURATION_MS && 
            !OvercurrentProtection.protection_active)
        {
            /* 关闭电机驱动 */
            HAL_GPIO_WritePin(Motor_EN_GPIO_Port, Motor_EN_Pin, GPIO_PIN_SET);
            OvercurrentProtection.protection_active = 1;
            
            /* 调试输出：过流保护激活 */
            USB_Debug_Printf(">>> OVERCURRENT PROTECTION ACTIVATED! I_Iq=%.3fA (threshold=%.3fA, duration=%.0fms)\r\n", 
                           current_magnitude, OVERCURRENT_THRESHOLD, 
                           (float)(now - OvercurrentProtection.overcurrent_start_time));
        }
    }
    else
    {
        /* 电流恢复正常，重置时间戳 */
        if (OvercurrentProtection.overcurrent_start_time != 0)
        {
            OvercurrentProtection.overcurrent_start_time = 0;
        }
        
        /* 如果保护曾激活且电流恢复，需要手动重启电机驱动（由上层应用决定） */
        /* 目前保留保护激活状态直到系统复位或手动清除 */
    }
}

/**
 * @brief  获取过流保护状态
 * @return 0: 保护未激活（正常）; 1: 保护已激活（过流发生）
 */
uint8_t OvercurrentProtection_GetStatus(void)
{
    return OvercurrentProtection.protection_active;
}

/* 可选：重置过流保护状态（需要重新启用电机驱动时调用） */
void OvercurrentProtection_Reset(void)
{
    OvercurrentProtection.overcurrent_start_time = 0;
    OvercurrentProtection.protection_active = 0;
    /* 手动重启电机驱动 */
    HAL_GPIO_WritePin(Motor_EN_GPIO_Port, Motor_EN_Pin, GPIO_PIN_RESET);
    USB_Debug_Printf(">>> Overcurrent Protection RESET. Motor EN applied.\r\n");
}


