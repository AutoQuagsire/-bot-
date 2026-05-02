#include "sys.h"
#include "FOC.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>
#include <stdint.h>







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


