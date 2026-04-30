#ifndef _MYINT_H
#define _MYINT_H

#include "stdint.h"

/* 左路保留（停用）
extern float left_raw_velocity;
extern float left_filtered_velocity;
*/
extern float right_raw_velocity;
extern float right_filtered_velocity;
extern volatile uint8_t FOC_Task;


extern uint8_t Test_flag ;
/* 左路保留（停用）
extern float Left_Raw_Iq;
extern float Left_Filtered_Iq;
*/
extern float Right_Raw_Iq;
extern float Right_Filtered_Iq;



extern float SMO_Angle ;
extern float SMO_Velocity ;
extern float elecangle ;

/* CSV数据用于AI调参 */
typedef struct {
	uint32_t timestamp_ms;     // 时间戳
	float setpoint;            // 目标值（Left_Target）
	float input;               // 反馈值（Left_Filtered_Iq）
	float pwm;                 // PWM输出（Left_Current_FOC_PID.output）
	float error;               // 误差
	float p_term;              // 比例项
	float i_term;              // 积分项
	float d_term;              // 微分项
} pid_csv_data_t;

extern pid_csv_data_t pid_csv_data;

/* 高频采样缓存：10kHz ISR采样，主循环慢速导出 */
#ifndef PID_FAST_LOG_ENABLE
#define PID_FAST_LOG_ENABLE 1U
#endif

#define PID_FAST_LOG_CAPACITY 2048U

typedef struct {
	float setpoint;      /* 电流目标(A) */
	float filtered_iq;   /* 电流反馈(A) */
	float raw_iq;        /* 原始Iq(A) */
	float uq_final;      /* 电压输出命令 */
} pid_fast_log_sample_t;

extern volatile pid_fast_log_sample_t pid_fast_log[PID_FAST_LOG_CAPACITY];
extern volatile uint16_t pid_fast_log_count;
extern volatile uint8_t pid_fast_log_full;
extern volatile uint8_t pid_fast_log_capture_enable;

#endif