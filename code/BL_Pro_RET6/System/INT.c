#include "as5047p.h"
#include "main.h"
#include "stm32g4xx_hal_gpio.h"
#include "sys.h"
#include "tim.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim3;
extern AS5047P_Handle encoder_left;
extern AS5047P_Handle encoder_right;




float left_elec_angle = 0;
float left_raw_velocity = 0;
float left_raw_velocity_test = 0;

float zero_elec_angle = 0.0f;



float left_filtered_velocity = 0;
float left_filtered_velocity_test = 0;
float Left_Target = 5.0f;

PID_t Left_Velocity_FOC_PID;
LowPassFilter_t left_velocity_filter;
LowPassFilter_t test_filter;



PID_t Left_Current_FOC_PID;
LowPassFilter_t left_current_filter;

float Left_Raw_Iq = 0;
float Left_Filtered_Iq = 0;
CurrentDetect_t Left_Current_Detect;

/* 右路变量（当前启用） */
float right_elec_angle = 0;
float right_raw_velocity = 0;
float right_filtered_velocity = 0;
float Right_Target = -0.6f;

PID_t Right_Velocity_FOC_PID;
LowPassFilter_t right_velocity_filter;

PID_t Right_Current_FOC_PID;
LowPassFilter_t right_current_filter;

float Right_Raw_Iq = 0;
float Right_Filtered_Iq = 0;
CurrentDetect_t Right_Current_Detect;

/* 诊断用缓存（中interrupt→主循环） */
volatile uint8_t diag_ready = 0;
volatile struct {
	float I_a, I_b, sint, cost, Raw_Iq_val;
	float raw_vel, filt_vel, uq_out;
	float pid_current_out, uq_final;
} diag_cache;

/* CSV数据用缓存（用于AI调参） */
pid_csv_data_t pid_csv_data = {0};

/* 高频采样缓存：ISR内只写入，不做打印 */
volatile pid_fast_log_sample_t pid_fast_log[PID_FAST_LOG_CAPACITY] = {0};
volatile uint16_t pid_fast_log_count = 0U;
volatile uint8_t pid_fast_log_full = 0U;
#if PID_FAST_LOG_ENABLE
volatile uint8_t pid_fast_log_capture_enable = 1U;
#else
volatile uint8_t pid_fast_log_capture_enable = 0U;
#endif

float R_uq_final = 0;
float L_uq_final = 0;

#define PID_TUNE_SETPOINT   (Left_Target)
#define PID_TUNE_INPUT      (left_filtered_velocity)
#define PID_TUNE_PID        (Left_Velocity_FOC_PID)

/* 开环旋转磁场分叉测试：1=开启(绕开速度/电流闭环)，0=关闭 */
#define OPEN_LOOP_SPIN_TEST_ENABLE  0U
#define OPEN_LOOP_TEST_UQ           8.0f  /* 固定小Uq */
#define OPEN_LOOP_TEST_W_ELEC       6.0f  /* 电角速度(rad/s)，尽量慢速 */

/* 闭环排查开关：
 * 0 = 电流环目标直接用 Left_Target（仅电流环）
 * 1 = 电流环目标使用速度环输出（速度级联）
 */
#define LEFT_CURRENT_TARGET_USE_SPEED_OUTPUT  1U
/* 闭环排查开关：Iq反馈符号，怀疑符号链路问题时可在 +1/-1 间切换 */
#define LEFT_IQ_FEEDBACK_SIGN                (1.0f)


AS5047P_Status left_sta_latched = AS5047P_ERROR;
AS5047P_Status right_sta_latched = AS5047P_ERROR;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t LoopFlag = 0;

	if(htim == &htim5)//10Khz
	{

		static float L_st=0,L_ct=0;
		static float R_st=0,R_ct=0;


		AS5047P_Status left_sta = left_sta_latched;
		AS5047P_Status right_sta = right_sta_latched;

		/* 编码器读取降频到 5kHz，其余周期复用上次角度与状态 */
		// encoder_read_div++;
		//  if (encoder_read_div >= 2U)
		//  {
			GPIO_SetBits(LED_GPIO_Port,LED_Pin);

			left_sta_latched = AS5047P_GetAngle(&encoder_left);
						GPIO_ResetBits(LED_GPIO_Port,LED_Pin);

			right_sta_latched = AS5047P_GetAngle(&encoder_right);

			left_sta = left_sta_latched;
			right_sta = right_sta_latched;
		//}

		left_elec_angle = normalizeAngle((float)(DIR * PolePair) * encoder_left.angle_rad - zero_elec_angle);
		right_elec_angle = normalizeAngle((float)(RIGHT_DIR * RIGHT_PolePair) * encoder_right.angle_rad - RIGHT_zero_elec_angle);


		Get_SinCos(right_elec_angle, &R_st, &R_ct);
		Get_SinCos(left_elec_angle, &L_st, &L_ct);
		



		/* 10kHz 主中断下，每 10 个周期执行一次速度环（1kHz） */
		if (++LoopFlag >= 10U)
		{
			static uint8_t Erro_Sign = 0;
			LoopFlag = 0U; 
			if(left_sta == AS5047P_ERROR)
			{
				Erro_Sign++;
			}
			else 
			{
				left_raw_velocity = AS5047P_GetVelocityWindowed(&encoder_left, FOC_PERIOD_S*(Erro_Sign+1)*10.0f);
				Erro_Sign = 0 ;
			}

			left_filtered_velocity = LowPassFilter_Update(&left_velocity_filter, left_raw_velocity);
		#if OPEN_LOOP_SPIN_TEST_ENABLE
			Left_Velocity_FOC_PID.error_integral = 0.0f;
			Left_Velocity_FOC_PID.last_error = 0.0f;
			Left_Velocity_FOC_PID.output = 0.0f;
		#else
			PID_Calculate(&Left_Velocity_FOC_PID, DIR*Left_Target, DIR*left_filtered_velocity,(left_sta == AS5047P_ERROR));
		#endif
		}




		Left_Current_Detect = GetPhaseCurrent(&L_Motor_CurrentCfg);
		Right_Current_Detect = GetPhaseCurrent(&R_Motor_CurrentCfg);



        // Get_SinCos(left_elec_angle, &L_st, &L_ct);
        // left_raw_velocity = AS5047_GetVelocity(&encoder_left, FOC_PERIOD_S);
        // left_filtered_velocity = LowPassFilter_Update(&left_velocity_filter, left_raw_velocity);
        // PID_Calculate(&Left_Velocity_FOC_PID, Left_Target, left_filtered_velocity);



		Left_Raw_Iq = cal_Iq_Id(Left_Current_Detect.I_a, Left_Current_Detect.I_b, L_st, L_ct);
		Right_Raw_Iq = cal_Iq_Id(Right_Current_Detect.I_a, Right_Current_Detect.I_b, R_st, R_ct);

		Left_Filtered_Iq = LowPassFilter_Update(&left_current_filter, Left_Raw_Iq);
		Right_Filtered_Iq = LowPassFilter_Update(&right_current_filter, Right_Raw_Iq);

	#if PID_FAST_LOG_ENABLE
		float left_log_iq_target = 0.0f;
		float left_log_iq_feedback = DIR * LEFT_IQ_FEEDBACK_SIGN * Left_Filtered_Iq;
	#endif

	#if OPEN_LOOP_SPIN_TEST_ENABLE
		static float left_open_loop_theta = 0.0f;

		left_open_loop_theta = normalizeAngle(left_open_loop_theta + OPEN_LOOP_TEST_W_ELEC * FOC_PERIOD_S);
		Get_SinCos(left_open_loop_theta, &L_st, &L_ct);

		Left_Current_FOC_PID.error_integral = 0.0f;
		Left_Current_FOC_PID.last_error = 0.0f;
		Left_Current_FOC_PID.output = 0.0f;

		L_uq_final = constrain(OPEN_LOOP_TEST_UQ, -Uq_max, Uq_max);
		FOC_SetSVPWM(&L_Motor, L_uq_final, L_st, L_ct, 0.0f);
	#if PID_FAST_LOG_ENABLE
		left_log_iq_target = L_uq_final;
	#endif
	#else


		float left_iq_target = DIR * Left_Target;
		float left_iq_feedback = DIR * LEFT_IQ_FEEDBACK_SIGN * Left_Filtered_Iq;
		if (LEFT_CURRENT_TARGET_USE_SPEED_OUTPUT != 0U)
		{
			left_iq_target = DIR * Left_Velocity_FOC_PID.output;
		}

		/*
		 * 降级策略：
		 *   AS5047P_OK    — 角度新鲜，正常闭环
		 *   AS5047P_STALE — 1~3 次失败，角度沿用上次有效值，控制继续（降级运行）
		 *   AS5047P_ERROR — 连续失败达阈值，清积分并停止输出，等后台重初始化
		 */

		if (left_sta != AS5047P_ERROR)
		{
			PID_Calculate(&Left_Current_FOC_PID, left_iq_target, left_iq_feedback,
			             (left_sta == AS5047P_ERROR));

			L_uq_final = constrain(Left_Current_FOC_PID.output, -Uq_max, Uq_max);


			FOC_SetSVPWM(&L_Motor, L_uq_final, L_st, L_ct, 0.0f);

		}

		else
		{
			Left_Current_FOC_PID.error_integral = 0.0f;
			Left_Current_FOC_PID.last_error     = 0.0f;
			Left_Current_FOC_PID.output         = 0.0f;
			L_uq_final = 0.0f;
			FOC_SetSVPWM(&L_Motor, 0.0f, L_st, L_ct, 0.0f);
		}
	#if PID_FAST_LOG_ENABLE
		left_log_iq_target = left_iq_target;
	#endif
	#endif

		if (right_sta != AS5047P_ERROR)
		{
			PID_Calculate(&Right_Current_FOC_PID, RIGHT_DIR*Right_Target, RIGHT_DIR*Right_Filtered_Iq,
			             (right_sta == AS5047P_ERROR));
			R_uq_final = constrain(Right_Current_FOC_PID.output, -Uq_max, Uq_max);
			FOC_SetSVPWM(&R_Motor, R_uq_final, R_st, R_ct, 0.0f);
		}
		else
		{
			Right_Current_FOC_PID.error_integral = 0.0f;
			Right_Current_FOC_PID.last_error     = 0.0f;
			Right_Current_FOC_PID.output         = 0.0f;
			R_uq_final = 0.0f;
			FOC_SetSVPWM(&R_Motor, 0.0f, R_st, R_ct, 0.0f);
		}





		/* 保存CSV数据用于AI调参 */
		pid_csv_data.timestamp_ms = HAL_GetTick();
		pid_csv_data.setpoint = PID_TUNE_SETPOINT;
		pid_csv_data.input = PID_TUNE_INPUT;
		pid_csv_data.pwm = PID_TUNE_PID.output;
		pid_csv_data.error = pid_csv_data.setpoint - pid_csv_data.input;
		/* 协议兼容：CSV列 p,i,d 上报 PID 参数 */
		pid_csv_data.p_term = PID_TUNE_PID.Kp;
		pid_csv_data.i_term = PID_TUNE_PID.Ki;
		pid_csv_data.d_term = PID_TUNE_PID.Kd;

		/* 10kHz采样：记录电流环关键量到RAM，主循环再导出 */
#if PID_FAST_LOG_ENABLE
		if (pid_fast_log_capture_enable && !pid_fast_log_full)
		{
			uint16_t idx = pid_fast_log_count;
			if (idx < PID_FAST_LOG_CAPACITY)
			{
				pid_fast_log[idx].setpoint = left_log_iq_target;
				pid_fast_log[idx].filtered_iq = left_log_iq_feedback;
				pid_fast_log[idx].raw_iq = DIR * LEFT_IQ_FEEDBACK_SIGN * Left_Raw_Iq;
				pid_fast_log[idx].uq_final = L_uq_final;
				idx++;
				pid_fast_log_count = idx;
				if (idx >= PID_FAST_LOG_CAPACITY)
				{
					pid_fast_log_full = 1U;
					pid_fast_log_capture_enable = 0U;
				}
			}
			else
			{
				pid_fast_log_full = 1U;
				pid_fast_log_capture_enable = 0U;
			}
		}
#endif

	}
}
