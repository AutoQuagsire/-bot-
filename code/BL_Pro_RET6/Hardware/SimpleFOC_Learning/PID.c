#include "PID.h"
#include "FOC.h"
#include <math.h>

#ifndef Uq_max
#define Uq_max 6.0f
#endif


__attribute__((optimize("O2,fast-math")))
void PID_Calculate(PID_t *pid, float target, float measure, uint8_t freeze_external)
{
    float error = target - measure;//计算误差


    const float UNWIND_GAIN = 1.0f;
 
    float i_band = pid->I_SEP_RATIO * fabsf(target);//计算允许积分的误差范围
    if (i_band < pid->I_ERR_MIN) i_band = pid->I_ERR_MIN;//限制最小积分带

    float derivative = error - pid->last_error;//计算微分项
    float p_term = pid->Kp * error;//比例输出
    float d_term = pid->Kd * derivative;//微分输出

    float i_term_pre = pid->Ki * pid->error_integral;//原始积分输出
    float u_raw_pre  = p_term + i_term_pre + d_term;//原始PID输出（未限幅）

	/*当输出饱和并且误差与输出方向一致(此时若允许积分恶化输出)时冻结积分*/
    uint8_t freeze_by_sat =
        ((u_raw_pre >  pid->output_limit) && (error > 0.0f)) ||
        ((u_raw_pre < -pid->output_limit) && (error < 0.0f));


    uint8_t freeze_integral = (freeze_external || freeze_by_sat);
    uint8_t allow_integrate = (fabsf(error) <= i_band);//仅当误差在积分带内时允许积分
    /*误差与积分积累项方向相反时允许纠正积分*/
	uint8_t allow_unwind    = (pid->error_integral * error < 0.0f);

	/*允许积分更新的情况*/
    if (!freeze_integral && allow_integrate) {
        pid->error_integral += error;
    } else if (allow_unwind) {
        pid->error_integral += UNWIND_GAIN * error;
    }

	float i_term = pid->Ki * pid->error_integral;//计算新的积分项输出
    
	/*积分限幅策略*/ 
	if (i_term > pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = pid->integral_limit / pid->Ki;
        }
    } else if (i_term < -pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = -pid->integral_limit / pid->Ki;
        }
    }
    i_term = pid->Ki * pid->error_integral;
    pid->output = p_term + i_term + d_term;//计算PID最终输出

	/*输出限幅*/
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    pid->last_error = error;
}






void PID_CalculateTest(PID_t *pid, float target, float measure)
{
    float error = target - measure;

    /* 基线版本：只保留传统 PID，不做积分分离/冻结与输出限幅 */
    pid->error_integral += error;

    float i_output = pid->Ki * pid->error_integral;
    if (i_output > pid->integral_limit)
    {
        if (fabsf(pid->Ki) > 1e-6f)
        {
            pid->error_integral = pid->integral_limit / pid->Ki;
        }
    }
    else if (i_output < -pid->integral_limit)
    {
        if (fabsf(pid->Ki) > 1e-6f)
        {
            pid->error_integral = -pid->integral_limit / pid->Ki;
        }
    }

    float derivative = error - pid->last_error;
    pid->last_error = error;

    pid->output = (pid->Kp * error) +
                  (pid->Ki * pid->error_integral) +
                  (pid->Kd * derivative);
}













void PID_ParameterInitEx(PID_t *pid,float kp,float ki,float kd,float integral_limit,
                 float output_limit,float i_err_min,float i_sep_ratio)
{
	pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->I_ERR_MIN = i_err_min;
    pid->I_SEP_RATIO = i_sep_ratio;
	pid->Kp = kp;	pid->Ki = ki;	pid->Kd = kd;
}

void PID_ParameterInit(PID_t *pid,float kp,float ki,float kd,float integral_limit)
{
    float output_limit = (pid->output_limit > 0.0f) ? pid->output_limit : Uq_max;
    float i_err_min = (pid->I_ERR_MIN > 0.0f) ? pid->I_ERR_MIN : 0.05f;
    float i_sep_ratio = (pid->I_SEP_RATIO > 0.0f) ? pid->I_SEP_RATIO : 0.5f;

    PID_ParameterInitEx(pid, kp, ki, kd, integral_limit,
                      output_limit, i_err_min, i_sep_ratio);
}
