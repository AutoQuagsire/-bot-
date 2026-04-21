#include "pid_autotune.h"

#include "sys.h"
#include "INT.h"
#include "FOC.h"
#include "usb_debug.h"

#include <math.h>

extern PID_t Left_Velocity_FOC_PID;
extern float Left_Target;

#define AUTOTUNE_SPEED_PID    Left_Velocity_FOC_PID
#define AUTOTUNE_TARGET       Left_Target
#define AUTOTUNE_SIDE_NAME    "LEFT_SPEED"

typedef enum {
    AUTOTUNE_IDLE = 0,
    AUTOTUNE_PREPARE,
    AUTOTUNE_RUN_TRIAL,
    AUTOTUNE_DONE,
    AUTOTUNE_ABORT
} AutoTuneState_t;

typedef enum {
    AUTOTUNE_PHASE_KP = 0,
    AUTOTUNE_PHASE_KI,
    AUTOTUNE_PHASE_KD,
    AUTOTUNE_PHASE_FINISH
} AutoTunePhase_t;

typedef struct {
    uint8_t active;
    AutoTuneState_t state;
    AutoTunePhase_t phase;

    float p[3];
    float dp[3];
    float trial_p[3];

    float best_global_p[3];
    float best_global_cost;
    uint8_t has_global_best;

    float target_backup;

    uint8_t round_idx;
    uint8_t param_idx;
    uint8_t cand_idx;
    uint8_t cand_count;

    float best_cost_for_param;
    float best_value_for_param;

    uint32_t prepare_start_ms;
    uint32_t trial_start_ms;
    uint32_t last_sample_ms;
    uint32_t prepare_stable_ms;

    float iae;
    float overshoot;
    float sat_time_s;
    float sat_hard_time_s;
    float tail_abs_error_int;
    float tail_time_s;

    float prev_error;
    uint8_t prev_error_valid;
    uint16_t zero_cross_count;

    uint8_t trial_failed;
    const char *fail_reason;
} AutoTuneCtx_t;

static AutoTuneCtx_t g_tune = {0};

#define AUTOTUNE_SETPOINT_A      0.8f
#define AUTOTUNE_PREPARE_MS      300U
#define AUTOTUNE_TRIAL_MS        1200U
#define AUTOTUNE_SAMPLE_MS       5U
#define AUTOTUNE_DP_SHRINK       0.7f
#define AUTOTUNE_DP_TOL_SUM      0.005f

#define AUTOTUNE_MAX_ROUNDS_KP   6U
#define AUTOTUNE_MAX_ROUNDS_KI   4U
#define AUTOTUNE_MAX_ROUNDS_KD   2U

#define AUTOTUNE_PREPARE_STABLE_MS       100U
#define AUTOTUNE_PREPARE_MAX_WAIT_MS     1200U
#define AUTOTUNE_PREPARE_INPUT_THRESH_A  0.08f

#define AUTOTUNE_OSC_OVERSHOOT_FAIL_GAIN 1.30f
#define AUTOTUNE_OSC_MIN_TIME_MS         80U
#define AUTOTUNE_SAT_HARD_DUTY_GAIN      0.98f
#define AUTOTUNE_SAT_HARD_TIME_S         0.15f
#define AUTOTUNE_ZERO_CROSS_FAIL_COUNT   16U

#define AUTOTUNE_TAIL_WINDOW_MS          200U

#define AUTOTUNE_FAIL_COST               1e9f

#define PARAM_KP_IDX 0U
#define PARAM_KI_IDX 1U
#define PARAM_KD_IDX 2U

static float clamp_positive(float v)
{
    return (v < 0.0f) ? 0.0f : v;
}

static uint8_t current_param_from_phase(void)
{
    if (g_tune.phase == AUTOTUNE_PHASE_KP) return PARAM_KP_IDX;
    if (g_tune.phase == AUTOTUNE_PHASE_KI) return PARAM_KI_IDX;
    return PARAM_KD_IDX;
}

static uint8_t phase_max_rounds(void)
{
    if (g_tune.phase == AUTOTUNE_PHASE_KP) return AUTOTUNE_MAX_ROUNDS_KP;
    if (g_tune.phase == AUTOTUNE_PHASE_KI) return AUTOTUNE_MAX_ROUNDS_KI;
    return AUTOTUNE_MAX_ROUNDS_KD;
}

static const char *phase_name(AutoTunePhase_t phase)
{
    if (phase == AUTOTUNE_PHASE_KP) return "KP";
    if (phase == AUTOTUNE_PHASE_KI) return "KI";
    if (phase == AUTOTUNE_PHASE_KD) return "KD";
    return "DONE";
}

static float candidate_from_index(float center, float delta, uint8_t idx)
{
    if (g_tune.phase == AUTOTUNE_PHASE_KI)
    {
        /* KI阶段只做保守上探，避免积分项激进变化 */
        if (idx == 0U) return clamp_positive(center);
        return clamp_positive(center + delta);
    }

    if (idx == 0U) return clamp_positive(center - delta);
    if (idx == 1U) return clamp_positive(center);
    return clamp_positive(center + delta);
}

static void apply_current_pid(float kp, float ki, float kd)
{
    float ilim = AUTOTUNE_SPEED_PID.integral_limit;
    PID_ParameterInit(&AUTOTUNE_SPEED_PID, kp, ki, kd, ilim);
}

static void update_global_best_if_needed(float cost)
{
    if ((!g_tune.has_global_best) || (cost < g_tune.best_global_cost))
    {
        g_tune.best_global_cost = cost;
        g_tune.best_global_p[0] = g_tune.trial_p[0];
        g_tune.best_global_p[1] = g_tune.trial_p[1];
        g_tune.best_global_p[2] = g_tune.trial_p[2];
        g_tune.has_global_best = 1U;
    }
}

static void begin_candidate_trial(uint32_t now)
{
    g_tune.param_idx = current_param_from_phase();

    g_tune.trial_p[0] = g_tune.p[0];
    g_tune.trial_p[1] = g_tune.p[1];
    g_tune.trial_p[2] = g_tune.p[2];
    g_tune.trial_p[g_tune.param_idx] = candidate_from_index(
        g_tune.p[g_tune.param_idx],
        g_tune.dp[g_tune.param_idx],
        g_tune.cand_idx
    );

    apply_current_pid(g_tune.trial_p[0], g_tune.trial_p[1], g_tune.trial_p[2]);

    g_tune.iae = 0.0f;
    g_tune.overshoot = 0.0f;
    g_tune.sat_time_s = 0.0f;
    g_tune.sat_hard_time_s = 0.0f;
    g_tune.tail_abs_error_int = 0.0f;
    g_tune.tail_time_s = 0.0f;
    g_tune.prev_error = 0.0f;
    g_tune.prev_error_valid = 0U;
    g_tune.zero_cross_count = 0U;
    g_tune.trial_failed = 0U;
    g_tune.fail_reason = "";

    AUTOTUNE_TARGET = 0.0f;
    g_tune.prepare_start_ms = now;
    g_tune.prepare_stable_ms = 0U;
    g_tune.last_sample_ms = now;
    g_tune.state = AUTOTUNE_PREPARE;

    /* KI阶段保持更保守的设定，降低积分激增风险 */
    if (g_tune.phase == AUTOTUNE_PHASE_KI && g_tune.trial_p[1] > 0.3f)
    {
        g_tune.trial_p[1] = 0.3f;
        apply_current_pid(g_tune.trial_p[0], g_tune.trial_p[1], g_tune.trial_p[2]);
    }

    USB_Debug_Printf(
        "AUTOTUNE trial: phase=%s round=%u param=%u cand=%u/%u Kp=%.5f Ki=%.5f Kd=%.5f\r\n",
        phase_name(g_tune.phase),
        g_tune.round_idx,
        g_tune.param_idx,
        g_tune.cand_idx,
        g_tune.cand_count,
        g_tune.trial_p[0], g_tune.trial_p[1], g_tune.trial_p[2]
    );
}

static float compute_trial_cost(void)
{
    float tail_mae = 0.0f;
    if (g_tune.tail_time_s > 1e-6f)
    {
        tail_mae = g_tune.tail_abs_error_int / g_tune.tail_time_s;
    }

    if (g_tune.trial_failed)
    {
        return AUTOTUNE_FAIL_COST;
    }

    /* 惩罚项：过零次数和尾段误差专门用于抑制振荡尾巴 */
    return g_tune.iae
         + 2.0f * g_tune.overshoot
         + 0.5f * g_tune.sat_time_s
         + 0.04f * (float)g_tune.zero_cross_count
         + 2.5f * tail_mae;
}

static void advance_phase(void)
{
    if (g_tune.phase == AUTOTUNE_PHASE_KP)
    {
        g_tune.phase = AUTOTUNE_PHASE_KI;
    }
    else if (g_tune.phase == AUTOTUNE_PHASE_KI)
    {
        g_tune.phase = AUTOTUNE_PHASE_KD;
    }
    else
    {
        g_tune.phase = AUTOTUNE_PHASE_FINISH;
        g_tune.state = AUTOTUNE_DONE;
        return;
    }

    g_tune.round_idx = 0U;
    g_tune.cand_idx = 0U;
    g_tune.param_idx = current_param_from_phase();
    g_tune.cand_count = (g_tune.phase == AUTOTUNE_PHASE_KI) ? 2U : 3U;
    g_tune.best_cost_for_param = 1e30f;
    g_tune.best_value_for_param = g_tune.p[g_tune.param_idx];

    USB_Debug_Printf("AUTOTUNE phase advance -> %s\r\n", phase_name(g_tune.phase));
}

static void finish_param_and_advance(void)
{
    float old_value;
    float dp_sum;

    old_value = g_tune.p[g_tune.param_idx];
    g_tune.p[g_tune.param_idx] = g_tune.best_value_for_param;

    if (fabsf(g_tune.best_value_for_param - old_value) < 1e-6f)
    {
        g_tune.dp[g_tune.param_idx] *= AUTOTUNE_DP_SHRINK;
    }

    /* 当前阶段收敛判断：步长太小或轮数达到上限则进入下一阶段 */
    dp_sum = g_tune.dp[g_tune.param_idx];
    g_tune.round_idx++;

    if ((g_tune.round_idx >= phase_max_rounds()) || (dp_sum < AUTOTUNE_DP_TOL_SUM))
    {
        advance_phase();
        return;
    }

    g_tune.cand_idx = 0U;
    g_tune.best_cost_for_param = 1e30f;
    g_tune.best_value_for_param = g_tune.p[g_tune.param_idx];
}

void PID_AutoTune_Start(void)
{
    if (g_tune.active)
    {
        USB_Debug_Printf("AUTOTUNE already active\r\n");
        return;
    }

    g_tune.active = 1U;
    g_tune.state = AUTOTUNE_IDLE;
    g_tune.phase = AUTOTUNE_PHASE_KP;

    g_tune.p[0] = AUTOTUNE_SPEED_PID.Kp;
    g_tune.p[1] = AUTOTUNE_SPEED_PID.Ki;
    g_tune.p[2] = AUTOTUNE_SPEED_PID.Kd;

    g_tune.dp[0] = (g_tune.p[0] > 0.2f) ? (0.2f * g_tune.p[0]) : 0.05f;
    g_tune.dp[1] = (g_tune.p[1] > 0.005f) ? (0.2f * g_tune.p[1]) : 0.001f;
    g_tune.dp[2] = (g_tune.p[2] > 0.001f) ? (0.2f * g_tune.p[2]) : 0.0002f;

    g_tune.round_idx = 0;
    g_tune.param_idx = PARAM_KP_IDX;
    g_tune.cand_idx = 0;
    g_tune.cand_count = 3U;
    g_tune.best_cost_for_param = 1e30f;
    g_tune.best_value_for_param = g_tune.p[0];
    g_tune.target_backup = AUTOTUNE_TARGET;
    g_tune.best_global_cost = 1e30f;
    g_tune.best_global_p[0] = g_tune.p[0];
    g_tune.best_global_p[1] = g_tune.p[1];
    g_tune.best_global_p[2] = g_tune.p[2];
    g_tune.has_global_best = 0U;

    USB_Debug_Printf(
        "AUTOTUNE start(%s): init Kp=%.5f Ki=%.5f Kd=%.5f\r\n",
        AUTOTUNE_SIDE_NAME,
        g_tune.p[0], g_tune.p[1], g_tune.p[2]
    );

    begin_candidate_trial(HAL_GetTick());
}

void PID_AutoTune_Stop(void)
{
    if (!g_tune.active)
    {
        return;
    }

    g_tune.active = 0U;
    g_tune.state = AUTOTUNE_ABORT;
    AUTOTUNE_TARGET = g_tune.target_backup;
    if (g_tune.has_global_best)
    {
        apply_current_pid(g_tune.best_global_p[0], g_tune.best_global_p[1], g_tune.best_global_p[2]);
    }
    USB_Debug_Printf("AUTOTUNE stopped by user\r\n");
}

uint8_t PID_AutoTune_IsActive(void)
{
    return g_tune.active;
}

void PID_AutoTune_Update(void)
{
    uint32_t now;
    float cost;
    float dt;
    float abs_setpoint;
    float abs_input;
    float abs_pwm;
    float abs_error;
    float cur_error;
    uint32_t elapsed_ms;

    if (!g_tune.active)
    {
        return;
    }

    now = HAL_GetTick();

    if (OvercurrentProtection_GetStatus())
    {
        g_tune.state = AUTOTUNE_ABORT;
        g_tune.active = 0U;
        AUTOTUNE_TARGET = 0.0f;
        if (g_tune.has_global_best)
        {
            apply_current_pid(g_tune.best_global_p[0], g_tune.best_global_p[1], g_tune.best_global_p[2]);
        }
        USB_Debug_Printf("AUTOTUNE abort: overcurrent protection active\r\n");
        return;
    }

    if (g_tune.state == AUTOTUNE_PREPARE)
    {
        if ((now - g_tune.last_sample_ms) >= AUTOTUNE_SAMPLE_MS)
        {
            uint32_t prep_dt_ms = (now - g_tune.last_sample_ms);
            g_tune.last_sample_ms = now;
            if (fabsf(pid_csv_data.input) < AUTOTUNE_PREPARE_INPUT_THRESH_A)
            {
                g_tune.prepare_stable_ms += prep_dt_ms;
            }
            else
            {
                g_tune.prepare_stable_ms = 0U;
            }
        }

        elapsed_ms = (now - g_tune.prepare_start_ms);
        if ((elapsed_ms >= AUTOTUNE_PREPARE_MS && g_tune.prepare_stable_ms >= AUTOTUNE_PREPARE_STABLE_MS) ||
            (elapsed_ms >= AUTOTUNE_PREPARE_MAX_WAIT_MS))
        {
            if (elapsed_ms >= AUTOTUNE_PREPARE_MAX_WAIT_MS)
            {
                USB_Debug_Printf("AUTOTUNE warn: prepare timeout, continue trial\r\n");
            }
            AUTOTUNE_TARGET = AUTOTUNE_SETPOINT_A;
            g_tune.trial_start_ms = now;
            g_tune.last_sample_ms = now;
            g_tune.state = AUTOTUNE_RUN_TRIAL;
        }
        return;
    }

    if (g_tune.state == AUTOTUNE_RUN_TRIAL)
    {
        if ((now - g_tune.last_sample_ms) >= AUTOTUNE_SAMPLE_MS)
        {
            dt = (float)(now - g_tune.last_sample_ms) * 0.001f;
            g_tune.last_sample_ms = now;

            abs_setpoint = fabsf(pid_csv_data.setpoint);
            abs_input = fabsf(pid_csv_data.input);
            abs_pwm = fabsf(pid_csv_data.pwm);
            abs_error = fabsf(pid_csv_data.error);
            cur_error = pid_csv_data.error;

            g_tune.iae += abs_error * dt;
            if (abs_input > abs_setpoint)
            {
                g_tune.overshoot += (abs_input - abs_setpoint) * dt;
            }
            if (abs_pwm > (0.95f * Uq_max))
            {
                g_tune.sat_time_s += dt;
            }

            if ((now - g_tune.trial_start_ms) >= (AUTOTUNE_TRIAL_MS - AUTOTUNE_TAIL_WINDOW_MS))
            {
                g_tune.tail_abs_error_int += abs_error * dt;
                g_tune.tail_time_s += dt;
            }

            if (g_tune.prev_error_valid)
            {
                if (((g_tune.prev_error > 0.0f) && (cur_error < 0.0f)) ||
                    ((g_tune.prev_error < 0.0f) && (cur_error > 0.0f)))
                {
                    g_tune.zero_cross_count++;
                }
            }
            g_tune.prev_error = cur_error;
            g_tune.prev_error_valid = 1U;

            /* ---------- 硬失败条件 ---------- */
            elapsed_ms = (now - g_tune.trial_start_ms);
            if ((elapsed_ms > AUTOTUNE_OSC_MIN_TIME_MS) &&
                (abs_setpoint > 1e-6f) &&
                (abs_input > (AUTOTUNE_OSC_OVERSHOOT_FAIL_GAIN * abs_setpoint)))
            {
                g_tune.trial_failed = 1U;
                g_tune.fail_reason = "overshoot_hard";
            }

            if (abs_pwm > (AUTOTUNE_SAT_HARD_DUTY_GAIN * Uq_max))
            {
                g_tune.sat_hard_time_s += dt;
                if (g_tune.sat_hard_time_s > AUTOTUNE_SAT_HARD_TIME_S)
                {
                    g_tune.trial_failed = 1U;
                    g_tune.fail_reason = "saturation_hard";
                }
            }

            if (g_tune.zero_cross_count > AUTOTUNE_ZERO_CROSS_FAIL_COUNT)
            {
                g_tune.trial_failed = 1U;
                g_tune.fail_reason = "oscillation_hard";
            }

            if (g_tune.trial_failed)
            {
                /* 硬失败提前结束该候选，减少风险暴露时间 */
                g_tune.trial_start_ms = now - AUTOTUNE_TRIAL_MS;
            }
        }

        if ((now - g_tune.trial_start_ms) >= AUTOTUNE_TRIAL_MS)
        {
            cost = compute_trial_cost();

            update_global_best_if_needed(cost);

            if (cost < g_tune.best_cost_for_param)
            {
                g_tune.best_cost_for_param = cost;
                g_tune.best_value_for_param = g_tune.trial_p[g_tune.param_idx];
            }

            USB_Debug_Printf(
                "AUTOTUNE eval: phase=%s round=%u param=%u cand=%u cost=%.6f iae=%.6f ov=%.6f sat=%.6f zc=%u tail=%.6f fail=%u %s\r\n",
                phase_name(g_tune.phase),
                g_tune.round_idx,
                g_tune.param_idx,
                g_tune.cand_idx,
                cost,
                g_tune.iae,
                g_tune.overshoot,
                g_tune.sat_time_s,
                g_tune.zero_cross_count,
                (g_tune.tail_time_s > 1e-6f) ? (g_tune.tail_abs_error_int / g_tune.tail_time_s) : 0.0f,
                g_tune.trial_failed,
                g_tune.fail_reason
            );

            AUTOTUNE_TARGET = 0.0f;
            g_tune.cand_idx++;
            if (g_tune.cand_idx >= g_tune.cand_count)
            {
                finish_param_and_advance();
            }

            if (g_tune.state == AUTOTUNE_DONE)
            {
                if (g_tune.has_global_best)
                {
                    apply_current_pid(g_tune.best_global_p[0], g_tune.best_global_p[1], g_tune.best_global_p[2]);
                }
                else
                {
                    apply_current_pid(g_tune.p[0], g_tune.p[1], g_tune.p[2]);
                }
                g_tune.active = 0U;
                AUTOTUNE_TARGET = g_tune.target_backup;
                USB_Debug_Printf(
                    "AUTOTUNE done: Kp=%.5f Ki=%.5f Kd=%.5f best_cost=%.6f\r\n",
                    g_tune.has_global_best ? g_tune.best_global_p[0] : g_tune.p[0],
                    g_tune.has_global_best ? g_tune.best_global_p[1] : g_tune.p[1],
                    g_tune.has_global_best ? g_tune.best_global_p[2] : g_tune.p[2],
                    g_tune.has_global_best ? g_tune.best_global_cost : 0.0f
                );
                return;
            }

            begin_candidate_trial(now);
        }
    }
}
