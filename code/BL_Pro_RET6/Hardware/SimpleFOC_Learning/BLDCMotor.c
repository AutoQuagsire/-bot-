#include "./BLDCMotor.h"
#include "./driver.h"
#include "./current_sense.h"
#include "./platform.h"
#include <math.h>
#include <stddef.h>

#if defined(__has_attribute)
  #if __has_attribute(optimize)
    #define ATTR_OPT_FAST __attribute__((optimize("O2,fast-math")))
  #else
    #define ATTR_OPT_FAST
  #endif
#else
  #define ATTR_OPT_FAST
#endif

#if defined(__has_attribute)
  #if __has_attribute(always_inline)
    #define ATTR_ALWAYS_INLINE __attribute__((always_inline)) inline
  #else
    #define ATTR_ALWAYS_INLINE inline
  #endif
#else
  #define ATTR_ALWAYS_INLINE inline
#endif




//预留代码：实现功能：把一个驱动器对象的指针，挂到电机对象�?
static const CurrentLoopSchedulePoint_t current_loop_schedule_table[] = {
    {0.03f, 0.400f, 0.080f},
    {0.10f, 0.475f, 0.125f},
    {0.30f, 0.465f, 0.350f},
    {0.60f, 0.525f, 0.300f},
    {0.90f, 0.550f, 0.500f},
    {1.20f, 0.550f, 0.700f},
    {1.50f, 0.550f, 0.650f},
    {1.80f, 0.550f, 0.650f},
};

static ATTR_ALWAYS_INLINE float current_loop_lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

void CurrentLoop_GetScheduledParams(float target_iq,
                                    float *ff_coef,
                                    float *integral_limit)
{
    const size_t table_size = sizeof(current_loop_schedule_table) / sizeof(current_loop_schedule_table[0]);
    const CurrentLoopSchedulePoint_t *last_point;
    float iq_abs;
    size_t idx;

    if ((ff_coef == NULL) || (integral_limit == NULL) || (table_size == 0U)) {
        return;
    }

    iq_abs = fabsf(target_iq);
    last_point = &current_loop_schedule_table[table_size - 1U];

    if (iq_abs <= current_loop_schedule_table[0].iq_abs) {
        *ff_coef = current_loop_schedule_table[0].ff_coef;
        *integral_limit = current_loop_schedule_table[0].integral_limit;
        return;
    }

    if (iq_abs >= last_point->iq_abs) {
        *ff_coef = last_point->ff_coef;
        *integral_limit = last_point->integral_limit;
        return;
    }

    for (idx = 0U; idx < (table_size - 1U); idx++) {
        const CurrentLoopSchedulePoint_t *p0 = &current_loop_schedule_table[idx];
        const CurrentLoopSchedulePoint_t *p1 = &current_loop_schedule_table[idx + 1U];

        if (iq_abs <= p1->iq_abs) {
            const float span = p1->iq_abs - p0->iq_abs;
            const float ratio = (span > 0.0f) ? ((iq_abs - p0->iq_abs) / span) : 0.0f;

            *ff_coef = current_loop_lerp(p0->ff_coef, p1->ff_coef, ratio);
            *integral_limit = current_loop_lerp(p0->integral_limit, p1->integral_limit, ratio);
            return;
        }
    }

    *ff_coef = last_point->ff_coef;
    *integral_limit = last_point->integral_limit;
}

void linkDriver(Driver_t *driver, Motor_t *motor)
{
    if (!driver || !motor) return;

    motor->driver = driver;
}

//预留代码：实现功能：把一个传感器对象的指针，挂到电机对象�?
void linkSensor(Sensor_t *sensor, Motor_t *motor)
{
    if (!sensor ||!motor) return;

    motor->sensor = sensor;
    motor->state.has_sensor = (sensor != NULL) ? 1U : 0U;
}

void linkCurrentSense(CurrentSense_t *Current_Sense, Motor_t *motor)
{
    if (!Current_Sense ||!motor) return;
    motor -> current_sense = Current_Sense;
    

}

//预留代码，实现功能：电机硬件可用性检�?+ 参数约束整理 + 进入可使能状�?
/*
�?）检�?driver 是否已经连接并初始化�?
     如果没有 driver，或�?driver->initialized 还没好，它直接报失败，电机状态置�?motor_init_failed，然后返�?0�?
�?）更�?motor 状态为“初始化中”：
    �?motor_status 置成 motor_initializing�?
�?）做电压限制的安全检查：
    它会检查：
    如果 motor.voltage_limit > driver.voltage_limit
    就把 motor 的限制压�?driver 的限制以�?
    voltage_sensor_align 也不能超�?voltage_limit
�?）更新控制器内部�?limit
    它会调用�?

    updateCurrentLimit(current_limit)
    updateVoltageLimit(voltage_limit)
    updateVelocityLimit(velocity_limit)

    也就是说，init() 不是只配硬件�?
    还顺便把控制器里依赖 limit 的东西同步好�?
�?）整理电机参�?
    如果只给了单个相电感 phase_inductance，但没有单独�?d/q 轴电感，它就�?d/q 都设成这个值�?

    这是个很典型的“初始化期参数整理”�?
�?）如果是开环而且没传感器，就给默认方�?

    如果�?

    没有 sensor
    控制模式是开环角�?开环速度
    方向还未�?

    它就默认 sensor_direction = CW

    这一步本质上是在补齐运行前提�?
�?）延时后调用 enable()
    这个很关键：
    init() 最后会�?
    延时
    enable()
    再延�?
    把状态设�?motor_uncalibrated
*/
uint8_t FOCMotor_init(Motor_t *FOC_Motor)
{
    if (!FOC_Motor) {
        return 0;
    }




    // 检�?driver 是否已连接并初始化完�?
    if (!FOC_Motor->driver || !(FOC_Motor->driver->initialized)) {
        FOC_Motor->state.motor_status = motor_init_failed;
        FOC_Motor->state.enabled = 0;
        return 0;
    }
    if (FOC_Motor->sensor && FOC_Motor->sensor->initialized) {
        FOC_Motor->state.has_sensor = 1U;
    } else {
        FOC_Motor->state.has_sensor = 0U;
    }



    // 通过第一步检查后，进入初始化中状�?
    FOC_Motor->state.motor_status = motor_initializing;

    // �?）电压限制安全检查：电机限制不能超过驱动限制
    if (FOC_Motor->config.voltage_limit > FOC_Motor->driver->voltage_limit) {
        FOC_Motor->config.voltage_limit = FOC_Motor->driver->voltage_limit;
    }
    if (FOC_Motor->config.voltage_sensor_align > FOC_Motor->config.voltage_limit) {
        FOC_Motor->config.voltage_sensor_align = FOC_Motor->config.voltage_limit;
    }
    //�?）更新控制器内部�?limit
    // 这里假设有全局函数可以更新 PID �?limit，实际可能需要更复杂的结构设�?

    //�?）整理电机参数：如果只给了单个相电感，就�?d/q 都设成这个�?
    if (FOC_Motor->param.Ld == 0.0f && FOC_Motor->param.Lq != 0.0f) {
        FOC_Motor->param.Ld = FOC_Motor->param.Lq;
    } else if (FOC_Motor->param.Lq == 0.0f && FOC_Motor->param.Ld != 0.0f) {
        FOC_Motor->param.Lq = FOC_Motor->param.Ld;
    }

    //�?）开环且无传感器时，若方向未知则给默认方�?
    if ((FOC_Motor->state.has_sensor == 0U) &&
        ((FOC_Motor->config.control_mode == motor_control_openloop_angle) ||
         (FOC_Motor->config.control_mode == motor_control_openloop_velocity)) &&
        (FOC_Motor->state.sensor_direction == sensor_direction_unknown)) {
        FOC_Motor->state.sensor_direction = sensor_direction_cw;
    }

    //�?）延�?-> 使能 -> 延时 -> 状态置为未校准
    Platform_DelayMs(10);
    FOCMotor_enable(FOC_Motor);
    Platform_DelayMs(10);
    FOC_Motor->state.motor_status = motor_uncalibrated;


    return 1;
}






//预留代码，实现功能：电机失能
/*
�?）如果有 current sense，就�?disable �?
�?）把 PWM 输出清零
�?）再禁用 driver
�?）更新状态把 enabled = 0
“先去能量，再断执行链�?
*/
void FOCMotor_disable(Motor_t *motor)
{
    if (!motor || !motor->driver) return;
    if (!motor->driver->initialized) return;

     //如果�?current sense，就�?disable �?
    if (motor->current_sense) {
        CurrentSense_Disable(motor->current_sense);
    }

    //�?PWM 输出清零
    Driver_SetPwm(motor->driver, 0.0f, 0.0f, 0.0f);

    //再禁�?driver
    if (motor->driver) {
        Driver_Disable(motor->driver);
    }

    //更新状态把 enabled = 0
    motor->state.enabled = 0;
}


//预留代码，实现功能：电机使能
/*
�?）使�?driver
先调用：
driver->enable()
�?）立刻把 PWM 清零
调用�?
driver->setPwm(0,0,0)
�?）如果有 current sense，就 enable
�?）重置控制器状�?
它会 reset�?
PID_velocity
P_angle
PID_current_q
PID_current_d
�?）更�?enabled 标志
�?enabled = 1

*/
void FOCMotor_enable(Motor_t *motor)
{
    if (!motor || !motor->driver) return;
    if (!motor->driver->initialized) return;

    Driver_Enable(motor->driver);
    Driver_SetPwm(motor->driver, 0.0f, 0.0f, 0.0f);

    if (motor->current_sense) {
        CurrentSense_Enable(motor->current_sense);
    }

    motor->state.enabled = 1;
}



void MotorParam_Init(Motor_t *motor, float pole_pairs, float phase_resistance,
                    float kv, float Ld, float Lq)
{
    if (!motor) return;

    motor->param.pole_pairs = pole_pairs;
    motor->param.phase_resistance = phase_resistance;
    motor->param.kv = kv;
    motor->param.Ld = Ld;
    motor->param.Lq = Lq;

}

ATTR_OPT_FAST
static float normalize_angle_0_2pi(float angle)
{
    float a = angle - (float)(int32_t)(angle * (1.0f / (2.0f * PI))) * (2.0f * PI);
    return (a >= 0.0f) ? a : (a + 2.0f * PI);
}

ATTR_OPT_FAST
static ATTR_ALWAYS_INLINE float Motor_CalcElectricalAngleUnchecked(const Motor_t *motor, const Sensor_t *sensor)
{
    float mech_angle = sensor->data.shaft_angle;
    float pole_pairs = motor->param.pole_pairs;

    if (motor->state.sensor_direction == sensor_direction_ccw) {
        pole_pairs = -pole_pairs;
    }

    float elec_angle = pole_pairs * mech_angle - motor->zero_electrical_angle;
    return normalize_angle_0_2pi(elec_angle);
}


float Motor_GetMechanicalAngle(Motor_t *motor)
{
    if (!motor || !motor->sensor) {
        return 0.0f;
    }

    if (!(motor->sensor->initialized)) {
        return 0.0f;
    }

    return Sensor_GetAngle(motor->sensor);
}

ATTR_OPT_FAST
float Motor_GetElectricalAngle(Motor_t *motor)
{
    if (!motor) {
        return 0.0f;
    }

    Sensor_t *sensor = motor->sensor;
    if (!sensor || !sensor->initialized) {
        return 0.0f;
    }

    return Motor_CalcElectricalAngleUnchecked(motor, sensor);
}



uint8_t Motor_UpdateSensor(Motor_t *motor, float dt)
{
    if (!motor || !motor->sensor) {
        return 0U;
    }

    if (!(motor->sensor->initialized)) {
        return 0U;
    }

    Sensor_Update(motor->sensor, dt);


    motor->electrical_angle = Motor_GetElectricalAngle(motor);


    return 1U;
}



static ATTR_OPT_FAST ATTR_ALWAYS_INLINE float clampf_fast(float x, float low, float high)
{
    return (x < low) ? low : ((x > high) ? high : x);
}

static ATTR_ALWAYS_INLINE void driver_set_pwm_fast(const Driver_t *driver,
                                                   uint32_t ccr_a,
                                                   uint32_t ccr_b,
                                                   uint32_t ccr_c)
{
    volatile uint32_t *ccr = &driver->htim->Instance->CCR1;
    ccr[(driver->chA >> 2U)] = ccr_a;
    ccr[(driver->chB >> 2U)] = ccr_b;
    ccr[(driver->chC >> 2U)] = ccr_c;
}

ATTR_OPT_FAST
static void BLDC_SetFVPWM(Motor_t *motor, float uq, float st, float ct)
{
    if (!motor || !motor->driver || !motor->driver->htim) {
        return;
    }
    float uq_limit = motor->driver->voltage_limit;
    if (uq_limit <= 0.0f || uq_limit > Uq_max) {
        uq_limit = Uq_max;
    }
    if (uq > uq_limit) {
        uq = uq_limit;
    } else if (uq < -uq_limit) {
        uq = -uq_limit;
    }

    /* 1) 由电角度计算 sin/cos */


    /* 2) 反Park+Clarke (Ud=0) 得到三相相电�?*/
    float Ualpha = -uq * st;
    float Ubeta  =  uq * ct;
    float t      = _SQRT3 * Ubeta;
    float Ua     = Ualpha;
    float Ub     = (-Ualpha + t) * 0.5f;
    float Uc     = (-Ualpha - t) * 0.5f;

    /* 3) SVPWM零序注入并映射到 [0, V_SUPPLY] */
    float Umax = Ua;
    if (Ub > Umax) Umax = Ub;
    if (Uc > Umax) Umax = Uc;
    float Umin = Ua;
    if (Ub < Umin) Umin = Ub;
    if (Uc < Umin) Umin = Uc;
    float Uzero = (V_SUPPLY * 0.5f) - (Umax + Umin) * 0.5f;

    Ua = clampf_fast(Ua + Uzero, 0.0f, V_SUPPLY);
    Ub = clampf_fast(Ub + Uzero, 0.0f, V_SUPPLY);
    Uc = clampf_fast(Uc + Uzero, 0.0f, V_SUPPLY);

    /* 4) 电压转换为定时器比较�?*/
    const float scale = (float)motor->driver->htim->Init.Period * (1.0f / V_SUPPLY);
    const uint32_t ccr_a = (uint32_t)(Ua * scale + 0.5f);
    const uint32_t ccr_b = (uint32_t)(Ub * scale + 0.5f);
    const uint32_t ccr_c = (uint32_t)(Uc * scale + 0.5f);

    driver_set_pwm_fast(motor->driver, ccr_a, ccr_b, ccr_c);
}



static void Motor_ApplyAlignVector(Motor_t *motor, float uq, float elec_angle)
{
    float st = sinf(elec_angle);
    float ct = cosf(elec_angle);
    BLDC_SetFVPWM(motor, uq, st, ct);
}

void Motor_SetPhaseVoltageQBySinCos(Motor_t *motor, float uq, float sin_el, float cos_el)
{
    BLDC_SetFVPWM(motor, uq, sin_el, cos_el);
}

void Motor_SetPhaseVoltageQ(Motor_t *motor, float uq, float elec_angle)
{
    float st = sinf(elec_angle);
    float ct = cosf(elec_angle);
    BLDC_SetFVPWM(motor, uq, st, ct);
}


uint8_t Motor_CalibrateZeroElectricalAngle(Motor_t *motor,
                                           float align_voltage,
                                           float align_angle,
                                           uint16_t settle_ms)
{
    if (!motor || !motor->sensor || !motor->driver) {
        return 0U;
    }

    if (!motor->sensor->initialized) {
        return 0U;
    }

    if (!motor->driver->initialized) {
        return 0U;
    }

    // 1. 使能电机
    FOCMotor_enable(motor);

    // 2. 施加固定电角矢量，让转子吸到目标位置
    Motor_ApplyAlignVector(motor, align_voltage, align_angle);

    // 3. 等待转子稳定
    HAL_Delay(settle_ms);

    // 4. 多次采样机械角，做圆均�?
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;

    for (uint16_t i = 0; i < 32; i++) {
        Sensor_Update(motor->sensor, 0.001f);   // 这里只是为了刷新角度，dt给个正值即�?
        float a = Sensor_GetAngle(motor->sensor);

        sum_sin += sinf(a);
        sum_cos += cosf(a);

        HAL_Delay(2);
    }

    float mech_align = atan2f(sum_sin, sum_cos);
    mech_align = normalize_angle_0_2pi(mech_align);

    // 5. 根据当前设定的方向，计算 zero_electrical_angle
    float dir = 1.0f;
    if (motor->state.sensor_direction == sensor_direction_ccw) {
        dir = -1.0f;
    }

    float theta_field = normalize_angle_0_2pi(align_angle + 0.5f * PI);
    motor->zero_electrical_angle =
        normalize_angle_0_2pi(dir * motor->param.pole_pairs * mech_align - theta_field);

    // 6. 去掉输出
    Driver_SetPwm(motor->driver, 0.0f, 0.0f, 0.0f);

    return 1U;
}



