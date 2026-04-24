#include "./BLDCMotor.h"
#include "./driver.h"
#include "./current_sense.h"
#include "./platform.h"





//预留代码：实现功能：把一个驱动器对象的指针，挂到电机对象上
void linkDriver(Driver_t *driver, Motor_t *motor)
{
    if (!driver || !motor) return;

    motor->driver = driver;
}

//预留代码：实现功能：把一个传感器对象的指针，挂到电机对象上
void linkSensor(Sensor_t *sensor, Motor_t *motor)
{
    if (!motor) return;

    motor->sensor = sensor;
    motor->state.has_sensor = (sensor != NULL) ? 1U : 0U;
}



//预留代码，实现功能：电机硬件可用性检查 + 参数约束整理 + 进入可使能状态
/*
（1）检查 driver 是否已经连接并初始化：
     如果没有 driver，或者 driver->initialized 还没好，它直接报失败，电机状态置为 motor_init_failed，然后返回 0。
（2）更新 motor 状态为“初始化中”：
    把 motor_status 置成 motor_initializing。
（3）做电压限制的安全检查：
    它会检查：
    如果 motor.voltage_limit > driver.voltage_limit
    就把 motor 的限制压到 driver 的限制以内
    voltage_sensor_align 也不能超过 voltage_limit
（4）更新控制器内部的 limit
    它会调用：

    updateCurrentLimit(current_limit)
    updateVoltageLimit(voltage_limit)
    updateVelocityLimit(velocity_limit)

    也就是说，init() 不是只配硬件，
    还顺便把控制器里依赖 limit 的东西同步好。
（5）整理电机参数
    如果只给了单个相电感 phase_inductance，但没有单独给 d/q 轴电感，它就把 d/q 都设成这个值。

    这是个很典型的“初始化期参数整理”。    
（6）如果是开环而且没传感器，就给默认方向

    如果：

    没有 sensor
    控制模式是开环角度/开环速度
    方向还未知

    它就默认 sensor_direction = CW

    这一步本质上是在补齐运行前提。    
（7）延时后调用 enable()
    这个很关键：
    init() 最后会：
    延时
    enable()
    再延时
    把状态设成 motor_uncalibrated
*/ 
uint8_t FOCMotor_init(Motor_t *FOC_Motor)
{
    if (!FOC_Motor) {
        return 0;
    }




    // 检查 driver 是否已连接并初始化完成
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



    // 通过第一步检查后，进入初始化中状态
    FOC_Motor->state.motor_status = motor_initializing;

    // （3）电压限制安全检查：电机限制不能超过驱动限制
    if (FOC_Motor->config.voltage_limit > FOC_Motor->driver->voltage_limit) {
        FOC_Motor->config.voltage_limit = FOC_Motor->driver->voltage_limit;
    }
    if (FOC_Motor->config.voltage_sensor_align > FOC_Motor->config.voltage_limit) {
        FOC_Motor->config.voltage_sensor_align = FOC_Motor->config.voltage_limit;
    }
    //（4）更新控制器内部的 limit
    // 这里假设有全局函数可以更新 PID 的 limit，实际可能需要更复杂的结构设计

    //（5）整理电机参数：如果只给了单个相电感，就把 d/q 都设成这个值
    if (FOC_Motor->param.Ld == 0.0f && FOC_Motor->param.Lq != 0.0f) {
        FOC_Motor->param.Ld = FOC_Motor->param.Lq;
    } else if (FOC_Motor->param.Lq == 0.0f && FOC_Motor->param.Ld != 0.0f) {
        FOC_Motor->param.Lq = FOC_Motor->param.Ld;
    }

    //（6）开环且无传感器时，若方向未知则给默认方向
    if ((FOC_Motor->state.has_sensor == 0U) &&
        ((FOC_Motor->config.control_mode == motor_control_openloop_angle) ||
         (FOC_Motor->config.control_mode == motor_control_openloop_velocity)) &&
        (FOC_Motor->state.sensor_direction == sensor_direction_unknown)) {
        FOC_Motor->state.sensor_direction = sensor_direction_cw;
    }

    //（7）延时 -> 使能 -> 延时 -> 状态置为未校准
    Platform_DelayMs(10);
    FOCMotor_enable(FOC_Motor);
    Platform_DelayMs(10);
    FOC_Motor->state.motor_status = motor_uncalibrated;


    return 1;
}






//预留代码，实现功能：电机失能
/*
（1）如果有 current sense，就先 disable 掉
（2）把 PWM 输出清零
（3）再禁用 driver
（4）更新状态把 enabled = 0
“先去能量，再断执行链”
*/
void FOCMotor_disable(Motor_t *motor)
{
    if (!motor || !motor->driver) return;
    if (!motor->driver->initialized) return;

     //如果有 current sense，就先 disable 掉
    if (motor->current_sense) {
        CurrentSense_Disable(motor->current_sense);
    }

    //把 PWM 输出清零
    Driver_SetPwm(motor->driver, 0.0f, 0.0f, 0.0f);

    //再禁用 driver
    if (motor->driver) {
        Driver_Disable(motor->driver);
    }

    //更新状态把 enabled = 0
    motor->state.enabled = 0;
}


//预留代码，实现功能：电机使能
/*
（1）使能 driver
先调用：
driver->enable()
（2）立刻把 PWM 清零
调用：
driver->setPwm(0,0,0)
（3）如果有 current sense，就 enable
（4）重置控制器状态
它会 reset：
PID_velocity
P_angle
PID_current_q
PID_current_d
（5）更新 enabled 标志
把 enabled = 1

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


static float normalize_angle_0_2pi(float angle)
{
    float a = angle - (float)(int32_t)(angle * (1.0f / (2.0f * PI))) * (2.0f * PI);
    return (a >= 0.0f) ? a : (a + 2.0f * PI);
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


float Motor_GetElectricalAngle(Motor_t *motor)
{
    if (!motor || !motor->sensor) {
        return 0.0f;
    }

    if (!(motor->sensor->initialized)) {
        return 0.0f;
    }

    float mech_angle = Sensor_GetAngle(motor->sensor);

    float direction_sign = 1.0f;
    if (motor->state.sensor_direction == sensor_direction_ccw) {
        direction_sign = -1.0f;
    }

    float elec_angle = direction_sign * motor->param.pole_pairs * mech_angle
                     - motor->zero_electrical_angle;

    return normalize_angle_0_2pi(elec_angle);
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



static void BLDC_SetFVPWM(Motor_t *motor, float uq, float elec_angle)
{
    if (!motor || !motor->driver || !motor->driver->htim) {
        return;
    }
    float uq_limit = motor->driver->voltage_limit;
    if (uq_limit <= 0.0f || uq_limit > Uq_max) {
        uq_limit = Uq_max;
    }
    uq = constrain(uq, -uq_limit, uq_limit);

    /* 1) 由电角度计算 sin/cos */
    float st = sinf(elec_angle);
    float ct = cosf(elec_angle);

    /* 2) 反Park+Clarke (Ud=0) 得到三相相电压 */
    float Ualpha = -uq * st;
    float Ubeta  =  uq * ct;
    float t      = _SQRT3 * Ubeta;
    float Ua     = Ualpha;
    float Ub     = (-Ualpha + t) * 0.5f;
    float Uc     = (-Ualpha - t) * 0.5f;

    /* 3) SVPWM零序注入并映射到 [0, V_SUPPLY] */
    float Umax  = fmaxf(Ua, fmaxf(Ub, Uc));
    float Umin  = fminf(Ua, fminf(Ub, Uc));
    float Uzero = (V_SUPPLY * 0.5f) - (Umax + Umin) * 0.5f;

    Ua = constrain(Ua + Uzero, 0.0f, V_SUPPLY);
    Ub = constrain(Ub + Uzero, 0.0f, V_SUPPLY);
    Uc = constrain(Uc + Uzero, 0.0f, V_SUPPLY);

    /* 4) 电压转换为定时器比较值 */
    float pwm_max = (float)motor->driver->htim->Init.Period;
    float scale = pwm_max / V_SUPPLY;
    float ccr_a = Ua * scale + 0.5f;
    float ccr_b = Ub * scale + 0.5f;
    float ccr_c = Uc * scale + 0.5f;

    Driver_SetPwm(motor->driver, ccr_a, ccr_b, ccr_c);
}



static void Motor_ApplyAlignVector(Motor_t *motor, float uq, float elec_angle)
{
    BLDC_SetFVPWM(motor, uq, elec_angle);
}

void Motor_SetPhaseVoltageQ(Motor_t *motor, float uq, float elec_angle)
{
    BLDC_SetFVPWM(motor, uq, elec_angle);
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

    // 4. 多次采样机械角，做圆均值
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;

    for (uint16_t i = 0; i < 32; i++) {
        Sensor_Update(motor->sensor, 0.001f);   // 这里只是为了刷新角度，dt给个正值即可
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

    motor->zero_electrical_angle =
        normalize_angle_0_2pi(dir * motor->param.pole_pairs * mech_align - align_angle);

    // 6. 去掉输出
    Driver_SetPwm(motor->driver, 0.0f, 0.0f, 0.0f);

    return 1U;
}
