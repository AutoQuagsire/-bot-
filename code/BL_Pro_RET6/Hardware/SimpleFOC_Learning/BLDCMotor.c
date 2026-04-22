#include "./BLDCMotor.h"
#include "./driver.h"
#include "./current_sense.h"
#include "stm32g4xx_hal.h"





//预留代码：实现功能：把一个驱动器对象的指针，挂到电机对象上
void linkDriver(Driver_t *driver, Motor_t *motor)
{
    if (!driver || !motor) return;

    motor->driver = driver;
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
    HAL_Delay(10);
    FOCMotor_enable(FOC_Motor);
    HAL_Delay(10);
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
void FOCMotor_disable(Motor_t *FOC_Motor)
{
    if (!FOC_Motor) return;

     //如果有 current sense，就先 disable 掉
    if (FOC_Motor->current_sense) {
        CurrentSense_Disable(FOC_Motor->current_sense);
    }

    //把 PWM 输出清零
    Driver_SetPwm(FOC_Motor->driver, 0.0f, 0.0f, 0.0f);

    //再禁用 driver
    if (FOC_Motor->driver) {
        Driver_Disable(FOC_Motor->driver);
    }

    //更新状态把 enabled = 0
    FOC_Motor->state.enabled = 0;
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