

//预留代码：实现功能：把一个驱动器对象的指针，挂到电机对象上
void linkDriver()
{


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
void init()
{

}






uint8_t enabled;
//预留代码，实现功能：电机失能
/*
（1）如果有 current sense，就先 disable 掉
（2）把 PWM 输出清零
（3）再禁用 driver
（4）更新状态把 enabled = 0
“先去能量，再断执行链”
*/
void disable()
{
    CurrentSense_Disable(&current_sense);

    // driver 可能尚未 link，避免空指针访问
    if (!driver) {
        enabled = 0;
        return;
    }

    Driver_Disable(driver);

    enabled = 0;
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
void enable()
{

}