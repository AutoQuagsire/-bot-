#include "app_foc.h"
#include "main.h"
#include "AS5047P_RW.h"
#include "sensor.h"
#include "driver.h"
#include "BLDCMotor.h"

/* 这些外设句柄由 CubeMX 生成并在别处定义
 * 这里用 extern 引用，供应用层初始化时使用 */
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;

/* =========================
 * 应用层 FOC 对象
 * =========================
 * 这一层的对象不属于某个单独模块，而是“把各模块组起来”
 * 所以统一放在 app_foc.c 内部静态保存
 */
static AS5047P_Handle_t g_enc1;     // AS5047P 底层驱动句柄
static Sensor_t         g_sensor1;  // 传感器公共层对象
static Motor_t          g_motor1;   // 电机控制对象
static Driver_t        *g_driver1 = NULL; // 三相驱动对象（由 Driver 模块提供实例）

/**
 * @brief  FOC 应用层对象初始化
 * @retval 1: 初始化成功
 *         0: 初始化失败
 *
 * 初始化顺序说明：
 * 1. 先拿到 driver 实例，并初始化 PWM 输出
 * 2. 再初始化编码器底层驱动
 * 3. 然后把编码器挂到 Sensor 层，并初始化 Sensor
 * 4. 最后配置 Motor 参数，并把 Sensor / Driver 链接给 Motor
 *
 * 这个函数只负责“对象装配”，不负责校准动作
 */
uint8_t App_FOCStack_Init(void)
{
    /* 1) 获取左电机对应的 driver 实例 */
    g_driver1 = Driver_GetInstance(DRIVER_LEFT);
    if (g_driver1 == NULL) {
        USB_Debug_Printf("Driver_GetInstance failed\r\n");
        return 0U;
    }

    /* 2) 初始化三相驱动输出
     * 参数含义：
     * - htim1：用于输出 PWM 的定时器
     * - TIM_CHANNEL_1/3/4：三相对应的通道
     * - 19 * 0.577f：最大相电压限制（你当前的写法）
     */
    if (!Driver_Init(g_driver1,
                     &htim1,
                     TIM_CHANNEL_1,
                     TIM_CHANNEL_3,
                     TIM_CHANNEL_4,
                     19 * 0.577f)) {
        USB_Debug_Printf("Driver_Init failed\r\n");
        return 0U;
    }

    /* 3) 初始化 AS5047P 底层驱动
     * 这里只做 SPI 与芯片基本通信确认 */
    if (!AS5047P_RW_Init(&g_enc1, &hspi3, EcdL_CS_GPIO_Port, EcdL_CS_Pin)) {
        USB_Debug_Printf("AS5047P_RW_Init failed\r\n");
        return 0U;
    }

    /* 4) 将 AS5047P 设备句柄挂到 Sensor 公共层 */
    Sensor_LinkAS5047P(&g_enc1, &g_sensor1);

    /* 5) 初始化 Sensor 层
     * Sensor_Init 只做链路确认和初始角记录
     * 不在这里做真正的速度估计 */
    if (!Sensor_Init(&g_sensor1)) {
        USB_Debug_Printf("Sensor_Init failed\r\n");
        return 0U;
    }

    /* 6) 初始化电机参数
     * 当前参数：
     * - 14.0f: 极对数
     * - 10.3f: 电源电压/母线电压（按你当前设定）
     * - 后面几个暂时为 0，后续再逐步补充
     */
    MotorParam_Init(&g_motor1, 14.0f, 10.3f, 0.0f, 0.0f, 0.0f);

    /* 7) 初始化电机当前状态
     * - 零位电角度先置 0，后面通过校准函数写入真实值
     * - 传感器方向先人工指定，后面可再做自动判定或验证
     */
    g_motor1.zero_electrical_angle = 0.0f;
    g_motor1.state.sensor_direction = sensor_direction_cw;

    /* 8) 把 Sensor 和 Driver 链接给 Motor
     * 从这一步开始，Motor 才真正拥有“读角度”和“打PWM”的能力 */
    linkSensor(&g_sensor1, &g_motor1);
    linkDriver(g_driver1, &g_motor1);

    USB_Debug_Printf("FOC stack init ok\r\n");
    return 1U;
}

/**
 * @brief  上电后的启动校准流程
 * @retval 1: 校准成功
 *         0: 校准失败
 *
 * 当前这里只做“零位电角度校准”：
 * - 给一个固定电压矢量
 * - 让转子吸到指定电角位置
 * - 读取此时机械角
 * - 反算 zero_electrical_angle
 */
uint8_t App_StartupCalibrate(void)
{
    /* 参数说明：
     * - &g_motor1            : 要校准的电机对象
     * - 2.0f                 : 对齐电压
     * - 3.0f * PI / 2.0f     : 对齐目标电角度（3π/2）
     * - 300                  : 对齐稳定等待时间，单位 ms
     */
    if (!Motor_CalibrateZeroElectricalAngle(&g_motor1, 2.0f, 3.0f * PI / 2.0f, 300)) {
        USB_Debug_Printf("Startup calibrate failed\r\n");
        return 0U;
    }

    /* 打印校准结果，便于调试确认 */
    USB_Debug_Printf("zero_elec = %.6f\r\n", g_motor1.zero_electrical_angle);
    return 1U;
}

/**
 * @brief 主循环中的应用层逻辑
 *
 * 当前先留空，后面可以逐步加入：
 * 1. Sensor_Update()
 * 2. 电角度更新
 * 3. 调试打印
 * 4. 开环测试
 * 5. 正式 FOC 闭环
 */
void App_Loop(void)
{
    // 先留空，后续逐步加
}