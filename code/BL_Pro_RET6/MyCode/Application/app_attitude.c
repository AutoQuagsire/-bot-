#include "app_attitude.h"

#include <math.h>
#include <string.h>

#include "INT.h"
#include "app_foc.h"
#include "stm32g4xx_hal.h"
#include "usb_debug.h"

/*
 * 姿态调试打印分频。
 *
 * 如果 IMU / Attitude 更新频率为 1kHz：
 * APP_ATTITUDE_PRINT_DIV = 20
 * 则打印频率约为 1000 / 20 = 50Hz。
 */
#if APP_ATTITUDE_USB_DEBUG_ENABLE
#define APP_ATTITUDE_PRINT_DIV 20U
#endif

/*
 * 姿态估计周期，单位：秒。
 *
 * 当前设置为 0.001s，对应 1kHz 更新频率。
 * 注意：这个 dt 必须和实际 Estimator_Update() 调用频率一致。
 * 如果后续改成 500Hz，则这里应改成 0.002f。
 */
#define APP_ATTITUDE_DT_SEC 0.001f

/*
 * First bring-up attitude loop parameters.
 *
 * Unit convention:
 *  - pitch: rad
 *  - pitch_rate: rad/s
 *  - iq_cmd: A
 */
#define APP_ATTITUDE_PITCH_RATE_TARGET_RADPS (0.0f)
#define APP_ATTITUDE_KP_A_PER_RAD            (9.6f)
#define APP_ATTITUDE_KD_A_PER_RADPS          (0.26f)
#define APP_ATTITUDE_IQ_LIMIT_A              (1.2f)
#define APP_ATTITUDE_SHUTDOWN_RAD            (2.0f) /* 30 deg */
#define APP_ATTITUDE_IQ_SIGN                 (1.0f)

#define APP_SPEED_TARGET_RADPS               (0.0f)
#define APP_SPEED_KP_RAD_PER_RADPS           (0.0006f)
#define APP_SPEED_KI_RAD_PER_RAD             (0.00f)
#define APP_SPEED_PITCH_LIMIT_RAD            (0.08f) /* 5 deg */
#define APP_SPEED_LOOP_SIGN                  (1.0f)


/* ICM42688 底层设备句柄 */
static ICM42688_Handle_t g_icm42688;

/* 姿态估计器实例，内部包含 IMU 链接信息和 Kalman 状态 */
static Attitude_Estimator_t g_estimator;

/*
 * 车体坐标系下的 IMU 物理量。
 *
 * 该变量在 SPI DMA 完成中断中更新，
 * 在主循环中被复制后用于姿态解算。
 */
IMU_PhysData_t g_phys_data;

/*
 * 姿态模块初始化完成标志。
 *
 * 用于防止初始化完成前 EXTI / DMA 中断误触发访问未初始化对象。
 */
static volatile uint8_t g_attitude_init_ready = 0U;
static volatile uint8_t g_attitude_control_enabled = 0U;
static float g_speed_loop_integral = 0.0f;
static volatile App_AttitudeTelemetry_t g_attitude_telemetry;

static float App_Attitude_ClampFloat(float value, float min_value, float max_value);
static float App_Attitude_CalculatePitchTarget(float wheel_speed_radps,
                                               float dt,
                                               float *pitch_target_p,
                                               float *pitch_target_i);
static float App_Attitude_CalculateIqCommand(float pitch_target_rad,
                                             float pitch_rad,
                                             float pitch_rate_radps,
                                             float *iq_cmd_p,
                                             float *iq_cmd_d,
                                             float *iq_cmd_clamped);

/**
 * @brief  姿态模块初始化
 *
 * 初始化内容：
 *  1. 清零 ICM42688 句柄、姿态估计器、物理量缓存
 *  2. 将 ICM42688 设备链接到姿态估计器
 *  3. 初始化 IMU 硬件
 *  4. 初始化 Kalman 姿态估计器
 *
 * @return IMU_OK 初始化成功；IMU_ERROR 初始化失败
 */
uint8_t App_Attitude_Init(void)
{
    memset(&g_icm42688, 0, sizeof(g_icm42688));
    memset(&g_estimator, 0, sizeof(g_estimator));
    memset(&g_phys_data, 0, sizeof(g_phys_data));
    memset((void *)&g_attitude_telemetry, 0, sizeof(g_attitude_telemetry));
    g_attitude_init_ready = 0U;
    g_attitude_control_enabled = 0U;
    g_speed_loop_integral = 0.0f;

    /* 将具体 IMU 设备实例绑定到 estimator */
    Estimator_LinkICM42688P(&g_icm42688, &g_estimator);

    /* 初始化 IMU 硬件：SPI、CS、ICM42688 寄存器配置等 */
    if (IMU_Init(&g_estimator) != IMU_OK) {
        return IMU_ERROR;
    }

    #ifdef USE_KALMAN_ESTIMATE
        /* 初始化姿态估计器内部的一维 Kalman 滤波器 */
        if (Estimator_Init(&g_estimator) != ESTIMATOR_OK) {
            return IMU_ERROR;
        }
    #endif

    /* 初始化 PID 控制器 */
    g_attitude_init_ready = 1U;
    return IMU_OK;
}

uint8_t App_Attitude_SetControlEnabled(uint8_t enable)
{
    if (enable > 1U) {
        return 0U;
    }

    if ((enable != 0U) && (App_FOC_IsPowerStageEnabled() == 0U)) {
        return 0U;
    }

    __disable_irq();
    g_attitude_control_enabled = enable;
    g_speed_loop_integral = 0.0f;
    __enable_irq();

    if (enable == 0U) {
        __disable_irq();
        g_attitude_telemetry.pitch_target_rad = 0.0f;
        g_attitude_telemetry.speed_p_term_rad = 0.0f;
        g_attitude_telemetry.speed_i_term_rad = 0.0f;
        g_attitude_telemetry.speed_target_radps = APP_SPEED_TARGET_RADPS;
        g_attitude_telemetry.speed_meas_radps = 0.0f;
        g_attitude_telemetry.attitude_p_term_a = 0.0f;
        g_attitude_telemetry.attitude_d_term_a = 0.0f;
        g_attitude_telemetry.iq_cmd_a = 0.0f;
        g_attitude_telemetry.iq_cmd_clamped_a = 0.0f;
        g_attitude_telemetry.speed_output_limit_rad = APP_SPEED_PITCH_LIMIT_RAD;
        g_attitude_telemetry.attitude_output_limit_a = APP_ATTITUDE_IQ_LIMIT_A;
        __enable_irq();
        App_FOC_SetIqTarget(0.0f, 0.0f);
    }

    return 1U;
}



















/**
 * @brief  IMU DRDY 外部中断回调入口
 *
 * ICM42688 数据准备好后触发 DRDY 中断。
 * 这里不直接读取数据，只启动一次 SPI DMA 读取。
 *
 * 数据流：
 * DRDY EXTI ISR
 *      -> IMU_Update()
 *      -> ICM42688_StartReadRawDataDMA()
 *      -> SPI DMA 完成中断
 *      -> App_Attitude_OnSpi2DmaCpltISR()
 */
void App_Attitude_OnDrdyExtiISR(void)
{
    if (g_attitude_init_ready == 0U) {
        return;
    }

    /* 发起一次 SPI DMA 读原始数据，不阻塞等待 */
    (void)IMU_Update(&g_estimator);
}


/**
 * @brief  SPI2 DMA 收发完成中断回调入口
 *
 * 该函数在 SPI DMA 读取 IMU 数据完成后调用。
 *
 * 主要工作：
 *  1. 解析 DMA 接收帧，得到 IMU 原始数据
 *  2. 将原始数据转换为物理量
 *  3. 将传感器坐标系映射到车体 FRD 坐标系
 *  4. 置位 IMU_DRDY_Flag，通知主循环可以进行姿态解算
 *
 * 注意：
 * 这里处于中断上下文，不做 Kalman 解算、不做 USB 打印。
 */
void App_Attitude_OnSpi2DmaCpltISR(void)
{
    IMU_PhysData_t imu_phys;

    if (g_attitude_init_ready == 0U) {
        return;
    }

    /* 处理 DMA 完成后的原始数据帧 */
    if (IMU_OnSpiTxRxCpltISR(&g_estimator) != IMU_OK) {
        return;
    }

    /* 原始 int16 数据转换为物理量：accel=m/s^2，gyro=rad/s */
    if (IMU_ConvertRawToPhysical(&g_estimator, &imu_phys) != IMU_OK) {
        return;
    }

    /*
     * 传感器坐标系 -> 车体 FRD 坐标系。
     *
     * 更新后的 g_phys_data 会在主循环中被复制，
     * 再送入 Estimator_Update() 进行姿态解算。
     */
    Estimator_MapPhysicalToBodyFrame(&imu_phys, &g_phys_data);

    /* 通知主循环：已有一帧新的 IMU 数据可以解算 */
    IMU_DRDY_Flag = 1U;
}


/**
 * @brief  SPI2 DMA 错误中断回调入口
 *
 * 用于通知底层 IMU 驱动 SPI / DMA 发生错误，
 * 释放 busy 状态，避免后续 DMA 读被卡死。
 */
void App_Attitude_OnSpi2DmaErrorISR(void)
{
    IMU_OnSpiErrorISR(&g_estimator);
}














/**
 * @brief  姿态主循环任务
 *
 * 该函数应在 while(1) 中高频调用。
 *
 * 工作流程：
 *  1. 检查 IMU_DRDY_Flag 是否置位
 *  2. 复制一份最新 IMU 物理量快照
 *  3. 调用 Estimator_Update() 进行 Kalman 姿态解算
 *  4. 按分频进行 USB 调试打印
 *
 * 注意：
 *  - Kalman 解算放在主循环，不放在 DMA ISR
 *  - dt 必须等于实际姿态解算周期
 */
void App_Attitude_Loop(void)
{
#if APP_ATTITUDE_USB_DEBUG_ENABLE
    static uint8_t print_div = 0U;
#endif
    IMU_PhysData_t phys_data;

    /*
     * 原子化获取一帧 IMU 数据。
     *
     * 这里关闭中断的时间很短，只做：
     *  1. 检查 flag
     *  2. 清 flag
     *  3. 复制 g_phys_data
     *
     * 这样可以避免主循环读取 g_phys_data 时，
     * DMA 完成中断刚好更新该结构体。
     */
    __disable_irq();

    if (IMU_DRDY_Flag == 0U) {
        __enable_irq();
        return;
    }

    IMU_DRDY_Flag = 0U;
    phys_data = g_phys_data;

    __enable_irq();
    /*
     * LED 拉高用于示波器/逻辑分析仪观察姿态解算耗时。
     * 高电平宽度约等于 Estimator_Update() 执行时间。
     */
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    /*
     * 姿态估计更新。
     *
     * 输入：
     *  - phys_data：车体 FRD 坐标系下的 IMU 物理量
     *  - APP_ATTITUDE_DT_SEC：姿态解算周期，单位秒
     */
    static uint8_t SpeedPID_Count = 0U;
    static float pitch_target_rad = 0.0f;
    static float wheel_speed_radps = 0.0f;
    float speed_meas_radps = 0.0f;
    float speed_error_radps;
    float speed_i_integral_term_rad;
    float pitch_target_cmd_rad;
    float pitch_meas_rad;
    float pitch_rate_meas_radps;
    float speed_p_term_rad;
    float speed_i_term_rad;
    float attitude_p_term_a;
    float attitude_d_term_a;
    float iq_cmd_a;
    float iq_cmd_clamped_a;

    Estimator_Update(&g_estimator, &phys_data, APP_ATTITUDE_DT_SEC);

    pitch_meas_rad = Estimator_GetPitch(&g_estimator);
    pitch_rate_meas_radps = Estimator_GetPitchRate(&g_estimator);

    if (g_attitude_control_enabled == 0U) {
        SpeedPID_Count = 0U;
        pitch_target_rad = 0.0f;
        wheel_speed_radps = App_FOC_GetAverageWheelSpeedRadps();
        g_speed_loop_integral = 0.0f;
        __disable_irq();
        g_attitude_telemetry.pitch_target_rad = 0.0f;
        g_attitude_telemetry.speed_p_term_rad = 0.0f;
        g_attitude_telemetry.speed_i_term_rad = 0.0f;
        g_attitude_telemetry.pitch_meas_rad = pitch_meas_rad;
        g_attitude_telemetry.pitch_rate_meas_radps = pitch_rate_meas_radps;
        g_attitude_telemetry.speed_target_radps = APP_SPEED_TARGET_RADPS;
        g_attitude_telemetry.speed_meas_radps = -wheel_speed_radps;
        g_attitude_telemetry.attitude_p_term_a = 0.0f;
        g_attitude_telemetry.attitude_d_term_a = 0.0f;
        g_attitude_telemetry.iq_cmd_a = 0.0f;
        g_attitude_telemetry.iq_cmd_clamped_a = 0.0f;
        g_attitude_telemetry.speed_output_limit_rad = APP_SPEED_PITCH_LIMIT_RAD;
        g_attitude_telemetry.attitude_output_limit_a = APP_ATTITUDE_IQ_LIMIT_A;
        __enable_irq();
        App_FOC_SetIqTarget(0.0f, 0.0f);
        return;
    }

    if (SpeedPID_Count++ >= 10U) 
    {
        SpeedPID_Count = 0U;
        wheel_speed_radps = App_FOC_GetAverageWheelSpeedRadps();
        speed_meas_radps = -wheel_speed_radps;
        pitch_target_rad = App_Attitude_CalculatePitchTarget(speed_meas_radps,
                                                             APP_ATTITUDE_DT_SEC * 10.0f,
                                                             &speed_p_term_rad,
                                                             &speed_i_term_rad);

    } else {
        speed_meas_radps = -wheel_speed_radps;
    }

    speed_error_radps = APP_SPEED_TARGET_RADPS - speed_meas_radps;
    speed_i_integral_term_rad = APP_SPEED_KI_RAD_PER_RAD * g_speed_loop_integral;
    if (SpeedPID_Count != 0U) {
        speed_p_term_rad = APP_SPEED_KP_RAD_PER_RADPS * speed_error_radps;
        speed_i_term_rad = speed_i_integral_term_rad;
    }
    pitch_target_cmd_rad = pitch_target_rad + 0.0f;

    iq_cmd_a = App_Attitude_CalculateIqCommand(pitch_target_cmd_rad,
                                               pitch_meas_rad,
                                               pitch_rate_meas_radps,
                                               &attitude_p_term_a,
                                               &attitude_d_term_a,
                                               &iq_cmd_clamped_a);
    __disable_irq();
    g_attitude_telemetry.pitch_target_rad = pitch_target_cmd_rad;
    g_attitude_telemetry.speed_p_term_rad = speed_p_term_rad;
    g_attitude_telemetry.speed_i_term_rad = speed_i_term_rad;
    g_attitude_telemetry.pitch_meas_rad = pitch_meas_rad;
    g_attitude_telemetry.pitch_rate_meas_radps = pitch_rate_meas_radps;
    g_attitude_telemetry.speed_target_radps = APP_SPEED_TARGET_RADPS;
    g_attitude_telemetry.speed_meas_radps = speed_meas_radps;
    g_attitude_telemetry.attitude_p_term_a = attitude_p_term_a;
    g_attitude_telemetry.attitude_d_term_a = attitude_d_term_a;
    g_attitude_telemetry.iq_cmd_a = iq_cmd_a;
    g_attitude_telemetry.iq_cmd_clamped_a = iq_cmd_clamped_a;
    g_attitude_telemetry.speed_output_limit_rad = APP_SPEED_PITCH_LIMIT_RAD;
    g_attitude_telemetry.attitude_output_limit_a = APP_ATTITUDE_IQ_LIMIT_A;
    __enable_irq();
    App_FOC_SetIqTarget(iq_cmd_clamped_a, -iq_cmd_clamped_a);



    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    /*
     * 调试打印分频。
     *
     * 姿态解算可以是 1kHz，
     * 但 USB 打印不能这么高，否则会严重影响实时性。
     */
#if APP_ATTITUDE_USB_DEBUG_ENABLE
    print_div++;
    if (print_div < APP_ATTITUDE_PRINT_DIV) {
        return;
    }
    print_div = 0U;

    /*
     * 打印格式：
     * time_ms,
     * kalman_pitch_deg,
     * kalman_pitch_rad,
     * accel_pitch_rad,
     * gyro_y_radps,
     * estimated_bias_radps,
     * residual_rad,
     * K0,
     * K1,
     * acc_norm
     */
    USB_Debug_Printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                     (unsigned long)HAL_GetTick(),
                     pitch_target_rad,
                     pitch_target_cmd_rad,
                     pitch_meas_rad,
                     pitch_rate_meas_radps,
                     wheel_speed_radps,
                     speed_error_radps,
                     g_speed_loop_integral,
                     speed_i_integral_term_rad,
                     iq_cmd_clamped_a,
                     g_estimator.kalman_pitch.acc_norm);
#endif
}







float App_Attitude_GetPitch()
{
    return Estimator_GetPitch(&g_estimator);
}

float  App_Attitude_GetPitchRate(void)
{   
    return Estimator_GetPitchRate(&g_estimator);
}

void App_Attitude_GetTelemetry(App_AttitudeTelemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }

    __disable_irq();
    *telemetry = g_attitude_telemetry;
    __enable_irq();
}

uint8_t App_Attitude_IsReady(void)
{
    return g_attitude_init_ready;
}

uint8_t App_Attitude_IsControlEnabled(void)
{
    return g_attitude_control_enabled;
}

static float App_Attitude_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static float App_Attitude_CalculatePitchTarget(float wheel_speed_radps,
                                               float dt,
                                               float *pitch_target_p,
                                               float *pitch_target_i)
{
    float speed_error = APP_SPEED_TARGET_RADPS - wheel_speed_radps;
    float pitch_target;
    float speed_i_term = 0.0f;
    float pitch_target_p_term;

    if (APP_SPEED_KI_RAD_PER_RAD != 0.0f) {
        g_speed_loop_integral += speed_error * dt;

        speed_i_term = APP_SPEED_KI_RAD_PER_RAD * g_speed_loop_integral;
        speed_i_term = App_Attitude_ClampFloat(speed_i_term,
                                               -APP_SPEED_PITCH_LIMIT_RAD,
                                               APP_SPEED_PITCH_LIMIT_RAD);

    } else {
        g_speed_loop_integral = 0.0f;
    }

    pitch_target_p_term = APP_SPEED_KP_RAD_PER_RADPS * speed_error;
    pitch_target = pitch_target_p_term + speed_i_term;
    pitch_target *= APP_SPEED_LOOP_SIGN;

    if (pitch_target_p != NULL) {
        *pitch_target_p = pitch_target_p_term * APP_SPEED_LOOP_SIGN;
    }
    if (pitch_target_i != NULL) {
        *pitch_target_i = speed_i_term * APP_SPEED_LOOP_SIGN;
    }

    return App_Attitude_ClampFloat(pitch_target,
                                   -APP_SPEED_PITCH_LIMIT_RAD,
                                   APP_SPEED_PITCH_LIMIT_RAD);
}

static float App_Attitude_CalculateIqCommand(float pitch_target_rad,
                                             float pitch_rad,
                                             float pitch_rate_radps,
                                             float *iq_cmd_p,
                                             float *iq_cmd_d,
                                             float *iq_cmd_clamped)
{
    float pitch_error = pitch_target_rad - pitch_rad;
    float pitch_rate_error = APP_ATTITUDE_PITCH_RATE_TARGET_RADPS - pitch_rate_radps;
    float iq_cmd;
    float iq_cmd_limited;
    float iq_cmd_p_term;
    float iq_cmd_d_term;

    if (fabsf(pitch_rad) > APP_ATTITUDE_SHUTDOWN_RAD) {
        g_speed_loop_integral = 0.0f;
        if (iq_cmd_p != NULL) {
            *iq_cmd_p = 0.0f;
        }
        if (iq_cmd_d != NULL) {
            *iq_cmd_d = 0.0f;
        }
        if (iq_cmd_clamped != NULL) {
            *iq_cmd_clamped = 0.0f;
        }
        return 0.0f;
    }

    iq_cmd_p_term = APP_ATTITUDE_KP_A_PER_RAD * pitch_error;
    iq_cmd_d_term = APP_ATTITUDE_KD_A_PER_RADPS * pitch_rate_error;
    iq_cmd = iq_cmd_p_term + iq_cmd_d_term;
    iq_cmd *= APP_ATTITUDE_IQ_SIGN;
    iq_cmd_p_term *= APP_ATTITUDE_IQ_SIGN;
    iq_cmd_d_term *= APP_ATTITUDE_IQ_SIGN;

    iq_cmd_limited = App_Attitude_ClampFloat(iq_cmd,
                                             -APP_ATTITUDE_IQ_LIMIT_A,
                                             APP_ATTITUDE_IQ_LIMIT_A);
    if (iq_cmd_p != NULL) {
        *iq_cmd_p = iq_cmd_p_term;
    }
    if (iq_cmd_d != NULL) {
        *iq_cmd_d = iq_cmd_d_term;
    }
    if (iq_cmd_clamped != NULL) {
        *iq_cmd_clamped = iq_cmd_limited;
    }

    return iq_cmd;
}
