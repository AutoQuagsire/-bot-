#include "app_attitude.h"

#include <string.h>

#include "INT.h"
#include "stm32g4xx_hal.h"
#include "usb_debug.h"

/*
 * 姿态调试打印分频。
 *
 * 如果 IMU / Attitude 更新频率为 1kHz：
 * APP_ATTITUDE_PRINT_DIV = 20
 * 则打印频率约为 1000 / 20 = 50Hz。
 */
#define APP_ATTITUDE_PRINT_DIV 20U

/*
 * 姿态估计周期，单位：秒。
 *
 * 当前设置为 0.001s，对应 1kHz 更新频率。
 * 注意：这个 dt 必须和实际 Estimator_Update() 调用频率一致。
 * 如果后续改成 500Hz，则这里应改成 0.002f。
 */
#define APP_ATTITUDE_DT_SEC 0.001f


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
    g_attitude_init_ready = 0U;

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

    g_attitude_init_ready = 1U;
    return IMU_OK;
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
    static uint8_t print_div = 0U;
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
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    /*
     * 姿态估计更新。
     *
     * 输入：
     *  - phys_data：车体 FRD 坐标系下的 IMU 物理量
     *  - APP_ATTITUDE_DT_SEC：姿态解算周期，单位秒
     */
    (void)Estimator_Update(&g_estimator, &phys_data, APP_ATTITUDE_DT_SEC);

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    /*
     * 调试打印分频。
     *
     * 姿态解算可以是 1kHz，
     * 但 USB 打印不能这么高，否则会严重影响实时性。
     */
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
    USB_Debug_Printf("%lu,%.3f,%.4f,%.4f,%f,%.9f,%.9f,%.9f,%.9f,%f\r\n",
                     (unsigned long)HAL_GetTick(),
                     g_estimator.kalman_pitch.angle_degree,
                     g_estimator.kalman_pitch.angle_rad,
                     g_estimator.kalman_pitch.accel_pitch_rad,
                     phys_data.gy_radps,
                     g_estimator.kalman_pitch.bias_radps,
                     g_estimator.kalman_pitch.residual,
                     g_estimator.kalman_pitch.K0,
                     g_estimator.kalman_pitch.K1,
                     g_estimator.kalman_pitch.acc_norm);
}