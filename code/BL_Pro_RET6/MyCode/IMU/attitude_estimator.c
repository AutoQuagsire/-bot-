#include "attitude_estimator.h"
#include "kalman_1d.h"
#include "main.h"
#include "usb_debug.h"


/**
 * @brief  将姿态估计器与具体 IMU 设备实例绑定
 *
 * 这里只做“链接”动作，不进行硬件初始化。
 * estimator 通过 imu_dev 指针持有底层 IMU 设备句柄，
 * 后续 IMU_Init / IMU_Update 会根据 imu_type 调用对应驱动。
 */
void Estimator_LinkICM42688P(ICM42688_Handle_t *dev, Attitude_Estimator_t *estimator)
{
    if (dev == NULL || estimator == NULL) {
        return;
    }

    estimator->imu_dev = dev;
    estimator->imu_type = IMU_MODULE_ID_ICM42688;
}


/**
 * @brief  初始化姿态估计器
 *
 * 当前版本主要初始化 pitch 方向的一维 Kalman 滤波器。
 * 初始角度和 gyro bias 暂时设为 0。
 *
 * 后续如果加入启动静态标定，可以把：
 *  - init_angle_rad 设置为静止时 accel 计算得到的 pitch
 *  - init_bias_radps 设置为静止时 gyro 均值
 */
uint8_t Estimator_Init(Attitude_Estimator_t *estimator)
{
    if (estimator == NULL) {
        return ESTIMATOR_ERROR;
    }

#ifdef USE_KALMAN_ESTIMATE
    Kalman1D_Init(&estimator->kalman_pitch, 0.0f, 0.0f);
#endif

    return ESTIMATOR_OK;
}


/**
 * @brief  初始化 IMU 硬件
 *
 * 根据 estimator->imu_type 选择对应的 IMU 驱动初始化函数。
 * 当前支持 ICM42688。
 *
 * 注意：
 * 这里会真正初始化 SPI 通信、CS 引脚、ICM42688 内部寄存器配置等。
 */
uint8_t IMU_Init(Attitude_Estimator_t *estimator)
{
    uint8_t state = IMU_ERROR;

    if (estimator == NULL || estimator->imu_dev == NULL) {
        return IMU_ERROR;
    }

    switch (estimator->imu_type) {
    case IMU_MODULE_ID_ICM42688:
        state = (ICM42688_Init((ICM42688_Handle_t *)estimator->imu_dev,
                               &hspi2,
                               IMU_CS_GPIO_Port,
                               IMU_CS_Pin) == ICM42688_OK) ? IMU_OK : IMU_ERROR;
        break;

    default:
        return IMU_ERROR;
    }

    USB_Debug_Printf("IMU_State:%u\r\n", state);
    return state;
}


/**
 * @brief  启动一次 IMU DMA 读取
 *
 * 该函数只负责“发起 SPI DMA 读数据”，不在这里等待数据返回。
 * 数据读取完成后，应由 SPI DMA 完成中断调用 IMU_OnSpiTxRxCpltISR() 处理。
 *
 * 推荐调用位置：
 *  - 500Hz 姿态任务
 *  - 或 DRDY 中断触发后启动 DMA
 */
uint8_t IMU_Update(Attitude_Estimator_t *estimator)
{
    if (estimator == NULL || estimator->imu_dev == NULL) {
        return IMU_ERROR;
    }

    switch (estimator->imu_type) {
    case IMU_MODULE_ID_ICM42688:
        return (ICM42688_StartReadRawDataDMA((ICM42688_Handle_t *)estimator->imu_dev) == ICM42688_OK)
                   ? IMU_OK
                   : IMU_ERROR;

    default:
        return IMU_ERROR;
    }
}


/**
 * @brief  SPI DMA 收发完成中断回调入口
 *
 * 该函数应在 HAL_SPI_TxRxCpltCallback() 中被调用。
 *
 * 作用：
 *  - 解析 DMA 接收到的原始数据帧
 *  - 更新 estimator->raw_data
 *  - 底层驱动内部可在此处释放 busy 标志
 *
 * 注意：
 * ISR 内不建议做 Kalman 解算、打印等耗时操作。
 * 推荐这里只做数据搬运和置位，姿态解算放到 while/scheduler 里执行。
 */
uint8_t IMU_OnSpiTxRxCpltISR(Attitude_Estimator_t *estimator)
{
    if (estimator == NULL || estimator->imu_dev == NULL) {
        return IMU_ERROR;
    }

    switch (estimator->imu_type) {
    case IMU_MODULE_ID_ICM42688:
        return (ICM42688_OnSpiTxRxCpltISR((ICM42688_Handle_t *)estimator->imu_dev,
                                          &estimator->raw_data) == ICM42688_OK)
                   ? IMU_OK
                   : IMU_ERROR;

    default:
        return IMU_ERROR;
    }
}


/**
 * @brief  SPI DMA 错误中断处理入口
 *
 * 该函数应在 HAL_SPI_ErrorCallback() 中被调用。
 * 主要用于通知底层 IMU 驱动 SPI 出错，并释放 DMA/SPI busy 状态。
 */
void IMU_OnSpiErrorISR(Attitude_Estimator_t *estimator)
{
    if (estimator == NULL || estimator->imu_dev == NULL) {
        return;
    }

    switch (estimator->imu_type) {
    case IMU_MODULE_ID_ICM42688:
        ICM42688_OnSpiErrorISR((ICM42688_Handle_t *)estimator->imu_dev);
        break;

    default:
        break;
    }
}


/**
 * @brief  将 IMU 原始数据转换为物理量
 *
 * 输入：
 *  - estimator->raw_data：DMA 读取并解析后的原始 int16 数据
 *
 * 输出：
 *  - phys：物理量数据
 *      accel: m/s^2
 *      gyro : rad/s
 *
 * 注意：
 * 该函数只完成“传感器坐标系下”的单位换算，
 * 不负责映射到车体坐标系。
 */
uint8_t IMU_ConvertRawToPhysical(Attitude_Estimator_t *estimator, IMU_PhysData_t *phys)
{
    if (estimator == NULL || estimator->imu_dev == NULL || phys == NULL) {
        return IMU_ERROR;
    }

    switch (estimator->imu_type) {
    case IMU_MODULE_ID_ICM42688:
        ICM42688_ConvertRawToPhysical(&estimator->raw_data, phys);
        return IMU_OK;

    default:
        return IMU_ERROR;
    }
}


/**
 * @brief  将 IMU 传感器坐标系映射到车体 FRD 坐标系
 *
 * FRD 坐标系定义：
 *  - X: Forward，车体前方为正
 *  - Y: Right，车体右方为正
 *  - Z: Down，车体下方为正
 *
 * 当前映射关系：
 *  accel:
 *      ax_body = -ay_sensor
 *      ay_body =  ax_sensor
 *      az_body = -az_sensor
 *
 *  gyro:
 *      gx_body = -gy_sensor
 *      gy_body =  gx_sensor
 *      gz_body = -gz_sensor
 *
 * 其中 gy_body 被定义为 pitch 角速度，
 * 需要满足“前倾为正”的工程约定。
 */
void Estimator_MapPhysicalToBodyFrame(const IMU_PhysData_t *imu_phys, IMU_PhysData_t *body_phys)
{
    if (imu_phys == NULL || body_phys == NULL) {
        return;
    }

    /* Sensor frame -> FRD body frame mapping used by estimator. */
    body_phys->ax_mps2 = -imu_phys->ay_mps2;
    body_phys->ay_mps2 = imu_phys->ax_mps2;
    body_phys->az_mps2 = -imu_phys->az_mps2;

    body_phys->gx_radps = -imu_phys->gy_radps;
    body_phys->gy_radps = imu_phys->gx_radps;
    body_phys->gz_radps = -imu_phys->gz_radps;
}


/**
 * @brief  姿态估计更新
 *
 * 输入：
 *  - phys_data：已经映射到车体 FRD 坐标系下的 IMU 物理量
 *      accel: m/s^2
 *      gyro : rad/s
 *  - dt：姿态估计周期，单位为秒
 *
 * 当前实现：
 *  - 使用一维 Kalman 滤波估计 pitch
 *  - 输入为 accel 计算得到的 pitch_acc 和 gyro_y
 *
 * 注意：
 * dt 必须是真实更新周期，单位必须是秒。
 * 例如 500Hz 更新时 dt = 0.002f。
 */
uint8_t Estimator_Update(Attitude_Estimator_t *estimator, const IMU_PhysData_t *phys_data, float dt)
{
    if (estimator == NULL || phys_data == NULL) {
        return ESTIMATOR_ERROR;
    }

#ifdef USE_KALMAN_ESTIMATE
    Kalman1D_Update(&estimator->kalman_pitch, *phys_data, dt);
#endif

    return ESTIMATOR_OK;
}

float Estimator_GetPitch(const Attitude_Estimator_t *estimator)
{
    if (estimator == NULL) {
        return 0.0f;
    }

#ifdef USE_KALMAN_ESTIMATE
    return estimator->kalman_pitch.angle_rad;
#else
    return 0.0f;
#endif
}

float Estimator_GetPitchRate(const Attitude_Estimator_t *estimator)
{
    if (estimator == NULL) {
        return 0.0f;
    }

#ifdef USE_KALMAN_ESTIMATE
    return estimator->kalman_pitch.rate_radps;
#else
    return 0.0f;
#endif
}


/**
 * @brief  获取最近一次 IMU 原始数据
 *
 * 用于调试打印、日志记录或上层读取原始传感器值。
 *
 * 返回：
 *  - 指向 estimator->raw_data 的只读指针
 */
const IMU_RawData_t *Estimator_GetRawData(const Attitude_Estimator_t *estimator)
{
    if (estimator == NULL) {
        return NULL;
    }

    return &estimator->raw_data;
}
