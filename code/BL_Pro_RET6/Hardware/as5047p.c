/**
 ******************************************************************************
 * @file    as5047p.c
 * @brief   AS5047P 14-bit 磁编码器驱动库实现（精简版）
 *
 * 设计原则
 * --------
 *  - spi_transfer：直接寄存器访问 + DWT 硬件超时（约 10 µs）
 *    * DWT->CYCCNT 由 CPU 时钟驱动，不依赖 SysTick，在任意优先级 ISR 内有效
 *    * 不使用 HAL_SPI_TransmitReceive：其超时依赖 HAL_GetTick()，
 *      在 TIM5 ISR（优先级 0）内 SysTick（优先级 15）被屏蔽，永远不递增，
 *      SPI 硬件故障时会轮询死循环导致 ISR 卡死
 *  - tCSH_delay：每次 spi_transfer 入口调用，保证 tCSH/tL ≥ 350 ns
 *  - pipeline_read：AS5047P 流水线模式（手册 Figure 15），需两次事务
 *  - 失败时读一次 ERRFL 清除粘性错误位，不做重试/重初始化
 ******************************************************************************
 */

#include "as5047p.h"
#include "sys.h"

/* ── 内部工具 ────────────────────────────────────────────────────── */

static inline void cs_low(AS5047P_Handle *dev)
{
    Pin_L(dev->cs_port, dev->cs_pin);
}

static inline void cs_high(AS5047P_Handle *dev)
{
    Pin_H(dev->cs_port, dev->cs_pin);
}

/**
 * tCSH ≥ 350 ns（手册 tCSH/tL 要求）
 * 60 NOP @170 MHz = 353 ns；展开避免 volatile 循环的栈读写开销（原写法实际约 1.2 us）
 */
__attribute__((optimize("O2")))
static inline void tCSH_delay(void)
{
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/**
 * 传输前恢复 SPI 到可用状态：
 *  - 清 RX FIFO/OVR/FRE 残留
 *  - MODF 后重新使能 SPE
 *  - 确保 SPE 已开启
 */
__attribute__((optimize("O2")))
static inline void spi_recover(AS5047P_Handle *dev)
{
    SPI_TypeDef *spi = dev->hspi->Instance;

    while ((spi->SR & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) { (void)spi->DR; }

    if (spi->SR & SPI_SR_OVR) { (void)spi->DR; (void)spi->SR; }
    if (spi->SR & SPI_SR_FRE) { (void)spi->SR; }

    if (spi->SR & SPI_SR_MODF)
    {
        (void)spi->SR;
        CLEAR_BIT(spi->CR1, SPI_CR1_SPE);
        SET_BIT(spi->CR1, SPI_CR1_SPE);
    }

    if ((spi->CR1 & SPI_CR1_SPE) == 0U) { SET_BIT(spi->CR1, SPI_CR1_SPE); }
}

/** 校验帧 bit15（偶校验覆盖 bit[14:0]） */
__attribute__((optimize("O2")))
static uint8_t frame_parity_ok(uint16_t frame)
{
    return (uint8_t)__builtin_parity(frame & 0x7FFFU) == (uint8_t)((frame >> 15U) & 1U);
}

/**
 * 单次 16-bit SPI 全双工事务（直接寄存器访问 + DWT 超时）
 *
 * 超时设为 30 µs（@170 MHz = 5100 ticks）：
 *   SPI 16 bit @10 MHz 实际传输 ≈ 1.6 µs，保留更大容错用于 ISR 抖动与异常恢复。
 *   注意：不可设为 1 s——ISR 内 1 s 阻塞会彻底冻结 10 kHz 控制环。
 *

 */
 __attribute__((optimize("O2")))
static HAL_StatusTypeDef spi_transfer(AS5047P_Handle *dev,
                                      uint16_t tx, uint16_t *rx)
{
    if (dev == NULL || dev->hspi == NULL || rx == NULL) return HAL_ERROR;

    SPI_TypeDef *spi = dev->hspi->Instance;
    const uint32_t TO = 3400U;   /* 约 20 us @170 MHz，足够覆盖 16bit@10MHz(1.6us) + 抖动 */
    uint32_t t0;
    uint32_t sr;

    /* ---------- 仅在必要时恢复，避免每次都做重恢复 ---------- */
    sr = spi->SR;

    if ((sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) ||
        ((sr & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) ||
        ((spi->CR1 & SPI_CR1_SPE) == 0U))
    {
        spi_recover(dev);
    }

    /* 满足 AS5047P 的 tCSH >= 350ns */
    tCSH_delay();
    cs_low(dev);

    /* ---------- 等待 TXE ---------- */
    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U)
    {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF))
            goto fail_error;

        if (DWT_GetElapsedTicks(t0) > TO)
            goto fail_timeout;
    }

    /* 16-bit 访问 DR，避免因为访问宽度不匹配带来额外问题 */
    *(__IO uint16_t *)&spi->DR = tx;

    /* ---------- 等待 RXNE ---------- */
    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U)
    {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF))
            goto fail_error;

        if (DWT_GetElapsedTicks(t0) > TO)
            goto fail_timeout;
    }

    *rx = *(__IO uint16_t *)&spi->DR;

    /* ---------- 等待 BSY 清零 ----------
     * BSY 清零即可保证最后一个时钟边沿结束，CS 可以安全拉高。
     * 这里不再额外等 FTLVL，减少轮询开销。
     */
    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U)
    {
        if (DWT_GetElapsedTicks(t0) > TO)
            goto fail_timeout;
    }

    cs_high(dev);
    return HAL_OK;

fail_error:
    cs_high(dev);
    spi_recover(dev);
    return HAL_ERROR;

fail_timeout:
    cs_high(dev);
    spi_recover(dev);
    return HAL_TIMEOUT;
}
/**
 * AS5047P 流水线读（手册 Figure 15）：
 *   事务1  MOSI: cmd      MISO: 上一命令残留（忽略）
 *   事务2  MOSI: NOP      MISO: cmd 对应的数据帧
 */
 __attribute__((optimize("O2")))
static HAL_StatusTypeDef pipeline_read(AS5047P_Handle *dev,
                                       uint16_t cmd, uint16_t *frame)
{
    uint16_t dummy = 0U;
    HAL_StatusTypeDef st = spi_transfer(dev, cmd, &dummy);
    if (st != HAL_OK) return st;
    return spi_transfer(dev, AS5047P_CMD_READ_NOP, frame);
}

/* ── 公开函数实现 ────────────────────────────────────────────────── */

void AS5047P_Init(AS5047P_Handle *dev,
                  SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port,
                  uint16_t cs_pin)
{
    uint16_t errfl = 0U;

    dev->hspi               = hspi;
    dev->cs_port            = cs_port;
    dev->cs_pin             = cs_pin;
    dev->raw_angle          = 0U;
    dev->angle_deg          = 0.0f;
    dev->angle_rad          = 0.0f;
    dev->prev_angle_rad     = 0.0f;
    dev->angle_rad_with_track = 0.0f;
    dev->full_rotations     = 0;
    dev->velocity_rad       = 0.0f;


    dev->vel_hist_head  = 0U;
    dev->vel_hist_count = 0U;
    dev->vel_initialized = 0U;

    for (uint16_t i = 0; i < AS5047P_VEL_WINDOW_SIZE; i++)
    {
        dev->vel_angle_hist[i] = 0.0f;
    }
    
    dev->vel_med3_idx   = 0U;
    dev->vel_med3_count = 0U;
    for (uint8_t i = 0; i < 3U; i++)
    {
        dev->vel_med3_hist[i] = 0.0f;
    }


    cs_high(dev);   /* 确保 CSn 初始为高 */

    /*
     * 上电时 GPIO 默认 RESET → AS5047P 检测到异常帧 → ERRFL 写入粘性错误位。
     * 读一次 ERRFL 使其自动清零，否则后续所有响应帧 EF=1。
     */
    (void)pipeline_read(dev, AS5047P_CMD_READ_ERRFL, &errfl);
}

/**
 * 3次连续 SPI 事务（批量版，仅入口做一次 spi_recover 检查）
 *   txn0: tx0 → 丢弃响应
 *   txn1: tx1 → *rx1（tx0 的响应）
 *   txn2: tx2 → *rx2（tx1 的响应）
 * 所有失败路径：CS 当前为低，统一由 fail 标签处理。
 */
__attribute__((optimize("O2"), always_inline))
static inline HAL_StatusTypeDef spi_transfer3(AS5047P_Handle *dev,
                                       uint16_t tx0,
                                       uint16_t tx1, uint16_t *rx1,
                                       uint16_t tx2, uint16_t *rx2)
{
    SPI_TypeDef *spi = dev->hspi->Instance;
    const uint32_t TO = 3400U;
    uint32_t t0, sr;

    sr = spi->SR;
    if ((sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) ||
        ((sr & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) ||
        ((spi->CR1 & SPI_CR1_SPE) == 0U))
    {
        spi_recover(dev);
    }

    /* ── txn0 ── */
    tCSH_delay(); cs_low(dev);
    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    *(__IO uint16_t *)&spi->DR = tx0;
    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    (void)*(__IO uint16_t *)&spi->DR;
    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U) { if (DWT_GetElapsedTicks(t0) > TO) goto fail; }
    cs_high(dev);

    /* ── txn1 ── */
    tCSH_delay(); cs_low(dev);
    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    *(__IO uint16_t *)&spi->DR = tx1;
    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    *rx1 = *(__IO uint16_t *)&spi->DR;
    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U) { if (DWT_GetElapsedTicks(t0) > TO) goto fail; }
    cs_high(dev);

    /* ── txn2 ── */
    tCSH_delay(); cs_low(dev);
    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    *(__IO uint16_t *)&spi->DR = tx2;
    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    *rx2 = *(__IO uint16_t *)&spi->DR;
    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U) { if (DWT_GetElapsedTicks(t0) > TO) goto fail; }
    cs_high(dev);
    return HAL_OK;

fail:
    cs_high(dev);
    spi_recover(dev);
    return HAL_ERROR;
}

/**
 * @brief  读取当前角度（ISR-safe）
 *
 * 3次 SPI 批量事务：txn0=ANGLECOM，txn1=ERRFL（同时收角度），txn2=NOP（同时收 ERRFL）。
 * ERRFL 每次无条件读清，失败时由 pipeline_read 兜底清零，返回 AS5047P_ERROR。
 */
 __attribute__((optimize("O2")))
AS5047P_Status AS5047P_GetAngleWithoutTrack(AS5047P_Handle *dev)
{
    uint16_t rx = 0U, errfl = 0U;

    if (dev == NULL) return AS5047P_ERROR;

    if (spi_transfer3(dev,
                      AS5047P_CMD_READ_ANGLECOM,
                      AS5047P_CMD_READ_ERRFL, &rx,
                      AS5047P_CMD_READ_NOP,   &errfl) != HAL_OK)
    {
        (void)pipeline_read(dev, AS5047P_CMD_READ_ERRFL, &errfl);
        return AS5047P_ERROR;
    }

    if (!frame_parity_ok(rx))
        return AS5047P_ERROR;   /* ERRFL 已在 txn2 无条件读清，无需额外清零 */

    dev->raw_angle      = rx & AS5047P_DATA_MASK;
    dev->angle_deg      = (float)dev->raw_angle * (360.0f      / (float)AS5047P_RESOLUTION);
    dev->prev_angle_rad = dev->angle_rad;
    dev->angle_rad      = (float)dev->raw_angle * (6.28318530f / (float)AS5047P_RESOLUTION);
    return AS5047P_OK;
}

/* ── 兼容/扩展接口 ───────────────────────────────────────────────── */

AS5047P_Status AS5047P_ReadAngle(AS5047P_Handle *dev)
{
    return AS5047P_GetAngleWithoutTrack(dev);
}

__attribute__((optimize("O2")))
AS5047P_Status AS5047P_GetAngle(AS5047P_Handle *dev)
{
    AS5047P_Status sta = AS5047P_GetAngleWithoutTrack(dev);
    if (sta != AS5047P_OK) return sta;

    float d_rad = dev->angle_rad - dev->prev_angle_rad;
    if (fabsf(d_rad) > (0.8f * 2.0f * PI))
    {
        dev->full_rotations += (d_rad > 0.0f) ? -1 : 1;
    }
    dev->angle_rad_with_track = (float)dev->full_rotations * 2.0f * PI + dev->angle_rad;
    return AS5047P_OK;
}
__attribute__((optimize("O2")))
float normalizeAngle(float angle)
{
    /* fmod(double,double) 会软件模拟 double 除法（~4 us）；
     * 改用 FPU 截断：VMUL + VCVT×2 + VMUL + VSUB ≈ 8 周期 */
    float a = angle - (float)(int32_t)(angle * (1.0f / (2.0f * PI))) * (2.0f * PI);
    return (a >= 0.0f) ? a : (a + 2.0f * PI);
}

float AS5047_GetVelocity(AS5047P_Handle *dev, float dt)
{
    float d_angle = dev->angle_rad - dev->prev_angle_rad;
    dev->prev_angle_rad = dev->angle_rad;
    if      (d_angle >  PI) d_angle -= 2.0f * PI;
    else if (d_angle < -PI) d_angle += 2.0f * PI;
    dev->velocity_rad_test = d_angle / dt;
    return dev->velocity_rad_test;
}



void AS5047P_ResetVelocityWindow(AS5047P_Handle *dev)
{
    if (dev == NULL) return;

    dev->vel_hist_head  = 0U;
    dev->vel_hist_count = 0U;
    dev->vel_initialized = 0U;
    dev->velocity_rad   = 0.0f;

    for (uint16_t i = 0; i < AS5047P_VEL_WINDOW_SIZE; i++)
    {
        dev->vel_angle_hist[i] = 0.0f;
    }

    dev->vel_med3_idx   = 0U;
    dev->vel_med3_count = 0U;
    for (uint8_t i = 0; i < 3U; i++)
    {
        dev->vel_med3_hist[i] = 0.0f;
    }

}





static inline float median3f(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}





/**
 * @brief 固定 N 的重叠窗口差分测速
 *
 * 前提：
 *   1. 本周期调用前，已经成功执行过 AS5047P_GetAngle(dev)
 *   2. 使用 dev->angle_rad_with_track（连续角度）避免跨 2π 跳变
 *
 * 公式：
 *   omega[k] = (theta[k] - theta[k-N]) / (N * dt)
 *
 * 特点：
 *   - 每个控制周期都输出一次（重叠窗口）
 *   - 噪声比 1 点差分明显更小
 *   - 前 N 个周期作为“灌满窗口”阶段，先输出 0
 */
float AS5047P_GetVelocityWindowed(AS5047P_Handle *dev, float dt)
{
    if (dev == NULL || dt <= 0.0f)
        return 0.0f;

    float theta_now = dev->angle_rad_with_track;

    /* 第一次调用：把整段历史初始化成当前角度 */
    if (dev->vel_initialized == 0U)
    {
        for (uint16_t i = 0; i < AS5047P_VEL_WINDOW_SIZE; i++)
        {
            dev->vel_angle_hist[i] = theta_now;
        }

        dev->vel_hist_head   = 0U;
        dev->vel_hist_count  = 1U;
        dev->vel_initialized = 1U;
        dev->velocity_rad    = 0.0f;
        return 0.0f;
    }

    /* 写入当前角度到环形缓冲区 */
    dev->vel_hist_head++;
    if (dev->vel_hist_head >= AS5047P_VEL_WINDOW_SIZE)
    {
        dev->vel_hist_head = 0U;
    }
    dev->vel_angle_hist[dev->vel_hist_head] = theta_now;

    if (dev->vel_hist_count < AS5047P_VEL_WINDOW_SIZE)
    {
        dev->vel_hist_count++;
        dev->velocity_rad = 0.0f;   /* 灌满窗口前先输出 0，最简单 */
        return 0.0f;
    }

    /* 满窗口后，“head 的下一个位置”就是 N 个周期前的样本 */
    uint16_t old_idx = dev->vel_hist_head + 1U;
    if (old_idx >= AS5047P_VEL_WINDOW_SIZE)
    {
        old_idx = 0U;
    }

    float theta_old = dev->vel_angle_hist[old_idx];
    float omega_raw = (theta_now - theta_old) / ((float)AS5047P_VEL_WINDOW_N * dt);

    /* ---------- 2) 3 点中值滤波 ---------- */

    dev->vel_med3_hist[dev->vel_med3_idx] = omega_raw;
    dev->vel_med3_idx++;
    if (dev->vel_med3_idx >= 3U)
    {
        dev->vel_med3_idx = 0U;
    }

    if (dev->vel_med3_count < 3U)
    {
        dev->vel_med3_count++;
    }

    /* 前两次不足 3 点时，先直接输出原值 */
    if (dev->vel_med3_count < 3U)
    {
        dev->velocity_rad = omega_raw;
        return dev->velocity_rad;
    }

    dev->velocity_rad = median3f(dev->vel_med3_hist[0],
                                 dev->vel_med3_hist[1],
                                 dev->vel_med3_hist[2]);

    return dev->velocity_rad;
}