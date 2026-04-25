#include "AS5047P_RW.h"
#include "stdint.h"
#include "sys.h"

/* 使能 DWT 周期计数器，用于微秒级超时 */
static inline void as5047p_enable_dwt_cycle_counter(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

/* 计算 AS5047P 偶校验位：根据 bit[14:0] 生成 bit15 */
static inline uint8_t as5047p_calc_parity(uint16_t value)
{
    value &= 0x7FFFU;
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1U;
}

/* 对 GCC 编译器给关键函数单独优化 */
#if defined(__GNUC__) && !defined(__clang__)
#define AS5047P_OPTIMIZE __attribute__((optimize("O2,fast-math")))
#else
#define AS5047P_OPTIMIZE
#endif

/* 已经带读位和校验位的常用命令 */
#define AS5047P_CMD_READ_ANGLEUNC  0x7FFEU
#define AS5047P_CMD_READ_NOP       0xC000U

/* 根据寄存器地址生成 AS5047P 读命令 */
static inline uint16_t as5047p_make_read_cmd(uint16_t reg)
{
    uint16_t cmd = reg | 0x4000U;

    if (as5047p_calc_parity(cmd)) {
        cmd |= 0x8000U;
    }

    return cmd;
}

/* CS 拉高后的保持时间，保证两帧之间满足 tCSH */
static inline void as5047p_tCSH_delay(void)
{
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/* 恢复 SPI 异常状态，清空残留数据和错误标志 */
static inline void as5047p_spi_recover(AS5047P_Handle_t *dev)
{
    SPI_TypeDef *spi = dev->hspi->Instance;

    /* 清空 RX FIFO，避免读到旧数据 */
    while ((spi->SR & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) {
        (void)*(__IO uint16_t *)&spi->DR;
    }

    /* 清除接收溢出错误 */
    if (spi->SR & SPI_SR_OVR) {
        (void)*(__IO uint16_t *)&spi->DR;
        (void)spi->SR;
    }

    /* 清除帧格式错误 */
    if (spi->SR & SPI_SR_FRE) {
        (void)spi->SR;
    }

    /* 清除模式错误，并重新使能 SPI */
    if (spi->SR & SPI_SR_MODF) {
        (void)spi->SR;
        CLEAR_BIT(spi->CR1, SPI_CR1_SPE);
        SET_BIT(spi->CR1, SPI_CR1_SPE);
    }

    /* 确保 SPI 外设处于使能状态 */
    if ((spi->CR1 & SPI_CR1_SPE) == 0U) {
        SET_BIT(spi->CR1, SPI_CR1_SPE);
    }
}

/* 单帧 16-bit SPI 传输，直接操作寄存器以减少 HAL 开销 */
AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_spi_transfer16(AS5047P_Handle_t *dev,
                                               uint16_t tx,
                                               uint16_t *rx)
{
    if (!dev || !dev->hspi || !rx) return AS5047P_ERROR;

    SPI_TypeDef *spi = dev->hspi->Instance;
    const uint32_t TO = 3400U;   /* 约 20us @170MHz */
    uint32_t t0;
    uint32_t sr;

    /* 传输前检查 SPI 是否有残留数据或错误状态 */
    sr = spi->SR;
    if ((sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) ||
        ((sr & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) ||
        ((spi->CR1 & SPI_CR1_SPE) == 0U))
    {
        as5047p_spi_recover(dev);
    }

    as5047p_tCSH_delay();

    /* CS 拉低，开始一帧 SPI 传输 */
    dev->cs_port->BSRR = ((uint32_t)dev->cs_pin) << 16U;

    /* 等待发送缓冲区可写 */
    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) {
            goto fail_error;
        }
        if (DWT_GetElapsedTicks(t0) > TO) {
            goto fail_timeout;
        }
    }

    /* 写入 16-bit 发送数据 */
    *(__IO uint16_t *)&spi->DR = tx;

    /* 等待接收完成 */
    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) {
            goto fail_error;
        }
        if (DWT_GetElapsedTicks(t0) > TO) {
            goto fail_timeout;
        }
    }

    /* 读取 16-bit 接收数据 */
    *rx = *(__IO uint16_t *)&spi->DR;

    /* 等待总线空闲，确保本帧真正结束 */
    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U) {
        if (DWT_GetElapsedTicks(t0) > TO) {
            goto fail_timeout;
        }
    }

    /* CS 拉高，结束本帧 */
    dev->cs_port->BSRR = dev->cs_pin;

    /* bit14 为 AS5047P 返回的错误标志 EF */
    dev->errorflag = ((*rx & 0x4000U) != 0U);

    return AS5047P_OK;

fail_error:
    dev->cs_port->BSRR = dev->cs_pin;
    as5047p_spi_recover(dev);
    return AS5047P_ERROR;

fail_timeout:
    dev->cs_port->BSRR = dev->cs_pin;
    as5047p_spi_recover(dev);
    return AS5047P_TIMEOUT;
}

/* 两帧式读取：第一帧发命令，第二帧发 READ_NOP 取回结果 */
AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_pipeline_read(AS5047P_Handle_t *dev,
                                              uint16_t cmd,
                                              uint16_t *frame)
{
    if (!dev || !frame) return AS5047P_ERROR;

    uint16_t dummy = 0U;

    /* 第一帧返回的是上一条命令的结果，这里丢弃 */
    AS5047P_Status_t st = as5047p_spi_transfer16(dev, cmd, &dummy);
    if (st != AS5047P_OK) return st;

    /* 第二帧返回第一帧命令对应的数据 */
    return as5047p_spi_transfer16(dev, AS5047P_CMD_READ_NOP, frame);
}

/* 读取 ANGLEUNC 原始角度 */
AS5047P_Status_t AS5047P_ReadRawAngle(AS5047P_Handle_t *dev, uint16_t *raw)
{
    if (!dev || !raw) return AS5047P_ERROR;

    uint16_t rx = 0U;

    AS5047P_Status_t st = as5047p_pipeline_read(dev, AS5047P_CMD_READ_ANGLEUNC, &rx);
    if (st != AS5047P_OK) 
    {
        return st;
    }

    /* 检查 AS5047P 返回帧错误标志 */
    if (rx & 0x4000U) {
        dev->errorflag = 1U;
        return AS5047P_ERROR;
    }

    /* 低 14 位为角度值 */
    *raw = rx & 0x3FFFU;

    dev->raw_angle = *raw;
    dev->angle_rad = (*raw) * AS5047P_ANGLE_RAD_SCALE;

    return AS5047P_OK;
}

/* 读取 ERRFL，通常用于清除 AS5047P 错误标志 */
AS5047P_Status_t AS5047P_ClearErrorFlag(AS5047P_Handle_t *dev)
{
    uint16_t dummy = 0U;
    return as5047p_pipeline_read(dev, as5047p_make_read_cmd(AS5047P_REG_ERRFL), &dummy);
}

/* 初始化 AS5047P 驱动，并做一次读角通信测试 */
uint8_t AS5047P_RW_Init(AS5047P_Handle_t *dev,
                        SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin)
{
    if (!dev || !hspi || !cs_port) {
        return 0U;
    }

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;

    as5047p_enable_dwt_cycle_counter();

    dev->initialized = 0U;
    dev->errorflag = 0U;
    dev->raw_angle = 0U;
    dev->angle_rad = 0.0f;

    /* CS 空闲态为高电平 */
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    /* 基础通信测试 */
    uint16_t raw = 0U;
    AS5047P_Status_t st = AS5047P_ReadRawAngle(dev, &raw);
    if (st != AS5047P_OK) {
        return 0U;
    }

    dev->initialized = 1U;
    return 1U;
}