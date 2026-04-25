#include "AS5047P_RW.h"
#include "stdint.h"
#include "sys.h"

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

static uint8_t as5047p_calc_parity(uint16_t value)
{
    uint8_t cnt = 0;
    for (int i = 0; i < 15; i++) {
        if (value & (1U << i)) cnt++;
    }
    return (cnt & 0x01U) ? 1U : 0U;
}
#if defined(__GNUC__) && !defined(__clang__)
#define AS5047P_OPTIMIZE __attribute__((optimize("O2,fast-math")))
#else
#define AS5047P_OPTIMIZE
#endif

static inline void as5047p_tCSH_delay(void)
{
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}


static inline void as5047p_spi_recover(AS5047P_Handle_t *dev)
{
    SPI_TypeDef *spi = dev->hspi->Instance;

    while ((spi->SR & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) {
        (void)*(__IO uint16_t *)&spi->DR;
    }

    if (spi->SR & SPI_SR_OVR) {
        (void)*(__IO uint16_t *)&spi->DR;
        (void)spi->SR;
    }
    if (spi->SR & SPI_SR_FRE) {
        (void)spi->SR;
    }
    if (spi->SR & SPI_SR_MODF) {
        (void)spi->SR;
        CLEAR_BIT(spi->CR1, SPI_CR1_SPE);
        SET_BIT(spi->CR1, SPI_CR1_SPE);
    }

    if ((spi->CR1 & SPI_CR1_SPE) == 0U) {
        SET_BIT(spi->CR1, SPI_CR1_SPE);
    }
}

AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_spi_transfer16(AS5047P_Handle_t *dev,
                                               uint16_t tx,
                                               uint16_t *rx)
{
    if (!dev || !dev->hspi || !rx) return AS5047P_ERROR;

    SPI_TypeDef *spi = dev->hspi->Instance;
    const uint32_t TO = 6800U;   /* ~40us @170MHz */
    uint32_t t0;
    uint32_t sr;

    sr = spi->SR;
    if ((sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) ||
        ((sr & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) ||
        ((spi->CR1 & SPI_CR1_SPE) == 0U))
    {
        as5047p_spi_recover(dev);
    }

    as5047p_tCSH_delay();
    dev->cs_port->BSRR = ((uint32_t)dev->cs_pin) << 16U;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) {
            goto fail_error;
        }
        if (DWT_GetElapsedTicks(t0) > TO) {
            goto fail_timeout;
        }
    }

    *(__IO uint16_t *)&spi->DR = tx;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) {
            goto fail_error;
        }
        if (DWT_GetElapsedTicks(t0) > TO) {
            goto fail_timeout;
        }
    }

    *rx = *(__IO uint16_t *)&spi->DR;

    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U) {
        if (DWT_GetElapsedTicks(t0) > TO) {
            as5047p_spi_recover(dev);
            break;
        }
    }

    dev->cs_port->BSRR = dev->cs_pin;

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

AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_read_register16(AS5047P_Handle_t *dev,
                                                uint16_t reg,
                                                uint16_t *data)
{
    if (!dev || !data) return AS5047P_ERROR;

    uint16_t cmd = reg | 0x4000U; /* read bit */
    uint16_t rx = 0U;

    if (as5047p_calc_parity(cmd)) {
        cmd |= 0x8000U;
    }

    AS5047P_Status_t st = as5047p_spi_transfer16(dev, cmd, &rx);
    if (st != AS5047P_OK) return st;

    /* AS5047P pipeline: second frame must be READ NOP (0x4000 + parity => 0xC000), not 0x0000 */
    uint16_t nop = 0x4000U;
    if (as5047p_calc_parity(nop)) {
        nop |= 0x8000U;
    }

    st = as5047p_spi_transfer16(dev, nop, &rx);
    if (st != AS5047P_OK) return st;

    if (rx & 0x4000U) {
        dev->errorflag = 1U;
        return AS5047P_ERROR;
    }

    *data = rx & 0x3FFFU;
    return AS5047P_OK;
}



AS5047P_Status_t AS5047P_ReadRawAngle(AS5047P_Handle_t *dev, uint16_t *raw)
{
    if (!dev || !raw) return AS5047P_ERROR;

    AS5047P_Status_t st =
        as5047p_read_register16(dev, AS5047P_REG_ANGLEUNC, raw);
    if (st != AS5047P_OK) return st;

    dev->raw_angle = *raw;
    dev->angle_rad = (*raw) * AS5047P_ANGLE_RAD_SCALE;
    return AS5047P_OK;
}


AS5047P_Status_t AS5047P_ReadAngleRad(AS5047P_Handle_t *dev, float *angle)
{
    uint16_t raw = 0;
    AS5047P_Status_t st = AS5047P_ReadRawAngle(dev, &raw);
    if (st != AS5047P_OK) return st;

    if (angle) *angle = dev->angle_rad;
    return AS5047P_OK;
}


AS5047P_Status_t AS5047P_ClearErrorFlag(AS5047P_Handle_t *dev)
{
    uint16_t dummy;
    return as5047p_read_register16(dev, AS5047P_REG_ERRFL, &dummy);
}


uint8_t AS5047P_RW_Init(AS5047P_Handle_t *dev,
                     SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port,
                     uint16_t cs_pin)
{
    if (!dev || !hspi || !cs_port) {
        return 0U;
    }

    /* 1. 绑定硬件资源 */
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;

    as5047p_enable_dwt_cycle_counter();

    /* 2. 清零运行状态 */
    dev->initialized = 0U;
    dev->errorflag = 0U;
    dev->raw_angle = 0U;
    dev->angle_rad = 0.0f;

    /* 3. 先确保 CS 为空闲态 */
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    /* 4. 做一次基础读角测试，确认 SPI 链路可用 */
    uint16_t raw = 0U;
    AS5047P_Status_t st = AS5047P_ReadRawAngle(dev, &raw);
    if (st != AS5047P_OK) {
        return 0U;
    }

    /* 5. 到这里说明设备基本可通信 */
    dev->initialized = 1U;
    return 1U;
}