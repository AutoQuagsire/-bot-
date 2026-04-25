#include "AS5047P_RW.h"
#include "stdint.h"
#include "sys.h"

#if defined(__GNUC__) && !defined(__clang__)
#define AS5047P_OPTIMIZE __attribute__((optimize("O2,fast-math")))
#else
#define AS5047P_OPTIMIZE
#endif

#define AS5047P_CMD_READ_ANGLEUNC  0x7FFEU
#define AS5047P_CMD_READ_NOP       0xC000U
#define AS5047P_READ_RETRY_MAX     3U
#define AS5047P_ENABLE_FRAME_PARITY_CHECK 0U

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

/* Parity bit for AS5047P SPI frames: even parity over lower 15 bits. */
static inline uint8_t as5047p_calc_parity(uint16_t value)
{
    value &= 0x7FFFU;
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1U;
}

static inline uint16_t as5047p_make_read_cmd(uint16_t reg)
{
    uint16_t cmd = reg | 0x4000U; /* READ bit */
    if (as5047p_calc_parity(cmd)) {
        cmd |= 0x8000U;
    }
    return cmd;
}

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
    const uint32_t TO = 3400U; /* ~20us @170MHz */
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
            goto fail_timeout;
        }
    }

    dev->cs_port->BSRR = dev->cs_pin;
    dev->errorflag = (uint8_t)((*rx & 0x4000U) != 0U);
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
static AS5047P_Status_t as5047p_pipeline_read(AS5047P_Handle_t *dev,
                                              uint16_t cmd,
                                              uint16_t *frame)
{
    if (!dev || !frame) return AS5047P_ERROR;

    uint16_t dummy = 0U;
    AS5047P_Status_t st = as5047p_spi_transfer16(dev, cmd, &dummy);
    if (st != AS5047P_OK) return st;
    return as5047p_spi_transfer16(dev, AS5047P_CMD_READ_NOP, frame);
}

AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_pipeline_read_angle_fast(AS5047P_Handle_t *dev,
                                                         uint16_t *raw)
{
    SPI_TypeDef *spi = dev->hspi->Instance;
    const uint32_t TO = 3400U; /* ~20us @170MHz */
    uint32_t t0;
    uint32_t sr;
    uint16_t frame;

    sr = spi->SR;
    if ((sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) ||
        ((sr & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) ||
        ((spi->CR1 & SPI_CR1_SPE) == 0U)) {
        as5047p_spi_recover(dev);
    }

    as5047p_tCSH_delay();
    dev->cs_port->BSRR = ((uint32_t)dev->cs_pin) << 16U;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    *(__IO uint16_t *)&spi->DR = AS5047P_CMD_READ_ANGLEUNC;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    (void)*(__IO uint16_t *)&spi->DR;

    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U) {
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    dev->cs_port->BSRR = dev->cs_pin;

    as5047p_tCSH_delay();
    dev->cs_port->BSRR = ((uint32_t)dev->cs_pin) << 16U;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    *(__IO uint16_t *)&spi->DR = AS5047P_CMD_READ_NOP;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }
    frame = *(__IO uint16_t *)&spi->DR;

    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U) {
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }

    dev->cs_port->BSRR = dev->cs_pin;
    if (frame & 0x4000U) {
        dev->errorflag = 1U;
        return AS5047P_ERROR;
    }

    *raw = frame & 0x3FFFU;
    dev->errorflag = 0U;
    return AS5047P_OK;

fail:
    dev->cs_port->BSRR = dev->cs_pin;
    as5047p_spi_recover(dev);
    return AS5047P_TIMEOUT;
}

/* Reading ERRFL clears sticky command-frame error bits (PARERR/INVCOMM/FRERR). */
AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_read_errfl_and_clear(AS5047P_Handle_t *dev, uint16_t *errfl)
{
    if (!dev) return AS5047P_ERROR;

    uint16_t frame = 0U;
    AS5047P_Status_t st = as5047p_pipeline_read(dev, as5047p_make_read_cmd(AS5047P_REG_ERRFL), &frame);
    if (st != AS5047P_OK) return st;

    if (errfl) {
        *errfl = (frame & 0x3FFFU);
    }
    return AS5047P_OK;
}

AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_read_register14_fast(AS5047P_Handle_t *dev,
                                                      uint16_t cmd,
                                                      uint16_t *data,
                                                      uint8_t check_ef)
{
    if (!dev || !data) return AS5047P_ERROR;

    uint16_t frame = 0U;
    AS5047P_Status_t st = as5047p_pipeline_read(dev, cmd, &frame);
    if (st != AS5047P_OK) return st;

    if (check_ef && (frame & 0x4000U)) {
        dev->errorflag = 1U;
        return AS5047P_ERROR;
    }

    *data = frame & 0x3FFFU;
    dev->errorflag = 0U;
    return AS5047P_OK;
}

AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_read_register14_recover(AS5047P_Handle_t *dev,
                                                         uint16_t cmd,
                                                         uint16_t *data,
                                                         uint8_t check_ef)
{
    if (!dev || !data) return AS5047P_ERROR;

    AS5047P_Status_t st = AS5047P_ERROR;
    uint16_t frame = 0U;
    uint16_t errfl = 0U;

    for (uint8_t i = 0U; i < AS5047P_READ_RETRY_MAX; i++) {
        st = as5047p_pipeline_read(dev, cmd, &frame);
        if (st != AS5047P_OK) {
            continue;
        }

#if AS5047P_ENABLE_FRAME_PARITY_CHECK
        if (as5047p_calc_parity(frame) != ((frame >> 15U) & 0x1U)) {
            dev->errorflag = 1U;
            (void)as5047p_read_errfl_and_clear(dev, &errfl);
            st = AS5047P_ERROR;
            continue;
        }
#endif

        if (check_ef && (frame & 0x4000U)) {
            dev->errorflag = 1U;
            (void)as5047p_read_errfl_and_clear(dev, &errfl);
            st = AS5047P_ERROR;
            continue;
        }

        *data = frame & 0x3FFFU;
        dev->errorflag = 0U;
        return AS5047P_OK;
    }

    return st;
}

AS5047P_Status_t AS5047P_ReadRawAngle(AS5047P_Handle_t *dev, uint16_t *raw)
{
    if (!dev || !raw) return AS5047P_ERROR;

    AS5047P_Status_t st = as5047p_pipeline_read_angle_fast(dev, raw);
    if (st != AS5047P_OK) {
        uint16_t errfl = 0U;
        (void)as5047p_read_errfl_and_clear(dev, &errfl);
        st = as5047p_read_register14_recover(dev, AS5047P_CMD_READ_ANGLEUNC, raw, 1U);
        if (st != AS5047P_OK) {
            return st;
        }
    }

    dev->raw_angle = *raw;
    dev->angle_rad = (*raw) * AS5047P_ANGLE_RAD_SCALE;
    return AS5047P_OK;
}

AS5047P_Status_t AS5047P_ReadMagnitude(AS5047P_Handle_t *dev, uint16_t *mag)
{
    if (!dev || !mag) return AS5047P_ERROR;

    AS5047P_Status_t st = as5047p_read_register14_fast(dev, as5047p_make_read_cmd(AS5047P_REG_MAG), mag, 1U);
    if (st == AS5047P_OK) return st;

    uint16_t errfl = 0U;
    (void)as5047p_read_errfl_and_clear(dev, &errfl);
    return as5047p_read_register14_recover(dev, as5047p_make_read_cmd(AS5047P_REG_MAG), mag, 1U);
}

AS5047P_Status_t AS5047P_ReadErrfl(AS5047P_Handle_t *dev, uint16_t *errfl)
{
    return as5047p_read_errfl_and_clear(dev, errfl);
}

AS5047P_Status_t AS5047P_ClearErrorFlag(AS5047P_Handle_t *dev)
{
    return as5047p_read_errfl_and_clear(dev, NULL);
}

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

    /* Keep CS high before first SPI transaction. */
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    /* Datasheet tpon: first valid angular value is available after power-on time. */
    HAL_Delay(10U);

    /* Clear startup sticky command errors before first angle read. */
    uint16_t errfl = 0U;
    (void)as5047p_read_errfl_and_clear(dev, &errfl);
    (void)as5047p_read_errfl_and_clear(dev, &errfl);

    uint16_t raw = 0U;
    AS5047P_Status_t st = AS5047P_ReadRawAngle(dev, &raw);
    if (st != AS5047P_OK) {
        return 0U;
    }

    dev->initialized = 1U;
    return 1U;
}
