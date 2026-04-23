#include "AS5047P_RW.h"
#include "stdint.h"
#include "stm32g4xx_hal.h"



static uint8_t as5047p_calc_parity(uint16_t value)
{
    uint8_t cnt = 0;
    for (int i = 0; i < 15; i++) {
        if (value & (1U << i)) cnt++;
    }
    return (cnt & 0x01U) ? 1U : 0U;
}


static AS5047P_Status_t as5047p_spi_transfer16(AS5047P_Handle_t *dev,
                                               uint16_t tx,
                                               uint16_t *rx)
{
    if (!dev || !dev->hspi || !rx) return AS5047P_ERROR;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi,
                                                   (uint8_t *)&tx,
                                                   (uint8_t *)rx,
                                                   1,
                                                   10);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    if (st == HAL_TIMEOUT) return AS5047P_TIMEOUT;
    if (st != HAL_OK) return AS5047P_ERROR;

    dev->errorflag = ((*rx & 0x4000U) != 0U);
    return AS5047P_OK;
}


static AS5047P_Status_t as5047p_read_register16(AS5047P_Handle_t *dev,
                                                uint16_t reg,
                                                uint16_t *data)
{
    uint16_t cmd = reg | 0x4000U; // read bit
    uint16_t rx = 0;

    if (as5047p_calc_parity(cmd)) {
        cmd |= 0x8000U;
    }

    AS5047P_Status_t st = as5047p_spi_transfer16(dev, cmd, &rx);
    if (st != AS5047P_OK) return st;

    uint16_t nop = 0x0000U;
    if (as5047p_calc_parity(nop)) {
        nop |= 0x8000U;
    }

    st = as5047p_spi_transfer16(dev, nop, &rx);
    if (st != AS5047P_OK) return st;

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