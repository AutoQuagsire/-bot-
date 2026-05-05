#include "icm42688p.h"
#include "spi.h"
#include "main.h"

/* ICM42688P register addresses (top bit = R/W: 0=write, 1=read) */
#define ICM_REG_WHO_AM_I        0x75
#define ICM_REG_READ_FLAG       0x80

/* CS pin: PC6 */
#define ICM_CS_LOW()   HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET)
#define ICM_CS_HIGH()  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET)

static uint8_t tx_buf[2];
static uint8_t rx_buf[2];

/**
 * @brief  Read a single register from ICM42688P
 * @param  reg  register address (7-bit, without read flag)
 * @return register value
 */
static uint8_t ICM_ReadReg(uint8_t reg)
{
    tx_buf[0] = reg | ICM_REG_READ_FLAG;
    tx_buf[1] = 0x00;

    ICM_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
    ICM_CS_HIGH();

    return rx_buf[1];
}

/**
 * @brief  Write a single register to ICM42688P
 */
static void ICM_WriteReg(uint8_t reg, uint8_t val)
{
    tx_buf[0] = reg & 0x7F;
    tx_buf[1] = val;

    ICM_CS_LOW();
    HAL_SPI_Transmit(&hspi2, tx_buf, 2, HAL_MAX_DELAY);
    ICM_CS_HIGH();
}

/**
 * @brief  SPI loopback test - send 0xAA, expect to read 0xAA
 *         (disconnect ICM42688P, connect MOSI to MISO for this test)
 * @return received byte
 */
uint8_t ICM_SPI_LoopbackTest(void)
{
    uint8_t tx = 0xAA;
    uint8_t rx = 0;

    ICM_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, HAL_MAX_DELAY);
    ICM_CS_HIGH();

    return rx;
}

/**
 * @brief  Read WHO_AM_I register (should return 0x47)
 * @return 0x47 on success, 0x00 on failure
 */
uint8_t ICM_WhoAmI(void)
{
    return ICM_ReadReg(ICM_REG_WHO_AM_I);
}

/**
 * @brief  Simple SPI test - call from main after MX_SPI2_Init()
 *         - Reads WHO_AM_I and returns result
 *         - On logic analyzer you should see:
 *           CS low -> MOSI: [0xF5, 0x00] -> MISO: [xx, 0x47] -> CS high
 */
uint8_t ICM_SPI_Test(void)
{
    /* CS idle high */
    ICM_CS_HIGH();
    HAL_Delay(1);

    /* Read WHO_AM_I (0x75 | 0x80 = 0xF5) */
    uint8_t id = ICM_WhoAmI();

    return id;
}
