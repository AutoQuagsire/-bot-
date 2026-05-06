#include "icm42688p.h"
#include "main.h"
#include <string.h>

/* ICM42688P register addresses (top bit = R/W: 0=write, 1=read) */

/* CS pin: PC6 */
static uint8_t tx_buf[2];
static uint8_t rx_buf[2];



static void ICM42688_CS_Low(ICM42688_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void ICM42688_CS_High(ICM42688_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static uint8_t ICM42688_WriteReg(ICM42688_Handle_t *dev, uint8_t reg, uint8_t data)
{
    uint8_t tx[2];

    tx[0] = reg & ICM42688_WRITE_FLAG;
    tx[1] = data;

    ICM42688_CS_Low(dev);
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(dev->hspi, tx, 2, 100);
    ICM42688_CS_High(dev);

    return (ret == HAL_OK) ? ICM42688_OK : ICM42688_ERR;
}

static uint8_t ICM42688_ReadReg(ICM42688_Handle_t *dev, uint8_t reg, uint8_t *data)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg | ICM42688_READ_FLAG;
    tx[1] = 0x00;

    ICM42688_CS_Low(dev);
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, 100);
    ICM42688_CS_High(dev);

    if (ret != HAL_OK) {
        return ICM42688_ERR;
    }

    *data = rx[1];
    return ICM42688_OK;
}

static uint8_t ICM42688_ReadRegs(ICM42688_Handle_t *dev,
                                 uint8_t start_reg,
                                 uint8_t *data,
                                 uint16_t len)
{
    uint8_t tx[1U + ICM42688_RAW_DATA_WORDS * 2U];
    uint8_t rx[1U + ICM42688_RAW_DATA_WORDS * 2U];

    if (dev == NULL || data == NULL || len == 0U || len > (uint16_t)(sizeof(tx) - 1U)) {
        return ICM42688_ERR;
    }

    tx[0] = start_reg | ICM42688_READ_FLAG;
    memset(&tx[1], 0, len);

    ICM42688_CS_Low(dev);
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len + 1U, 100);
    ICM42688_CS_High(dev);

    if (ret != HAL_OK) {
        return ICM42688_ERR;
    }

    memcpy(data, &rx[1], len);
    return ICM42688_OK;
}

static uint8_t ICM42688_SelectBank(ICM42688_Handle_t *dev, uint8_t bank)
{
    return ICM42688_WriteReg(dev, ICM42688_REG_BANK_SEL, bank & 0x07u);
}

uint8_t ICM42688_Init(ICM42688_Handle_t *dev,
                      SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port,
                      uint16_t cs_pin)
{
    uint8_t who_am_i = 0;
    uint8_t status = 0;

    if (dev == NULL || hspi == NULL || cs_port == NULL) {
        return ICM42688_ERR;
    }

    memset(dev, 0, sizeof(ICM42688_Handle_t));

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->module_id = IMU_MODULE_ID_ICM42688;
    dev->initialized = 0;

    /*
     * 先让 CS 空闲为高。
     * 注意：CubeMX 里 NSS 用 Software，CS 由 GPIO 手动控制。
     */
    ICM42688_CS_High(dev);
    HAL_Delay(10);

    /*
     * 确保在 Bank0。
     */
    if (ICM42688_SelectBank(dev, 0) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    /*
     * 软复位。
     * DEVICE_CONFIG bit0 = 1。
     * 数据手册要求写入后至少等待 1ms，这里保守等 10ms。
     */
    if (ICM42688_WriteReg(dev, ICM42688_REG_DEVICE_CONFIG, 0x01) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    HAL_Delay(10);

    /*
     * 复位后重新选择 Bank0。
     */
    if (ICM42688_SelectBank(dev, 0) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    /*
     * 读 WHO_AM_I。
     * ICM-42688-P 应该返回 0x47。
     */
    if (ICM42688_ReadReg(dev, ICM42688_REG_WHO_AM_I, &who_am_i) != ICM42688_OK) {
        return ICM42688_ERR;
    }



    if (who_am_i != ICM42688P_WHO_AM_I_VAL) {
        return ICM42688_ERR;
    }

    /*
     * INTF_CONFIG0:
     * bit5 = 1: Sensor data Big Endian
     * bit4 = 1: FIFO count Big Endian
     * bit1:0 = 11: Disable I2C
     *
     * 0x30 是默认大端配置，0x33 是在此基础上禁用 I2C。
     * 用 SPI 时建议禁用 I2C，避免接口模式干扰。
     */
    if (ICM42688_WriteReg(dev, ICM42688_REG_INTF_CONFIG0, 0x33) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    /*
     * FIFO 暂时不用，保持 Bypass Mode。
     */
    if (ICM42688_WriteReg(dev, ICM42688_REG_FIFO_CONFIG, 0x00) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    /*
     * GYRO_CONFIG0:
     * bit7:5 = 001 -> ±1000 dps
     * bit3:0 = 0110 -> 1kHz
     * 0x26 = 0010 0110
     */
    if (ICM42688_WriteReg(dev, ICM42688_REG_GYRO_CONFIG0, 0x26) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    /*
     * ACCEL_CONFIG0:
     * bit7:5 = 010 -> ±4g
     * bit3:0 = 0110 -> 1kHz
     * 0x46 = 0100 0110
     */
    if (ICM42688_WriteReg(dev, ICM42688_REG_ACCEL_CONFIG0, 0x46) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    /*
     * GYRO_ACCEL_CONFIG0:
     * Accel UI filter BW = 4 -> max(400Hz, ODR)/10
     * Gyro  UI filter BW = 4 -> max(400Hz, ODR)/10
     * ODR = 1kHz 时，大约 100Hz。
     */
    if (ICM42688_WriteReg(dev, ICM42688_REG_GYRO_ACCEL_CONFIG0, 0x44) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    /*
     * PWR_MGMT0:
     * bit3:2 = 11 -> Gyro Low Noise Mode
     * bit1:0 = 11 -> Accel Low Noise Mode
     * bit5   = 0  -> 温度传感器启用
     *
     * 0x0F = gyro LN + accel LN。
     */
    if (ICM42688_WriteReg(dev, ICM42688_REG_PWR_MGMT0, 0x0F) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    /*
     * 从 OFF 切到工作模式后，数据手册要求短时间内不要继续写寄存器；
     * 陀螺仪启动也需要等待。这里保守等待 45ms。
     */
    HAL_Delay(45);

    /*
     * 可选：读一次 INT_STATUS，顺手清掉部分状态位。
     */
    (void)ICM42688_ReadReg(dev, ICM42688_REG_INT_STATUS, &status);

    /*
     * 量程比例系数。
     *
     * ±1000 dps: 32.8 LSB/(deg/s)
     * ±4g:       8192 LSB/g
     */
    dev->initialized = 1;

    return ICM42688_OK;
}

uint8_t ICM42688_ReadRawData(ICM42688_Handle_t *dev, int16_t *raw)
{
    uint8_t frame[12];

    if (dev == NULL || dev->initialized == 0U) {
        return ICM42688_ERR;
    }

    /*
     * 第一版为了稳，可以保留。
     * 后面如果确定所有配置函数退出后都会切回 Bank0，可以去掉这一步。
     */
    if (ICM42688_SelectBank(dev, 0) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    if (ICM42688_ReadRegs(dev,
                          ICM42688_REG_ACCEL_DATA_X1,
                          frame,
                          sizeof(frame)) != ICM42688_OK) {
        return ICM42688_ERR;
    }

    dev->accel_raw[0] = (int16_t)(((uint16_t)frame[0]  << 8) | frame[1]);
    dev->accel_raw[1] = (int16_t)(((uint16_t)frame[2]  << 8) | frame[3]);
    dev->accel_raw[2] = (int16_t)(((uint16_t)frame[4]  << 8) | frame[5]);

    dev->gyro_raw[0]  = (int16_t)(((uint16_t)frame[6]  << 8) | frame[7]);
    dev->gyro_raw[1]  = (int16_t)(((uint16_t)frame[8]  << 8) | frame[9]);
    dev->gyro_raw[2]  = (int16_t)(((uint16_t)frame[10] << 8) | frame[11]);

    if (raw != NULL) {
        raw[0] = dev->accel_raw[0];
        raw[1] = dev->accel_raw[1];
        raw[2] = dev->accel_raw[2];

        raw[3] = dev->gyro_raw[0];
        raw[4] = dev->gyro_raw[1];
        raw[5] = dev->gyro_raw[2];
    }

    return ICM42688_OK;
}