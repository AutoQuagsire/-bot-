#ifndef ICM42688P_H
#define ICM42688P_H

#include <stdint.h>
#include "spi.h"
#include "imu_types.h"


#define ICM42688_OK                 1u
#define ICM42688_ERR                0u

#define IMU_MODULE_ID_NONE          0u
#define IMU_MODULE_ID_ICM42688      1u


/* Expected WHO_AM_I value for ICM42688P */
#define ICM42688P_WHO_AM_I_VAL  0x47

#define ICM42688_REG_DEVICE_CONFIG  0x11u
#define ICM42688_REG_DRIVE_CONFIG   0x13u
#define ICM42688_REG_INT_CONFIG     0x14u
#define ICM42688_REG_FIFO_CONFIG    0x16u
#define ICM42688_REG_ACCEL_DATA_X1  0x1Fu
#define ICM42688_REG_INT_STATUS     0x2Du
#define ICM42688_REG_INTF_CONFIG0   0x4Cu
#define ICM42688_REG_PWR_MGMT0      0x4Eu
#define ICM42688_REG_GYRO_CONFIG0   0x4Fu
#define ICM42688_REG_ACCEL_CONFIG0  0x50u
#define ICM42688_REG_GYRO_ACCEL_CONFIG0 0x52u
#define ICM42688_REG_INT_CONFIG1    0x64u
#define ICM42688_REG_INT_SOURCE0    0x65u
#define ICM42688_REG_WHO_AM_I       0x75u
#define ICM42688_REG_BANK_SEL       0x76u

#define ICM42688_READ_FLAG          0x80u
#define ICM42688_WRITE_FLAG         0x7Fu

#define ICM42688_GRAVITY            9.80665f
#define ICM42688_DEG_TO_RAD         0.017453292519943295f
#define ICM42688_RAW_DATA_WORDS     6u
#define ICM42688_BURST_DATA_BYTES   12u
#define ICM42688_BURST_XFER_BYTES   (1u + ICM42688_BURST_DATA_BYTES)



typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    uint8_t module_id;
    uint8_t initialized;

    int16_t gyro_raw[3];
    int16_t accel_raw[3];

    uint8_t dma_tx[ICM42688_BURST_XFER_BYTES];
    uint8_t dma_rx[ICM42688_BURST_XFER_BYTES];
    volatile uint8_t dma_busy;

} ICM42688_Handle_t;





uint8_t ICM42688_Init(ICM42688_Handle_t *dev,
                     SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port,
                     uint16_t cs_pin);

uint8_t ICM42688_ReadRawData(ICM42688_Handle_t *dev, IMU_RawData_t *raw);
void ICM42688_ConvertRawToPhysical(const IMU_RawData_t *raw, IMU_PhysData_t *phys);
uint8_t ICM42688_StartReadRawDataDMA(ICM42688_Handle_t *dev);
uint8_t ICM42688_OnSpiTxRxCpltISR(ICM42688_Handle_t *dev, IMU_RawData_t *raw);
void ICM42688_OnSpiErrorISR(ICM42688_Handle_t *dev);







#endif
