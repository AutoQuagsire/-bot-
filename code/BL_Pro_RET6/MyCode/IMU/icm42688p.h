#ifndef ICM42688P_H
#define ICM42688P_H

#include <stdint.h>

/* Expected WHO_AM_I value for ICM42688P */
#define ICM42688P_WHO_AM_I_VAL  0x47

uint8_t ICM_SPI_Test(void);
uint8_t ICM_WhoAmI(void);
uint8_t ICM_SPI_LoopbackTest(void);

#endif
