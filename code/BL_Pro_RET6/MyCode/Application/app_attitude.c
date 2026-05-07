#include "app_attitude.h"

#include <string.h>
#include "INT.h"
#include "usb_debug.h"
#include "stm32g4xx_hal.h"

static ICM42688_Handle_t g_icm42688;
static Attitude_Estimator_t g_estimator;


uint8_t App_Attitude_Init(void)
{
    memset(&g_icm42688, 0, sizeof(g_icm42688));
    memset(&g_estimator, 0, sizeof(g_estimator));

    Estimator_LinkICM42688P(&g_icm42688, &g_estimator);
    return IMU_Init(&g_estimator);
}

void App_Attitude_Loop(void)
{
    static uint8_t print_div = 0U;
    const IMU_RawData_t *raw_data;

    if (IMU_DRDY_Flag == 0U) {
        return;
    }
    IMU_DRDY_Flag = 0U;

    if (IMU_Update(&g_estimator) != IMU_OK) {
        return;
    }

    /* USB CDC bandwidth is limited: downsample print rate for stable FireWater streaming. */
    print_div++;
    if (print_div < 20U) {
        return;
    }
    print_div = 0U;

    raw_data = Estimator_GetRawData(&g_estimator);
    if (raw_data == NULL) {
        return;
    }

    /* VOFA+ FireWater: one frame per line, comma separated numeric channels. */
    USB_Debug_Printf("%lu,%d,%d,%d,%d,%d,%d\r\n",
                     (unsigned long)HAL_GetTick(),
                     (int)raw_data->accel[0],
                     (int)raw_data->accel[1],
                     (int)raw_data->accel[2],
                     (int)raw_data->gyro[0],
                     (int)raw_data->gyro[1],
                     (int)raw_data->gyro[2]);
}
