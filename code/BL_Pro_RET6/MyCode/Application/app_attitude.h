#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include <stdint.h>

#include "attitude_estimator.h"

uint8_t App_Attitude_Init(void);
void App_Attitude_Loop(void);
#endif
