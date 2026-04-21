#ifndef __PID_AUTOTUNE_H
#define __PID_AUTOTUNE_H

#include <stdint.h>

void PID_AutoTune_Start(void);
void PID_AutoTune_Stop(void);
void PID_AutoTune_Update(void);
uint8_t PID_AutoTune_IsActive(void);

#endif
