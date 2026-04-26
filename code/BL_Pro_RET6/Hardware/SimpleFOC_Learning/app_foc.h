#ifndef APP_FOC_H
#define APP_FOC_H

#include <stdint.h>

uint8_t App_FOCStack_Init(void);
uint8_t App_StartupCalibrate(void);
void App_Loop(void);
void App_CurrentSenseSignTest(void);
void App_FOCControlIT_Enable(void);
void App_LoopForIT(void);
void DebuginWhile(void);

#endif
