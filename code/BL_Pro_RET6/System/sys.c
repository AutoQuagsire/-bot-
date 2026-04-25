#include "sys.h"




extern volatile uint32_t usTicks;

void GPIO_ResetBits(GPIO_TypeDef *GPIO_PORT,uint16_t GPIO_PIN)
{
	HAL_GPIO_WritePin(GPIO_PORT,GPIO_PIN,0);
}

void GPIO_SetBits(GPIO_TypeDef *GPIO_PORT,uint16_t GPIO_PIN)
{
	HAL_GPIO_WritePin(GPIO_PORT,GPIO_PIN,1);
}


