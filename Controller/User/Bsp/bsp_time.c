#include "bsp_time.h"

#include "main.h"
#include "tim.h"

void Bsp_Time_InitTick(void)
{
  HAL_TIM_Base_Start_IT(&htim1);
}

uint32_t Bsp_Time_GetMs(void)
{
  return HAL_GetTick();
}
