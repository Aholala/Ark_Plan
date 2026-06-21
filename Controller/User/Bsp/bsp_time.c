#include "bsp_time.h"

#include "main.h"

uint32_t Bsp_Time_GetMs(void)
{
  return HAL_GetTick();
}
