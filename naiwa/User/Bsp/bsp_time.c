/**
 * @file bsp_time.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-06-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_time.h"

#include "main.h"

uint32_t Bsp_Time_GetMs(void)
{
  return HAL_GetTick();
}
