#ifndef __BSP_TIME_H
#define __BSP_TIME_H

#include <stdint.h>

void Bsp_Time_InitTick(void);
uint32_t Bsp_Time_GetMs(void);

#endif
