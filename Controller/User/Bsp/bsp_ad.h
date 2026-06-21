#ifndef __AD_H
#define __AD_H

#include <stdint.h>

typedef enum
{
  AD_CHANNEL_LH = 0,
  AD_CHANNEL_LV,
  AD_CHANNEL_RH,
  AD_CHANNEL_RV,
  AD_CHANNEL_COUNT,
} AdChannel_t;

void AD_Init(void);
uint16_t AD_GetValue(AdChannel_t channel);

#endif
