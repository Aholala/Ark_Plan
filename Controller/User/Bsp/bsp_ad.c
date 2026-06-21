#include "bsp_ad.h"

#include "adc.h"

static uint16_t ad_values[AD_CHANNEL_COUNT] = {2048U, 2048U, 2048U, 2048U};

void AD_Init(void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ad_values, AD_CHANNEL_COUNT) != HAL_OK)
  {
    Error_Handler();
  }
}

uint16_t AD_GetValue(AdChannel_t channel)
{
  if ((uint8_t)channel >= AD_CHANNEL_COUNT)
  {
    return 2048U;
  }

  return ad_values[channel];
}
