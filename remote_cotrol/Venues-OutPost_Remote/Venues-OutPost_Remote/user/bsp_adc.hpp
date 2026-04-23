#ifndef __BSP_ADC_HPP
#define __BSP_ADC_HPP

/* Includes ------------------------------------------------------------------*/

#include <algorithm>
#include <cmath>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "main.h"
#include "adc.h"

#ifdef __cplusplus
}
#endif

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

class BSP_ADC {
public:
    BSP_ADC(ADC_HandleTypeDef* hadc);
    void init(uint16_t* buffer, uint8_t channelCount); // 提供缓冲区

    uint16_t getValue(uint8_t channelIndex); // 获取某个通道的值

private:
    ADC_HandleTypeDef* _hadc;
    uint16_t* _adcBuffer = nullptr;
    uint8_t _channelCount = 0;
};

#endif