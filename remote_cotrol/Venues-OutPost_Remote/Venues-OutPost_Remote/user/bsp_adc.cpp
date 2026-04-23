/*  @file arm_task.cpp
* @brief arm_task驱动实现文件
 * author: 椰汁城传奇
 *
 * 使用示例:
 * 不需要使用示例
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_adc.hpp"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

BSP_ADC::BSP_ADC(ADC_HandleTypeDef* hadc)
    : _hadc(hadc) {}

void BSP_ADC::init(uint16_t* buffer, uint8_t channelCount) {
    _adcBuffer = buffer;
    _channelCount = channelCount;
    HAL_ADC_Start_DMA(_hadc, (uint32_t*)_adcBuffer, _channelCount);
}

uint16_t BSP_ADC::getValue(uint8_t channelIndex) {
    if (channelIndex >= _channelCount || _adcBuffer == nullptr)
        return 0;
    return _adcBuffer[channelIndex];
}
