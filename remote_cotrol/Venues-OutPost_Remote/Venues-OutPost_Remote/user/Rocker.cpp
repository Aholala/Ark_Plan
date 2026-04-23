/*  @file arm_task.cpp
* @brief arm_task驱动实现文件
 * author: 椰汁城传奇
 *
 * 使用示例:
#include "adc.h"

Rocker rocker(&adc, ADC_CHANNEL_0, ADC_CHANNEL_1); // PA0, PA1

rocker.setCenter(2048, 2048);
rocker.setDeadzone(50);

int8_t x = rocker.getXMapped();
int8_t y = rocker.getYMapped();

 *
 */

/* Includes ------------------------------------------------------------------*/

#include "Rocker.hpp"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Rocker::Rocker(BSP_ADC* adc, uint32_t xChannel, uint32_t yChannel)
    : _adc(adc), _xChannel(xChannel), _yChannel(yChannel) {
}

void Rocker::init() {

    // 默认中心点和死区
    _xCenter = 2048; // 默认中心值
    _yCenter = 2048; // 默认中心值
    _deadzone = 50;  // 默认死区
}

void Rocker::setCenter(uint16_t xCenter, uint16_t yCenter) {
    _xCenter = xCenter;
    _yCenter = yCenter;
}

void Rocker::setDeadzone(uint16_t dz) {
    _deadzone = dz;
}

uint16_t Rocker::getXRaw() {
    return _adc->getValue(_xChannel);
}

uint16_t Rocker::getYRaw() {
    return _adc->getValue(_yChannel);
}

int8_t Rocker::getXMapped() {
    int16_t diff = static_cast<int16_t>(getXRaw()) - static_cast<int16_t>(_xCenter);
    return normalizeToSigned100(diff);
}

int8_t Rocker::getYMapped() {
    int16_t diff = static_cast<int16_t>(getYRaw()) - static_cast<int16_t>(_yCenter);
    return normalizeToSigned100(diff);
}

int8_t Rocker::normalizeToSigned100(int16_t diff) {
    if (abs(diff) < _deadzone) return 0;

    // ±2048 → ±100 归一化
    float ratio = static_cast<float>(diff) / 2048.0f;
    float mapped = ratio * 100.0f;

    // 限制范围
    if (mapped > 100.0f) mapped = 100.0f;
    if (mapped < -100.0f) mapped = -100.0f;

    return static_cast<int8_t>(mapped);
}





