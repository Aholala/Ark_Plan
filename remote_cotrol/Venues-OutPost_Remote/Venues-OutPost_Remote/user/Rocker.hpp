#ifndef __ROCKER_HPP
#define __ROCKER_HPP

/* Includes ------------------------------------------------------------------*/

#include <algorithm>
#include <cmath>
#include <stdio.h>
#include  "bsp_adc.hpp"

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

class Rocker {
public:
    Rocker(BSP_ADC* adc, uint32_t xChannel, uint32_t yChannel);

    void init();
    void setCenter(uint16_t xCenter, uint16_t yCenter);
    void setDeadzone(uint16_t dz);

    uint16_t getXRaw();
    uint16_t getYRaw();
    int8_t getXMapped(); // -100 ~ 100
    int8_t getYMapped();
    int8_t normalizeToSigned100(int16_t diff);

private:
    BSP_ADC* _adc;
    uint32_t _xChannel;
    uint32_t _yChannel;

    uint16_t _xCenter = 2048;
    uint16_t _yCenter = 2048;
    uint16_t _deadzone = 50;

// public:
//     // static Rocker& instance() ;
//     static Rocker instance1;
//     static Rocker instance2;
// public:
//     Rocker() = default;
//     ~Rocker() = default;
//     Rocker(const Rocker&) = delete;            // 禁止拷贝构造
//     Rocker& operator=(const Rocker&) = delete; // 禁止赋值
//
//     int8_t normalizeToSigned100(int16_t diff);
};

#endif



