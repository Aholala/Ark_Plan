#ifndef __BSP_ENCODE_HPP
#define __BSP_ENCODE_HPP

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <algorithm>
#include <cmath>

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
}
#endif

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Encoder {
public:
    Encoder(TIM_HandleTypeDef* htim, int32_t max_count = 0xFFFF);
    void start();                         // 启动编码器
    int32_t getCount() const;                   // 获取当前编码器计数值
    float getDelta();                   // 获取本次与上次计数值的差值（处理溢出）
    float getPos();                     // 获取当前编码器位置
    void reset();                         // 重置编码器计数

private:
    TIM_HandleTypeDef* _htim;
    float _lastCount;
    float _maxCount;
    float _delta = 0;             // 当前一次位置变化值
    float _position = 0.0f;         // 累计位置（单位：圈）
};

#endif
