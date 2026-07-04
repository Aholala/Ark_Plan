/**
 * @file bsp_pwm.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包 - PWM输出接口头文件
 * @version 1.0
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块提供PWM通道的枚举定义和初始化为及占空比/脉冲设置接口。
 *          支持12路PWM通道，映射到TIM8/TIM4/TIM5/TIM1定时器的指定通道。
 *          SetDuty()接口包含死区/有效占空比限幅策略（5%~95%），
 *          而SetPulse()为原始脉冲设置接口，不应用该策略。
 */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief PWM通道枚举
 * @note 顺序必须与 bsp_pwm.c 中的硬件映射表严格一致
 */
typedef enum {
  BSP_PWM_TIM8_CH1 = 0, /**< TIM8 通道1 */
  BSP_PWM_TIM8_CH2,     /**< TIM8 通道2 */
  BSP_PWM_TIM8_CH3,     /**< TIM8 通道3 */
  BSP_PWM_TIM8_CH4,     /**< TIM8 通道4 */
  BSP_PWM_TIM4_CH1,     /**< TIM4 通道1 */
  BSP_PWM_TIM4_CH2,     /**< TIM4 通道2 */
  BSP_PWM_TIM4_CH3,     /**< TIM4 通道3 */
  BSP_PWM_TIM4_CH4,     /**< TIM4 通道4 */
  BSP_PWM_TIM5_CH3,     /**< TIM5 通道3 */
  BSP_PWM_TIM5_CH4,     /**< TIM5 通道4 */
  BSP_PWM_TIM1_CH1,     /**< TIM1 通道1 */
  BSP_PWM_TIM1_CH3,     /**< TIM1 通道3 */
  BSP_PWM_COUNT         /**< PWM通道总数（用于边界检查） */
} BspPwmChannel_t;

/**
 * @brief 初始化所有PWM通道
 * @note 将所有通道输出设置为0脉冲，并启动PWM定时器
 * @warning 若某个通道启动失败，将调用 Error_Handler() 进入死循环
 */
void Bsp_Pwm_Init(void);

/**
 * @brief 设置指定PWM通道的占空比（百分比形式）
 *
 * @param channel       通道号（BspPwmChannel_t枚举）
 * @param duty_percent  占空比（0~100%）
 * @note 占空比应用限幅策略：
 *       - 0% 表示停止/制动，保持为0；
 *       - 非零值小于5% 会被提升至5%，避免死区；
 *       - 大于95% 会被钳位至95%，防止驱动饱和。
 * @note 若通道号无效则函数直接返回
 */
void Bsp_Pwm_SetDuty(BspPwmChannel_t channel, uint8_t duty_percent);

/**
 * @brief 设置指定PWM通道的原始脉冲宽度（定时器计数值）
 *
 * @param channel 通道号（BspPwmChannel_t枚举）
 * @param pulse   脉冲计数值（0 ~ 自动重载值）
 * @note 脉冲值会自动限幅到定时器周期范围内，但不会应用5%/95%的占空比策略。
 *       此接口适用于需要精细控制或绕过策略限制的场景。
 * @note 若通道号无效则函数直接返回
 */
void Bsp_Pwm_SetPulse(BspPwmChannel_t channel, uint32_t pulse);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */