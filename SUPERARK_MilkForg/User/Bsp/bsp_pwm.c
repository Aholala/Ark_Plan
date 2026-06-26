/**
 * @file bsp_pwm.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包 - PWM驱动实现
 * @version 1.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块提供PWM通道的初始化、占空比/脉冲设置功能。
 *          支持PWM模式1和模式2（极性相反），通过配置表统一管理。
 *          所有通道输出值均以脉冲宽度（计数值）或百分比占空比进行设置。
 */

#include "bsp_pwm.h"
#include "tim.h"

/**
 * @brief PWM通道配置结构体
 * @note 包含定时器句柄、通道号和PWM模式标志
 */
typedef struct {
  TIM_HandleTypeDef *htim; /**< 定时器句柄指针 */
  uint32_t channel;        /**< 定时器通道（如 TIM_CHANNEL_1） */
  uint8_t
      pwm2_mode; /**< PWM模式选择：0=模式1（高电平有效），1=模式2（低电平有效）
                  */
} BspPwmConfig_t;

/**
 * @brief PWM通道硬件映射表
 * @note 顺序必须与 BspPwmChannel_t 枚举定义一致
 * @note pwm2_mode=1表示该通道使用PWM模式2，输出极性翻转
 */
static const BspPwmConfig_t bsp_pwm_configs[BSP_PWM_COUNT] = {
    {&htim8, TIM_CHANNEL_1, 0U}, /* BSP_PWM_TIM8_CH1 */
    {&htim8, TIM_CHANNEL_2, 1U}, /* BSP_PWM_TIM8_CH2 */
    {&htim8, TIM_CHANNEL_3, 0U}, /* BSP_PWM_TIM8_CH3 */
    {&htim8, TIM_CHANNEL_4, 1U}, /* BSP_PWM_TIM8_CH4 */
    {&htim4, TIM_CHANNEL_1, 0U}, /* BSP_PWM_TIM4_CH1 */
    {&htim4, TIM_CHANNEL_2, 1U}, /* BSP_PWM_TIM4_CH2 */
    {&htim4, TIM_CHANNEL_3, 0U}, /* BSP_PWM_TIM4_CH3 */
    {&htim4, TIM_CHANNEL_4, 1U}, /* BSP_PWM_TIM4_CH4 */
    {&htim5, TIM_CHANNEL_3, 0U}, /* BSP_PWM_TIM5_CH3 */
    {&htim5, TIM_CHANNEL_4, 1U}, /* BSP_PWM_TIM5_CH4 */
    {&htim1, TIM_CHANNEL_1, 0U}, /* BSP_PWM_TIM1_CH1 */
    {&htim1, TIM_CHANNEL_3, 1U}, /* BSP_PWM_TIM1_CH3 */
};

/**
 * @brief 获取定时器的PWM周期（自动重载值 + 1）
 *
 * @param htim 定时器句柄
 * @return uint32_t 周期计数值（ARR + 1）
 */
static uint32_t Bsp_Pwm_GetPeriod(TIM_HandleTypeDef *htim) {
  return __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
}

/**
 * @brief 将占空比百分比转换为脉冲计数值
 *
 * @param htim          定时器句柄
 * @param duty_percent  占空比（0~100%）
 * @return uint32_t 对应的脉冲计数值（0 ~ 周期值）
 * @note 若 duty_percent > 100，自动截断为100
 */
static uint32_t Bsp_Pwm_DutyToPulse(TIM_HandleTypeDef *htim,
                                    uint8_t duty_percent) {
  if (duty_percent > 100U) {
    duty_percent = 100U;
  }

  return (Bsp_Pwm_GetPeriod(htim) * duty_percent) / 100U;
}

/**
 * @brief 将脉冲宽度转换为定时器比较值（考虑PWM模式极性）
 *
 * @param config  PWM通道配置结构体指针
 * @param pulse   脉冲宽度计数值（0 ~ 周期值）
 * @return uint32_t 写入比较寄存器的值
 * @note 模式1（高有效）：比较值 = pulse
 * @note 模式2（低有效）：比较值 = period - pulse
 * @note 若 pulse > period，则截断为 period
 */
static uint32_t Bsp_Pwm_PulseToCompare(const BspPwmConfig_t *config,
                                       uint32_t pulse) {
  uint32_t period = Bsp_Pwm_GetPeriod(config->htim);

  /* 脉冲值限幅 */
  if (pulse > period) {
    pulse = period;
  }

  /* 根据PWM模式选择比较值 */
  if (config->pwm2_mode != 0U) {
    /* 模式2：极性反转，比较值 = 周期 - 脉冲 */
    return period - pulse;
  }

  /* 模式1：比较值 = 脉冲 */
  return pulse;
}

/**
 * @brief 初始化所有PWM通道
 * @note 将所有通道输出设置为0脉冲（实际根据极性可能输出低或高），并启动PWM输出
 * @warning 若某通道启动失败，将调用 Error_Handler() 进入死循环
 */
void Bsp_Pwm_Init(void) {
  for (uint8_t i = 0U; i < (uint8_t)BSP_PWM_COUNT; i++) {
    /* 设置初始脉冲为0（内部会处理极性） */
    Bsp_Pwm_SetPulse((BspPwmChannel_t)i, 0U);

    /* 启动PWM输出 */
    if (HAL_TIM_PWM_Start(bsp_pwm_configs[i].htim,
                          bsp_pwm_configs[i].channel) != HAL_OK) {
      Error_Handler();
    }
  }
}

/**
 * @brief 设置指定PWM通道的占空比（百分比形式）
 *
 * @param channel       通道号（BspPwmChannel_t枚举）
 * @param duty_percent  占空比（0~100%）
 * @note 内部转换为脉冲值，并自动处理极性
 * @note 若通道号无效或占空比超出范围，会进行限幅或直接返回
 */
void Bsp_Pwm_SetDuty(BspPwmChannel_t channel, uint8_t duty_percent) {
  if ((uint8_t)channel >= (uint8_t)BSP_PWM_COUNT) {
    return;
  }

  Bsp_Pwm_SetPulse(channel, Bsp_Pwm_DutyToPulse(bsp_pwm_configs[channel].htim,
                                                duty_percent));
}

/**
 * @brief 设置指定PWM通道的脉冲宽度（计数值）
 *
 * @param channel 通道号（BspPwmChannel_t枚举）
 * @param pulse   脉冲计数值（0 ~ 周期值）
 * @note 内部会根据通道的PWM模式（模式1/模式2）自动转换比较值
 * @note 若 pulse > 周期值，会自动截断为周期值
 * @note 若通道号无效则直接返回
 */
void Bsp_Pwm_SetPulse(BspPwmChannel_t channel, uint32_t pulse) {
  if ((uint8_t)channel >= (uint8_t)BSP_PWM_COUNT) {
    return;
  }

  __HAL_TIM_SET_COMPARE(
      bsp_pwm_configs[channel].htim, bsp_pwm_configs[channel].channel,
      Bsp_Pwm_PulseToCompare(&bsp_pwm_configs[channel], pulse));
}