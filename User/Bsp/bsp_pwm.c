/**
 * @file bsp_pwm.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包 - PWM输出驱动实现
 * @version 1.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块提供PWM通道的初始化和占空比/脉冲设置功能。
 *          针对电机或执行器驱动，加入了占空比限幅机制：
 *          - 最小有效占空比（5%），避免死区或无效驱动；
 *          - 最大占空比（95%），留有余量防止饱和。
 *          所有PWM通道启动时输出为0，确保安全。
 */

#include "bsp_pwm.h"
#include "tim.h"

/*==================== 宏定义 ====================*/

/** @brief 最小有效占空比（%），低于此值视为0，避免死区或非线性区 */
#define BSP_PWM_MIN_EFFECTIVE_DUTY 5U

/** @brief 最大允许占空比（%），防止100%占空比导致驱动饱和或发热 */
#define BSP_PWM_MAX_DUTY 95U

/*==================== 类型定义 ====================*/

/**
 * @brief PWM通道配置结构体
 * @note 包含定时器句柄和对应通道号
 */
typedef struct {
  TIM_HandleTypeDef *htim; /**< 定时器句柄指针 */
  uint32_t channel;        /**< 定时器通道（如 TIM_CHANNEL_1） */
} BspPwmConfig_t;

/*==================== 硬件映射表 ====================*/

/**
 * @brief PWM通道硬件映射表
 * @note 顺序必须与 BspPwmChannel_t 枚举定义一致
 */
static const BspPwmConfig_t bsp_pwm_configs[BSP_PWM_COUNT] = {
    {&htim8, TIM_CHANNEL_1}, /* BSP_PWM_TIM8_CH1 */
    {&htim8, TIM_CHANNEL_2}, /* BSP_PWM_TIM8_CH2 */
    {&htim8, TIM_CHANNEL_3}, /* BSP_PWM_TIM8_CH3 */
    {&htim8, TIM_CHANNEL_4}, /* BSP_PWM_TIM8_CH4 */
    {&htim4, TIM_CHANNEL_1}, /* BSP_PWM_TIM4_CH1 */
    {&htim4, TIM_CHANNEL_2}, /* BSP_PWM_TIM4_CH2 */
    {&htim4, TIM_CHANNEL_3}, /* BSP_PWM_TIM4_CH3 */
    {&htim4, TIM_CHANNEL_4}, /* BSP_PWM_TIM4_CH4 */
    {&htim5, TIM_CHANNEL_3}, /* BSP_PWM_TIM5_CH3 */
    {&htim5, TIM_CHANNEL_4}, /* BSP_PWM_TIM5_CH4 */
    {&htim1, TIM_CHANNEL_1}, /* BSP_PWM_TIM1_CH1 */
    {&htim1, TIM_CHANNEL_3}, /* BSP_PWM_TIM1_CH3 */
};

/*==================== 静态函数（工具函数） ====================*/

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
 * @brief 对脉冲宽度进行限幅，防止超出周期范围
 *
 * @param htim  定时器句柄
 * @param pulse 待限幅的脉冲值
 * @return uint32_t 限幅后的脉冲值（0 ~ 周期值）
 */
static uint32_t Bsp_Pwm_LimitPulse(TIM_HandleTypeDef *htim, uint32_t pulse) {
  uint32_t period = Bsp_Pwm_GetPeriod(htim);

  if (pulse > period) {
    pulse = period;
  }

  return pulse;
}

/**
 * @brief 对占空比进行限幅（应用最小/最大有效占空比）
 *
 * @param duty_percent 原始占空比（0~100%）
 * @return uint8_t 限幅后的占空比
 * @retval 0 当 duty_percent == 0 时直接返回0
 * @retval BSP_PWM_MIN_EFFECTIVE_DUTY 当原始值在1~4%之间时
 * @retval BSP_PWM_MAX_DUTY 当原始值大于95%时
 * @retval 原始值 当原始值在5%~95%之间时
 *
 * @note 此限幅用于保护电机或执行器，避免在死区或饱和区工作。
 */
static uint8_t Bsp_Pwm_LimitDuty(uint8_t duty_percent) {
  if (duty_percent == 0U) {
    return 0U;
  }
  if (duty_percent < BSP_PWM_MIN_EFFECTIVE_DUTY) {
    return BSP_PWM_MIN_EFFECTIVE_DUTY;
  }
  if (duty_percent > BSP_PWM_MAX_DUTY) {
    return BSP_PWM_MAX_DUTY;
  }
  return duty_percent;
}

/**
 * @brief 将占空比百分比转换为脉冲计数值（内部自动限幅）
 *
 * @param htim          定时器句柄
 * @param duty_percent  占空比（0~100%）
 * @return uint32_t 对应的脉冲计数值（经过限幅后计算）
 * @note 内部调用 Bsp_Pwm_LimitDuty 进行占空比限幅
 */
static uint32_t Bsp_Pwm_DutyToPulse(TIM_HandleTypeDef *htim,
                                    uint8_t duty_percent) {
  duty_percent = Bsp_Pwm_LimitDuty(duty_percent);

  return (Bsp_Pwm_GetPeriod(htim) * duty_percent) / 100U;
}

/*==================== 全局API函数 ====================*/

/**
 * @brief 初始化所有PWM通道
 * @note 将所有通道输出设置为0脉冲，并启动PWM定时器
 * @warning 若某通道启动失败，将调用 Error_Handler() 进入死循环
 */
void Bsp_Pwm_Init(void) {
  for (uint8_t i = 0U; i < (uint8_t)BSP_PWM_COUNT; i++) {
    /* 初始输出为0，确保安全 */
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
 * @note 占空比会经过限幅处理（5%~95%，0保持为0）
 * @note 若通道号无效则函数直接返回
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
 * @note 内部会自动限幅，确保 pulse 不超过周期值
 * @note 若通道号无效则函数直接返回
 */
void Bsp_Pwm_SetPulse(BspPwmChannel_t channel, uint32_t pulse) {
  if ((uint8_t)channel >= (uint8_t)BSP_PWM_COUNT) {
    return;
  }

  __HAL_TIM_SET_COMPARE(
      bsp_pwm_configs[channel].htim, bsp_pwm_configs[channel].channel,
      Bsp_Pwm_LimitPulse(bsp_pwm_configs[channel].htim, pulse));
}