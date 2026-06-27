/**
 * @file bsp_pwm.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包 - PWM驱动实现（支持IR2104桥式驱动）
 * @version 1.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块提供PWM通道的初始化、占空比设置以及电机控制功能。
 *          支持多路PWM输出，并将PWM通道配对为6组电机控制信号（IN_A/IN_B），
 *          用于驱动IR2104等桥式驱动器。电机控制接口支持正反转和速度调节，
 *          占空比范围-100%～+100%，负值表示反转。
 */

#include "bsp_pwm.h"
#include "tim.h"

/**
 * @brief PWM通道配置结构体
 * @note 包含定时器句柄和对应通道号
 */
typedef struct {
  TIM_HandleTypeDef *htim; /**< 定时器句柄指针 */
  uint32_t channel;        /**< 定时器通道（如 TIM_CHANNEL_1） */
} BspPwmConfig_t;

/**
 * @brief 电机PWM配对配置结构体
 * @note 每个电机由两个PWM通道控制（IN_A和IN_B），
 *       对应桥式驱动器的上下臂或正反方向控制。
 */
typedef struct {
  BspPwmChannel_t in_a; /**< 电机A相控制通道（正向） */
  BspPwmChannel_t in_b; /**< 电机B相控制通道（反向） */
} BspPwmMotorConfig_t;

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

/**
 * @brief 电机PWM配对映射表
 * @note 顺序必须与 BspPwmMotor_t 枚举定义一致
 */
static const BspPwmMotorConfig_t bsp_pwm_motor_configs[BSP_PWM_MOTOR_COUNT] = {
    {BSP_PWM_TIM8_CH1, BSP_PWM_TIM8_CH2}, /* 电机 M1 */
    {BSP_PWM_TIM8_CH3, BSP_PWM_TIM8_CH4}, /* 电机 M2 */
    {BSP_PWM_TIM4_CH1, BSP_PWM_TIM4_CH2}, /* 电机 M3 */
    {BSP_PWM_TIM4_CH3, BSP_PWM_TIM4_CH4}, /* 电机 M4 */
    {BSP_PWM_TIM5_CH3, BSP_PWM_TIM5_CH4}, /* 电机 M5 */
    {BSP_PWM_TIM1_CH1, BSP_PWM_TIM1_CH3}, /* 电机 M6 */
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
 * @brief 初始化所有PWM通道
 * @note 将所有通道输出设置为0脉冲，并启动PWM定时器
 * @warning 若某通道启动失败，将调用 Error_Handler() 进入死循环
 */
void Bsp_Pwm_Init(void) {
  for (uint8_t i = 0U; i < (uint8_t)BSP_PWM_COUNT; i++) {
    /* 设置初始脉冲为0 */
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
 * @note 若 duty_percent > 100 会被截断为100
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

/**
 * @brief 设置电机转速和方向（带符号占空比）
 *
 * @param motor         电机编号（BspPwmMotor_t枚举）
 * @param duty_percent  占空比（-100% ~ +100%）
 *                      - 正值：正向旋转（IN_A有效，IN_B为0）
 *                      - 负值：反向旋转（IN_B有效，IN_A为0）
 *                      - 0：停止（两路均为0）
 * @note 若 duty_percent 超出 [-100, 100] 范围，自动限幅
 * @note 若电机编号无效则函数直接返回
 *
 * @details 本函数适用于半桥或全桥驱动器，通过控制IN_A和IN_B的占空比实现正反转。
 *          正向时，IN_A输出占空比 = duty_percent，IN_B为0；
 *          反向时，IN_B输出占空比 = -duty_percent，IN_A为0。
 *          注意：此驱动方式为互补模式，若需死区控制，需在硬件或定时器配置中实现。
 */
void Bsp_Pwm_SetMotorDuty(BspPwmMotor_t motor, int8_t duty_percent) {
  const BspPwmMotorConfig_t *config;
  uint8_t duty;

  /* 通道号有效性检查 */
  if ((uint8_t)motor >= (uint8_t)BSP_PWM_MOTOR_COUNT) {
    return;
  }

  /* 占空比限幅 */
  if (duty_percent > 100) {
    duty_percent = 100;
  } else if (duty_percent < -100) {
    duty_percent = -100;
  }

  config = &bsp_pwm_motor_configs[motor];

  if (duty_percent >= 0) {
    /* 正向：IN_A = duty, IN_B = 0 */
    duty = (uint8_t)duty_percent;
    Bsp_Pwm_SetDuty(config->in_b, 0U);
    Bsp_Pwm_SetDuty(config->in_a, duty);
  } else {
    /* 反向：IN_A = 0, IN_B = -duty */
    duty = (uint8_t)(-duty_percent);
    Bsp_Pwm_SetDuty(config->in_a, 0U);
    Bsp_Pwm_SetDuty(config->in_b, duty);
  }
}

/**
 * @brief 停止指定电机（两路PWM均输出0）
 *
 * @param motor 电机编号（BspPwmMotor_t枚举）
 * @note 若电机编号无效则函数直接返回
 */
void Bsp_Pwm_StopMotor(BspPwmMotor_t motor) {
  const BspPwmMotorConfig_t *config;

  if ((uint8_t)motor >= (uint8_t)BSP_PWM_MOTOR_COUNT) {
    return;
  }

  config = &bsp_pwm_motor_configs[motor];
  Bsp_Pwm_SetDuty(config->in_a, 0U);
  Bsp_Pwm_SetDuty(config->in_b, 0U);
}

/**
 * @brief 停止所有电机
 * @note 遍历所有电机，调用 Bsp_Pwm_StopMotor()
 */
void Bsp_Pwm_StopAllMotors(void) {
  for (uint8_t i = 0U; i < (uint8_t)BSP_PWM_MOTOR_COUNT; i++) {
    Bsp_Pwm_StopMotor((BspPwmMotor_t)i);
  }
}