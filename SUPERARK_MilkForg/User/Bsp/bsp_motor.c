/**
 * @file bsp_motor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）电机PWM驱动源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本文件实现了电机驱动的PWM输出功能，基于多个定时器（TIM1、TIM4、TIM5、TIM8）
 * 的PWM通道，支持最多6个电机，每个电机包含两个PWM输入（正转/反转或方向+速度）。
 * 通过模块化接口（module_motor.h）向上层提供统一的电机控制API。
 */

#include "bsp_motor.h"

#include "tim.h" /* 包含HAL库生成的定时器句柄（htim1, htim4, htim5, htim8） */

/**
 * @brief 电机PWM通道配置结构体
 * @note  用于描述每个电机每个PWM通道所对应的定时器、输出通道和极性反转标志
 */
typedef struct {
  TIM_HandleTypeDef *htim; /**< 定时器句柄指针 */
  uint32_t channel;        /**< 定时器输出通道（如 TIM_CHANNEL_1） */
  uint8_t inverted;        /**< 反转标志（1=占空比反向，0=正常） */
} BspMotorPwmChannel_t;

/**
 * @brief 电机PWM通道映射表 [电机编号][PWM编号]
 * @note  共6个电机（MODULE_MOTOR_COUNT），每个电机2个PWM通道（PWM1和PWM2）
 *        映射关系：
 *        电机0：TIM8 CH1（正向），TIM8 CH2（反向）
 *        电机1：TIM8 CH3（正向），TIM8 CH4（反向）
 *        电机2：TIM4 CH1（正向），TIM4 CH2（反向）
 *        电机3：TIM4 CH3（正向），TIM4 CH4（反向）
 *        电机4：TIM5 CH3（正向），TIM5 CH4（反向）
 *        电机5：TIM1 CH1（正向），TIM1 CH3（反向）
 *        反转标志设置为1的通道在计算占空比时取反，以实现方向控制或适配硬件极性。
 */
static const BspMotorPwmChannel_t motor_pwm_channels[MODULE_MOTOR_COUNT][2] = {
    {{&htim8, TIM_CHANNEL_1, 0U}, {&htim8, TIM_CHANNEL_2, 1U}}, // 电机0
    {{&htim8, TIM_CHANNEL_3, 0U}, {&htim8, TIM_CHANNEL_4, 1U}}, // 电机1
    {{&htim4, TIM_CHANNEL_1, 0U}, {&htim4, TIM_CHANNEL_2, 1U}}, // 电机2
    {{&htim4, TIM_CHANNEL_3, 0U}, {&htim4, TIM_CHANNEL_4, 1U}}, // 电机3
    {{&htim5, TIM_CHANNEL_3, 0U}, {&htim5, TIM_CHANNEL_4, 1U}}, // 电机4
    {{&htim1, TIM_CHANNEL_1, 0U}, {&htim1, TIM_CHANNEL_3, 1U}}, // 电机5
};

/**
 * @brief 将占空比百分比转换为定时器比较值（脉冲宽度）
 * @param htim        定时器句柄
 * @param duty_percent 占空比百分比（0~100）
 * @param inverted    反转标志（1=反向占空比，即 duty_percent 对应互补值）
 * @return uint32_t   比较寄存器值（脉冲宽度）
 * @note  根据自动重载值计算脉冲宽度，若 inverted 为1，则输出 (period - pulse)
 *        以达到反向效果（例如用于H桥的互补PWM）
 */
static uint32_t Bsp_Motor_DutyToPulse(TIM_HandleTypeDef *htim,
                                      uint8_t duty_percent, uint8_t inverted) {
  uint32_t period =
      __HAL_TIM_GET_AUTORELOAD(htim) + 1U; /* 自动重载值 + 1 = 周期计数值 */
  uint32_t pulse;

  /* 限幅 */
  if (duty_percent > 100U) {
    duty_percent = 100U;
  }

  /* 计算正常脉冲宽度 */
  pulse = (period * duty_percent) / 100U;

  /* 若需要反转，则输出互补值 */
  if (inverted != 0U) {
    pulse = period - pulse;
  }

  return pulse;
}

/**
 * @brief 设置指定电机PWM通道的占空比（模块回调函数）
 * @param context      上下文指针（本驱动中未使用，传入NULL）
 * @param motor        电机编号（0 ~ MODULE_MOTOR_COUNT-1）
 * @param pwm          PWM通道编号（MODULE_MOTOR_PWM1 或 MODULE_MOTOR_PWM2）
 * @param duty_percent 占空比百分比（0~100）
 * @retval 无
 * @note  该函数通过模块层（ModuleMotor）调用，更新对应定时器比较寄存器的值
 *        以实现PWM占空比调节。若参数无效则直接返回。
 */
static void Bsp_Motor_SetPwm(void *context, uint8_t motor, uint8_t pwm,
                             uint8_t duty_percent) {
  const BspMotorPwmChannel_t *channel;

  (void)context;

  /* 检查电机编号和PWM编号有效性 */
  if ((motor >= MODULE_MOTOR_COUNT) || (pwm > MODULE_MOTOR_PWM2)) {
    return;
  }

  /* 获取对应通道配置 */
  channel = &motor_pwm_channels[motor][pwm];

  /* 更新比较值（占空比） */
  __HAL_TIM_SET_COMPARE(
      channel->htim, channel->channel,
      Bsp_Motor_DutyToPulse(channel->htim, duty_percent, channel->inverted));
}

/**
 * @brief 启动单个PWM通道（初始化时调用）
 * @param channel PWM通道配置指针
 * @retval 无
 * @note  先设置比较值为0（占空比0%），然后启动PWM输出。
 *        若启动失败则调用 Error_Handler。
 */
static void Bsp_Motor_StartPwm(const BspMotorPwmChannel_t *channel) {
  /* 初始占空比设为0（考虑反转标志） */
  __HAL_TIM_SET_COMPARE(
      channel->htim, channel->channel,
      Bsp_Motor_DutyToPulse(channel->htim, 0U, channel->inverted));

  /* 启动PWM输出 */
  if (HAL_TIM_PWM_Start(channel->htim, channel->channel) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief 电机操作函数表（虚函数表，供模块抽象层使用）
 * @note  仅包含 SetPwm 回调，版本号预留为0
 */
static const ModuleMotorOps_t motor_ops = {
    0,                /* 版本号 */
    Bsp_Motor_SetPwm, /* 设置PWM占空比回调 */
};

/**
 * @brief 初始化所有电机PWM通道
 * @retval 无
 * @note  遍历所有电机，对每个电机的两个PWM通道调用 Bsp_Motor_StartPwm
 * 启动输出。 所有通道初始占空比为0%，电机处于停止状态。
 */
void Bsp_Motor_Init(void) {
  for (uint8_t motor = 0U; motor < MODULE_MOTOR_COUNT; motor++) {
    /* 启动PWM1通道 */
    Bsp_Motor_StartPwm(&motor_pwm_channels[motor][MODULE_MOTOR_PWM1]);
    /* 启动PWM2通道 */
    Bsp_Motor_StartPwm(&motor_pwm_channels[motor][MODULE_MOTOR_PWM2]);
  }
}

/**
 * @brief 获取电机操作函数表（供其他模块使用）
 * @return const ModuleMotorOps_t* 指向操作函数表的指针
 * @note  通常由模块层调用以获取底层驱动回调
 */
const ModuleMotorOps_t *Bsp_Motor_GetOps(void) { return &motor_ops; }