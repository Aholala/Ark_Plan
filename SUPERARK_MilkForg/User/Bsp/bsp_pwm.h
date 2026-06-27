/**
 * @file bsp_pwm.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包 - PWM驱动头文件
 * @version 1.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 定义PWM通道枚举及初始化、占空比/脉冲设置接口。
 *          支持多个定时器通道映射，提供统一的PWM控制API。
 */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief PWM通道枚举
 * @note 顺序必须与 bsp_pwm.c 中的硬件映射表一致
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

typedef enum {
  BSP_PWM_MOTOR_1 = 0,
  BSP_PWM_MOTOR_2,
  BSP_PWM_MOTOR_3,
  BSP_PWM_MOTOR_4,
  BSP_PWM_MOTOR_5,
  BSP_PWM_MOTOR_6,
  BSP_PWM_MOTOR_COUNT
} BspPwmMotor_t;

/**
 * @brief 初始化所有PWM通道
 * @note 将所有通道输出初始为0脉冲，并启动PWM定时器
 * @warning 若某个通道启动失败，会进入 Error_Handler()
 */
void Bsp_Pwm_Init(void);

/**
 * @brief 设置指定PWM通道的占空比（百分比形式）
 *
 * @param channel       通道号（参见 BspPwmChannel_t）
 * @param duty_percent  占空比（0~100%）
 * @note 占空比会自动限幅到0~100%，内部转换为脉冲计数值
 * @note 若通道号无效则函数直接返回
 */
void Bsp_Pwm_SetDuty(BspPwmChannel_t channel, uint8_t duty_percent);

void Bsp_Pwm_SetMotorDuty(BspPwmMotor_t motor, int8_t duty_percent);

void Bsp_Pwm_StopMotor(BspPwmMotor_t motor);

void Bsp_Pwm_StopAllMotors(void);

/**
 * @brief 设置指定PWM通道的脉冲宽度（定时器计数值）
 *
 * @param channel 通道号（参见 BspPwmChannel_t）
 * @param pulse   脉冲计数值（0 ~ 自动重载值）
 * @note 直接写入比较寄存器，不进行限幅，调用者需保证值合法
 * @note 若通道号无效则函数直接返回
 */
void Bsp_Pwm_SetPulse(BspPwmChannel_t channel, uint32_t pulse);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
