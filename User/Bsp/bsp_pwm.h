/**
 * @file bsp_pwm.h
 * @brief Board-level PWM output interface.
 */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum {
  BSP_PWM_TIM8_CH1 = 0,
  BSP_PWM_TIM8_CH2,
  BSP_PWM_TIM8_CH3,
  BSP_PWM_TIM8_CH4,
  BSP_PWM_TIM4_CH1,
  BSP_PWM_TIM4_CH2,
  BSP_PWM_TIM4_CH3,
  BSP_PWM_TIM4_CH4,
  BSP_PWM_TIM5_CH3,
  BSP_PWM_TIM5_CH4,
  BSP_PWM_TIM1_CH1,
  BSP_PWM_TIM1_CH3,
  BSP_PWM_COUNT
} BspPwmChannel_t;

void Bsp_Pwm_Init(void);

/**
 * @brief Set channel duty by percent.
 * @note 0 means stop/brake. Non-zero values below 5% are raised to 5%, and
 *       values above 95% are clamped to 95%.
 */
void Bsp_Pwm_SetDuty(BspPwmChannel_t channel, uint8_t duty_percent);

/**
 * @brief Set raw compare pulse.
 * @note Pulse is clamped to the timer period. This raw API does not apply the
 *       5%/95% duty policy.
 */
void Bsp_Pwm_SetPulse(BspPwmChannel_t channel, uint32_t pulse);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
