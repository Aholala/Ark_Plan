/**
 * @file module_motor.c
 * @brief Motor module implementation.
 */

#include "module_motor.h"

#include "bsp_encoder.h"
#include "bsp_pwm.h"

#define MODULE_MOTOR_NO_ENCODER 0xFFU

typedef struct {
  BspPwmChannel_t in_a;
  BspPwmChannel_t in_b;
  uint8_t encoder;
} ModuleMotorConfig_t;

static const ModuleMotorConfig_t module_motor_configs[MODULE_MOTOR_COUNT] = {
    {BSP_PWM_TIM4_CH3, BSP_PWM_TIM4_CH4, MODULE_MOTOR_NO_ENCODER}, /* M1: PB8 / PB9, open loop */
    {BSP_PWM_TIM4_CH1, BSP_PWM_TIM4_CH2, MODULE_MOTOR_NO_ENCODER}, /* M2: PB6 / PB7, open loop */
    {BSP_PWM_TIM8_CH3, BSP_PWM_TIM8_CH4, MODULE_MOTOR_NO_ENCODER}, /* M3: PC8 / PC9, open loop */
    {BSP_PWM_TIM8_CH1, BSP_PWM_TIM8_CH2, MODULE_MOTOR_NO_ENCODER}, /* M4: PC6 / PC7, open loop */
    {BSP_PWM_TIM5_CH3, BSP_PWM_TIM5_CH4, BSP_ENCODER_M5},          /* M5: PA2 / PA3, encoder TIM2 */
    {BSP_PWM_TIM1_CH1, BSP_PWM_TIM1_CH3, BSP_ENCODER_M6},          /* M6: PA8 / PA10, encoder TIM3 */
};

static uint8_t Module_Motor_IsValid(ModuleMotorId_t motor) {
  return (uint8_t)motor < (uint8_t)MODULE_MOTOR_COUNT;
}

static uint8_t Module_Motor_AbsDuty(int8_t duty_percent) {
  if (duty_percent < 0) {
    return (uint8_t)(-duty_percent);
  }

  return (uint8_t)duty_percent;
}

void Module_Motor_Init(void) {
  Module_Motor_StopAll();
}

void Module_Motor_SetDuty(ModuleMotorId_t motor, int8_t duty_percent) {
  const ModuleMotorConfig_t *config;
  uint8_t duty;

  if (!Module_Motor_IsValid(motor)) {
    return;
  }

  if (duty_percent > 95) {
    duty_percent = 95;
  } else if (duty_percent < -95) {
    duty_percent = -95;
  }

  config = &module_motor_configs[motor];
  duty = Module_Motor_AbsDuty(duty_percent);

  if (duty_percent > 0) {
    Bsp_Pwm_SetDuty(config->in_b, 0U);
    Bsp_Pwm_SetDuty(config->in_a, duty);
  } else if (duty_percent < 0) {
    Bsp_Pwm_SetDuty(config->in_a, 0U);
    Bsp_Pwm_SetDuty(config->in_b, duty);
  } else {
    Bsp_Pwm_SetDuty(config->in_a, 0U);
    Bsp_Pwm_SetDuty(config->in_b, 0U);
  }
}

void Module_Motor_Stop(ModuleMotorId_t motor) {
  Module_Motor_SetDuty(motor, 0);
}

void Module_Motor_StopAll(void) {
  for (uint8_t i = 0U; i < (uint8_t)MODULE_MOTOR_COUNT; i++) {
    Module_Motor_Stop((ModuleMotorId_t)i);
  }
}

uint8_t Module_Motor_HasEncoder(ModuleMotorId_t motor) {
  if (!Module_Motor_IsValid(motor)) {
    return 0U;
  }

  return module_motor_configs[motor].encoder != MODULE_MOTOR_NO_ENCODER;
}

uint32_t Module_Motor_GetEncoderCountsPerRev(ModuleMotorId_t motor) {
  if (!Module_Motor_HasEncoder(motor)) {
    return 0U;
  }

  return MODULE_MOTOR_ENCODER_COUNTS_PER_REV;
}

int32_t Module_Motor_GetEncoderCount(ModuleMotorId_t motor) {
  if (!Module_Motor_HasEncoder(motor)) {
    return 0;
  }

  return Bsp_Encoder_GetCount((BspEncoderId_t)module_motor_configs[motor].encoder);
}

int32_t Module_Motor_GetEncoderDelta(ModuleMotorId_t motor) {
  if (!Module_Motor_HasEncoder(motor)) {
    return 0;
  }

  return Bsp_Encoder_GetDelta((BspEncoderId_t)module_motor_configs[motor].encoder);
}

void Module_Motor_ResetEncoder(ModuleMotorId_t motor) {
  if (!Module_Motor_HasEncoder(motor)) {
    return;
  }

  Bsp_Encoder_Reset((BspEncoderId_t)module_motor_configs[motor].encoder);
}
