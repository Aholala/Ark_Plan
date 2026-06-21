/**
 * @file module_motor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-06-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "module_motor.h"

static uint8_t Module_Motor_ClampDuty(uint8_t duty_percent)
{
  return (duty_percent > 100U) ? 100U : duty_percent;
}

static int8_t Module_Motor_ClampSpeed(int8_t speed_percent)
{
  if (speed_percent > MODULE_MOTOR_MAX_SPEED)
  {
    return MODULE_MOTOR_MAX_SPEED;
  }

  if (speed_percent < -MODULE_MOTOR_MAX_SPEED)
  {
    return -MODULE_MOTOR_MAX_SPEED;
  }

  return speed_percent;
}

static void Module_Motor_SetPwm(ModuleMotor_t *motor, ModuleMotorId_t id,
                                ModuleMotorPwm_t pwm, uint8_t duty_percent)
{
  if ((motor == 0) || (motor->ops == 0) || (motor->ops->set_pwm == 0))
  {
    return;
  }

  if ((uint8_t)id >= MODULE_MOTOR_COUNT)
  {
    return;
  }

  motor->ops->set_pwm(motor->ops->context, (uint8_t)id, (uint8_t)pwm,
                      Module_Motor_ClampDuty(duty_percent));
}

void Module_Motor_Init(ModuleMotor_t *motor, const ModuleMotorOps_t *ops)
{
  if (motor == 0)
  {
    return;
  }

  motor->ops = ops;
}

void Module_Motor_SetSpeed(ModuleMotor_t *motor, ModuleMotorId_t id,
                           int8_t speed_percent)
{
  uint8_t abs_speed;

  speed_percent = Module_Motor_ClampSpeed(speed_percent);
  abs_speed = (speed_percent < 0) ? (uint8_t)(-speed_percent) : (uint8_t)speed_percent;

  if (speed_percent > 0)
  {
    Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM1, abs_speed);
    Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM2, 0U);
  }
  else if (speed_percent < 0)
  {
    Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM1, 0U);
    Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM2, abs_speed);
  }
  else
  {
    Module_Motor_Brake(motor, id);
  }
}

void Module_Motor_Brake(ModuleMotor_t *motor, ModuleMotorId_t id)
{
  Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM1, 0U);
  Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM2, 0U);
}

void Module_Motor_BrakeAll(ModuleMotor_t *motor)
{
  for (uint8_t i = 0U; i < MODULE_MOTOR_COUNT; i++)
  {
    Module_Motor_Brake(motor, (ModuleMotorId_t)i);
  }
}

void Module_Motor_SetComplementaryTest(ModuleMotor_t *motor, uint8_t duty_percent)
{
  duty_percent = Module_Motor_ClampDuty(duty_percent);

  for (uint8_t i = 0U; i < MODULE_MOTOR_COUNT; i++)
  {
    Module_Motor_SetPwm(motor, (ModuleMotorId_t)i, MODULE_MOTOR_PWM1, duty_percent);
    Module_Motor_SetPwm(motor, (ModuleMotorId_t)i, MODULE_MOTOR_PWM2, duty_percent);
  }
}
