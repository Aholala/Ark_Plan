/**
 * @file module_motor.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-06-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __MODULE_MOTOR_H
#define __MODULE_MOTOR_H

#include <stdint.h>

#define MODULE_MOTOR_COUNT 6U
#define MODULE_MOTOR_MAX_SPEED 100

typedef enum
{
  MODULE_MOTOR_1 = 0,
  MODULE_MOTOR_2,
  MODULE_MOTOR_3,
  MODULE_MOTOR_4,
  MODULE_MOTOR_5,
  MODULE_MOTOR_6,
} ModuleMotorId_t;

typedef enum
{
  MODULE_MOTOR_PWM1 = 0,
  MODULE_MOTOR_PWM2,
} ModuleMotorPwm_t;

typedef struct
{
  void *context;
  void (*set_pwm)(void *context, uint8_t motor, uint8_t pwm, uint8_t duty_percent);
} ModuleMotorOps_t;

typedef struct
{
  const ModuleMotorOps_t *ops;
} ModuleMotor_t;

void Module_Motor_Init(ModuleMotor_t *motor, const ModuleMotorOps_t *ops);
void Module_Motor_SetSpeed(ModuleMotor_t *motor, ModuleMotorId_t id,
                           int8_t speed_percent);
void Module_Motor_Brake(ModuleMotor_t *motor, ModuleMotorId_t id);
void Module_Motor_BrakeAll(ModuleMotor_t *motor);
void Module_Motor_SetComplementaryTest(ModuleMotor_t *motor, uint8_t duty_percent);

#endif
