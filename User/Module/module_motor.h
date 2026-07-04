/**
 * @file module_motor.h
 * @brief Motor module with PWM drive and optional encoder feedback.
 */

#ifndef __MODULE_MOTOR_H
#define __MODULE_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
  MODULE_MOTOR_1 = 0,
  MODULE_MOTOR_2,
  MODULE_MOTOR_3,
  MODULE_MOTOR_4,
  MODULE_MOTOR_5,
  MODULE_MOTOR_6,
  MODULE_MOTOR_COUNT
} ModuleMotorId_t;

#define MODULE_MOTOR_ENCODER_PHASE_PPR 12U
#define MODULE_MOTOR_ENCODER_QUADRATURE_MULTIPLIER 4U
#define MODULE_MOTOR_ENCODER_COUNTS_PER_REV                          \
  (MODULE_MOTOR_ENCODER_PHASE_PPR *                                  \
   MODULE_MOTOR_ENCODER_QUADRATURE_MULTIPLIER)

void Module_Motor_Init(void);
void Module_Motor_SetDuty(ModuleMotorId_t motor, int8_t duty_percent);
void Module_Motor_Stop(ModuleMotorId_t motor);
void Module_Motor_StopAll(void);
uint8_t Module_Motor_HasEncoder(ModuleMotorId_t motor);
uint32_t Module_Motor_GetEncoderCountsPerRev(ModuleMotorId_t motor);
int32_t Module_Motor_GetEncoderCount(ModuleMotorId_t motor);
int32_t Module_Motor_GetEncoderDelta(ModuleMotorId_t motor);
void Module_Motor_ResetEncoder(ModuleMotorId_t motor);

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_MOTOR_H */
