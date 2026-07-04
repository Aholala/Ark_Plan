/**
 * @file app_chassis_task.h
 * @brief Chassis open-loop motor task interface.
 */

#ifndef __APP_CHASSIS_TASK_H
#define __APP_CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "module_motor.h"

void App_Chassis_Init(void);
void App_Chassis_SetMotorDuty(ModuleMotorId_t motor, int8_t duty_percent);
void App_Chassis_StopMotor(ModuleMotorId_t motor);
void App_Chassis_StopAllMotors(void);
void App_Chassis_Task(void);
void StartChassicTask(void *argument);
void StartChassisTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CHASSIS_TASK_H */
