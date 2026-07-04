/**
 * @file app_chassis_task.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 底盘开环控制任务 - 头文件
 * @version 1.0
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 * @details 声明底盘控制任务的初始化、电机控制接口及任务入口函数。
 *          支持对四轮底盘电机进行开环占空比控制，配合遥控器输入和运动学解算。
 */

#ifndef __APP_CHASSIS_TASK_H
#define __APP_CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "module_motor.h"

/**
 * @brief 初始化底盘电机状态（停止所有电机，清除内部状态）
 * @note 应在任务启动前调用，确保电机处于安全状态。
 */
void App_Chassis_Init(void);

/**
 * @brief 设置底盘电机的目标占空比（外部调用接口）
 *
 * @param motor         电机编号（必须为 MODULE_MOTOR_1~4，否则无效）
 * @param duty_percent  目标占空比（-100~100），内部会自动限幅和死区处理
 * @note 若电机编号不在底盘范围内，函数直接返回。
 * @note 实际输出会经过斜坡处理，不会立即跳变。
 */
void App_Chassis_SetMotorDuty(ModuleMotorId_t motor, int8_t duty_percent);

/**
 * @brief 停止指定底盘电机（目标占空比设为0）
 *
 * @param motor 电机编号
 * @note 内部调用 App_Chassis_SetMotorDuty(motor, 0)，会经过斜坡减速。
 */
void App_Chassis_StopMotor(ModuleMotorId_t motor);

/**
 * @brief 停止所有底盘电机（目标占空比归零）
 * @note 不会立即停止输出，而是通过斜坡逐渐降至0，避免急停冲击。
 */
void App_Chassis_StopAllMotors(void);

/**
 * @brief 底盘任务主函数（周期性调用）
 * @note 此函数需以固定周期（CHASSIS_MOTOR_RAMP_PERIOD_MS）被调用。
 *       其工作流程：
 *       1. 读取遥控器数据并更新目标占空比；
 *       2. 遍历所有电机，执行状态更新（斜坡/刹车/输出）。
 */
void App_Chassis_Task(void);

/**
 * @brief 底盘任务入口函数（FreeRTOS任务）
 *
 * @param argument 任务参数（未使用）
 * @note 任务初始化后周期性调用 App_Chassis_Task。
 * @warning 此函数名拼写为 StartChassicTask（历史遗留），
 *          为保持兼容同时提供了 StartChassisTask 作为别名。
 */
void StartChassicTask(void *argument);

/**
 * @brief 底盘任务入口函数（正确拼写，供外部调用）
 *
 * @param argument 任务参数（未使用）
 * @note 直接调用 StartChassicTask 实现，以兼容两种命名。
 */
void StartChassisTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CHASSIS_TASK_H */