/**
 * @file bsp_motor.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）电机PWM驱动头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件定义了电机PWM驱动的硬件接口，基于模块化电机抽象层（module_motor.h）。
 * 支持6个电机（编号0~5），每个电机包含两个PWM输入通道（用于方向/速度控制）。
 * 底层使用TIM1、TIM4、TIM5、TIM8的多个PWM输出通道，具体映射参见源文件。
 * 提供初始化接口和操作函数表供上层调用。
 */

#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h"         /* HAL库及硬件定义 */
#include "module_motor.h" /* 模块化电机抽象层 */

/**
 * @brief 初始化所有电机PWM通道
 * @retval 无
 * @note   - 遍历6个电机，对每个电机的两个PWM通道启动PWM输出
 *         - 初始占空比均为0%，电机处于停止状态
 *         - 若某个PWM通道启动失败，则调用 Error_Handler 进入死循环
 */
void Bsp_Motor_Init(void);

/**
 * @brief 获取电机操作函数表（供模块抽象层使用）
 * @return const ModuleMotorOps_t* 指向包含 SetPwm 回调的函数表指针
 * @note  该操作表被 Module_Motor 模块调用，用于底层PWM占空比设置
 */
const ModuleMotorOps_t *Bsp_Motor_GetOps(void);

#endif /* __BSP_MOTOR_H */