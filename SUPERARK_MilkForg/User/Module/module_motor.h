/**
 * @file module_motor.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块化电机控制抽象层头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本模块提供了电机的统一控制接口，通过操作函数表（ops）实现底层硬件与
 * 上层应用的解耦。支持多个电机的速度控制（带方向）、刹车、全速测试等功能。
 * 速度值范围为 -100 ~ +100（由 MODULE_MOTOR_MAX_SPEED 定义），通过两个PWM通道
 * （PWM1/PWM2）实现正反转控制（或方向+使能）。
 */

#ifndef __MODULE_MOTOR_H
#define __MODULE_MOTOR_H

#include <stdint.h> /* 标准整数类型定义 */

/**
 * @brief 电机数量（支持6个电机）
 */
#define MODULE_MOTOR_COUNT 6U

/**
 * @brief 最大速度百分比（绝对值）
 * @note  速度输入值将被限幅到此范围内，取值范围为 -100 ~ +100
 */
#define MODULE_MOTOR_MAX_SPEED 100

/**
 * @brief 电机ID枚举
 * @note  用于标识不同的电机通道
 */
typedef enum {
  MODULE_MOTOR_1 = 0, /**< 电机1 */
  MODULE_MOTOR_2,     /**< 电机2 */
  MODULE_MOTOR_3,     /**< 电机3 */
  MODULE_MOTOR_4,     /**< 电机4 */
  MODULE_MOTOR_5,     /**< 电机5 */
  MODULE_MOTOR_6,     /**< 电机6 */
} ModuleMotorId_t;

/**
 * @brief 每个电机的PWM通道枚举
 * @note  通常使用双PWM实现正反转控制（PWM1=正向，PWM2=反向）
 *        或PWM+方向信号方式（PWM1=占空比，PWM2=方向）
 */
typedef enum {
  MODULE_MOTOR_PWM1 = 0, /**< PWM通道1（通常为正向驱动） */
  MODULE_MOTOR_PWM2,     /**< PWM通道2（通常为反向驱动） */
} ModuleMotorPwm_t;

/**
 * @brief 电机操作函数表（虚函数表）
 * @note  底层驱动需要实现这些回调函数，供模块层调用
 */
typedef struct {
  void *context; /**< 上下文指针（底层驱动可传递任意数据） */

  /**
   * @brief 设置指定电机PWM通道占空比的回调函数
   * @param context      上下文指针（由底层驱动使用）
   * @param motor        电机ID（0~MODULE_MOTOR_COUNT-1）
   * @param pwm          PWM通道（0或1，对应 ModuleMotorPwm_t）
   * @param duty_percent 占空比（0~100）
   * @retval 无
   */
  void (*set_pwm)(void *context, uint8_t motor, uint8_t pwm,
                  uint8_t duty_percent);
} ModuleMotorOps_t;

/**
 * @brief 电机模块结构体
 * @note  存储操作函数表指针，无其他状态数据（电机状态由应用层管理）
 */
typedef struct {
  const ModuleMotorOps_t *ops; /**< 操作函数表指针 */
} ModuleMotor_t;

/**
 * @brief 初始化电机模块
 * @param motor 指向电机模块结构体的指针（输出）
 * @param ops   指向操作函数表的指针（输入）
 * @retval 无
 * @note  保存操作函数表指针，若 motor 为空则直接返回
 */
void Module_Motor_Init(ModuleMotor_t *motor, const ModuleMotorOps_t *ops);

/**
 * @brief 设置指定电机的速度（带方向）
 * @param motor         指向电机模块结构体的指针
 * @param id            电机ID
 * @param speed_percent 速度百分比（-100 ~ +100，负值反转）
 * @retval 无
 * @note   - 正速度：PWM1输出占空比，PWM2置0
 *         - 负速度：PWM1置0，PWM2输出占空比（绝对值）
 *         - 零速度：调用刹车函数（两路PWM均置0）
 *         - 速度值自动限幅到 ±MODULE_MOTOR_MAX_SPEED 范围内
 */
void Module_Motor_SetSpeed(ModuleMotor_t *motor, ModuleMotorId_t id,
                           int8_t speed_percent);

/**
 * @brief 刹车（停止）指定电机
 * @param motor 指向电机模块结构体的指针
 * @param id    电机ID
 * @retval 无
 * @note  将两个PWM通道的占空比均设为0，实现制动
 */
void Module_Motor_Brake(ModuleMotor_t *motor, ModuleMotorId_t id);

/**
 * @brief 刹车所有电机
 * @param motor 指向电机模块结构体的指针
 * @retval 无
 * @note  遍历所有电机，逐个调用 Module_Motor_Brake
 */
void Module_Motor_BrakeAll(ModuleMotor_t *motor);

/**
 * @brief 互补PWM测试模式（两个通道同时输出相同占空比）
 * @param motor         指向电机模块结构体的指针
 * @param duty_percent  占空比（0~100）
 * @retval 无
 * @note   - 该功能通常用于测试PWM硬件是否正常工作
 *         - 所有电机的两个PWM通道输出相同的占空比，电机可能不转（正反转抵消）
 *         - 占空比自动限幅到0~100
 */
void Module_Motor_SetComplementaryTest(ModuleMotor_t *motor,
                                       uint8_t duty_percent);

#endif /* __MODULE_MOTOR_H */