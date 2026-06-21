/**
 * @file module_motor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块化电机控制抽象层源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本模块提供了电机的统一控制接口，通过操作函数表（ops）实现底层硬件与
 * 上层应用的解耦。支持多个电机的速度控制（带方向）、刹车、全速测试等功能。
 * 速度值范围为 -100 ~ +100（由 MODULE_MOTOR_MAX_SPEED 定义，通常为100），
 * 通过两个PWM通道（PWM1/PWM2）实现正反转控制。
 */

#include "module_motor.h"

/**
 * @brief 限幅占空比（限制到0~100%）
 * @param duty_percent 输入占空比（0~100）
 * @return uint8_t 限幅后的占空比
 */
static uint8_t Module_Motor_ClampDuty(uint8_t duty_percent) {
  return (duty_percent > 100U) ? 100U : duty_percent;
}

/**
 * @brief 限幅速度值（限制到 -MAX_SPEED ~ +MAX_SPEED）
 * @param speed_percent 输入速度百分比（可正可负）
 * @return int8_t 限幅后的速度值
 * @note  MAX_SPEED 由宏 MODULE_MOTOR_MAX_SPEED 定义（通常为100）
 */
static int8_t Module_Motor_ClampSpeed(int8_t speed_percent) {
  if (speed_percent > MODULE_MOTOR_MAX_SPEED) {
    return MODULE_MOTOR_MAX_SPEED;
  }

  if (speed_percent < -MODULE_MOTOR_MAX_SPEED) {
    return -MODULE_MOTOR_MAX_SPEED;
  }

  return speed_percent;
}

/**
 * @brief 设置指定电机某个PWM通道的占空比（内部调用）
 * @param motor        指向电机模块结构体的指针
 * @param id           电机ID
 * @param pwm          PWM通道（MODULE_MOTOR_PWM1 或 PWM2）
 * @param duty_percent 占空比百分比（0~100）
 * @retval 无
 * @note   - 该函数通过 ops->set_pwm 回调写入硬件
 *         - 自动限幅占空比到有效范围
 *         - 若 motor/ops 无效，函数直接返回
 */
static void Module_Motor_SetPwm(ModuleMotor_t *motor, ModuleMotorId_t id,
                                ModuleMotorPwm_t pwm, uint8_t duty_percent) {
  if ((motor == 0) || (motor->ops == 0) || (motor->ops->set_pwm == 0)) {
    return;
  }

  if ((uint8_t)id >= MODULE_MOTOR_COUNT) {
    return;
  }

  motor->ops->set_pwm(motor->ops->context, (uint8_t)id, (uint8_t)pwm,
                      Module_Motor_ClampDuty(duty_percent));
}

/**
 * @brief 初始化电机模块
 * @param motor 指向电机模块结构体的指针（输出）
 * @param ops   指向操作函数表的指针（输入）
 * @retval 无
 * @note  保存操作函数表指针，若 motor 为空则直接返回
 */
void Module_Motor_Init(ModuleMotor_t *motor, const ModuleMotorOps_t *ops) {
  if (motor == 0) {
    return;
  }

  motor->ops = ops;
}

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
                           int8_t speed_percent) {
  uint8_t abs_speed;

  speed_percent = Module_Motor_ClampSpeed(speed_percent);
  abs_speed =
      (speed_percent < 0) ? (uint8_t)(-speed_percent) : (uint8_t)speed_percent;

  if (speed_percent > 0) {
    Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM1, abs_speed);
    Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM2, 0U);
  } else if (speed_percent < 0) {
    Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM1, 0U);
    Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM2, abs_speed);
  } else {
    Module_Motor_Brake(motor, id);
  }
}

/**
 * @brief 刹车（停止）指定电机
 * @param motor 指向电机模块结构体的指针
 * @param id    电机ID
 * @retval 无
 * @note  将两个PWM通道的占空比均设为0，实现制动
 */
void Module_Motor_Brake(ModuleMotor_t *motor, ModuleMotorId_t id) {
  Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM1, 0U);
  Module_Motor_SetPwm(motor, id, MODULE_MOTOR_PWM2, 0U);
}

/**
 * @brief 刹车所有电机
 * @param motor 指向电机模块结构体的指针
 * @retval 无
 * @note  遍历所有电机，逐个调用 Module_Motor_Brake
 */
void Module_Motor_BrakeAll(ModuleMotor_t *motor) {
  for (uint8_t i = 0U; i < MODULE_MOTOR_COUNT; i++) {
    Module_Motor_Brake(motor, (ModuleMotorId_t)i);
  }
}

/**
 * @brief 互补PWM测试模式（两个通道同时输出相同占空比）
 * @param motor         指向电机模块结构体的指针
 * @param duty_percent  占空比（0~100）
 * @retval 无
 * @note   - 该功能通常用于测试PWM硬件是否正常工作
 *         - 所有电机的两个PWM通道输出相同的占空比，电机可能不转（正反转抵消）
 *         - 占空比自动限幅
 */
void Module_Motor_SetComplementaryTest(ModuleMotor_t *motor,
                                       uint8_t duty_percent) {
  duty_percent = Module_Motor_ClampDuty(duty_percent);

  for (uint8_t i = 0U; i < MODULE_MOTOR_COUNT; i++) {
    Module_Motor_SetPwm(motor, (ModuleMotorId_t)i, MODULE_MOTOR_PWM1,
                        duty_percent);
    Module_Motor_SetPwm(motor, (ModuleMotorId_t)i, MODULE_MOTOR_PWM2,
                        duty_percent);
  }
}