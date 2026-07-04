/**
 * @file module_motor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 电机模块实现 - 提供电机PWM控制、编码器读取及状态管理
 * @version 1.0
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块对6路电机进行统一管理，每路电机包含：
 *          - 两路PWM输入（IN_A/IN_B）实现正反转控制；
 *          - 可选编码器接口（部分电机配有正交编码器）；
 *          提供占空比设置（带符号，-95%～+95%）、停止、编码器数据读取等功能。
 *          支持开环控制（无编码器）和闭环反馈读取（有编码器）。
 */

#include "module_motor.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"

/*==================== 宏定义 ====================*/

/** @brief 无编码器标识值（用于配置表中表示该电机无编码器） */
#define MODULE_MOTOR_NO_ENCODER 0xFFU

/*==================== 类型定义 ====================*/

/**
 * @brief 电机配置结构体
 * @note 定义每个电机的PWM控制通道及对应的编码器ID（如有）
 */
typedef struct {
  BspPwmChannel_t in_a; /**< 正向PWM控制通道（IN_A） */
  BspPwmChannel_t in_b; /**< 反向PWM控制通道（IN_B） */
  uint8_t encoder; /**< 编码器ID（BspEncoderId_t），若为 MODULE_MOTOR_NO_ENCODER
                      则表示无编码器 */
} ModuleMotorConfig_t;

/*==================== 硬件映射表 ====================*/

/**
 * @brief 电机硬件配置表
 * @note 顺序必须与 ModuleMotorId_t 枚举定义一致
 * @note 前4个电机为开环控制（无编码器），后2个电机带有正交编码器反馈
 */
static const ModuleMotorConfig_t module_motor_configs[MODULE_MOTOR_COUNT] = {
    {BSP_PWM_TIM4_CH3, BSP_PWM_TIM4_CH4,
     MODULE_MOTOR_NO_ENCODER}, /* M1: PB8/PB9, 开环 */
    {BSP_PWM_TIM4_CH1, BSP_PWM_TIM4_CH2,
     MODULE_MOTOR_NO_ENCODER}, /* M2: PB6/PB7, 开环 */
    {BSP_PWM_TIM8_CH3, BSP_PWM_TIM8_CH4,
     MODULE_MOTOR_NO_ENCODER}, /* M3: PC8/PC9, 开环 */
    {BSP_PWM_TIM8_CH1, BSP_PWM_TIM8_CH2,
     MODULE_MOTOR_NO_ENCODER}, /* M4: PC6/PC7, 开环 */
    {BSP_PWM_TIM5_CH3, BSP_PWM_TIM5_CH4,
     BSP_ENCODER_M5}, /* M5: PA2/PA3, 编码器 TIM2 */
    {BSP_PWM_TIM1_CH1, BSP_PWM_TIM1_CH3,
     BSP_ENCODER_M6}, /* M6: PA8/PA10, 编码器 TIM3 */
};

/*==================== 静态函数 ====================*/

/**
 * @brief 检查电机编号是否有效
 *
 * @param motor 电机编号
 * @return uint8_t 有效性
 * @retval 1 有效（0 ~ MODULE_MOTOR_COUNT-1）
 * @retval 0 无效
 */
static uint8_t Module_Motor_IsValid(ModuleMotorId_t motor) {
  return (uint8_t)motor < (uint8_t)MODULE_MOTOR_COUNT;
}

/**
 * @brief 取有符号整数的绝对值（返回uint8_t）
 *
 * @param duty_percent 有符号占空比值（-100~100）
 * @return uint8_t 绝对值的无符号表示
 */
static uint8_t Module_Motor_AbsDuty(int8_t duty_percent) {
  if (duty_percent < 0) {
    return (uint8_t)(-duty_percent);
  }

  return (uint8_t)duty_percent;
}

/*==================== 全局API函数 ====================*/

/**
 * @brief 初始化所有电机（停止所有电机）
 * @note 内部调用 Module_Motor_StopAll() 确保上电时电机处于安全状态
 */
void Module_Motor_Init(void) { Module_Motor_StopAll(); }

/**
 * @brief 设置指定电机的转速和方向（带符号占空比）
 *
 * @param motor         电机编号（ModuleMotorId_t枚举）
 * @param duty_percent  占空比（-95% ~ +95%）
 *                      - 正值：正转（IN_A有效，IN_B为0）
 *                      - 负值：反转（IN_B有效，IN_A为0）
 *                      - 0：停止（两路均为0）
 * @note 占空比自动限幅在 ±95% 以内，避免驱动饱和。
 * @note 若电机编号无效则函数直接返回。
 * @note 实际PWM占空比会经过 bsp_pwm 层的5%/95%限幅策略。
 */
void Module_Motor_SetDuty(ModuleMotorId_t motor, int8_t duty_percent) {
  const ModuleMotorConfig_t *config;
  uint8_t duty;

  if (!Module_Motor_IsValid(motor)) {
    return;
  }

  /* 占空比限幅（±95%） */
  if (duty_percent > 95) {
    duty_percent = 95;
  } else if (duty_percent < -95) {
    duty_percent = -95;
  }

  config = &module_motor_configs[motor];
  duty = Module_Motor_AbsDuty(duty_percent);

  if (duty_percent > 0) {
    /* 正转：IN_B = 0, IN_A = duty */
    Bsp_Pwm_SetDuty(config->in_b, 0U);
    Bsp_Pwm_SetDuty(config->in_a, duty);
  } else if (duty_percent < 0) {
    /* 反转：IN_A = 0, IN_B = -duty */
    Bsp_Pwm_SetDuty(config->in_a, 0U);
    Bsp_Pwm_SetDuty(config->in_b, duty);
  } else {
    /* 停止：两路均为0 */
    Bsp_Pwm_SetDuty(config->in_a, 0U);
    Bsp_Pwm_SetDuty(config->in_b, 0U);
  }
}

/**
 * @brief 停止指定电机（占空比设为0）
 *
 * @param motor 电机编号（ModuleMotorId_t枚举）
 * @note 内部调用 Module_Motor_SetDuty(motor, 0)
 */
void Module_Motor_Stop(ModuleMotorId_t motor) {
  Module_Motor_SetDuty(motor, 0);
}

/**
 * @brief 停止所有电机
 * @note 遍历所有电机，依次调用 Module_Motor_Stop()
 */
void Module_Motor_StopAll(void) {
  for (uint8_t i = 0U; i < (uint8_t)MODULE_MOTOR_COUNT; i++) {
    Module_Motor_Stop((ModuleMotorId_t)i);
  }
}

/**
 * @brief 判断指定电机是否配备编码器
 *
 * @param motor 电机编号
 * @return uint8_t 是否具有编码器
 * @retval 1 有编码器
 * @retval 0 无编码器或编号无效
 */
uint8_t Module_Motor_HasEncoder(ModuleMotorId_t motor) {
  if (!Module_Motor_IsValid(motor)) {
    return 0U;
  }

  return module_motor_configs[motor].encoder != MODULE_MOTOR_NO_ENCODER;
}

/**
 * @brief 获取指定电机编码器的单圈脉冲数（每转计数）
 *
 * @param motor 电机编号
 * @return uint32_t 每转脉冲数（若电机无编码器，返回0）
 * @note 该值为固定常数，由 MODULE_MOTOR_ENCODER_COUNTS_PER_REV
 * 定义（在头文件中）。
 */
uint32_t Module_Motor_GetEncoderCountsPerRev(ModuleMotorId_t motor) {
  if (!Module_Motor_HasEncoder(motor)) {
    return 0U;
  }

  return MODULE_MOTOR_ENCODER_COUNTS_PER_REV;
}

/**
 * @brief 获取指定电机编码器的当前累计计数值
 *
 * @param motor 电机编号
 * @return int32_t 当前编码器累计值（若电机无编码器，返回0）
 * @note 该值为有符号数，表示相对初始位置的偏移。
 * @note 计数值可通过 Module_Motor_ResetEncoder() 清零。
 */
int32_t Module_Motor_GetEncoderCount(ModuleMotorId_t motor) {
  if (!Module_Motor_HasEncoder(motor)) {
    return 0;
  }

  return Bsp_Encoder_GetCount(
      (BspEncoderId_t)module_motor_configs[motor].encoder);
}

/**
 * @brief 获取指定电机编码器上次读取后的增量值
 *
 * @param motor 电机编号
 * @return int32_t 编码器增量（若电机无编码器，返回0）
 * @note 该值表示自上次调用此函数以来编码器的变化量，
 *       通常在周期性任务中调用以获取速度反馈。
 */
int32_t Module_Motor_GetEncoderDelta(ModuleMotorId_t motor) {
  if (!Module_Motor_HasEncoder(motor)) {
    return 0;
  }

  return Bsp_Encoder_GetDelta(
      (BspEncoderId_t)module_motor_configs[motor].encoder);
}

/**
 * @brief 重置指定电机编码器的累计计数值（清零）
 *
 * @param motor 电机编号
 * @note 若电机无编码器，函数直接返回。
 * @note 重置后，后续调用 GetEncoderCount() 将返回0。
 */
void Module_Motor_ResetEncoder(ModuleMotorId_t motor) {
  if (!Module_Motor_HasEncoder(motor)) {
    return;
  }

  Bsp_Encoder_Reset((BspEncoderId_t)module_motor_configs[motor].encoder);
}