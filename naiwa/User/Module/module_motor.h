/**
 * @file module_motor.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 电机模块 - 提供PWM驱动及可选编码器反馈接口
 * @version 1.0
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块定义6路电机的统一控制接口，支持：
 *          - 带符号占空比设置（正反转）
 *          - 电机停止控制（单路/全部）
 *          - 可选正交编码器反馈（部分电机）
 *          - 编码器累计计数、增量读取和重置功能
 *          编码器每转脉冲数由物理PPR和四倍频系数计算得出。
 */

#ifndef __MODULE_MOTOR_H
#define __MODULE_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 电机编号枚举
 * @note 顺序与硬件配置表一致，共6路电机
 */
typedef enum {
  MODULE_MOTOR_1 = 0, /**< 电机1（开环） */
  MODULE_MOTOR_2,     /**< 电机2（开环） */
  MODULE_MOTOR_3,     /**< 电机3（开环） */
  MODULE_MOTOR_4,     /**< 电机4（开环） */
  MODULE_MOTOR_5,     /**< 电机5（带编码器） */
  MODULE_MOTOR_6,     /**< 电机6（带编码器） */
  MODULE_MOTOR_COUNT  /**< 电机总数（用于边界检查） */
} ModuleMotorId_t;

/**
 * @name 编码器参数宏
 * @brief 定义编码器的物理参数及每转计数值
 * @{
 */
#define MODULE_MOTOR_ENCODER_PHASE_PPR                                         \
  12U /**< 编码器物理线数（每转AB相脉冲数） */
#define MODULE_MOTOR_ENCODER_QUADRATURE_MULTIPLIER                             \
  4U /**< 四倍频系数（上升沿+下降沿计数） */
#define MODULE_MOTOR_ENCODER_COUNTS_PER_REV                                    \
  (MODULE_MOTOR_ENCODER_PHASE_PPR *                                            \
   MODULE_MOTOR_ENCODER_QUADRATURE_MULTIPLIER) /**< 每转总计数（48       \
                                                  counts/rev） */
/** @} */

/**
 * @brief 初始化所有电机（停止所有电机，进入安全状态）
 * @note 上电或系统复位后需调用此函数，确保电机不会意外运转。
 */
void Module_Motor_Init(void);

/**
 * @brief 设置指定电机的转速和方向（带符号占空比）
 *
 * @param motor         电机编号（ModuleMotorId_t枚举）
 * @param duty_percent  占空比（-95% ~ +95%）
 *                      - 正值：正转（IN_A有效）
 *                      - 负值：反转（IN_B有效）
 *                      - 0：停止
 * @note 占空比自动限幅在 ±95% 以内，防止驱动饱和。
 * @note 若电机编号无效则函数直接返回。
 */
void Module_Motor_SetDuty(ModuleMotorId_t motor, int8_t duty_percent);

/**
 * @brief 停止指定电机（占空比设为0）
 *
 * @param motor 电机编号
 * @note 内部调用 Module_Motor_SetDuty(motor, 0)
 */
void Module_Motor_Stop(ModuleMotorId_t motor);

/**
 * @brief 停止所有电机
 * @note 遍历所有电机并调用 Module_Motor_Stop()
 */
void Module_Motor_StopAll(void);

/**
 * @brief 判断指定电机是否配备编码器
 *
 * @param motor 电机编号
 * @return uint8_t 是否具有编码器
 * @retval 1 有编码器
 * @retval 0 无编码器或编号无效
 */
uint8_t Module_Motor_HasEncoder(ModuleMotorId_t motor);

/**
 * @brief 获取指定电机编码器的单圈脉冲数（每转计数）
 *
 * @param motor 电机编号
 * @return uint32_t 每转脉冲数（若有编码器）；0（若无编码器）
 * @note 该值为固定值，由宏 MODULE_MOTOR_ENCODER_COUNTS_PER_REV 定义。
 */
uint32_t Module_Motor_GetEncoderCountsPerRev(ModuleMotorId_t motor);

/**
 * @brief 获取指定电机编码器的当前累计计数值
 *
 * @param motor 电机编号
 * @return int32_t 当前编码器累计值（若有编码器）；0（若无编码器）
 * @note 值表示相对初始位置的偏移，可通过 Module_Motor_ResetEncoder() 清零。
 */
int32_t Module_Motor_GetEncoderCount(ModuleMotorId_t motor);

/**
 * @brief 获取指定电机编码器上次读取后的增量值
 *
 * @param motor 电机编号
 * @return int32_t 编码器增量（若有编码器）；0（若无编码器）
 * @note 该值表示自上次调用此函数以来编码器的变化量，
 *       适用于周期性速度计算。
 */
int32_t Module_Motor_GetEncoderDelta(ModuleMotorId_t motor);

/**
 * @brief 重置指定电机编码器的累计计数值（清零）
 *
 * @param motor 电机编号
 * @note 若电机无编码器，函数直接返回。
 * @note 重置后，后续调用 GetEncoderCount() 将返回0。
 */
void Module_Motor_ResetEncoder(ModuleMotorId_t motor);

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_MOTOR_H */