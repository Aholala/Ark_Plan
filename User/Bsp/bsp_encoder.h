/**
 * @file bsp_encoder.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）编码器驱动头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件定义了编码器硬件驱动的接口，基于模块化编码器抽象层（module_encoder.h）。
 * 支持两个电机编码器（M5和M6），分别对应TIM2和TIM3的编码器模式。
 * 提供计数值读取、增量计算、复位等操作。
 */

#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "main.h"           /* HAL库及硬件定义 */
#include "module_encoder.h" /* 模块化编码器抽象层 */

/**
 * @brief BSP层编码器标识枚举
 * @note  值映射到模块抽象层的枚举，便于统一管理
 */
typedef enum {
  BSP_ENCODER_M5 = MODULE_ENCODER_M5, /**< M5电机编码器（对应TIM2） */
  BSP_ENCODER_M6 = MODULE_ENCODER_M6, /**< M6电机编码器（对应TIM3） */
} BspEncoderId_t;

/**
 * @brief 初始化编码器硬件及模块
 * @retval 无
 * @note   - 启动TIM2和TIM3的编码器模式（全部通道）
 *         - 若启动失败则进入 Error_Handler
 *         - 初始化模块抽象层并复位所有编码器计数值
 */
void Bsp_Encoder_Init(void);

/**
 * @brief 获取编码器操作函数表（供外部直接使用底层回调）
 * @return const ModuleEncoderOps_t* 指向操作函数表的指针
 * @note  包含读计数和复位回调，通常用于模块化框架内部
 */
const ModuleEncoderOps_t *Bsp_Encoder_GetOps(void);

/**
 * @brief 获取指定编码器的当前计数值
 * @param encoder 编码器ID（BSP_ENCODER_M5 或 BSP_ENCODER_M6）
 * @return int32_t 当前计数值（M5为32位，M6为16位扩展为32位）
 */
int32_t Bsp_Encoder_GetCount(BspEncoderId_t encoder);

/**
 * @brief 获取指定编码器的增量值（自上次调用该函数以来的变化量）
 * @param encoder 编码器ID（BSP_ENCODER_M5 或 BSP_ENCODER_M6）
 * @return int32_t 增量值（可正可负）
 * @note  该函数通过模块层自动计算两次读取的差值，适用于速度估算
 */
int32_t Bsp_Encoder_GetDelta(BspEncoderId_t encoder);

/**
 * @brief 复位指定编码器的计数值
 * @param encoder 编码器ID（BSP_ENCODER_M5 或 BSP_ENCODER_M6）
 * @retval 无
 */
void Bsp_Encoder_Reset(BspEncoderId_t encoder);

/**
 * @brief 复位所有编码器的计数值
 * @retval 无
 * @note  通常在系统初始化或需要归零时调用
 */
void Bsp_Encoder_ResetAll(void);

#endif /* __BSP_ENCODER_H */