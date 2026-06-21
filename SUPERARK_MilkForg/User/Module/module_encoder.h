/**
 * @file module_encoder.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块化编码器抽象层头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本模块提供了编码器的统一抽象接口，通过操作函数表（ops）实现底层硬件与
 * 上层应用的解耦。支持多个编码器的计数值读取、增量计算和复位操作。
 * 上层应用通过模块化接口调用，无需关心具体硬件实现（如定时器编码器模式、
 * 外部中断计数等），便于跨平台移植和代码复用。
 */

#ifndef __MODULE_ENCODER_H
#define __MODULE_ENCODER_H

#include <stdint.h> /* 标准整数类型定义 */

/**
 * @brief 编码器数量（本实现支持2个编码器）
 */
#define MODULE_ENCODER_COUNT 2U

/**
 * @brief 编码器ID枚举
 * @note  用于标识不同的编码器通道
 */
typedef enum {
  MODULE_ENCODER_M5 = 0, /**< M5电机编码器（如TIM2） */
  MODULE_ENCODER_M6,     /**< M6电机编码器（如TIM3） */
} ModuleEncoderId_t;

/**
 * @brief 编码器操作函数表（虚函数表）
 * @note  底层驱动需要实现这些回调函数，供模块层调用
 */
typedef struct {
  void *context; /**< 上下文指针（底层驱动可传递任意数据） */

  /**
   * @brief 读取编码器当前计数值的回调函数
   * @param context 上下文指针（由底层驱动使用）
   * @param encoder 编码器ID（0~MODULE_ENCODER_COUNT-1）
   * @return int32_t 当前计数值
   */
  int32_t (*read_count)(void *context, uint8_t encoder);

  /**
   * @brief 复位编码器计数值的回调函数
   * @param context 上下文指针（由底层驱动使用）
   * @param encoder 编码器ID（0~MODULE_ENCODER_COUNT-1）
   * @retval 无
   */
  void (*reset_count)(void *context, uint8_t encoder);
} ModuleEncoderOps_t;

/**
 * @brief 编码器模块结构体
 * @note  存储操作函数表指针和每个编码器的上次计数值（用于增量计算）
 */
typedef struct {
  const ModuleEncoderOps_t *ops; /**< 操作函数表指针 */
  int32_t
      last_count[MODULE_ENCODER_COUNT]; /**< 上次读取的计数值（用于增量计算） */
} ModuleEncoder_t;

/**
 * @brief 初始化编码器模块
 * @param encoder 指向编码器模块结构体的指针（输出）
 * @param ops     指向操作函数表的指针（输入）
 * @retval 无
 * @note   - 保存操作函数表指针，并将所有编码器的上次计数值清零
 *         - 若 encoder 为空指针，函数直接返回
 */
void Module_Encoder_Init(ModuleEncoder_t *encoder,
                         const ModuleEncoderOps_t *ops);

/**
 * @brief 读取指定编码器的当前计数值
 * @param encoder 指向编码器模块结构体的指针
 * @param id      编码器ID
 * @return int32_t 当前计数值（若无效则返回0）
 * @note  调用 ops->read_count 回调函数获取硬件计数值
 */
int32_t Module_Encoder_GetCount(ModuleEncoder_t *encoder, ModuleEncoderId_t id);

/**
 * @brief 获取指定编码器的增量值（自上次调用以来的变化量）
 * @param encoder 指向编码器模块结构体的指针
 * @param id      编码器ID
 * @return int32_t 增量值（可为正或负）
 * @note   - 读取当前计数值，计算与上次存储值的差值
 *         - 更新 last_count 为当前值，供下次调用使用
 *         - 该函数通常用于速度估算（单位时间内的脉冲增量）
 */
int32_t Module_Encoder_GetDelta(ModuleEncoder_t *encoder, ModuleEncoderId_t id);

/**
 * @brief 复位指定编码器的计数值
 * @param encoder 指向编码器模块结构体的指针
 * @param id      编码器ID
 * @retval 无
 * @note   - 若提供了 reset_count 回调，则调用硬件复位
 *         - 将软件缓存的 last_count 清零，避免后续增量计算出现偏差
 */
void Module_Encoder_Reset(ModuleEncoder_t *encoder, ModuleEncoderId_t id);

/**
 * @brief 复位所有编码器的计数值
 * @param encoder 指向编码器模块结构体的指针
 * @retval 无
 * @note  遍历所有编码器ID，依次调用 Module_Encoder_Reset
 */
void Module_Encoder_ResetAll(ModuleEncoder_t *encoder);

#endif /* __MODULE_ENCODER_H */