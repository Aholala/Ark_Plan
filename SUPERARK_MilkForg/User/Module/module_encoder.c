/**
 * @file module_encoder.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块化编码器抽象层源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本模块提供了编码器的统一抽象接口，通过操作函数表（ops）实现底层硬件与
 * 上层应用的解耦。支持多个编码器的计数值读取、增量计算和复位操作。
 * 每个编码器在初始化后，可通过 GetDelta 函数获取自上次调用以来的脉冲增量，
 * 便于速度估算和位置控制。
 */

#include "module_encoder.h"

/**
 * @brief 检查编码器模块和ID是否有效
 * @param encoder 指向编码器模块结构体的指针
 * @param id      编码器ID（枚举类型）
 * @return uint8_t 1表示有效，0表示无效
 * @note  检查条件：encoder非空、ops非空、id在有效范围内
 */
static uint8_t Module_Encoder_IsValid(ModuleEncoder_t *encoder,
                                      ModuleEncoderId_t id) {
  return (encoder != 0) && (encoder->ops != 0) &&
         ((uint8_t)id < MODULE_ENCODER_COUNT);
}

/**
 * @brief 初始化编码器模块
 * @param encoder 指向编码器模块结构体的指针（输出）
 * @param ops     指向操作函数表的指针（输入）
 * @retval 无
 * @note   - 保存操作函数表指针，并将所有编码器的上次计数值清零
 *         - 若 encoder 为空指针，函数直接返回
 */
void Module_Encoder_Init(ModuleEncoder_t *encoder,
                         const ModuleEncoderOps_t *ops) {
  if (encoder == 0) {
    return;
  }

  encoder->ops = ops;
  for (uint8_t i = 0U; i < MODULE_ENCODER_COUNT; i++) {
    encoder->last_count[i] = 0;
  }
}

/**
 * @brief 读取指定编码器的当前计数值（直接通过硬件回调）
 * @param encoder 指向编码器模块结构体的指针
 * @param id      编码器ID
 * @return int32_t 当前计数值（若无效则返回0）
 * @note  调用 ops->read_count 回调函数获取硬件计数值
 */
int32_t Module_Encoder_GetCount(ModuleEncoder_t *encoder,
                                ModuleEncoderId_t id) {
  if (!Module_Encoder_IsValid(encoder, id) || (encoder->ops->read_count == 0)) {
    return 0;
  }

  return encoder->ops->read_count(encoder->ops->context, (uint8_t)id);
}

/**
 * @brief 获取指定编码器的增量值（自上次调用以来的变化量）
 * @param encoder 指向编码器模块结构体的指针
 * @param id      编码器ID
 * @return int32_t 增量值（可为正或负）
 * @note   - 读取当前计数值，计算与上次存储值的差值
 *         - 更新 last_count 为当前值，供下次调用使用
 *         - 若编码器无效，返回0
 */
int32_t Module_Encoder_GetDelta(ModuleEncoder_t *encoder,
                                ModuleEncoderId_t id) {
  int32_t current;
  int32_t delta;

  if (!Module_Encoder_IsValid(encoder, id)) {
    return 0;
  }

  current = Module_Encoder_GetCount(encoder, id);
  delta = current - encoder->last_count[id];
  encoder->last_count[id] = current;

  return delta;
}

/**
 * @brief 复位指定编码器的计数值（硬件复位 + 软件缓存清零）
 * @param encoder 指向编码器模块结构体的指针
 * @param id      编码器ID
 * @retval 无
 * @note   - 若提供了 reset_count 回调，则调用硬件复位
 *         - 将软件缓存的 last_count 清零，避免后续增量计算出现偏差
 *         - 若编码器无效，函数直接返回
 */
void Module_Encoder_Reset(ModuleEncoder_t *encoder, ModuleEncoderId_t id) {
  if (!Module_Encoder_IsValid(encoder, id)) {
    return;
  }

  if (encoder->ops->reset_count != 0) {
    encoder->ops->reset_count(encoder->ops->context, (uint8_t)id);
  }
  encoder->last_count[id] = 0;
}

/**
 * @brief 复位所有编码器的计数值
 * @param encoder 指向编码器模块结构体的指针
 * @retval 无
 * @note  遍历所有编码器ID，依次调用 Module_Encoder_Reset
 */
void Module_Encoder_ResetAll(ModuleEncoder_t *encoder) {
  for (uint8_t i = 0U; i < MODULE_ENCODER_COUNT; i++) {
    Module_Encoder_Reset(encoder, (ModuleEncoderId_t)i);
  }
}