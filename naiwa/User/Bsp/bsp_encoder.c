/**
 * @file bsp_encoder.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）编码器驱动源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本文件实现了电机编码器的硬件驱动，使用TIM2和TIM3的编码器模式
 * 分别读取M5和M6两个电机的编码器计数值。通过模块化接口向上层
 * 提供统一的编码器操作函数（读取、增量、复位等）。
 */

#include "bsp_encoder.h"

#include "tim.h"   /* 包含HAL库生成的定时器句柄（htim2, htim3） */

/**
 * @brief 编码器模块实例（全局静态对象）
 * @note  通过 Module_Encoder 抽象层管理编码器状态和操作
 */
static ModuleEncoder_t bsp_encoder_module;

/**
 * @brief 读取指定编码器的当前计数值（硬件读取回调函数）
 * @param context 上下文指针（本驱动中未使用，传入NULL）
 * @param encoder 编码器ID（MODULE_ENCODER_M5 或 MODULE_ENCODER_M6）
 * @return int32_t 当前编码器计数值（M5为32位，M6为16位扩展为32位返回）
 * @note   - M5使用TIM2（32位计数器），直接返回HAL_GetCounter值
 *         - M6使用TIM3（16位计数器），转换为int16_t后再提升为int32_t
 *         - 该函数作为回调，由Module_Encoder层调用
 */
static int32_t Bsp_Encoder_ReadCount(void *context, uint8_t encoder)
{
  (void)context;   /* 避免未使用参数警告 */

  if (encoder == MODULE_ENCODER_M5)
  {
    /* TIM2为32位计数器，直接返回 */
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  }

  if (encoder == MODULE_ENCODER_M6)
  {
    /* TIM3为16位计数器，先转换为int16_t再提升为int32_t */
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  }

  return 0;   /* 无效编码器ID返回0 */
}

/**
 * @brief 复位指定编码器的计数值（硬件复位回调函数）
 * @param context 上下文指针（本驱动中未使用，传入NULL）
 * @param encoder 编码器ID（MODULE_ENCODER_M5 或 MODULE_ENCODER_M6）
 * @retval 无
 * @note   将对应定时器的计数器寄存器清零
 */
static void Bsp_Encoder_ResetCount(void *context, uint8_t encoder)
{
  (void)context;   /* 避免未使用参数警告 */

  if (encoder == MODULE_ENCODER_M5)
  {
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
  }
  else if (encoder == MODULE_ENCODER_M6)
  {
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
  }
}

/**
 * @brief 编码器操作函数表（虚函数表）
 * @note  包含版本号、读取和复位回调函数，供Module_Encoder层调用
 */
static const ModuleEncoderOps_t encoder_ops =
{
  0,                           /* 版本号（预留） */
  Bsp_Encoder_ReadCount,       /* 读取计数值回调 */
  Bsp_Encoder_ResetCount,      /* 复位计数值回调 */
};

/**
 * @brief 初始化编码器硬件及模块
 * @retval 无
 * @note   - 启动TIM2和TIM3的编码器模式（全部通道）
 *         - 若启动失败则调用Error_Handler进入死循环
 *         - 初始化Module_Encoder抽象层，并复位所有编码器计数值
 */
void Bsp_Encoder_Init(void)
{
  /* 启动TIM2编码器模式（M5电机） */
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();   /* 启动失败，错误处理 */
  }

  /* 启动TIM3编码器模式（M6电机） */
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();   /* 启动失败，错误处理 */
  }

  /* 初始化编码器模块抽象层 */
  Module_Encoder_Init(&bsp_encoder_module, &encoder_ops);
  
  /* 将所有编码器计数值清零 */
  Module_Encoder_ResetAll(&bsp_encoder_module);
}

/**
 * @brief 获取编码器操作函数表（供其他模块使用）
 * @return const ModuleEncoderOps_t* 指向编码器操作函数表的指针
 * @note  可用于外部直接调用底层回调函数
 */
const ModuleEncoderOps_t *Bsp_Encoder_GetOps(void)
{
  return &encoder_ops;
}

/**
 * @brief 获取指定编码器的当前计数值
 * @param encoder 编码器ID（BspEncoderId_t枚举，如 ENCODER_M5）
 * @return int32_t 当前计数值
 * @note  封装了 Module_Encoder_GetCount 函数调用
 */
int32_t Bsp_Encoder_GetCount(BspEncoderId_t encoder)
{
  return Module_Encoder_GetCount(&bsp_encoder_module, (ModuleEncoderId_t)encoder);
}

/**
 * @brief 获取指定编码器的增量值（相对于上次调用）
 * @param encoder 编码器ID（BspEncoderId_t枚举，如 ENCODER_M5）
 * @return int32_t 自上次读取后的增量值
 * @note  封装了 Module_Encoder_GetDelta 函数调用
 *        该函数通常用于速度计算（单位时间内的脉冲增量）
 */
int32_t Bsp_Encoder_GetDelta(BspEncoderId_t encoder)
{
  return Module_Encoder_GetDelta(&bsp_encoder_module, (ModuleEncoderId_t)encoder);
}

/**
 * @brief 复位指定编码器的计数值
 * @param encoder 编码器ID（BspEncoderId_t枚举，如 ENCODER_M5）
 * @retval 无
 * @note  封装了 Module_Encoder_Reset 函数调用
 */
void Bsp_Encoder_Reset(BspEncoderId_t encoder)
{
  Module_Encoder_Reset(&bsp_encoder_module, (ModuleEncoderId_t)encoder);
}

/**
 * @brief 复位所有编码器的计数值
 * @retval 无
 * @note  封装了 Module_Encoder_ResetAll 函数调用
 *        通常在系统初始化或需要归零时调用
 */
void Bsp_Encoder_ResetAll(void)
{
  Module_Encoder_ResetAll(&bsp_encoder_module);
}