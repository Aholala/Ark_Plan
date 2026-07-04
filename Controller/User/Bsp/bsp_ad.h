/**
 * @file bsp_ad.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）ADC驱动头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件定义了ADC驱动的通道枚举和接口函数。
 * 使用ADC1的DMA连续采集模式，支持最多4个通道的模拟量输入。
 * 通常用于读取双摇杆的X/Y轴电压值（左摇杆水平/垂直，右摇杆水平/垂直）。
 */

#ifndef __AD_H
#define __AD_H

#include <stdint.h> /* 标准整数类型定义 */

/**
 * @brief ADC通道枚举
 * @note  顺序必须与硬件连接和DMA传输顺序一致
 *        使用时通过该枚举索引获取对应通道的转换值
 */
typedef enum {
  AD_CHANNEL_LH = 0, /**< 左摇杆水平（Left Horizontal） */
  AD_CHANNEL_LV,     /**< 左摇杆垂直（Left Vertical） */
  AD_CHANNEL_RH,     /**< 右摇杆水平（Right Horizontal） */
  AD_CHANNEL_RV,     /**< 右摇杆垂直（Right Vertical） */
  AD_CHANNEL_COUNT,  /**< 通道总数（用于数组大小和循环控制） */
} AdChannel_t;

/**
 * @brief 初始化ADC外设，执行校准并启动DMA连续转换
 * @retval 无
 * @note   - 校准后启动DMA循环模式，转换结果自动更新
 *         - 若初始化失败则进入 Error_Handler
 */
void AD_Init(void);

/**
 * @brief 获取指定通道的ADC转换值
 * @param channel 通道枚举（AdChannel_t类型，如 AD_CHANNEL_LH）
 * @return uint16_t 最新转换结果（12位，0~4095）
 * @note   - 若通道枚举无效，返回中间值2048
 *         - 转换值由DMA自动更新，无需额外同步
 *         - 实际值范围取决于硬件电路（通常为0~3.3V对应0~4095）
 */
uint16_t AD_GetValue(AdChannel_t channel);

#endif /* __AD_H */