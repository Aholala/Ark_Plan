/**
 * @file bsp_ad.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）ADC驱动源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本文件实现了ADC的初始化、校准和DMA连续采集功能。
 * 使用ADC1的DMA循环模式采集多个通道（由AD_CHANNEL_COUNT定义），
 * 并将转换结果存入静态数组，供上层通过通道枚举读取。
 */

#include "bsp_ad.h"

#include "adc.h" /* HAL库生成的ADC句柄（hadc1） */

/**
 * @brief ADC转换结果缓存数组
 * @note 数组大小由 AD_CHANNEL_COUNT
 * 决定（通常为4），初始值设为中间值2048（12位ADC）
 *        通过DMA自动更新，用户无需关心底层更新细节
 */
static uint16_t ad_values[AD_CHANNEL_COUNT] = {2048U, 2048U, 2048U, 2048U};

/**
 * @brief 初始化ADC外设，执行校准并启动DMA连续转换
 * @retval 无
 * @note   - 调用HAL_ADCEx_Calibration_Start进行ADC校准，确保转换精度
 *         - 通过HAL_ADC_Start_DMA启动DMA循环模式，转换结果自动存入ad_values数组
 *         - 若校准或DMA启动失败则调用Error_Handler进入死循环
 *         - DMA传输完成中断由HAL库自动处理，无需用户干预
 */
void AD_Init(void) {
  /* 执行ADC1硬件校准（偏移校准） */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    Error_Handler(); /* 校准失败，错误处理 */
  }

  /* 启动DMA连续转换，转换结果自动搬运到ad_values数组，共AD_CHANNEL_COUNT个通道
   */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ad_values, AD_CHANNEL_COUNT) !=
      HAL_OK) {
    Error_Handler(); /* DMA启动失败，错误处理 */
  }
}

/**
 * @brief 根据通道枚举获取最新的ADC转换值
 * @param channel 通道枚举（如 AD_CHANNEL_LH, AD_CHANNEL_LV 等，定义于bsp_ad.h）
 * @return uint16_t 该通道的最近一次转换结果（12位，0~4095）
 * @note   - 若传入的通道索引超出范围（>= AD_CHANNEL_COUNT），则返回中间值2048
 *         - 转换结果由DMA自动更新，读取时无需额外同步操作
 *         - 适用于摇杆、电位计等模拟量采集
 */
uint16_t AD_GetValue(AdChannel_t channel) {
  /* 通道有效性检查，防止数组越界 */
  if ((uint8_t)channel >= AD_CHANNEL_COUNT) {
    return 2048U; /* 无效通道返回中间值 */
  }

  return ad_values[channel];
}