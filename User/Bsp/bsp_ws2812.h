/**
 * @file bsp_ws2812.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级 WS2812 驱动头文件（SPI1/PB5）
 * @version 1.0
 * @date 2026-06-17
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef __BSP_WS2812_H__
#define __BSP_WS2812_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "module_ws2812.h"

/* 若未定义 LED 数量，默认 30 个 */
#ifndef BSP_WS2812_LED_COUNT
#define BSP_WS2812_LED_COUNT 30U
#endif

/**
 * @brief 初始化板级 WS2812 实例（SPI1 上）
 * @return WS2812_Status_t 初始化状态
 */
WS2812_Status_t BSP_WS2812_Init(void);

/**
 * @brief 获取默认板级 WS2812 驱动句柄
 * @return WS2812_Handle_t* 句柄指针
 */
WS2812_Handle_t *BSP_WS2812_GetHandle(void);

/**
 * @brief HAL SPI 发送完成回调桥接函数
 * @param hspi SPI 句柄
 */
void BSP_WS2812_OnSpiTxComplete(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_WS2812_H__ */