/**
 * @file bsp_ws2812.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级 WS2812 驱动实现（基于 SPI1）
 * @version 1.0
 * @date 2026-06-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "bsp_ws2812.h"
#include "spi.h"

/* 静态变量：驱动句柄、像素缓冲区、发送缓冲区、复位缓冲区 */
static WS2812_Handle_t bsp_ws2812_handle;
static WS2812_Color_t bsp_ws2812_pixels[BSP_WS2812_LED_COUNT];
static uint8_t bsp_ws2812_tx_buffer[WS2812_TX_BUFFER_SIZE(BSP_WS2812_LED_COUNT)];
static uint8_t bsp_ws2812_reset_buffer[WS2812_RESET_BYTES_DEFAULT];

/**
 * @brief SPI 非阻塞发送回调（供模块层使用）
 * @param user 用户数据指针，实际为 SPI_HandleTypeDef*
 * @param data 待发送数据
 * @param size 数据大小（字节）
 * @return WS2812_Status_t 发送结果
 * 
 * 该函数由模块层调用，启动 SPI 中断发送。
 */
static WS2812_Status_t BSP_WS2812_TransmitIt(void *user, const uint8_t *data, uint16_t size)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)user;
  HAL_StatusTypeDef status;

  if ((hspi == 0) || (data == 0) || (size == 0U))
  {
    return WS2812_STATUS_INVALID_PARAM;
  }

  status = HAL_SPI_Transmit_DMA(hspi, (uint8_t *)data, size);
  if (status == HAL_OK)
  {
    return WS2812_STATUS_OK;
  }

  if (status == HAL_BUSY)
  {
    return WS2812_STATUS_BUSY;
  }

  return WS2812_STATUS_ERROR;
}

/**
 * @brief 获取当前系统毫秒时钟（供模块层使用）
 * @param user 未使用
 * @return uint32_t HAL_GetTick() 返回值
 */
static uint32_t BSP_WS2812_GetTickMs(void *user)
{
  (void)user;
  return HAL_GetTick();
}

/**
 * @brief 初始化板级 WS2812 实例
 * @return WS2812_Status_t 初始化结果
 * 
 * 配置模块层所需参数，包括 LED 数量、像素缓冲区、发送/复位缓冲区、
 * 以及操作接口（发送和时钟回调）。
 */
WS2812_Status_t BSP_WS2812_Init(void)
{
  WS2812_Config_t config = {
    .led_count = BSP_WS2812_LED_COUNT,
    .pixels = bsp_ws2812_pixels,
    .pixel_count = BSP_WS2812_LED_COUNT,
    .tx_buffer = bsp_ws2812_tx_buffer,
    .tx_buffer_size = sizeof(bsp_ws2812_tx_buffer),
    .reset_buffer = bsp_ws2812_reset_buffer,
    .reset_buffer_size = sizeof(bsp_ws2812_reset_buffer),
    .ops = {
      .user = &hspi1,
      .transmit_it = BSP_WS2812_TransmitIt,
      .get_tick_ms = BSP_WS2812_GetTickMs,
    },
  };

  return WS2812_Init(&bsp_ws2812_handle, &config);
}

/**
 * @brief 获取板级 WS2812 驱动句柄
 * @return WS2812_Handle_t* 句柄指针
 */
WS2812_Handle_t *BSP_WS2812_GetHandle(void)
{
  return &bsp_ws2812_handle;
}

/**
 * @brief SPI 发送完成回调桥接函数
 * @param hspi 发生中断的 SPI 句柄
 * 
 * 该函数在 HAL 回调中被调用，用于通知模块层发送完成。
 */
void BSP_WS2812_OnSpiTxComplete(SPI_HandleTypeDef *hspi)
{
  if ((hspi != 0) && (hspi->Instance == SPI1))
  {
    WS2812_OnTxComplete(&bsp_ws2812_handle);
  }
}

/**
 * @brief HAL SPI 发送完成中断回调
 * @param hspi SPI 句柄
 * 
 * 该函数由 HAL 库自动调用，桥接到 BSP 层回调。
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BSP_WS2812_OnSpiTxComplete(hspi);
}
