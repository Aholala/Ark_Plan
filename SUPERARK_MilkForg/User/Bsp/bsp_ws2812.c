#include "bsp_ws2812.h"
#include "spi.h"

static WS2812_Handle_t bsp_ws2812_handle;
static WS2812_Color_t bsp_ws2812_pixels[BSP_WS2812_LED_COUNT];
static uint8_t bsp_ws2812_tx_buffer[WS2812_TX_BUFFER_SIZE(BSP_WS2812_LED_COUNT)];
static uint8_t bsp_ws2812_reset_buffer[WS2812_RESET_BYTES_DEFAULT];

static WS2812_Status_t BSP_WS2812_TransmitIt(void *user, const uint8_t *data, uint16_t size)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)user;
  HAL_StatusTypeDef status;

  if ((hspi == 0) || (data == 0) || (size == 0U))
  {
    return WS2812_STATUS_INVALID_PARAM;
  }

  status = HAL_SPI_Transmit_IT(hspi, (uint8_t *)data, size);
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

static uint32_t BSP_WS2812_GetTickMs(void *user)
{
  (void)user;
  return HAL_GetTick();
}

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

WS2812_Handle_t *BSP_WS2812_GetHandle(void)
{
  return &bsp_ws2812_handle;
}

void BSP_WS2812_OnSpiTxComplete(SPI_HandleTypeDef *hspi)
{
  if ((hspi != 0) && (hspi->Instance == SPI1))
  {
    WS2812_OnTxComplete(&bsp_ws2812_handle);
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BSP_WS2812_OnSpiTxComplete(hspi);
}
