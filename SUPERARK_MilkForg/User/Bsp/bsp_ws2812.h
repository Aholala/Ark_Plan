#ifndef __BSP_WS2812_H__
#define __BSP_WS2812_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "module_ws2812.h"

#ifndef BSP_WS2812_LED_COUNT
#define BSP_WS2812_LED_COUNT 30U
#endif

/* Initialize the board-level WS2812 instance on SPI1/PB5. */
WS2812_Status_t BSP_WS2812_Init(void);

/* Return the default board WS2812 handle for Module/App calls. */
WS2812_Handle_t *BSP_WS2812_GetHandle(void);

/* HAL SPI callback bridge. Called by HAL_SPI_TxCpltCallback(). */
void BSP_WS2812_OnSpiTxComplete(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_WS2812_H__ */
