#ifndef __WS2812_H__
#define __WS2812_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define WS2812_BYTES_PER_LED       9U
#define WS2812_RESET_BYTES_DEFAULT 64U
#define WS2812_TX_BUFFER_SIZE(led_count) ((uint16_t)((led_count) * WS2812_BYTES_PER_LED))

typedef enum
{
  WS2812_STATUS_OK = 0,
  WS2812_STATUS_BUSY,
  WS2812_STATUS_ERROR,
  WS2812_STATUS_INVALID_PARAM
} WS2812_Status_t;

typedef struct
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} WS2812_Color_t;

typedef enum
{
  WS2812_COLOR_OFF = 0,
  WS2812_COLOR_RED,
  WS2812_COLOR_GREEN,
  WS2812_COLOR_BLUE,
  WS2812_COLOR_WHITE,
  WS2812_COLOR_WARM_WHITE,
  WS2812_COLOR_YELLOW,
  WS2812_COLOR_CYAN,
  WS2812_COLOR_MAGENTA,
  WS2812_COLOR_ORANGE,
  WS2812_COLOR_PURPLE,
  WS2812_COLOR_PINK
} WS2812_ColorName_t;

typedef enum
{
  WS2812_EFFECT_NONE = 0,
  WS2812_EFFECT_BLINK,
  WS2812_EFFECT_COLOR_WIPE,
  WS2812_EFFECT_BREATH,
  WS2812_EFFECT_RAINBOW,
  WS2812_EFFECT_THEATER_CHASE
} WS2812_Effect_t;

typedef enum
{
  WS2812_TX_IDLE = 0,
  WS2812_TX_DATA,
  WS2812_TX_RESET
} WS2812_TxState_t;

typedef struct
{
  WS2812_Effect_t type;
  WS2812_Color_t color;
  uint32_t step_ms;
  uint32_t last_tick;
  uint16_t index;
  uint16_t offset;
  uint8_t phase;
  uint8_t brightness;
  int8_t direction;
  uint8_t enabled;
} WS2812_EffectState_t;

typedef struct
{
  void *user;

  /* Start a non-blocking byte transfer. Completion must call WS2812_OnTxComplete(). */
  WS2812_Status_t (*transmit_it)(void *user, const uint8_t *data, uint16_t size);

  /* Return a monotonically increasing millisecond tick. */
  uint32_t (*get_tick_ms)(void *user);
} WS2812_Ops_t;

typedef struct
{
  uint16_t led_count;

  WS2812_Color_t *pixels;
  uint16_t pixel_count;

  uint8_t *tx_buffer;
  uint16_t tx_buffer_size;

  uint8_t *reset_buffer;
  uint16_t reset_buffer_size;

  WS2812_Ops_t ops;
} WS2812_Config_t;

typedef struct
{
  WS2812_Config_t config;
  uint8_t brightness;
  volatile WS2812_TxState_t tx_state;
  WS2812_EffectState_t effect;
} WS2812_Handle_t;

/* Initialize a driver instance. This only clears RAM state; call WS2812_Show() to output. */
WS2812_Status_t WS2812_Init(WS2812_Handle_t *handle, const WS2812_Config_t *config);

/* Non-blocking effect scheduler. Call periodically from main loop or an RTOS task. */
void WS2812_Task(WS2812_Handle_t *handle);

/* Must be called by the BSP when the non-blocking transfer completes. */
void WS2812_OnTxComplete(WS2812_Handle_t *handle);

/* Build a color value using RGB order. The driver converts it to WS2812 GRB internally. */
WS2812_Color_t WS2812_RGB(uint8_t red, uint8_t green, uint8_t blue);

/* Get one of the built-in colors from the color table. */
WS2812_Color_t WS2812_GetColor(WS2812_ColorName_t color);

/* Return color scaled by brightness. brightness range: 0~255. */
WS2812_Color_t WS2812_ScaleColor(WS2812_Color_t color, uint8_t brightness);

/* Set global brightness. Every later refresh is scaled by this value. */
void WS2812_SetBrightness(WS2812_Handle_t *handle, uint8_t brightness);
uint8_t WS2812_GetBrightness(const WS2812_Handle_t *handle);

/* Return 1 while the transport is still sending a WS2812 frame. */
uint8_t WS2812_IsBusy(const WS2812_Handle_t *handle);

/* Write one LED into the RAM buffer. Call WS2812_Show() to refresh the real strip. */
WS2812_Status_t WS2812_SetPixel(WS2812_Handle_t *handle, uint16_t index,
                                uint8_t red, uint8_t green, uint8_t blue);
WS2812_Status_t WS2812_SetPixelColor(WS2812_Handle_t *handle, uint16_t index,
                                     WS2812_Color_t color);

/* Fill the RAM buffer. Call WS2812_Show() to refresh the real strip. */
WS2812_Status_t WS2812_Fill(WS2812_Handle_t *handle, uint8_t red, uint8_t green, uint8_t blue);
WS2812_Status_t WS2812_FillColor(WS2812_Handle_t *handle, WS2812_Color_t color);
WS2812_Status_t WS2812_Clear(WS2812_Handle_t *handle);

/* Start a non-blocking refresh. Returns WS2812_STATUS_BUSY if a refresh is active. */
WS2812_Status_t WS2812_Show(WS2812_Handle_t *handle);

/* Convenience display functions. They stop the current effect before showing the color. */
WS2812_Status_t WS2812_ShowRGB(WS2812_Handle_t *handle, uint8_t red, uint8_t green, uint8_t blue);
WS2812_Status_t WS2812_ShowColor(WS2812_Handle_t *handle, WS2812_Color_t color);
WS2812_Status_t WS2812_ShowNamedColor(WS2812_Handle_t *handle, WS2812_ColorName_t color);
WS2812_Status_t WS2812_ShowOff(WS2812_Handle_t *handle);

/* Non-blocking effects. Start once, then call WS2812_Task() periodically. */
void WS2812_StopEffect(WS2812_Handle_t *handle);
void WS2812_StartBlink(WS2812_Handle_t *handle, WS2812_Color_t color, uint32_t period_ms);
void WS2812_StartColorWipe(WS2812_Handle_t *handle, WS2812_Color_t color, uint32_t step_ms);
void WS2812_StartBreath(WS2812_Handle_t *handle, WS2812_Color_t color, uint32_t step_ms);
void WS2812_StartRainbow(WS2812_Handle_t *handle, uint32_t step_ms);
void WS2812_StartTheaterChase(WS2812_Handle_t *handle, WS2812_Color_t color, uint32_t step_ms);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H__ */
