#include "module_ws2812.h"

#define WS2812_SPI_ZERO_CODE 0x4U
#define WS2812_SPI_ONE_CODE  0x6U

/* Common colors used by WS2812_GetColor(). */
static const WS2812_Color_t ws2812_color_table[] = {
  [WS2812_COLOR_OFF] = {0U, 0U, 0U},
  [WS2812_COLOR_RED] = {255U, 0U, 0U},
  [WS2812_COLOR_GREEN] = {0U, 255U, 0U},
  [WS2812_COLOR_BLUE] = {0U, 0U, 255U},
  [WS2812_COLOR_WHITE] = {255U, 255U, 255U},
  [WS2812_COLOR_WARM_WHITE] = {255U, 180U, 80U},
  [WS2812_COLOR_YELLOW] = {255U, 180U, 0U},
  [WS2812_COLOR_CYAN] = {0U, 255U, 255U},
  [WS2812_COLOR_MAGENTA] = {255U, 0U, 255U},
  [WS2812_COLOR_ORANGE] = {255U, 80U, 0U},
  [WS2812_COLOR_PURPLE] = {120U, 0U, 255U},
  [WS2812_COLOR_PINK] = {255U, 40U, 120U},
};

static uint8_t WS2812_IsValidHandle(const WS2812_Handle_t *handle)
{
  return (handle != 0) &&
         (handle->config.pixels != 0) &&
         (handle->config.tx_buffer != 0) &&
         (handle->config.reset_buffer != 0) &&
         (handle->config.ops.transmit_it != 0) &&
         (handle->config.ops.get_tick_ms != 0);
}

static void WS2812_EncodeByte(uint8_t value, uint8_t *encoded)
{
  uint32_t bits = 0U;

  /* Each WS2812 data bit is encoded into 3 SPI bits: 100 for 0, 110 for 1. */
  for (uint8_t i = 0U; i < 8U; i++)
  {
    bits <<= 3U;
    bits |= ((value & 0x80U) != 0U) ? WS2812_SPI_ONE_CODE : WS2812_SPI_ZERO_CODE;
    value <<= 1U;
  }

  encoded[0] = (uint8_t)(bits >> 16U);
  encoded[1] = (uint8_t)(bits >> 8U);
  encoded[2] = (uint8_t)bits;
}

static WS2812_Color_t WS2812_Wheel(uint8_t position)
{
  /* Convert a 0~255 position to a smooth rainbow color. */
  position = 255U - position;

  if (position < 85U)
  {
    return WS2812_RGB((uint8_t)(255U - position * 3U), 0U, (uint8_t)(position * 3U));
  }

  if (position < 170U)
  {
    position -= 85U;
    return WS2812_RGB(0U, (uint8_t)(position * 3U), (uint8_t)(255U - position * 3U));
  }

  position -= 170U;
  return WS2812_RGB((uint8_t)(position * 3U), (uint8_t)(255U - position * 3U), 0U);
}

static void WS2812_BuildTxBuffer(WS2812_Handle_t *handle)
{
  for (uint16_t i = 0U; i < handle->config.led_count; i++)
  {
    WS2812_Color_t color = WS2812_ScaleColor(handle->config.pixels[i], handle->brightness);
    uint8_t *pixel = &handle->config.tx_buffer[i * WS2812_BYTES_PER_LED];

    /* WS2812 expects GRB byte order, while the public API uses RGB. */
    WS2812_EncodeByte(color.green, &pixel[0]);
    WS2812_EncodeByte(color.red, &pixel[3]);
    WS2812_EncodeByte(color.blue, &pixel[6]);
  }
}

static uint8_t WS2812_IsStepDue(WS2812_Handle_t *handle, uint32_t now)
{
  /* Unsigned subtraction keeps timing safe when the millisecond tick wraps. */
  if ((uint32_t)(now - handle->effect.last_tick) < handle->effect.step_ms)
  {
    return 0U;
  }

  handle->effect.last_tick = now;
  return 1U;
}

static void WS2812_EffectReset(WS2812_Handle_t *handle, WS2812_Effect_t type,
                               WS2812_Color_t color, uint32_t step_ms)
{
  if (handle == 0)
  {
    return;
  }

  /* A zero step would update every call; clamp it to 1 ms. */
  handle->effect.type = type;
  handle->effect.color = color;
  handle->effect.step_ms = (step_ms == 0U) ? 1U : step_ms;
  handle->effect.last_tick = handle->config.ops.get_tick_ms(handle->config.ops.user);
  handle->effect.index = 0U;
  handle->effect.offset = 0U;
  handle->effect.phase = 0U;
  handle->effect.brightness = 0U;
  handle->effect.direction = 1;
  handle->effect.enabled = 1U;
}

WS2812_Status_t WS2812_Init(WS2812_Handle_t *handle, const WS2812_Config_t *config)
{
  if ((handle == 0) || (config == 0) ||
      (config->led_count == 0U) ||
      (config->pixel_count < config->led_count) ||
      (config->tx_buffer_size < WS2812_TX_BUFFER_SIZE(config->led_count)) ||
      (config->reset_buffer_size < WS2812_RESET_BYTES_DEFAULT) ||
      (config->pixels == 0) ||
      (config->tx_buffer == 0) ||
      (config->reset_buffer == 0) ||
      (config->ops.transmit_it == 0) ||
      (config->ops.get_tick_ms == 0))
  {
    return WS2812_STATUS_INVALID_PARAM;
  }

  handle->config = *config;
  handle->brightness = 255U;
  handle->tx_state = WS2812_TX_IDLE;
  handle->effect.enabled = 0U;
  handle->effect.type = WS2812_EFFECT_NONE;

  /* Reset bytes must stay low so the strip latches the previous frame. */
  for (uint16_t i = 0U; i < handle->config.reset_buffer_size; i++)
  {
    handle->config.reset_buffer[i] = 0U;
  }

  return WS2812_Clear(handle);
}

void WS2812_Task(WS2812_Handle_t *handle)
{
  uint32_t now;

  if ((WS2812_IsValidHandle(handle) == 0U) ||
      (handle->effect.enabled == 0U) ||
      WS2812_IsBusy(handle))
  {
    return;
  }

  now = handle->config.ops.get_tick_ms(handle->config.ops.user);
  if (WS2812_IsStepDue(handle, now) == 0U)
  {
    return;
  }

  switch (handle->effect.type)
  {
    case WS2812_EFFECT_BLINK:
      if (handle->effect.phase == 0U)
      {
        (void)WS2812_FillColor(handle, handle->effect.color);
        handle->effect.phase = 1U;
      }
      else
      {
        (void)WS2812_Clear(handle);
        handle->effect.phase = 0U;
      }
      (void)WS2812_Show(handle);
      break;

    case WS2812_EFFECT_COLOR_WIPE:
      if (handle->effect.index == 0U)
      {
        (void)WS2812_Clear(handle);
      }
      (void)WS2812_SetPixelColor(handle, handle->effect.index, handle->effect.color);
      handle->effect.index++;
      if (handle->effect.index >= handle->config.led_count)
      {
        handle->effect.index = 0U;
      }
      (void)WS2812_Show(handle);
      break;

    case WS2812_EFFECT_BREATH:
      (void)WS2812_FillColor(handle, WS2812_ScaleColor(handle->effect.color, handle->effect.brightness));
      if (handle->effect.direction > 0)
      {
        if (handle->effect.brightness >= 250U)
        {
          handle->effect.brightness = 255U;
          handle->effect.direction = -1;
        }
        else
        {
          handle->effect.brightness += 5U;
        }
      }
      else
      {
        if (handle->effect.brightness <= 5U)
        {
          handle->effect.brightness = 0U;
          handle->effect.direction = 1;
        }
        else
        {
          handle->effect.brightness -= 5U;
        }
      }
      (void)WS2812_Show(handle);
      break;

    case WS2812_EFFECT_RAINBOW:
      for (uint16_t i = 0U; i < handle->config.led_count; i++)
      {
        uint8_t position = (uint8_t)(((i * 256U) / handle->config.led_count + handle->effect.offset) & 0xFFU);
        (void)WS2812_SetPixelColor(handle, i, WS2812_Wheel(position));
      }
      handle->effect.offset = (uint16_t)((handle->effect.offset + 1U) & 0xFFU);
      (void)WS2812_Show(handle);
      break;

    case WS2812_EFFECT_THEATER_CHASE:
      (void)WS2812_Clear(handle);
      for (uint16_t i = handle->effect.phase; i < handle->config.led_count; i += 3U)
      {
        (void)WS2812_SetPixelColor(handle, i, handle->effect.color);
      }
      handle->effect.phase++;
      if (handle->effect.phase >= 3U)
      {
        handle->effect.phase = 0U;
      }
      (void)WS2812_Show(handle);
      break;

    default:
      break;
  }
}

void WS2812_OnTxComplete(WS2812_Handle_t *handle)
{
  if (WS2812_IsValidHandle(handle) == 0U)
  {
    return;
  }

  if (handle->tx_state == WS2812_TX_DATA)
  {
    handle->tx_state = WS2812_TX_RESET;
    if (handle->config.ops.transmit_it(handle->config.ops.user,
                                       handle->config.reset_buffer,
                                       handle->config.reset_buffer_size) != WS2812_STATUS_OK)
    {
      handle->tx_state = WS2812_TX_IDLE;
    }
    return;
  }

  handle->tx_state = WS2812_TX_IDLE;
}

WS2812_Color_t WS2812_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
  WS2812_Color_t color = {red, green, blue};
  return color;
}

WS2812_Color_t WS2812_GetColor(WS2812_ColorName_t color)
{
  if ((uint32_t)color >= (sizeof(ws2812_color_table) / sizeof(ws2812_color_table[0])))
  {
    return ws2812_color_table[WS2812_COLOR_OFF];
  }

  return ws2812_color_table[color];
}

WS2812_Color_t WS2812_ScaleColor(WS2812_Color_t color, uint8_t brightness)
{
  color.red = (uint8_t)(((uint16_t)color.red * brightness) / 255U);
  color.green = (uint8_t)(((uint16_t)color.green * brightness) / 255U);
  color.blue = (uint8_t)(((uint16_t)color.blue * brightness) / 255U);
  return color;
}

void WS2812_SetBrightness(WS2812_Handle_t *handle, uint8_t brightness)
{
  if (handle != 0)
  {
    handle->brightness = brightness;
  }
}

uint8_t WS2812_GetBrightness(const WS2812_Handle_t *handle)
{
  return (handle != 0) ? handle->brightness : 0U;
}

uint8_t WS2812_IsBusy(const WS2812_Handle_t *handle)
{
  return ((handle != 0) && (handle->tx_state != WS2812_TX_IDLE)) ? 1U : 0U;
}

WS2812_Status_t WS2812_SetPixel(WS2812_Handle_t *handle, uint16_t index,
                                uint8_t red, uint8_t green, uint8_t blue)
{
  return WS2812_SetPixelColor(handle, index, WS2812_RGB(red, green, blue));
}

WS2812_Status_t WS2812_SetPixelColor(WS2812_Handle_t *handle, uint16_t index,
                                     WS2812_Color_t color)
{
  if ((WS2812_IsValidHandle(handle) == 0U) || (index >= handle->config.led_count))
  {
    return WS2812_STATUS_INVALID_PARAM;
  }

  handle->config.pixels[index] = color;
  return WS2812_STATUS_OK;
}

WS2812_Status_t WS2812_Fill(WS2812_Handle_t *handle, uint8_t red, uint8_t green, uint8_t blue)
{
  return WS2812_FillColor(handle, WS2812_RGB(red, green, blue));
}

WS2812_Status_t WS2812_FillColor(WS2812_Handle_t *handle, WS2812_Color_t color)
{
  if (WS2812_IsValidHandle(handle) == 0U)
  {
    return WS2812_STATUS_INVALID_PARAM;
  }

  for (uint16_t i = 0U; i < handle->config.led_count; i++)
  {
    handle->config.pixels[i] = color;
  }

  return WS2812_STATUS_OK;
}

WS2812_Status_t WS2812_Clear(WS2812_Handle_t *handle)
{
  return WS2812_Fill(handle, 0U, 0U, 0U);
}

WS2812_Status_t WS2812_Show(WS2812_Handle_t *handle)
{
  WS2812_Status_t status;

  if (WS2812_IsValidHandle(handle) == 0U)
  {
    return WS2812_STATUS_INVALID_PARAM;
  }

  if (WS2812_IsBusy(handle))
  {
    return WS2812_STATUS_BUSY;
  }

  /* Build the encoded frame locally, then ask the BSP transport to send it. */
  WS2812_BuildTxBuffer(handle);
  handle->tx_state = WS2812_TX_DATA;
  status = handle->config.ops.transmit_it(handle->config.ops.user,
                                          handle->config.tx_buffer,
                                          WS2812_TX_BUFFER_SIZE(handle->config.led_count));
  if (status != WS2812_STATUS_OK)
  {
    handle->tx_state = WS2812_TX_IDLE;
  }

  return status;
}

WS2812_Status_t WS2812_ShowRGB(WS2812_Handle_t *handle, uint8_t red, uint8_t green, uint8_t blue)
{
  WS2812_StopEffect(handle);
  (void)WS2812_Fill(handle, red, green, blue);
  return WS2812_Show(handle);
}

WS2812_Status_t WS2812_ShowColor(WS2812_Handle_t *handle, WS2812_Color_t color)
{
  WS2812_StopEffect(handle);
  (void)WS2812_FillColor(handle, color);
  return WS2812_Show(handle);
}

WS2812_Status_t WS2812_ShowNamedColor(WS2812_Handle_t *handle, WS2812_ColorName_t color)
{
  return WS2812_ShowColor(handle, WS2812_GetColor(color));
}

WS2812_Status_t WS2812_ShowOff(WS2812_Handle_t *handle)
{
  WS2812_StopEffect(handle);
  (void)WS2812_Clear(handle);
  return WS2812_Show(handle);
}

void WS2812_StopEffect(WS2812_Handle_t *handle)
{
  if (handle != 0)
  {
    handle->effect.enabled = 0U;
    handle->effect.type = WS2812_EFFECT_NONE;
  }
}

void WS2812_StartBlink(WS2812_Handle_t *handle, WS2812_Color_t color, uint32_t period_ms)
{
  WS2812_EffectReset(handle, WS2812_EFFECT_BLINK, color, period_ms / 2U);
}

void WS2812_StartColorWipe(WS2812_Handle_t *handle, WS2812_Color_t color, uint32_t step_ms)
{
  WS2812_EffectReset(handle, WS2812_EFFECT_COLOR_WIPE, color, step_ms);
}

void WS2812_StartBreath(WS2812_Handle_t *handle, WS2812_Color_t color, uint32_t step_ms)
{
  WS2812_EffectReset(handle, WS2812_EFFECT_BREATH, color, step_ms);
}

void WS2812_StartRainbow(WS2812_Handle_t *handle, uint32_t step_ms)
{
  WS2812_EffectReset(handle, WS2812_EFFECT_RAINBOW, WS2812_GetColor(WS2812_COLOR_OFF), step_ms);
}

void WS2812_StartTheaterChase(WS2812_Handle_t *handle, WS2812_Color_t color, uint32_t step_ms)
{
  WS2812_EffectReset(handle, WS2812_EFFECT_THEATER_CHASE, color, step_ms);
}
