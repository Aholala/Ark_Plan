/**
 * @file module_ws2812.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief WS2812 核心驱动模块实现（非阻塞、效果调度）
 * @version 1.0
 * @date 2026-06-18
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "module_ws2812.h"

/* SPI 编码常量：WS2812 每个数据位对应 3 个 SPI 位 */
#define WS2812_SPI_ZERO_CODE 0x4U /* 100 -> 逻辑 0 */
#define WS2812_SPI_ONE_CODE 0x6U  /* 110 -> 逻辑 1 */

/* 预定义常用颜色表（RGB 顺序） */
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

/**
 * @brief 检查句柄是否有效
 * @param handle 驱动句柄
 * @return uint8_t 1=有效，0=无效
 */
static uint8_t WS2812_IsValidHandle(const WS2812_Handle_t *handle) {
  return (handle != 0) && (handle->config.pixels != 0) &&
         (handle->config.tx_buffer != 0) &&
         (handle->config.reset_buffer != 0) &&
         (handle->config.ops.transmit_it != 0) &&
         (handle->config.ops.get_tick_ms != 0);
}

/**
 * @brief 将一个字节编码为 SPI 数据（每 bit -> 3 bits）
 * @param value 待编码字节
 * @param encoded 输出缓冲区（需 3 字节）
 *
 * 编码规则：WS2812 逻辑 0 对应 SPI 0x4 (100)，逻辑 1 对应 0x6 (110)
 */
static void WS2812_EncodeByte(uint8_t value, uint8_t *encoded) {
  uint32_t bits = 0U;

  for (uint8_t i = 0U; i < 8U; i++) {
    bits <<= 3U;
    bits |=
        ((value & 0x80U) != 0U) ? WS2812_SPI_ONE_CODE : WS2812_SPI_ZERO_CODE;
    value <<= 1U;
  }

  encoded[0] = (uint8_t)(bits >> 16U);
  encoded[1] = (uint8_t)(bits >> 8U);
  encoded[2] = (uint8_t)bits;
}

/**
 * @brief 生成彩虹色轮（0~255 位置）
 * @param position 色轮位置
 * @return WS2812_Color_t 对应的 RGB 颜色
 */
static WS2812_Color_t WS2812_Wheel(uint8_t position) {
  position = 255U - position;

  if (position < 85U) {
    return WS2812_RGB((uint8_t)(255U - position * 3U), 0U,
                      (uint8_t)(position * 3U));
  }

  if (position < 170U) {
    position -= 85U;
    return WS2812_RGB(0U, (uint8_t)(position * 3U),
                      (uint8_t)(255U - position * 3U));
  }

  position -= 170U;
  return WS2812_RGB((uint8_t)(position * 3U), (uint8_t)(255U - position * 3U),
                    0U);
}

/**
 * @brief 构建 SPI 发送缓冲区（将像素数据编码为 SPI 帧）
 * @param handle 驱动句柄
 *
 * 将像素缓冲区中的 RGB 数据按 WS2812 协议编码（GRB 顺序），
 * 并存入 tx_buffer，同时应用全局亮度。
 */
static void WS2812_BuildTxBuffer(WS2812_Handle_t *handle) {
  for (uint16_t i = 0U; i < handle->config.led_count; i++) {
    WS2812_Color_t color =
        WS2812_ScaleColor(handle->config.pixels[i], handle->brightness);
    uint8_t *pixel = &handle->config.tx_buffer[i * WS2812_BYTES_PER_LED];

    /* WS2812 实际顺序为 GRB，而 API 使用 RGB，故先绿后红再蓝 */
    WS2812_EncodeByte(color.green, &pixel[0]);
    WS2812_EncodeByte(color.red, &pixel[3]);
    WS2812_EncodeByte(color.blue, &pixel[6]);
  }
}

/**
 * @brief 判断效果是否应该更新（基于 tick）
 * @param handle 驱动句柄
 * @param now 当前毫秒数
 * @return uint8_t 1=需要更新，0=未到时间
 */
static uint8_t WS2812_IsStepDue(WS2812_Handle_t *handle, uint32_t now) {
  /* 使用无符号减法处理 tick 溢出 */
  if ((uint32_t)(now - handle->effect.last_tick) < handle->effect.step_ms) {
    return 0U;
  }

  handle->effect.last_tick = now;
  return 1U;
}

/**
 * @brief 重置效果状态（通用初始化）
 * @param handle 驱动句柄
 * @param type 效果类型
 * @param color 颜色（部分效果使用）
 * @param step_ms 步进间隔（毫秒），若为 0 则强制为 1
 */
static void WS2812_EffectReset(WS2812_Handle_t *handle, WS2812_Effect_t type,
                               WS2812_Color_t color, uint32_t step_ms) {
  if (handle == 0) {
    return;
  }

  handle->effect.type = type;
  handle->effect.color = color;
  handle->effect.step_ms = (step_ms == 0U) ? 1U : step_ms;
  handle->effect.last_tick =
      handle->config.ops.get_tick_ms(handle->config.ops.user);
  handle->effect.index = 0U;
  handle->effect.offset = 0U;
  handle->effect.phase = 0U;
  handle->effect.brightness = 0U;
  handle->effect.direction = 1;
  handle->effect.enabled = 1U;
}

/**
 * @brief 初始化驱动实例（仅清空 RAM 状态，需调用 WS2812_Show() 输出）
 * @param handle 驱动句柄
 * @param config 配置参数
 * @return WS2812_Status_t 初始化结果
 */
WS2812_Status_t WS2812_Init(WS2812_Handle_t *handle,
                            const WS2812_Config_t *config) {
  if ((handle == 0) || (config == 0) || (config->led_count == 0U) ||
      (config->pixel_count < config->led_count) ||
      (config->tx_buffer_size < WS2812_TX_BUFFER_SIZE(config->led_count)) ||
      (config->reset_buffer_size < WS2812_RESET_BYTES_DEFAULT) ||
      (config->pixels == 0) || (config->tx_buffer == 0) ||
      (config->reset_buffer == 0) || (config->ops.transmit_it == 0) ||
      (config->ops.get_tick_ms == 0)) {
    return WS2812_STATUS_INVALID_PARAM;
  }

  handle->config = *config;
  handle->brightness = 255U;
  handle->tx_state = WS2812_TX_IDLE;
  handle->effect.enabled = 0U;
  handle->effect.type = WS2812_EFFECT_NONE;

  /* 复位缓冲区全零，确保 WS2812 锁存数据 */
  for (uint16_t i = 0U; i < handle->config.reset_buffer_size; i++) {
    handle->config.reset_buffer[i] = 0U;
  }

  return WS2812_Clear(handle);
}

/**
 * @brief 非阻塞效果调度任务，需在主循环或 RTOS 任务中周期调用
 * @param handle 驱动句柄
 */
void WS2812_Task(WS2812_Handle_t *handle) {
  uint32_t now;

  if ((WS2812_IsValidHandle(handle) == 0U) || (handle->effect.enabled == 0U) ||
      WS2812_IsBusy(handle)) {
    return;
  }

  now = handle->config.ops.get_tick_ms(handle->config.ops.user);
  if (WS2812_IsStepDue(handle, now) == 0U) {
    return;
  }

  /* 根据不同效果类型执行相应更新 */
  switch (handle->effect.type) {
  case WS2812_EFFECT_BLINK: /* 闪烁 */
    if (handle->effect.phase == 0U) {
      (void)WS2812_FillColor(handle, handle->effect.color);
      handle->effect.phase = 1U;
    } else {
      (void)WS2812_Clear(handle);
      handle->effect.phase = 0U;
    }
    (void)WS2812_Show(handle);
    break;

  case WS2812_EFFECT_COLOR_WIPE: /* 颜色填充 */
    if (handle->effect.index == 0U) {
      (void)WS2812_Clear(handle);
    }
    (void)WS2812_SetPixelColor(handle, handle->effect.index,
                               handle->effect.color);
    handle->effect.index++;
    if (handle->effect.index >= handle->config.led_count) {
      handle->effect.index = 0U;
    }
    (void)WS2812_Show(handle);
    break;

  case WS2812_EFFECT_BREATH: /* 呼吸灯 */
    (void)WS2812_FillColor(
        handle,
        WS2812_ScaleColor(handle->effect.color, handle->effect.brightness));
    if (handle->effect.direction > 0) {
      if (handle->effect.brightness >= 250U) {
        handle->effect.brightness = 255U;
        handle->effect.direction = -1;
      } else {
        handle->effect.brightness += 5U;
      }
    } else {
      if (handle->effect.brightness <= 5U) {
        handle->effect.brightness = 0U;
        handle->effect.direction = 1;
      } else {
        handle->effect.brightness -= 5U;
      }
    }
    (void)WS2812_Show(handle);
    break;

  case WS2812_EFFECT_RAINBOW: /* 彩虹循环 */
    for (uint16_t i = 0U; i < handle->config.led_count; i++) {
      uint8_t position = (uint8_t)(((i * 256U) / handle->config.led_count +
                                    handle->effect.offset) &
                                   0xFFU);
      (void)WS2812_SetPixelColor(handle, i, WS2812_Wheel(position));
    }
    handle->effect.offset = (uint16_t)((handle->effect.offset + 1U) & 0xFFU);
    (void)WS2812_Show(handle);
    break;

  case WS2812_EFFECT_THEATER_CHASE: /* 剧院追逐 */
    (void)WS2812_Clear(handle);
    for (uint16_t i = handle->effect.phase; i < handle->config.led_count;
         i += 3U) {
      (void)WS2812_SetPixelColor(handle, i, handle->effect.color);
    }
    handle->effect.phase++;
    if (handle->effect.phase >= 3U) {
      handle->effect.phase = 0U;
    }
    (void)WS2812_Show(handle);
    break;

  default:
    break;
  }
}

/**
 * @brief 发送完成回调（由 BSP 层调用）
 * @param handle 驱动句柄
 *
 * 当数据发送完成后，若当前是数据阶段，则自动发送复位信号；
 * 若复位信号发送完成，则置空闲状态。
 */
void WS2812_OnTxComplete(WS2812_Handle_t *handle) {
  if (WS2812_IsValidHandle(handle) == 0U) {
    return;
  }

  if (handle->tx_state == WS2812_TX_DATA) {
    handle->tx_state = WS2812_TX_RESET;
    if (handle->config.ops.transmit_it(
            handle->config.ops.user, handle->config.reset_buffer,
            handle->config.reset_buffer_size) != WS2812_STATUS_OK) {
      handle->tx_state = WS2812_TX_IDLE;
    }
    return;
  }

  handle->tx_state = WS2812_TX_IDLE;
}

/**
 * @brief 创建 RGB 颜色结构体
 * @param red 红色分量 (0-255)
 * @param green 绿色分量 (0-255)
 * @param blue 蓝色分量 (0-255)
 * @return WS2812_Color_t RGB 颜色
 */
WS2812_Color_t WS2812_RGB(uint8_t red, uint8_t green, uint8_t blue) {
  WS2812_Color_t color = {red, green, blue};
  return color;
}

/**
 * @brief 获取内置颜色名称对应的颜色值
 * @param color 颜色枚举
 * @return WS2812_Color_t 对应的 RGB，若无效返回黑色
 */
WS2812_Color_t WS2812_GetColor(WS2812_ColorName_t color) {
  if ((uint32_t)color >=
      (sizeof(ws2812_color_table) / sizeof(ws2812_color_table[0]))) {
    return ws2812_color_table[WS2812_COLOR_OFF];
  }

  return ws2812_color_table[color];
}

/**
 * @brief 按亮度缩放颜色
 * @param color 原始颜色
 * @param brightness 亮度 (0-255)
 * @return WS2812_Color_t 缩放后的颜色
 */
WS2812_Color_t WS2812_ScaleColor(WS2812_Color_t color, uint8_t brightness) {
  color.red = (uint8_t)(((uint16_t)color.red * brightness) / 255U);
  color.green = (uint8_t)(((uint16_t)color.green * brightness) / 255U);
  color.blue = (uint8_t)(((uint16_t)color.blue * brightness) / 255U);
  return color;
}

/**
 * @brief 设置全局亮度
 * @param handle 驱动句柄
 * @param brightness 亮度 (0-255)
 */
void WS2812_SetBrightness(WS2812_Handle_t *handle, uint8_t brightness) {
  if (handle != 0) {
    handle->brightness = brightness;
  }
}

/**
 * @brief 获取全局亮度
 * @param handle 驱动句柄
 * @return uint8_t 当前亮度
 */
uint8_t WS2812_GetBrightness(const WS2812_Handle_t *handle) {
  return (handle != 0) ? handle->brightness : 0U;
}

/**
 * @brief 检查驱动是否正在发送（忙碌）
 * @param handle 驱动句柄
 * @return uint8_t 1=忙碌，0=空闲
 */
uint8_t WS2812_IsBusy(const WS2812_Handle_t *handle) {
  return ((handle != 0) && (handle->tx_state != WS2812_TX_IDLE)) ? 1U : 0U;
}

/**
 * @brief 设置单个 LED 颜色（通过 RGB 分量）
 * @param handle 驱动句柄
 * @param index LED 索引
 * @param red 红色
 * @param green 绿色
 * @param blue 蓝色
 * @return WS2812_Status_t 操作状态
 */
WS2812_Status_t WS2812_SetPixel(WS2812_Handle_t *handle, uint16_t index,
                                uint8_t red, uint8_t green, uint8_t blue) {
  return WS2812_SetPixelColor(handle, index, WS2812_RGB(red, green, blue));
}

/**
 * @brief 设置单个 LED 颜色（颜色结构体）
 * @param handle 驱动句柄
 * @param index LED 索引
 * @param color RGB 颜色
 * @return WS2812_Status_t 操作状态
 */
WS2812_Status_t WS2812_SetPixelColor(WS2812_Handle_t *handle, uint16_t index,
                                     WS2812_Color_t color) {
  if ((WS2812_IsValidHandle(handle) == 0U) ||
      (index >= handle->config.led_count)) {
    return WS2812_STATUS_INVALID_PARAM;
  }

  handle->config.pixels[index] = color;
  return WS2812_STATUS_OK;
}

/**
 * @brief 填充所有 LED 为同一颜色（RGB 分量）
 * @param handle 驱动句柄
 * @param red 红色
 * @param green 绿色
 * @param blue 蓝色
 * @return WS2812_Status_t 操作状态
 */
WS2812_Status_t WS2812_Fill(WS2812_Handle_t *handle, uint8_t red, uint8_t green,
                            uint8_t blue) {
  return WS2812_FillColor(handle, WS2812_RGB(red, green, blue));
}

/**
 * @brief 填充所有 LED 为同一颜色（颜色结构体）
 * @param handle 驱动句柄
 * @param color RGB 颜色
 * @return WS2812_Status_t 操作状态
 */
WS2812_Status_t WS2812_FillColor(WS2812_Handle_t *handle,
                                 WS2812_Color_t color) {
  if (WS2812_IsValidHandle(handle) == 0U) {
    return WS2812_STATUS_INVALID_PARAM;
  }

  for (uint16_t i = 0U; i < handle->config.led_count; i++) {
    handle->config.pixels[i] = color;
  }

  return WS2812_STATUS_OK;
}

/**
 * @brief 清空所有 LED（关闭）
 * @param handle 驱动句柄
 * @return WS2812_Status_t 操作状态
 */
WS2812_Status_t WS2812_Clear(WS2812_Handle_t *handle) {
  return WS2812_Fill(handle, 0U, 0U, 0U);
}

/**
 * @brief 刷新 LED 显示（非阻塞发送）
 * @param handle 驱动句柄
 * @return WS2812_Status_t 操作状态（若忙碌则返回 BUSY）
 *
 * 该函数构建 SPI 帧并启动非阻塞发送，后续通过回调完成。
 */
WS2812_Status_t WS2812_Show(WS2812_Handle_t *handle) {
  WS2812_Status_t status;

  if (WS2812_IsValidHandle(handle) == 0U) {
    return WS2812_STATUS_INVALID_PARAM;
  }

  if (WS2812_IsBusy(handle)) {
    return WS2812_STATUS_BUSY;
  }

  WS2812_BuildTxBuffer(handle);
  handle->tx_state = WS2812_TX_DATA;
  status = handle->config.ops.transmit_it(
      handle->config.ops.user, handle->config.tx_buffer,
      WS2812_TX_BUFFER_SIZE(handle->config.led_count));
  if (status != WS2812_STATUS_OK) {
    handle->tx_state = WS2812_TX_IDLE;
  }

  return status;
}

/**
 * @brief 便捷函数：显示单一 RGB 颜色（停止效果）
 * @param handle 驱动句柄
 * @param red 红色
 * @param green 绿色
 * @param blue 蓝色
 * @return WS2812_Status_t 操作状态
 */
WS2812_Status_t WS2812_ShowRGB(WS2812_Handle_t *handle, uint8_t red,
                               uint8_t green, uint8_t blue) {
  WS2812_StopEffect(handle);
  (void)WS2812_Fill(handle, red, green, blue);
  return WS2812_Show(handle);
}

/**
 * @brief 便捷函数：显示单一颜色（颜色结构体，停止效果）
 * @param handle 驱动句柄
 * @param color RGB 颜色
 * @return WS2812_Status_t 操作状态
 */
WS2812_Status_t WS2812_ShowColor(WS2812_Handle_t *handle,
                                 WS2812_Color_t color) {
  WS2812_StopEffect(handle);
  (void)WS2812_FillColor(handle, color);
  return WS2812_Show(handle);
}

/**
 * @brief 便捷函数：显示预定义颜色名称（停止效果）
 * @param handle 驱动句柄
 * @param color 颜色枚举
 * @return WS2812_Status_t 操作状态
 */
WS2812_Status_t WS2812_ShowNamedColor(WS2812_Handle_t *handle,
                                      WS2812_ColorName_t color) {
  return WS2812_ShowColor(handle, WS2812_GetColor(color));
}

/**
 * @brief 便捷函数：关闭所有 LED（停止效果）
 * @param handle 驱动句柄
 * @return WS2812_Status_t 操作状态
 */
WS2812_Status_t WS2812_ShowOff(WS2812_Handle_t *handle) {
  WS2812_StopEffect(handle);
  (void)WS2812_Clear(handle);
  return WS2812_Show(handle);
}

/**
 * @brief 停止当前运行的效果
 * @param handle 驱动句柄
 */
void WS2812_StopEffect(WS2812_Handle_t *handle) {
  if (handle != 0) {
    handle->effect.enabled = 0U;
    handle->effect.type = WS2812_EFFECT_NONE;
  }
}

/**
 * @brief 启动闪烁效果
 * @param handle 驱动句柄
 * @param color 闪烁颜色
 * @param period_ms 闪烁周期（毫秒）
 */
void WS2812_StartBlink(WS2812_Handle_t *handle, WS2812_Color_t color,
                       uint32_t period_ms) {
  WS2812_EffectReset(handle, WS2812_EFFECT_BLINK, color, period_ms / 2U);
}

/**
 * @brief 启动颜色填充效果（逐个点亮）
 * @param handle 驱动句柄
 * @param color 填充颜色
 * @param step_ms 每步间隔（毫秒）
 */
void WS2812_StartColorWipe(WS2812_Handle_t *handle, WS2812_Color_t color,
                           uint32_t step_ms) {
  WS2812_EffectReset(handle, WS2812_EFFECT_COLOR_WIPE, color, step_ms);
}

/**
 * @brief 启动呼吸灯效果
 * @param handle 驱动句柄
 * @param color 呼吸颜色
 * @param step_ms 每步间隔（毫秒）
 */
void WS2812_StartBreath(WS2812_Handle_t *handle, WS2812_Color_t color,
                        uint32_t step_ms) {
  WS2812_EffectReset(handle, WS2812_EFFECT_BREATH, color, step_ms);
}

/**
 * @brief 启动彩虹循环效果
 * @param handle 驱动句柄
 * @param step_ms 每步间隔（毫秒）
 */
void WS2812_StartRainbow(WS2812_Handle_t *handle, uint32_t step_ms) {
  WS2812_EffectReset(handle, WS2812_EFFECT_RAINBOW,
                     WS2812_GetColor(WS2812_COLOR_OFF), step_ms);
}

/**
 * @brief 启动剧院追逐效果
 * @param handle 驱动句柄
 * @param color 追逐颜色
 * @param step_ms 每步间隔（毫秒）
 */
void WS2812_StartTheaterChase(WS2812_Handle_t *handle, WS2812_Color_t color,
                              uint32_t step_ms) {
  WS2812_EffectReset(handle, WS2812_EFFECT_THEATER_CHASE, color, step_ms);
}