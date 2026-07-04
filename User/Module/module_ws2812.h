/**
 * @file module_ws2812.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief WS2812 核心驱动模块头文件（API 定义）
 * @version 1.0
 * @date 2026-06-17
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef __WS2812_H__
#define __WS2812_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 每个 LED 需要的 SPI 字节数（3 字节编码） */
#define WS2812_BYTES_PER_LED 9U
/* 复位信号最小字节数（典型 64 字节） */
#define WS2812_RESET_BYTES_DEFAULT 64U
/* 计算发送缓冲区大小宏 */
#define WS2812_TX_BUFFER_SIZE(led_count)                                       \
  ((uint16_t)((led_count) * WS2812_BYTES_PER_LED))

/**
 * @brief 驱动状态码
 */
typedef enum {
  WS2812_STATUS_OK = 0,
  WS2812_STATUS_BUSY,         /* 发送忙碌 */
  WS2812_STATUS_ERROR,        /* 发送错误 */
  WS2812_STATUS_INVALID_PARAM /* 参数无效 */
} WS2812_Status_t;

/**
 * @brief RGB 颜色结构体（红绿蓝顺序）
 */
typedef struct {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} WS2812_Color_t;

/**
 * @brief 内置颜色名称枚举
 */
typedef enum {
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

/**
 * @brief 支持的效果类型
 */
typedef enum {
  WS2812_EFFECT_NONE = 0,
  WS2812_EFFECT_BLINK,        /* 闪烁 */
  WS2812_EFFECT_COLOR_WIPE,   /* 颜色填充 */
  WS2812_EFFECT_BREATH,       /* 呼吸 */
  WS2812_EFFECT_RAINBOW,      /* 彩虹 */
  WS2812_EFFECT_THEATER_CHASE /* 剧院追逐 */
} WS2812_Effect_t;

/**
 * @brief 发送状态机
 */
typedef enum {
  WS2812_TX_IDLE = 0, /* 空闲 */
  WS2812_TX_DATA,     /* 正在发送数据 */
  WS2812_TX_RESET     /* 正在发送复位信号 */
} WS2812_TxState_t;

/**
 * @brief 效果运行时状态
 */
typedef struct {
  WS2812_Effect_t type; /* 效果类型 */
  WS2812_Color_t color; /* 效果颜色 */
  uint32_t step_ms;     /* 步进间隔（毫秒） */
  uint32_t last_tick;   /* 上次更新时间戳 */
  uint16_t index;       /* 当前索引（用于 wipe） */
  uint16_t offset;      /* 偏移（用于彩虹） */
  uint8_t phase;        /* 阶段（用于闪烁/追逐） */
  uint8_t brightness;   /* 当前亮度（用于呼吸） */
  int8_t direction;     /* 亮度增减方向（呼吸） */
  uint8_t enabled;      /* 效果使能标志 */
} WS2812_EffectState_t;

/**
 * @brief 底层操作接口（由 BSP 层实现）
 */
typedef struct {
  void *user; /* 用户数据（如 SPI 句柄） */

  /**
   * @brief 启动非阻塞字节发送，完成后必须调用 WS2812_OnTxComplete()
   */
  WS2812_Status_t (*transmit_it)(void *user, const uint8_t *data,
                                 uint16_t size);

  /**
   * @brief 获取单调递增的毫秒数
   */
  uint32_t (*get_tick_ms)(void *user);
} WS2812_Ops_t;

/**
 * @brief 驱动配置结构体
 */
typedef struct {
  uint16_t led_count; /* LED 数量 */

  WS2812_Color_t *pixels; /* 像素缓冲区（用户分配） */
  uint16_t pixel_count;   /* 缓冲区大小（至少 led_count） */

  uint8_t *tx_buffer;      /* 发送缓冲区（用户分配） */
  uint16_t tx_buffer_size; /* 发送缓冲区大小（需 >= TX_BUFFER_SIZE） */

  uint8_t *reset_buffer;      /* 复位缓冲区（用户分配） */
  uint16_t reset_buffer_size; /* 复位缓冲区大小（需 >= RESET_BYTES_DEFAULT） */

  WS2812_Ops_t ops; /* 底层操作接口 */
} WS2812_Config_t;

/**
 * @brief 驱动句柄结构体
 */
typedef struct {
  WS2812_Config_t config;             /* 配置副本 */
  uint8_t brightness;                 /* 全局亮度 (0-255) */
  volatile WS2812_TxState_t tx_state; /* 发送状态 */
  WS2812_EffectState_t effect;        /* 效果状态 */
} WS2812_Handle_t;

/* -------- 核心 API -------- */

/**
 * @brief 初始化驱动实例
 * @param handle 驱动句柄
 * @param config 配置参数
 * @return WS2812_Status_t 初始化结果
 */
WS2812_Status_t WS2812_Init(WS2812_Handle_t *handle,
                            const WS2812_Config_t *config);

/**
 * @brief 非阻塞效果任务，需周期性调用
 * @param handle 驱动句柄
 */
void WS2812_Task(WS2812_Handle_t *handle);

/**
 * @brief 发送完成回调，由 BSP 层调用
 * @param handle 驱动句柄
 */
void WS2812_OnTxComplete(WS2812_Handle_t *handle);

/* -------- 颜色工具 -------- */

/**
 * @brief 构造 RGB 颜色
 */
WS2812_Color_t WS2812_RGB(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief 根据名称获取内置颜色
 */
WS2812_Color_t WS2812_GetColor(WS2812_ColorName_t color);

/**
 * @brief 按亮度缩放颜色
 */
WS2812_Color_t WS2812_ScaleColor(WS2812_Color_t color, uint8_t brightness);

/* -------- 亮度控制 -------- */

void WS2812_SetBrightness(WS2812_Handle_t *handle, uint8_t brightness);
uint8_t WS2812_GetBrightness(const WS2812_Handle_t *handle);

/* -------- 状态查询 -------- */

/**
 * @brief 检查驱动是否正在发送
 */
uint8_t WS2812_IsBusy(const WS2812_Handle_t *handle);

/* -------- 像素缓冲操作 -------- */

WS2812_Status_t WS2812_SetPixel(WS2812_Handle_t *handle, uint16_t index,
                                uint8_t red, uint8_t green, uint8_t blue);
WS2812_Status_t WS2812_SetPixelColor(WS2812_Handle_t *handle, uint16_t index,
                                     WS2812_Color_t color);
WS2812_Status_t WS2812_Fill(WS2812_Handle_t *handle, uint8_t red, uint8_t green,
                            uint8_t blue);
WS2812_Status_t WS2812_FillColor(WS2812_Handle_t *handle, WS2812_Color_t color);
WS2812_Status_t WS2812_Clear(WS2812_Handle_t *handle);

/* -------- 刷新显示 -------- */

/**
 * @brief 非阻塞刷新显示，返回 BUSY 表示正在发送
 */
WS2812_Status_t WS2812_Show(WS2812_Handle_t *handle);

/* -------- 快捷显示（停止效果） -------- */

WS2812_Status_t WS2812_ShowRGB(WS2812_Handle_t *handle, uint8_t red,
                               uint8_t green, uint8_t blue);
WS2812_Status_t WS2812_ShowColor(WS2812_Handle_t *handle, WS2812_Color_t color);
WS2812_Status_t WS2812_ShowNamedColor(WS2812_Handle_t *handle,
                                      WS2812_ColorName_t color);
WS2812_Status_t WS2812_ShowOff(WS2812_Handle_t *handle);

/* -------- 效果控制 -------- */

void WS2812_StopEffect(WS2812_Handle_t *handle);
void WS2812_StartBlink(WS2812_Handle_t *handle, WS2812_Color_t color,
                       uint32_t period_ms);
void WS2812_StartColorWipe(WS2812_Handle_t *handle, WS2812_Color_t color,
                           uint32_t step_ms);
void WS2812_StartBreath(WS2812_Handle_t *handle, WS2812_Color_t color,
                        uint32_t step_ms);
void WS2812_StartRainbow(WS2812_Handle_t *handle, uint32_t step_ms);
void WS2812_StartTheaterChase(WS2812_Handle_t *handle, WS2812_Color_t color,
                              uint32_t step_ms);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H__ */