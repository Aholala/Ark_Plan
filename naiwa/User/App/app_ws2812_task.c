/**
 * @file app_ws2812_task.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief WS2812灯带控制任务 - 实现视觉颜色显示与闪烁效果
 * @version 2.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本任务负责：
 *          1. 初始化WS2812灯带；
 *          2. 通过USB CDC接收视觉协议帧，解析颜色信息；
 *          3. 按分区显示颜色：
 *             核心(1-21,50-53) / 基础(22-29,42-49) / 球框1(38-41), 球框2(34-38), 球框3(30-33)；
 *          4. 实现呼吸/闪烁效果，以突出显示目标颜色。
 */

#include "app_ws2812_task.h"

#include "bsp_time.h"
#include "bsp_usb_cdc.h"
#include "bsp_ws2812.h"
#include "cmsis_os.h"
#include "module_vision_protocol.h"

/*==================== 宏定义 ====================*/

/** @brief 彩虹动态效果步进间隔（毫秒） */
#define APP_WS2812_RAINBOW_STEP_MS 20U

/** @brief USB CDC接收缓冲区大小（字节） */
#define APP_WS2812_RX_BUFFER_SIZE 64U

/** @brief 任务主循环延时（毫秒） */
#define APP_WS2812_TASK_DELAY_MS 1U

/** @brief 颜色闪烁周期（毫秒） */
#define APP_WS2812_BLINK_PERIOD_MS 500U

/** @brief 数据超时进入闪烁的时间（毫秒） */
#define APP_WS2812_DATA_TIMEOUT_MS 1000U

/** @brief 核心框灯珠数量（前N颗显示芯色，其余显示底色） */
#define APP_WS2812_CORE_LED_COUNT 17U

/*==================== 灯珠分区定义（0-indexed） ====================*/

/** @brief 核心区：索引 [0,20] ∪ [49,52] 显示芯色 */
#define ZONE_CORE_LO    0U
#define ZONE_CORE_HI   20U
#define ZONE_CORE2_LO  49U
#define ZONE_CORE2_HI  52U

/** @brief 基础区：索引 [21,28] ∪ [41,48] 显示底色 */
#define ZONE_BASE_LO   21U
#define ZONE_BASE_HI   28U
#define ZONE_BASE2_LO  41U
#define ZONE_BASE2_HI  48U

/** @brief 球框颜色1：索引 [37,40] */
#define ZONE_BALL1_LO  37U
#define ZONE_BALL1_HI  40U

/** @brief 球框颜色2：索引 [33,37] */
#define ZONE_BALL2_LO  33U
#define ZONE_BALL2_HI  36U

/** @brief 球框颜色3：索引 [29,32] */
#define ZONE_BALL3_LO  29U
#define ZONE_BALL3_HI  32U

/*==================== 外部函数声明 ====================*/

/*==================== 静态变量 ====================*/

/** @brief 当前显示的底色（WS2812颜色格式） */
static WS2812_Color_t app_ws2812_base_color;

/** @brief 当前显示的芯色（WS2812颜色格式） */
static WS2812_Color_t app_ws2812_core_color;

/** @brief 球框颜色1 */
static WS2812_Color_t app_ws2812_ball_color1;

/** @brief 球框颜色2 */
static WS2812_Color_t app_ws2812_ball_color2;

/** @brief 球框颜色3 */
static WS2812_Color_t app_ws2812_ball_color3;

/** @brief 上次闪烁切换的时刻（毫秒） */
static uint32_t app_ws2812_last_blink_tick;

/** @brief 闪烁功能使能标志（1=使能，0=禁止） */
static uint8_t app_ws2812_blink_enabled;

/** @brief 当前闪烁状态（1=亮，0=灭） */
static uint8_t app_ws2812_blink_on;

/** @brief 是否曾经收到过视觉数据（0=从未，1=收到过） */
static uint8_t app_ws2812_data_ever;

/*==================== Ozone 调试变量（volatile，可直接 Watch） ====================*/

/** @brief 接收到视觉数据的帧计数 */
volatile uint32_t ws2812_debug_rx_count = 0U;

/** @brief 当前显示的颜色：data_source, base, core, ball1, ball2, ball3 */
volatile uint8_t ws2812_debug_data_source = 0U;
volatile uint8_t ws2812_debug_base_color = 0U;
volatile uint8_t ws2812_debug_core_color = 0U;
volatile uint8_t ws2812_debug_ball_color1 = 0U;
volatile uint8_t ws2812_debug_ball_color2 = 0U;
volatile uint8_t ws2812_debug_ball_color3 = 0U;

/*==================== 静态函数原型 ====================*/

/**
 * @brief 将视觉颜色枚举转换为WS2812颜色值
 */
static WS2812_Color_t App_Ws2812Task_ColorToWs2812(VisionColor_t color);

/**
 * @brief 按分区显示所有颜色
 * @note 核心区显示 core_color，基础区显示 base_color，
 *       球框区显示 ball1/ball2/ball3
 */
static void App_Ws2812Task_ShowZones(WS2812_Handle_t *ws2812,
                                     WS2812_Color_t base_color,
                                     WS2812_Color_t core_color,
                                     WS2812_Color_t ball1,
                                     WS2812_Color_t ball2,
                                     WS2812_Color_t ball3);

/**
 * @brief 闪烁状态更新任务（周期性调用）
 */
static void App_Ws2812Task_BlinkTask(WS2812_Handle_t *ws2812);

/*==================== 静态函数实现 ====================*/

/**
 * @brief 颜色转换函数
 */
static WS2812_Color_t App_Ws2812Task_ColorToWs2812(VisionColor_t color) {
  switch (color) {
  case VISION_COLOR_RED:
    return WS2812_GetColor(WS2812_COLOR_RED);
  case VISION_COLOR_ORANGE:
    return WS2812_GetColor(WS2812_COLOR_ORANGE);
  case VISION_COLOR_YELLOW:
    return WS2812_GetColor(WS2812_COLOR_YELLOW);
  case VISION_COLOR_GREEN:
    return WS2812_GetColor(WS2812_COLOR_GREEN);
  case VISION_COLOR_CYAN:
    return WS2812_GetColor(WS2812_COLOR_CYAN);
  case VISION_COLOR_BLUE:
    return WS2812_GetColor(WS2812_COLOR_BLUE);
  case VISION_COLOR_PURPLE:
    return WS2812_GetColor(WS2812_COLOR_PURPLE);
  case VISION_COLOR_NONE:
  default:
    return WS2812_GetColor(WS2812_COLOR_OFF);
  }
}

/**
 * @brief 判断索引是否在区间内（闭区间）
 */
static uint8_t App_Ws2812Task_InRange(uint16_t i, uint16_t lo, uint16_t hi) {
  return (i >= lo && i <= hi) ? 1U : 0U;
}

/**
 * @brief 按分区显示所有颜色
 *
 * 分区布局（1-indexed → 0-indexed）：
 *   核心: 1-21→[0,20], 50-53→[49,52]
 *   基础: 22-29→[21,28], 42-49→[41,48]
 *   球框1: 38-41→[37,40]
 *   球框2: 34-38→[33,37]
 *   球框3: 30-33→[29,32]
 */
static void App_Ws2812Task_ShowZones(WS2812_Handle_t *ws2812,
                                     WS2812_Color_t base_color,
                                     WS2812_Color_t core_color,
                                     WS2812_Color_t ball1,
                                     WS2812_Color_t ball2,
                                     WS2812_Color_t ball3) {
  /* 参数检查及设备忙检测 */
  if ((ws2812 == 0) || (WS2812_IsBusy(ws2812) != 0U)) {
    return;
  }

  for (uint16_t i = 0U; i < BSP_WS2812_LED_COUNT; i++) {
    WS2812_Color_t color;

    if (App_Ws2812Task_InRange(i, ZONE_CORE_LO, ZONE_CORE_HI) ||
        App_Ws2812Task_InRange(i, ZONE_CORE2_LO, ZONE_CORE2_HI)) {
      color = core_color;
    } else if (App_Ws2812Task_InRange(i, ZONE_BASE_LO, ZONE_BASE_HI) ||
               App_Ws2812Task_InRange(i, ZONE_BASE2_LO, ZONE_BASE2_HI)) {
      color = base_color;
    } else if (App_Ws2812Task_InRange(i, ZONE_BALL1_LO, ZONE_BALL1_HI)) {
      color = ball1;
    } else if (App_Ws2812Task_InRange(i, ZONE_BALL2_LO, ZONE_BALL2_HI)) {
      color = ball2;
    } else if (App_Ws2812Task_InRange(i, ZONE_BALL3_LO, ZONE_BALL3_HI)) {
      color = ball3;
    } else {
      color = WS2812_GetColor(WS2812_COLOR_OFF);
    }

    (void)WS2812_SetPixelColor(ws2812, i, color);
  }

  /* 刷新灯带显示 */
  (void)WS2812_Show(ws2812);
}

/**
 * @brief 闪烁/常亮状态更新（周期性调用）
 * @note 有数据时常亮，超时无数据则闪烁
 */
static void App_Ws2812Task_BlinkTask(WS2812_Handle_t *ws2812) {
  uint32_t now = Bsp_Time_GetMs();

  /* 句柄有效性检查 */
  if ((ws2812 == 0) || (WS2812_IsBusy(ws2812) != 0U)) {
    return;
  }

  /* 常亮模式：检查是否超时（仅收到过数据后才超时）
     使用 BSP 层中断级时间戳，任何 USB 数据到达都会刷新 */
  if (app_ws2812_blink_enabled == 0U) {
    uint32_t last_rx = bsp_usb_cdc_last_rx_tick;
    if ((app_ws2812_data_ever == 0U) ||
        ((uint32_t)(now - last_rx) < APP_WS2812_DATA_TIMEOUT_MS)) {
      return; /* 从未收过数据 或 未超时，保持当前状态 */
    }
    /* 收到过数据且超时 → 进入闪烁模式 */
    app_ws2812_blink_enabled = 1U;
    app_ws2812_last_blink_tick = 0U;
    app_ws2812_blink_on = 0U;
  }

  /* 闪烁模式：检查切换周期 */
  if ((app_ws2812_last_blink_tick != 0U) &&
      ((uint32_t)(now - app_ws2812_last_blink_tick) <
       APP_WS2812_BLINK_PERIOD_MS)) {
    return;
  }

  app_ws2812_last_blink_tick = now;
  app_ws2812_blink_on = (app_ws2812_blink_on == 0U) ? 1U : 0U;

  if (app_ws2812_blink_on != 0U) {
    /* 亮：显示目标颜色 */
    App_Ws2812Task_ShowZones(ws2812, app_ws2812_base_color,
                             app_ws2812_core_color,
                             app_ws2812_ball_color1,
                             app_ws2812_ball_color2,
                             app_ws2812_ball_color3);
  } else {
    /* 灭：全部熄灭 */
    WS2812_Color_t off_color = WS2812_GetColor(WS2812_COLOR_OFF);
    App_Ws2812Task_ShowZones(ws2812, off_color, off_color,
                             off_color, off_color, off_color);
  }
}

/*==================== RTOS任务函数 ====================*/

/**
 * @brief WS2812控制任务入口函数（FreeRTOS任务）
 *
 * @param argument 任务参数（未使用）
 */
void StartWs2812Task(void *argument) {
  WS2812_Handle_t *ws2812;
  uint8_t usb_rx_buffer[APP_WS2812_RX_BUFFER_SIZE];
  uint16_t usb_rx_len;
  VisionFrame_t vision_frame;

  (void)argument;

  /* 获取WS2812设备句柄并初始化灯带 */
  ws2812 = BSP_WS2812_GetHandle();
  if (BSP_WS2812_Init() == WS2812_STATUS_OK) {
    WS2812_StartRainbow(ws2812, APP_WS2812_RAINBOW_STEP_MS);
  }

  /* 任务主循环 */
  for (;;) {
    /* 从USB CDC接收数据（非阻塞） */
    if (Bsp_UsbCdc_TakeRx(usb_rx_buffer, sizeof(usb_rx_buffer), &usb_rx_len) !=
        0U) {
      /* 尝试解析视觉帧 */
      VisionParseError_t err = VisionProtocol_ParseFrame(
          usb_rx_buffer, usb_rx_len, &vision_frame);

      if (err == VISION_PARSE_OK) {
        ws2812_debug_rx_count++;
        ws2812_debug_data_source = (uint8_t)vision_frame.data_source;

        WS2812_StopEffect(ws2812);

        if (vision_frame.data_source == VISION_SOURCE_OWN_QR) {
          /* 自家二维码：存储核心色+基础色，球框区灭 */
          app_ws2812_base_color = App_Ws2812Task_ColorToWs2812(vision_frame.base_color);
          app_ws2812_core_color = App_Ws2812Task_ColorToWs2812(vision_frame.core_color);
          /* Ozone debug */
          ws2812_debug_base_color  = (uint8_t)vision_frame.base_color;
          ws2812_debug_core_color  = (uint8_t)vision_frame.core_color;
        } else {
          /* 球框二维码：存储球框三色，核心/基础区灭 */
          app_ws2812_ball_color1 = App_Ws2812Task_ColorToWs2812(vision_frame.ball_color1);
          app_ws2812_ball_color2 = App_Ws2812Task_ColorToWs2812(vision_frame.ball_color2);
          app_ws2812_ball_color3 = App_Ws2812Task_ColorToWs2812(vision_frame.ball_color3);
          /* Ozone debug */
          ws2812_debug_ball_color1 = (uint8_t)vision_frame.ball_color1;
          ws2812_debug_ball_color2 = (uint8_t)vision_frame.ball_color2;
          ws2812_debug_ball_color3 = (uint8_t)vision_frame.ball_color3;
        }

        /* 有数据 → 常亮显示 */
        app_ws2812_data_ever = 1U;
        app_ws2812_blink_enabled = 0U;
        App_Ws2812Task_ShowZones(ws2812, app_ws2812_base_color,
                                 app_ws2812_core_color,
                                 app_ws2812_ball_color1,
                                 app_ws2812_ball_color2,
                                 app_ws2812_ball_color3);
      } else {
        /* 解析失败 -> 将错误码通过 USB 发送给上位机 */
      }
    }

    /* 处理闪烁状态切换 */
    App_Ws2812Task_BlinkTask(ws2812);

    /* 处理WS2812底层状态机（如DMA刷新） */
    WS2812_Task(ws2812);

    /* 任务延时1ms，释放CPU */
    osDelay(APP_WS2812_TASK_DELAY_MS);
  }
}
