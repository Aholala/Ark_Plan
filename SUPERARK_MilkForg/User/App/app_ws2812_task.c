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
 *          1. 初始化WS2812灯带并运行彩虹动态效果作为默认显示；
 *          2. 通过USB CDC接收视觉协议帧，解析颜色信息；
 *          3.
 * 根据解析的颜色更新灯带显示（前半部分显示底色，后半部分显示芯色）；
 *          4. 实现呼吸/闪烁效果，以突出显示目标颜色；
 *          5. 通过USB返回解析成功或失败的可读响应。
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

/*==================== 外部函数声明 ====================*/

/** @brief USB设备初始化函数（由MX生成） */
extern void MX_USB_DEVICE_Init(void);

/*==================== 静态变量 ====================*/

/** @brief 当前显示的底色（WS2812颜色格式） */
static WS2812_Color_t app_ws2812_base_color;

/** @brief 当前显示的芯色（WS2812颜色格式） */
static WS2812_Color_t app_ws2812_core_color;

/** @brief 上次闪烁切换的时刻（毫秒） */
static uint32_t app_ws2812_last_blink_tick;

/** @brief 闪烁功能使能标志（1=使能，0=禁止） */
static uint8_t app_ws2812_blink_enabled;

/** @brief 当前闪烁状态（1=亮，0=灭） */
static uint8_t app_ws2812_blink_on;

/** @brief USB响应静态缓冲区，保证异步发送时数据有效。 */

/** @brief 待发送USB响应长度。 */

/** @brief USB响应待发送标志。 */

/*==================== 静态函数原型 ====================*/

/**
 * @brief 将视觉颜色枚举转换为WS2812颜色值
 *
 * @param color 视觉颜色枚举值
 * @return WS2812_Color_t 对应的WS2812颜色数据
 * @note 若枚举无效或为 VISION_COLOR_NONE，则返回关闭颜色（全灭）
 */
static WS2812_Color_t App_Ws2812Task_ColorToWs2812(VisionColor_t color);

/**
 * @brief 在灯带上按半区显示两种颜色
 *
 * @param ws2812     WS2812设备句柄
 * @param base_color 底色（前半部分显示）
 * @param core_color 芯色（后半部分显示）
 * @note 函数会检查句柄有效性和设备空闲状态，忙时直接返回
 */
static void App_Ws2812Task_ShowHalves(WS2812_Handle_t *ws2812,
                                      WS2812_Color_t base_color,
                                      WS2812_Color_t core_color);

/**
 * @brief 启动闪烁模式
 *
 * @param ws2812     WS2812设备句柄
 * @param base_color 闪烁时显示的底色
 * @param core_color 闪烁时显示的芯色
 * @note 停止当前正在运行的特效，并重置闪烁控制变量
 */
static void App_Ws2812Task_StartBlink(WS2812_Handle_t *ws2812,
                                      WS2812_Color_t base_color,
                                      WS2812_Color_t core_color);

/**
 * @brief 闪烁状态更新任务（周期性调用）
 *
 * @param ws2812 WS2812设备句柄
 * @note 根据当前时间和闪烁周期切换亮灭状态，并更新灯带显示
 */
static void App_Ws2812Task_BlinkTask(WS2812_Handle_t *ws2812);

/*==================== 静态函数实现 ====================*/

/**
 * @brief 颜色转换函数
 *
 * @param color 视觉颜色枚举
 * @return WS2812_Color_t 转换后的颜色值
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
 * @brief 按半区显示两种颜色（前一半显示底色，后一半显示芯色）
 *
 * @param ws2812     WS2812设备句柄
 * @param base_color 底色
 * @param core_color 芯色
 */
static void App_Ws2812Task_ShowHalves(WS2812_Handle_t *ws2812,
                                      WS2812_Color_t base_color,
                                      WS2812_Color_t core_color) {
  uint16_t half = BSP_WS2812_LED_COUNT / 2U;

  /* 参数检查及设备忙检测 */
  if ((ws2812 == 0) || (WS2812_IsBusy(ws2812) != 0U)) {
    return;
  }

  /* 逐灯珠设置颜色 */
  for (uint16_t i = 0U; i < BSP_WS2812_LED_COUNT; i++) {
    (void)WS2812_SetPixelColor(ws2812, i, (i < half) ? base_color : core_color);
  }

  /* 刷新灯带显示 */
  (void)WS2812_Show(ws2812);
}

/**
 * @brief 启动闪烁模式
 *
 * @param ws2812     WS2812设备句柄
 * @param base_color 底色
 * @param core_color 芯色
 */
static void App_Ws2812Task_StartBlink(WS2812_Handle_t *ws2812,
                                      WS2812_Color_t base_color,
                                      WS2812_Color_t core_color) {
  /* 保存目标颜色 */
  app_ws2812_base_color = base_color;
  app_ws2812_core_color = core_color;

  /* 重置闪烁控制变量 */
  app_ws2812_last_blink_tick = 0U;
  app_ws2812_blink_enabled = 1U;
  app_ws2812_blink_on = 0U;

  /* 停止当前运行的任何动态特效（如彩虹） */
  WS2812_StopEffect(ws2812);
}

/**
 * @brief 闪烁任务处理函数（需周期性调用）
 *
 * @param ws2812 WS2812设备句柄
 */
static void App_Ws2812Task_BlinkTask(WS2812_Handle_t *ws2812) {
  uint32_t now;
  WS2812_Color_t off_color;

  /* 检查闪烁使能、句柄有效性和设备忙状态 */
  if ((app_ws2812_blink_enabled == 0U) || (ws2812 == 0) ||
      (WS2812_IsBusy(ws2812) != 0U)) {
    return;
  }

  now = Bsp_Time_GetMs();

  /* 判断是否达到切换周期 */
  if ((app_ws2812_last_blink_tick != 0U) &&
      ((uint32_t)(now - app_ws2812_last_blink_tick) <
       APP_WS2812_BLINK_PERIOD_MS)) {
    return;
  }

  app_ws2812_last_blink_tick = now;

  /* 翻转亮灭状态 */
  app_ws2812_blink_on = (app_ws2812_blink_on == 0U) ? 1U : 0U;

  if (app_ws2812_blink_on != 0U) {
    /* 亮：显示目标颜色 */
    App_Ws2812Task_ShowHalves(ws2812, app_ws2812_base_color,
                              app_ws2812_core_color);
  } else {
    /* 灭：全部熄灭 */
    off_color = WS2812_GetColor(WS2812_COLOR_OFF);
    App_Ws2812Task_ShowHalves(ws2812, off_color, off_color);
  }
}

/*==================== RTOS任务函数 ====================*/

/**
 * @brief WS2812控制任务入口函数（FreeRTOS任务）
 *
 * @param argument 任务参数（未使用）
 *
 * @note 任务执行流程：
 *       1. 初始化USB设备；
 *       2. 初始化WS2812并启动彩虹特效；
 *       3. 主循环：
 *          - 尝试从USB CDC读取一帧数据；
 *          - 若解析到有效颜色帧，则启动闪烁显示；
 *          - 通过USB返回 OK 或 ERR 文本响应；
 *          - 处理闪烁状态机；
 *          - 调用WS2812底层任务（如更新DMA传输）；
 *          - 延时1ms。
 */
void StartWs2812Task(void *argument) {
  WS2812_Handle_t *ws2812;
  uint8_t usb_rx_buffer[APP_WS2812_RX_BUFFER_SIZE];
  uint16_t usb_rx_len;
  VisionColorFrame_t vision_frame;

  (void)argument; /* 防止未使用参数警告 */

  /* 初始化USB CDC设备 */
  MX_USB_DEVICE_Init();

  /* 获取WS2812设备句柄并初始化灯带 */
  ws2812 = BSP_WS2812_GetHandle();
  if (BSP_WS2812_Init() == WS2812_STATUS_OK) {
    /* 启动彩虹动态效果（默认背景） */
    WS2812_StartRainbow(ws2812, APP_WS2812_RAINBOW_STEP_MS);
  }

  /* 任务主循环 */
  for (;;) {
    /* 从USB CDC接收数据（非阻塞） */
    if (Bsp_UsbCdc_TakeRx(usb_rx_buffer, sizeof(usb_rx_buffer), &usb_rx_len) !=
        0U) {
      /* 尝试解析视觉颜色帧 */
      VisionParseError_t err = VisionProtocol_ParseColorFrame(
          usb_rx_buffer, usb_rx_len, &vision_frame);
      if (err == VISION_PARSE_OK) {
        /* 解析成功 -> 启动闪烁模式显示目标颜色 */
        App_Ws2812Task_StartBlink(
            ws2812, App_Ws2812Task_ColorToWs2812(vision_frame.base_color),
            App_Ws2812Task_ColorToWs2812(vision_frame.core_color));
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
