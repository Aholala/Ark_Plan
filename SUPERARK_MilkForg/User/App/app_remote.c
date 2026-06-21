/**
 * @file app_remote.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 遥控器应用层模块（接收端处理）
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本模块负责接收来自遥控器发送端的数据包（通过NRF24L01），
 * 解析摇杆值、按键信息，并维护连接状态和调试信息。
 * 该模块通常运行在机器人底盘等接收设备上。
 */

#include "app_remote.h"

#include "bsp_remote_radio.h" /* 无线通信底层驱动 */
#include "bsp_time.h"         /* 时间基准（毫秒） */

#include <string.h>

/*------------------------ 应用参数宏定义 ------------------------*/
#define APP_REMOTE_TIMEOUT_MS                                                  \
  500U /* 连接超时时间（毫秒），超过此时间未收到数据则视为断开 */
#define APP_REMOTE_RX_DRAIN_LIMIT                                              \
  3U /* 单次任务中最多从接收FIFO取出的包数（防止积压） */

/* 应用数据对象（静态） */
static AppRemoteData_t app_remote_data; /* 存储最新接收到的遥控数据及状态 */
static uint8_t app_remote_last_key_seq; /* 上一次处理过的按键序列号，用于去重 */

/**
 * @brief 遥控器应用初始化
 * @retval 无
 * @note  清空数据对象，并初始化无线通信模块（设置为接收模式）
 */
void App_Remote_Init(void) {
  memset(&app_remote_data, 0, sizeof(app_remote_data));
  app_remote_last_key_seq = 0U;

  Bsp_RemoteRadio_Init(); /* 初始化NRF24L01并设置为接收模式 */
}

/**
 * @brief 遥控器应用主任务（需在主循环中周期调用）
 * @retval 无
 * @note   - 尝试从接收FIFO中取出最多 APP_REMOTE_RX_DRAIN_LIMIT 个数据包
 *         - 若收到有效包，则更新摇杆值、模式、按键和序列号，并记录接收时间
 *         - 更新来自底层的调试信息（寄存器状态、IRQ引脚等）
 *         - 检查超时：若距离上次接收超过
 * APP_REMOTE_TIMEOUT_MS，则标记为断开连接并清零摇杆值
 */
void App_Remote_Task(void) {
  BspRemoteRadioPacket_t packet;            /* 存放接收到的数据包 */
  const BspRemoteRadioDebug_t *radio_debug; /* 指向调试信息结构体的指针 */
  uint32_t now = Bsp_Time_GetMs();          /* 获取当前系统时间（毫秒） */

  /* 连续取出多个包，防止FIFO积压，确保使用最新数据 */
  for (uint8_t i = 0U; i < APP_REMOTE_RX_DRAIN_LIMIT; i++) {
    /* 尝试接收一包数据，成功返回1，无数据返回0 */
    if (Bsp_RemoteRadio_Receive(&packet) == 0U) {
      break; /* 无更多数据，退出循环 */
    }

    /* 更新摇杆值和模式（直接覆盖） */
    app_remote_data.mode = packet.mode;
    app_remote_data.lh = packet.lh;
    app_remote_data.lv = packet.lv;
    app_remote_data.rh = packet.rh;
    app_remote_data.rv = packet.rv;

    /* 按键处理：只有当按键值非零且序列号发生变化时才更新，避免重复触发 */
    if (packet.key != 0U) {
      if (packet.key_seq != app_remote_last_key_seq) {
        app_remote_last_key_seq = packet.key_seq;
        app_remote_data.key = packet.key;
        app_remote_data.key_seq = packet.key_seq;
        app_remote_data.key_event_count++; /* 按键事件计数（可用于统计） */
      }
    }

    app_remote_data.last_rx_tick = now; /* 更新最后接收时间 */
    app_remote_data.connected = 1U;     /* 标记连接有效 */
  }

  /* 获取底层NRF24L01的调试信息（状态寄存器、引脚电平、FIFO状态等） */
  radio_debug = Bsp_RemoteRadio_GetDebug();
  app_remote_data.nrf_status = radio_debug->status;
  app_remote_data.nrf_config = radio_debug->config;
  app_remote_data.nrf_irq_pin = radio_debug->irq_pin;
  app_remote_data.nrf_csn_pin = radio_debug->csn_pin;
  app_remote_data.nrf_ce_pin = radio_debug->ce_pin;
  app_remote_data.nrf_fifo_status = radio_debug->fifo_status;
  app_remote_data.nrf_last_result = radio_debug->last_result;
  app_remote_data.nrf_rx_count = radio_debug->rx_count; /* 接收数据包总数 */

  /* 超时检测：若距离上次接收超过设定时间，则视为连接断开 */
  if ((uint32_t)(now - app_remote_data.last_rx_tick) > APP_REMOTE_TIMEOUT_MS) {
    app_remote_data.connected = 0U; /* 标记断开 */
    /* 将摇杆值安全清零，避免误动作 */
    app_remote_data.lh = 0;
    app_remote_data.lv = 0;
    app_remote_data.rh = 0;
    app_remote_data.rv = 0;
    /* 注意：按键值不清零，保留最后一次有效按键，但连接标志已清除，上层可据此判断
     */
  }
}

/**
 * @brief 获取最新的遥控器数据指针（只读）
 * @return const AppRemoteData_t* 指向包含最新数据、状态和调试信息的结构体
 * @note  该函数返回的指针指向静态内部数据，上层应用可安全读取
 */
const AppRemoteData_t *App_Remote_GetData(void) { return &app_remote_data; }