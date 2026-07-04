/**
 * @file app_remote_task.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 遥控器应用层模块头文件（接收端）
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件定义了遥控器接收端的数据结构及外部接口。
 * 该模块负责从NRF24L01接收遥控数据包，解析摇杆值、按键信息，
 * 并维护连接状态、调试信息等，供上层控制算法使用。
 */

#ifndef __APP_REMOTE_TASK_H
#define __APP_REMOTE_TASK_H

#include "main.h" /* 包含HAL库及基础类型定义 */

/**
 * @brief 遥控器接收数据及状态结构体
 * @note  该结构体保存最新接收到的遥控数据、连接状态及NRF24L01调试信息
 */
typedef struct {
  /* 遥控数据 */
  uint8_t mode;    /**< 工作模式（来自发送端，0或1） */
  int8_t lh;       /**< 左摇杆水平分量（-100 ~ +100） */
  int8_t lv;       /**< 左摇杆垂直分量（-100 ~ +100） */
  int8_t rh;       /**< 右摇杆水平分量（-100 ~ +100） */
  int8_t rv;       /**< 右摇杆垂直分量（-100 ~ +100） */
  uint8_t key;     /**< 当前按下的按键编号（1~12，0表示无按键） */
  uint8_t key_seq; /**< 按键序列号（用于去重和防抖） */

  /* 统计与状态 */
  uint32_t key_event_count; /**< 按键事件累计计数（可用于检测按键变化） */
  uint8_t connected;        /**< 连接标志（1=已连接，0=超时断开） */

  /* NRF24L01调试信息（从底层驱动获取） */
  uint8_t nrf_status;      /**< NRF24L01状态寄存器（STATUS） */
  uint8_t nrf_config;      /**< NRF24L01配置寄存器（CONFIG） */
  uint8_t nrf_irq_pin;     /**< IRQ引脚电平（1=高，0=低） */
  uint8_t nrf_csn_pin;     /**< CSN引脚电平 */
  uint8_t nrf_ce_pin;      /**< CE引脚电平 */
  uint8_t nrf_fifo_status; /**< FIFO状态寄存器（FIFO_STATUS） */
  uint8_t nrf_last_result; /**< 最近一次通信操作结果（0成功，非0失败） */
  uint32_t nrf_rx_count;   /**< 已接收数据包总数（累计） */

  uint32_t last_rx_tick; /**< 最近一次成功接收数据包的系统时间（毫秒） */
} AppRemoteData_t;

/**
 * @brief 遥控器应用初始化
 * @retval 无
 * @note  清空应用数据，并初始化无线模块（设置为接收模式）
 */
void App_Remote_Init(void);

/**
 * @brief 遥控器应用主任务（需在主循环中周期调用）
 * @retval 无
 * @note   - 从NRF24L01 FIFO中取出一至多个数据包
 *         - 更新摇杆值、按键、连接状态和调试信息
 *         - 若超时未收到数据，则标记断开并清零摇杆值
 */
void App_Remote_Task(void);

/**
 * @brief 获取最新遥控器数据的只读指针
 * @return const AppRemoteData_t* 指向内部数据对象的指针
 * @note  返回的指针指向静态数据，上层应用可安全读取但不应修改
 */
const AppRemoteData_t *App_Remote_GetData(void);

void StartRemoteTask(void *argument);

#endif /* __APP_REMOTE_TASK_H */
