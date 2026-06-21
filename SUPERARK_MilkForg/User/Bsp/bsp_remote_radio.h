/**
 * @file bsp_remote_radio.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）遥控器无线接收模块驱动头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件定义了遥控无线接收模块的数据结构和外部接口。
 * 该模块基于NRF24L01，专门用于接收遥控器发送的控制数据包，
 * 并提供中断处理、调试信息采集等功能。
 */

#ifndef __BSP_REMOTE_RADIO_H
#define __BSP_REMOTE_RADIO_H

#include <stdint.h> /* 标准整数类型定义 */

/**
 * @brief 遥控数据包结构体（从接收FIFO中解析得到）
 * @note  对应发送端的数据格式：mode, lh, lv, rh, rv, key, key_seq
 */
typedef struct {
  uint8_t mode;    /**< 工作模式（来自发送端，0或1） */
  int8_t lh;       /**< 左摇杆水平分量（-100 ~ +100） */
  int8_t lv;       /**< 左摇杆垂直分量（-100 ~ +100） */
  int8_t rh;       /**< 右摇杆水平分量（-100 ~ +100） */
  int8_t rv;       /**< 右摇杆垂直分量（-100 ~ +100） */
  uint8_t key;     /**< 按键编号（1~12，0表示无按键） */
  uint8_t key_seq; /**< 按键序列号（用于去重和防抖，每次按键递增） */
} BspRemoteRadioPacket_t;

/**
 * @brief 调试信息结构体（用于诊断通信状态）
 * @note  保存NRF24L01的寄存器值、引脚电平和统计数据
 */
typedef struct {
  uint8_t status;      /**< NRF24L01状态寄存器（STATUS） */
  uint8_t config;      /**< NRF24L01配置寄存器（CONFIG） */
  uint8_t irq_pin;     /**< IRQ引脚电平（1=高，0=低） */
  uint8_t csn_pin;     /**< CSN引脚电平（1=高，0=低） */
  uint8_t ce_pin;      /**< CE引脚电平（1=高，0=低） */
  uint8_t fifo_status; /**< FIFO状态寄存器（FIFO_STATUS） */
  uint8_t last_result; /**< 最近一次接收操作的结果（0失败，1成功，2异常） */
  uint32_t rx_count;   /**< 成功接收数据包的总数（累计） */
} BspRemoteRadioDebug_t;

/**
 * @brief 初始化遥控无线接收模块
 * @retval 无
 * @note   - 调用 NRF24L01_Init() 配置模块为接收模式
 *         - 采集初始调试信息（寄存器、引脚状态）
 *         - 清空统计数据
 */
void Bsp_RemoteRadio_Init(void);

/**
 * @brief 从接收FIFO中取出一包数据（非阻塞）
 * @param packet
 * 指向数据包结构体的指针，用于存放接收到的数据（若为NULL则只执行底层读取，不返回数据）
 * @return uint8_t 是否成功接收数据
 *         - 1：成功接收到一包有效数据，packet已填充
 *         - 0：无数据或接收失败
 * @note   - 函数会更新调试信息（寄存器、引脚电平）和 last_result
 *         - 成功接收后累加 rx_count 计数
 *         - 若 packet 为 NULL，则仅清空FIFO但不返回数据，可用于丢弃旧包
 */
uint8_t Bsp_RemoteRadio_Receive(BspRemoteRadioPacket_t *packet);

/**
 * @brief 外部中断触发回调（由IRQ引脚下降沿触发）
 * @retval 无
 * @note   - 该函数在中断上下文中调用，仅置位内部待处理标志
 *         - 主循环通过轮询该标志或调用 Receive 来读取数据
 */
void Bsp_RemoteRadio_OnIrq(void);

/**
 * @brief 获取调试信息结构体指针（只读）
 * @return const BspRemoteRadioDebug_t* 指向调试数据对象的指针
 * @note   可被外部调用以显示诊断信息（如寄存器值、引脚电平、接收计数等）
 */
const BspRemoteRadioDebug_t *Bsp_RemoteRadio_GetDebug(void);

#endif /* __BSP_REMOTE_RADIO_H */