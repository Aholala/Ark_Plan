/**
 * @file bsp_radio_link.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）无线链路驱动头文件（接收端专用）
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件定义了无线链路BSP层的接收端接口，基于NRF24L01硬件实现。
 * 与发送端（bsp_remote_radio）不同，本模块专用于接收遥控器发来的
 * 控制数据包，并提供调试信息采集功能。
 *
 * 数据包格式由 module_remote_protocol 模块定义，本层负责协议解码。
 * 支持中断触发和轮询两种方式接收，并提供寄存器状态、引脚电平、
 * 接收计数等诊断信息。
 */

#ifndef __BSP_RADIO_LINK_H
#define __BSP_RADIO_LINK_H

#include "module_remote_protocol.h" /* 遥控协议编解码模块 */

/**
 * @brief BSP层接收数据包类型（重命名自 RemoteProtocolControlPacket_t）
 * @note  包含摇杆值、按键编号和按键序列号，由遥控器发送
 */
typedef RemoteProtocolControlPacket_t BspRadioLinkPacket_t;

/**
 * @brief 无线链路调试信息结构体
 * @note  保存NRF24L01的寄存器值、引脚电平和接收统计，便于诊断
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
} BspRadioLinkDebug_t;

/**
 * @brief 初始化无线链路模块（接收模式）
 * @retval 无
 * @note   - 调用 NRF24L01_Init() 完成硬件初始化，默认进入接收模式
 *         - 采集初始调试信息，清空统计计数
 */
void Bsp_RadioLink_Init(void);

/**
 * @brief 接收一包控制数据并进行协议解码
 * @param packet
 * 指向控制数据包结构体的指针（若为NULL，则只执行底层接收但不返回数据）
 * @return uint8_t 是否成功接收
 *         - 1：成功接收并解码，packet已填充（若不为NULL）
 *         - 0：无数据或接收失败
 * @note   - 非阻塞调用，立即返回
 *         - 调用前会更新调试信息，调用后再次更新，便于追踪状态变化
 *         - 若接收成功，通过 RemoteProtocol_DecodeControl
 * 将原始载荷解析为结构体
 *         - 成功接收后累加 rx_count 计数
 */
uint8_t Bsp_RadioLink_Receive(BspRadioLinkPacket_t *packet);

/**
 * @brief 外部中断触发回调（由IRQ引脚下降沿触发）
 * @retval 无
 * @note   - 该函数在中断上下文中调用，仅置位内部待处理标志
 *         - 主循环可通过轮询该标志或调用 Receive 来读取数据
 *         - 该函数由 HAL_GPIO_EXTI_Callback 调用
 */
void Bsp_RadioLink_OnIrq(void);

/**
 * @brief 获取调试信息结构体指针（只读）
 * @return const BspRadioLinkDebug_t* 指向调试数据对象的指针
 * @note   可被外部调用以显示诊断信息（如寄存器值、引脚电平、接收计数等）
 */
const BspRadioLinkDebug_t *Bsp_RadioLink_GetDebug(void);

#endif /* __BSP_RADIO_LINK_H */