/**
 * @file bsp_radio_link.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）无线链路驱动源文件（含调试扩展）
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本文件在基础无线链路功能之上扩展了调试信息采集和中断处理能力。
 * 提供接收数据包（解码为协议结构体）、中断标志管理、寄存器状态及引脚电平采集、
 * 接收计数统计等功能，便于系统诊断和状态监控。
 * 数据包格式由 RemoteProtocol 模块定义，本层负责协议解码。
 */

#include "bsp_radio_link.h"

#include "bsp_nrf24l01.h"
#include "main.h"

/* 中断待处理标志（由外部中断置位） */
static volatile uint8_t radio_link_irq_pending;

/* 调试信息结构体（包含寄存器、引脚、接收计数等） */
static BspRadioLinkDebug_t radio_link_debug;

/**
 * @brief 更新调试信息（读取NRF24L01寄存器及引脚电平）
 * @retval 无
 * @note  采集 STATUS、CONFIG、FIFO_STATUS 寄存器值以及 IRQ、CSN、CE 引脚电平
 */
static void Bsp_RadioLink_UpdateDebug(void) {
  radio_link_debug.irq_pin =
      (HAL_GPIO_ReadPin(SPI3_IRQ_GPIO_Port, SPI3_IRQ_Pin) == GPIO_PIN_RESET)
          ? 0U
          : 1U;
  radio_link_debug.csn_pin =
      (HAL_GPIO_ReadPin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin) == GPIO_PIN_RESET)
          ? 0U
          : 1U;
  radio_link_debug.ce_pin =
      (HAL_GPIO_ReadPin(SPI3_CE_GPIO_Port, SPI3_CE_Pin) == GPIO_PIN_RESET) ? 0U
                                                                           : 1U;
  radio_link_debug.status = NRF24L01_ReadStatus();
  radio_link_debug.config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  radio_link_debug.fifo_status = NRF24L01_ReadFifoStatus();
}

/**
 * @brief 初始化无线链路模块（含调试信息采集）
 * @retval 无
 * @note   - 调用 NRF24L01_Init() 完成硬件初始化
 *         - 清空中断待处理标志，采集初始调试信息
 *         - 清零 last_result 和 rx_count 统计
 */
void Bsp_RadioLink_Init(void) {
  radio_link_irq_pending = 1U; /* 初始认为有中断待处理，以便首次读取 */
  NRF24L01_Init();
  Bsp_RadioLink_UpdateDebug();
  radio_link_debug.last_result = 0U;
  radio_link_debug.rx_count = 0U;
}

/**
 * @brief 接收一包数据并进行协议解码
 * @param packet 指向控制数据包结构体的指针（若为NULL，则只执行底层接收）
 * @return uint8_t 是否成功接收
 *         - 1：成功接收并解码（packet已填充，若不为NULL）
 *         - 0：无数据或接收失败
 * @note   - 调用前更新调试信息，调用后再次更新
 *         - 若底层 NRF24L01_Receive 成功，使用 RemoteProtocol_DecodeControl
 *           将原始载荷解析为控制数据包结构体
 *         - 成功接收后累加 rx_count 计数
 *         - 该函数设计用于接收端（底盘）获取遥控器的控制指令
 */
uint8_t Bsp_RadioLink_Receive(BspRadioLinkPacket_t *packet) {
  uint8_t result;

  /* 更新调试信息（调用前） */
  Bsp_RadioLink_UpdateDebug();
  radio_link_irq_pending = 0U; /* 清除中断待处理标志 */

  /* 调用底层接收函数 */
  result = NRF24L01_Receive();
  radio_link_debug.last_result = result; /* 保存本次操作结果 */

  /* 再次更新调试信息（底层可能修改了寄存器） */
  Bsp_RadioLink_UpdateDebug();

  /* 若接收失败，直接返回0 */
  if (result != 1U) {
    return 0U;
  }

  /* 成功接收：若调用者提供了存储指针，则解码数据包 */
  if (packet != 0) {
    RemoteProtocol_DecodeControl(NRF24L01_RxPacket, packet);
  }

  radio_link_debug.rx_count++; /* 累加成功接收计数 */
  return 1U;
}

/**
 * @brief 外部中断触发回调（由IRQ引脚下降沿触发）
 * @retval 无
 * @note  将中断待处理标志置1，提示主循环有数据到达。
 *        该函数在中断上下文中调用，应保持轻量。
 */
void Bsp_RadioLink_OnIrq(void) { radio_link_irq_pending = 1U; }

/**
 * @brief 获取调试信息结构体指针（只读）
 * @return const BspRadioLinkDebug_t* 指向调试数据对象的指针
 * @note  可用于外部显示寄存器状态、引脚电平、接收计数等诊断信息
 */
const BspRadioLinkDebug_t *Bsp_RadioLink_GetDebug(void) {
  return &radio_link_debug;
}

/**
 * @brief HAL库外部中断回调函数（覆盖弱定义）
 * @param GPIO_Pin 触发中断的引脚
 * @retval 无
 * @note  当检测到IRQ引脚中断时，调用 Bsp_RadioLink_OnIrq()
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == SPI3_IRQ_Pin) {
    Bsp_RadioLink_OnIrq();
  }
}