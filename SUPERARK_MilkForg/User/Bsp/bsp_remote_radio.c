/**
 * @file bsp_remote_radio.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）遥控器无线接收模块驱动源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本文件封装了NRF24L01无线模块，专用于接收遥控器发送的数据包。
 * 提供数据接收、中断处理、调试信息采集等功能，并向上层应用提供
 * 统一的数据结构（BspRemoteRadioPacket_t）。
 */

#include "bsp_remote_radio.h"

#include "bsp_nrf24l01.h" /* NRF24L01底层驱动 */
#include "main.h"         /* HAL库及硬件引脚定义 */

/* 中断待处理标志（由外部中断置位，在主循环中轮询） */
static volatile uint8_t remote_radio_irq_pending;

/* 调试信息结构体（存储当前寄存器状态、引脚电平、统计等） */
static BspRemoteRadioDebug_t remote_radio_debug;

/**
 * @brief 初始化遥控无线接收模块
 * @retval 无
 * @note   - 调用 NRF24L01_Init() 配置模块为接收模式
 *         - 读取初始状态（STATUS、CONFIG、FIFO_STATUS）以及各引脚电平
 *         - 清空中断标志和统计数据
 */
void Bsp_RemoteRadio_Init(void) {
  remote_radio_irq_pending =
      1U; /* 初始认为有中断待处理，以便第一次进入时立即读取 */
  NRF24L01_Init();

  /* 采集初始调试信息 */
  remote_radio_debug.status = NRF24L01_ReadStatus();
  remote_radio_debug.config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  remote_radio_debug.fifo_status = NRF24L01_ReadFifoStatus();
  remote_radio_debug.irq_pin =
      (HAL_GPIO_ReadPin(SPI3_IRQ_GPIO_Port, SPI3_IRQ_Pin) == GPIO_PIN_RESET)
          ? 0U
          : 1U;
  remote_radio_debug.csn_pin =
      (HAL_GPIO_ReadPin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin) == GPIO_PIN_RESET)
          ? 0U
          : 1U;
  remote_radio_debug.ce_pin =
      (HAL_GPIO_ReadPin(SPI3_CE_GPIO_Port, SPI3_CE_Pin) == GPIO_PIN_RESET) ? 0U
                                                                           : 1U;
  remote_radio_debug.last_result = 0U;
  remote_radio_debug.rx_count = 0U;
}

/**
 * @brief 从接收FIFO中取出一包数据（非阻塞）
 * @param packet
 * 指向数据包结构体的指针，用于存放接收到的数据（若为NULL则只执行底层读取而不返回数据）
 * @return uint8_t 是否成功接收数据
 *         - 1：成功接收到一包有效数据，packet已填充
 *         - 0：无数据或接收失败
 * @note   - 函数执行时会先更新调试信息（引脚电平、寄存器状态），然后调用
 * NRF24L01_Receive()
 *         - 若底层返回成功（result == 1），则解析 NRF24L01_RxPacket 并填充到
 * packet 结构体中
 *         - 若 packet 为 NULL，则只进行底层接收（清空FIFO等操作）但不返回数据
 *         - 接收成功后累加 rx_count 计数
 *         - 无论成功与否，都会更新 last_result 和寄存器状态
 */
uint8_t Bsp_RemoteRadio_Receive(BspRemoteRadioPacket_t *packet) {
  uint8_t result;

  /* 采集引脚电平（用于调试） */
  remote_radio_debug.irq_pin =
      (HAL_GPIO_ReadPin(SPI3_IRQ_GPIO_Port, SPI3_IRQ_Pin) == GPIO_PIN_RESET)
          ? 0U
          : 1U;
  remote_radio_debug.csn_pin =
      (HAL_GPIO_ReadPin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin) == GPIO_PIN_RESET)
          ? 0U
          : 1U;
  remote_radio_debug.ce_pin =
      (HAL_GPIO_ReadPin(SPI3_CE_GPIO_Port, SPI3_CE_Pin) == GPIO_PIN_RESET) ? 0U
                                                                           : 1U;

  /* 读取寄存器状态 */
  remote_radio_debug.status = NRF24L01_ReadStatus();
  remote_radio_debug.config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  remote_radio_debug.fifo_status = NRF24L01_ReadFifoStatus();

  /* 清除中断待处理标志（表示本次已处理） */
  remote_radio_irq_pending = 0U;

  /* 调用底层接收函数，尝试从FIFO中读取一包数据 */
  result = NRF24L01_Receive();
  remote_radio_debug.last_result = result; /* 保存本次操作结果 */

  /* 更新寄存器状态（底层函数可能已修改） */
  remote_radio_debug.status = NRF24L01_ReadStatus();
  remote_radio_debug.config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  remote_radio_debug.fifo_status = NRF24L01_ReadFifoStatus();

  /* 若接收失败，直接返回0 */
  if (result != 1U) {
    return 0U;
  }

  /* 成功接收：若packet指针非空，则解析数据并填充 */
  if (packet != 0) {
    packet->mode = NRF24L01_RxPacket[0];        /* 工作模式 */
    packet->lh = (int8_t)NRF24L01_RxPacket[1];  /* 左摇杆水平 */
    packet->lv = (int8_t)NRF24L01_RxPacket[2];  /* 左摇杆垂直 */
    packet->rh = (int8_t)NRF24L01_RxPacket[3];  /* 右摇杆水平 */
    packet->rv = (int8_t)NRF24L01_RxPacket[4];  /* 右摇杆垂直 */
    packet->key = NRF24L01_RxPacket[5] & 0x3FU; /* 按键编号，取低6位有效 */
    packet->key_seq = NRF24L01_RxPacket[6];     /* 按键序列号（用于去重） */
  }

  remote_radio_debug.rx_count++; /* 累计成功接收包数 */
  return 1U;
}

/**
 * @brief 外部中断触发回调（由IRQ引脚下降沿触发）
 * @retval 无
 * @note   将中断待处理标志置1，提示主循环有数据到达。
 *         该函数在中断上下文中调用，应保持轻量。
 */
void Bsp_RemoteRadio_OnIrq(void) { remote_radio_irq_pending = 1U; }

/**
 * @brief 获取调试信息结构体指针（只读）
 * @return const BspRemoteRadioDebug_t* 指向调试数据对象的指针
 * @note   可用于外部显示状态或诊断通信问题
 */
const BspRemoteRadioDebug_t *Bsp_RemoteRadio_GetDebug(void) {
  return &remote_radio_debug;
}

/**
 * @brief HAL库外部中断回调函数（覆盖弱定义）
 * @param GPIO_Pin 触发中断的引脚
 * @retval 无
 * @note   当检测到IRQ引脚中断时，调用 Bsp_RemoteRadio_OnIrq()
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == SPI3_IRQ_Pin) {
    Bsp_RemoteRadio_OnIrq();
  }
}