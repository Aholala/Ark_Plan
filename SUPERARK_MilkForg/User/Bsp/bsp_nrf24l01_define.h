/**
 * @file bsp_nrf24l01_define.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief NRF24L01 寄存器及指令宏定义头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件定义了NRF24L01无线模块的SPI指令码和寄存器地址。
 * 所有宏命名参考官方数据手册，便于上层驱动直接调用。
 */

#ifndef __NRF24L01_DEFINE_H
#define __NRF24L01_DEFINE_H

/*------------------------ SPI 操作指令 ------------------------*/
#define NRF24L01_R_REGISTER 0x00U   /**< 读寄存器指令（后跟5位寄存器地址） */
#define NRF24L01_W_REGISTER 0x20U   /**< 写寄存器指令（后跟5位寄存器地址） */
#define NRF24L01_R_RX_PAYLOAD 0x61U /**< 读取接收FIFO载荷数据 */
#define NRF24L01_W_TX_PAYLOAD 0xA0U /**< 写入发送FIFO载荷数据 */
#define NRF24L01_FLUSH_TX 0xE1U     /**< 清空发送FIFO */
#define NRF24L01_FLUSH_RX 0xE2U     /**< 清空接收FIFO */
#define NRF24L01_REUSE_TX_PL 0xE3U /**< 重复使用上次发送的载荷（用于ACK模式） \
                                    */
#define NRF24L01_R_RX_PL_WID                                                   \
  0x60U /**< 读取接收FIFO中当前载荷的宽度（字节数） */
#define NRF24L01_W_ACK_PAYLOAD                                                 \
  0xA8U /**< 写带自动应答的载荷（用于PTX模式，动态载荷） */
#define NRF24L01_W_TX_PAYLOAD_NOACK                                            \
  0xB0U                    /**< 写不带自动应答的载荷（不触发ACK） */
#define NRF24L01_NOP 0xFFU /**< 空操作（可用于读取状态寄存器） */

/*------------------------ 寄存器地址 ------------------------*/
#define NRF24L01_CONFIG                                                        \
  0x00U /**< 配置寄存器（PRIM_RX, PWR_UP, CRC, 中断使能等） */
#define NRF24L01_EN_AA 0x01U /**< 自动应答使能寄存器（各通道是否启用ACK） */
#define NRF24L01_EN_RXADDR                                                     \
  0x02U                         /**< 接收通道使能寄存器（各通道是否启用接收） */
#define NRF24L01_SETUP_AW 0x03U /**< 地址宽度配置寄存器（3/4/5字节） */
#define NRF24L01_SETUP_RETR 0x04U /**< 重发配置寄存器（重发延时和重发次数） */
#define NRF24L01_RF_CH 0x05U      /**< 射频通道频率设置（2400MHz + RF_CH） */
#define NRF24L01_RF_SETUP 0x06U   /**< 射频配置寄存器（数据速率、发射功率等） */
#define NRF24L01_STATUS 0x07U     /**< 状态寄存器（中断标志、TX_FULL等） */
#define NRF24L01_OBSERVE_TX 0x08U /**< 发送观察寄存器（重发计数、丢失包计数） \
                                   */
#define NRF24L01_RPD 0x09U        /**< 接收功率检测寄存器（RSSI指示） */
#define NRF24L01_RX_ADDR_P0 0x0AU /**< 接收通道0地址 */
#define NRF24L01_RX_ADDR_P1 0x0BU /**< 接收通道1地址 */
#define NRF24L01_RX_ADDR_P2 0x0CU /**< 接收通道2地址（低字节可配置） */
#define NRF24L01_RX_ADDR_P3 0x0DU /**< 接收通道3地址（低字节可配置） */
#define NRF24L01_RX_ADDR_P4 0x0EU /**< 接收通道4地址（低字节可配置） */
#define NRF24L01_RX_ADDR_P5 0x0FU /**< 接收通道5地址（低字节可配置） */
#define NRF24L01_TX_ADDR 0x10U    /**< 发送地址（PTX模式） */
#define NRF24L01_RX_PW_P0 0x11U   /**< 接收通道0数据载荷宽度（字节） */
#define NRF24L01_RX_PW_P1 0x12U   /**< 接收通道1数据载荷宽度 */
#define NRF24L01_RX_PW_P2 0x13U   /**< 接收通道2数据载荷宽度 */
#define NRF24L01_RX_PW_P3 0x14U   /**< 接收通道3数据载荷宽度 */
#define NRF24L01_RX_PW_P4 0x15U   /**< 接收通道4数据载荷宽度 */
#define NRF24L01_RX_PW_P5 0x16U   /**< 接收通道5数据载荷宽度 */
#define NRF24L01_FIFO_STATUS                                                   \
  0x17U /**< FIFO状态寄存器（TX_FULL, TX_EMPTY, RX_EMPTY等） */
#define NRF24L01_DYNPD                                                         \
  0x1CU /**< 动态载荷使能寄存器（各通道是否支持动态载荷） */
#define NRF24L01_FEATURE 0x1DU /**< 功能特性寄存器（动态载荷、应答载荷等） */

#endif /* __NRF24L01_DEFINE_H */