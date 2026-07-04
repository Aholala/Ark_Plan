/**
 * @file bsp_radio_link.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）无线链路驱动源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本文件是无线通信链路的BSP层封装，基于NRF24L01模块实现。
 * 与 bsp_remote_radio.c 不同的是，本模块将数据包编解码逻辑委托给
 * RemoteProtocol 模块处理，实现了协议与硬件的解耦。
 * 上层应用只需关心控制数据包和反馈数据包的结构体，而无需关心
 * 数据打包/解包的细节。
 */

#include "bsp_radio_link.h"

#include "bsp_nrf24l01.h" /* NRF24L01底层驱动 */

/**
 * @brief 初始化无线链路模块
 * @retval 无
 * @note  调用 NRF24L01_Init() 完成模块上电、寄存器配置并进入接收模式
 */
void Bsp_RadioLink_Init(void) { NRF24L01_Init(); }

/**
 * @brief 发送控制数据包（摇杆值 + 按键信息）
 * @param packet 指向控制数据包结构体的指针（不能为NULL）
 * @return uint8_t 发送结果（由 NRF24L01_Send 返回）
 *         - 0：发送进行中（未完成）
 *         - 1：发送成功并收到ACK
 *         - 2：达到最大重发次数（MAX_RT）
 *         - 3：同时出现TX_DS和MAX_RT（异常）
 * @note   - 调用 RemoteProtocol_EncodeControl() 将结构体数据打包到
 * NRF24L01_TxPacket
 *         - 然后调用 NRF24L01_Send() 启动非阻塞发送
 *         - 如果 packet 为空指针，直接返回0
 */
uint8_t Bsp_RadioLink_SendControl(const BspRadioLinkControlPacket_t *packet) {
  if (packet == 0) {
    return 0U;
  }

  /* 将控制数据编码为无线协议格式（打包到发送缓冲区） */
  RemoteProtocol_EncodeControl(packet, NRF24L01_TxPacket);
  return NRF24L01_Send();
}

/**
 * @brief 接收底盘的反馈数据包
 * @param packet
 * 指向反馈数据包结构体的指针（若为NULL，则只执行接收但不保存数据）
 * @return uint8_t 是否成功接收
 *         - 1：成功接收到一包反馈数据，packet已填充（若不为NULL）
 *         - 0：无数据或接收失败
 * @note   - 调用 NRF24L01_Receive() 尝试从FIFO读取一包数据
 *         - 若接收成功，调用 RemoteProtocol_DecodeFeedback()
 * 将原始字节流解析为结构体
 *         - 实现协议层与硬件层的分离，便于协议升级或更换无线模块
 */
uint8_t Bsp_RadioLink_ReceiveFeedback(BspRadioLinkFeedbackPacket_t *packet) {
  /* 尝试接收一包数据，若失败则返回0 */
  if (NRF24L01_Receive() != 1U) {
    return 0U;
  }

  /* 若调用者提供了存储指针，则解码数据 */
  if (packet != 0) {
    RemoteProtocol_DecodeFeedback(NRF24L01_RxPacket, packet);
  }

  return 1U;
}