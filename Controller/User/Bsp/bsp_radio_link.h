/**
 * @file bsp_radio_link.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）无线链路驱动头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件定义了无线链路BSP层的接口，基于NRF24L01硬件实现。
 * 与 bsp_remote_radio 不同的是，本模块使用 module_remote_protocol
 * 进行数据编解码，实现了协议层与硬件层的解耦。
 *
 * 控制数据包用于发送摇杆和按键状态，反馈数据包用于接收底盘状态。
 * 数据包结构由 RemoteProtocol 模块定义，本层仅负责传输。
 */

#ifndef __BSP_RADIO_LINK_H
#define __BSP_RADIO_LINK_H

#include "module_remote_protocol.h" /* 遥控协议编解码模块 */

/**
 * @brief BSP层控制数据包类型（重命名自 RemoteProtocolControlPacket_t）
 * @note  包含摇杆值、按键编号和按键序列号
 */
typedef RemoteProtocolControlPacket_t BspRadioLinkControlPacket_t;

/**
 * @brief BSP层反馈数据包类型（重命名自 RemoteProtocolFeedbackPacket_t）
 * @note  包含速度等级、PWM值、角度、轮速等底盘状态信息
 */
typedef RemoteProtocolFeedbackPacket_t BspRadioLinkFeedbackPacket_t;

/**
 * @brief 初始化无线链路模块
 * @retval 无
 * @note  调用 NRF24L01_Init() 完成硬件初始化，默认进入接收模式
 */
void Bsp_RadioLink_Init(void);

/**
 * @brief 发送控制数据包（摇杆+按键）
 * @param packet 指向控制数据包结构体的指针（不可为NULL）
 * @return uint8_t 发送结果（由 NRF24L01_Send 返回）
 *         - 0：发送进行中（未完成）
 *         - 1：发送成功（已收到ACK）
 *         - 2：达到最大重发次数（失败）
 *         - 3：异常状态（同时触发TX_DS和MAX_RT）
 * @note  非阻塞函数，第一次调用启动发送，后续需轮询检查直到返回非0
 *        使用 RemoteProtocol_EncodeControl 将结构体打包为字节流
 */
uint8_t Bsp_RadioLink_SendControl(const BspRadioLinkControlPacket_t *packet);

/**
 * @brief 接收底盘的反馈数据包
 * @param packet 指向反馈数据包结构体的指针（若为NULL，则只清空FIFO不返回数据）
 * @return uint8_t 是否成功接收
 *         - 1：成功接收一包反馈数据（packet已填充）
 *         - 0：无数据或接收失败
 * @note  非阻塞调用，立即返回。若接收成功，使用 RemoteProtocol_DecodeFeedback
 *        将原始字节流解析为结构体
 */
uint8_t Bsp_RadioLink_ReceiveFeedback(BspRadioLinkFeedbackPacket_t *packet);

#endif /* __BSP_RADIO_LINK_H */