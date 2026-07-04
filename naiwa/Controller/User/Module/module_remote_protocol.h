/**
 * @file module_remote_protocol.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 遥控协议编解码模块头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本模块定义了遥控数据包的结构体及编解码函数，用于在应用层与无线传输层之间
 * 进行数据序列化和反序列化。协议载荷固定为32字节（符合NRF24L01最大载荷）。
 *
 * 控制数据包（发送端→接收端）：包含摇杆值、按键编号和序列号。
 * 反馈数据包（接收端→发送端）：包含4个PWM值和4个速度值（float数组）。
 * 所有编解码函数均进行指针有效性检查，避免空指针访问。
 */

#ifndef __MODULE_REMOTE_PROTOCOL_H
#define __MODULE_REMOTE_PROTOCOL_H

#include <stdint.h> /* 标准整数类型定义 */

/**
 * @brief 无线载荷固定大小（32字节，符合NRF24L01最大载荷）
 */
#define REMOTE_PROTOCOL_PAYLOAD_SIZE 32U

/**
 * @brief 控制数据包结构体（遥控器 → 底盘）
 * @note  用于传输实时操控指令，共7字节有效数据
 */
typedef struct {
  uint8_t mode;    /**< 工作模式（0=发送模式，1=接收模式） */
  int8_t lh;       /**< 左摇杆水平分量（-100 ~ +100） */
  int8_t lv;       /**< 左摇杆垂直分量（-100 ~ +100） */
  int8_t rh;       /**< 右摇杆水平分量（-100 ~ +100） */
  int8_t rv;       /**< 右摇杆垂直分量（-100 ~ +100） */
  uint8_t key;     /**< 按键编号（1~12，0表示无按键） */
  uint8_t key_seq; /**< 按键序列号（每次按键递增，用于接收端去重） */
} RemoteProtocolControlPacket_t;

/**
 * @brief 反馈数据包结构体（底盘 → 遥控器）
 * @note  用于传输底盘状态信息，包含4组PWM和速度数据（float数组）
 */
typedef struct {
  int8_t pwm[4];  /**< 4个电机的PWM占空比（-100 ~ +100） */
  float speed[4]; /**< 4个电机的速度值（单位取决于具体实现，如 m/s 或 rpm） */
} RemoteProtocolFeedbackPacket_t;

/**
 * @brief 将控制数据包编码为字节流载荷
 * @param packet  指向控制数据包结构体的指针（输入）
 * @param payload 指向载荷缓冲区的指针（输出，大小为
 * REMOTE_PROTOCOL_PAYLOAD_SIZE）
 * @retval 无
 * @note  载荷格式：[mode][lh][lv][rh][rv][key][key_seq]，共7字节
 *        未使用的字节被清零，按键值仅保留低6位
 */
void RemoteProtocol_EncodeControl(
    const RemoteProtocolControlPacket_t *packet,
    uint8_t payload[REMOTE_PROTOCOL_PAYLOAD_SIZE]);

/**
 * @brief 将字节流载荷解码为控制数据包结构体
 * @param payload 指向载荷缓冲区的指针（输入，大小为
 * REMOTE_PROTOCOL_PAYLOAD_SIZE）
 * @param packet  指向控制数据包结构体的指针（输出）
 * @retval 无
 */
void RemoteProtocol_DecodeControl(
    const uint8_t payload[REMOTE_PROTOCOL_PAYLOAD_SIZE],
    RemoteProtocolControlPacket_t *packet);

/**
 * @brief 将反馈数据包编码为字节流载荷
 * @param packet  指向反馈数据包结构体的指针（输入）
 * @param payload 指向载荷缓冲区的指针（输出，大小为
 * REMOTE_PROTOCOL_PAYLOAD_SIZE）
 * @retval 无
 * @note  载荷格式：[pwm0][pwm1][pwm2][pwm3] +
 * [speed0..speed3]（每个float占4字节） 共4 + 4×4 = 20字节有效数据，剩余字节置零
 */
void RemoteProtocol_EncodeFeedback(
    const RemoteProtocolFeedbackPacket_t *packet,
    uint8_t payload[REMOTE_PROTOCOL_PAYLOAD_SIZE]);

/**
 * @brief 将字节流载荷解码为反馈数据包结构体
 * @param payload 指向载荷缓冲区的指针（输入，大小为
 * REMOTE_PROTOCOL_PAYLOAD_SIZE）
 * @param packet  指向反馈数据包结构体的指针（输出）
 * @retval 无
 */
void RemoteProtocol_DecodeFeedback(
    const uint8_t payload[REMOTE_PROTOCOL_PAYLOAD_SIZE],
    RemoteProtocolFeedbackPacket_t *packet);

#endif /* __MODULE_REMOTE_PROTOCOL_H */