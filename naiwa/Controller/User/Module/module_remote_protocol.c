/**
 * @file module_remote_protocol.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 遥控协议编解码模块源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本模块实现了遥控数据包与字节流之间的双向转换（编码/解码）。
 * 将应用层结构体（控制数据包/反馈数据包）序列化为无线传输载荷，
 * 或将接收到的载荷反序列化为结构体，便于上层应用处理。
 *
 * 协议格式：
 * - 控制数据包（7字节）：[mode][lh][lv][rh][rv][key][key_seq]
 * - 反馈数据包（20字节）：[pwm[0..3] (4字节)] + [speed[0..3] (16字节)]
 */

#include "module_remote_protocol.h"

#include <string.h> /* memset, memcpy */

/**
 * @brief 将控制数据包结构体编码为字节流载荷
 * @param packet  指向控制数据包结构体的指针（输入）
 * @param payload 指向载荷缓冲区的指针（输出，大小为
 * REMOTE_PROTOCOL_PAYLOAD_SIZE）
 * @retval 无
 * @note   - 载荷格式：[mode][lh][lv][rh][rv][key][key_seq]，共7字节
 *         - 先清空整个载荷缓冲区，再填充数据，确保未使用字节为0
 *         - 按键值仅保留低6位（有效范围0~63），高位被屏蔽
 *         - 若传入空指针，函数直接返回
 */
void RemoteProtocol_EncodeControl(
    const RemoteProtocolControlPacket_t *packet,
    uint8_t payload[REMOTE_PROTOCOL_PAYLOAD_SIZE]) {
  if ((packet == 0) || (payload == 0)) {
    return;
  }

  /* 清空整个载荷缓冲区，确保未使用字节为0 */
  memset(payload, 0, REMOTE_PROTOCOL_PAYLOAD_SIZE);

  /* 按协议顺序填充各字段 */
  payload[0] = packet->mode;
  payload[1] = (uint8_t)packet->lh;
  payload[2] = (uint8_t)packet->lv;
  payload[3] = (uint8_t)packet->rh;
  payload[4] = (uint8_t)packet->rv;
  payload[5] = packet->key & 0x3FU; /* 仅保留低6位，高位截断 */
  payload[6] = packet->key_seq;
}

/**
 * @brief 将字节流载荷解码为控制数据包结构体
 * @param payload 指向载荷缓冲区的指针（输入，大小为
 * REMOTE_PROTOCOL_PAYLOAD_SIZE）
 * @param packet  指向控制数据包结构体的指针（输出）
 * @retval 无
 * @note   - 载荷格式与编码函数保持一致
 *         - 从载荷中提取各字段并赋值给结构体成员
 *         - 若传入空指针，函数直接返回
 */
void RemoteProtocol_DecodeControl(
    const uint8_t payload[REMOTE_PROTOCOL_PAYLOAD_SIZE],
    RemoteProtocolControlPacket_t *packet) {
  if ((payload == 0) || (packet == 0)) {
    return;
  }

  /* 从载荷中提取各字段 */
  packet->mode = payload[0];
  packet->lh = (int8_t)payload[1];
  packet->lv = (int8_t)payload[2];
  packet->rh = (int8_t)payload[3];
  packet->rv = (int8_t)payload[4];
  packet->key = payload[5] & 0x3FU; /* 确保按键值在有效范围内 */
  packet->key_seq = payload[6];
}

/**
 * @brief 将反馈数据包结构体编码为字节流载荷
 * @param packet  指向反馈数据包结构体的指针（输入）
 * @param payload 指向载荷缓冲区的指针（输出，大小为
 * REMOTE_PROTOCOL_PAYLOAD_SIZE）
 * @retval 无
 * @note   - 载荷格式：[pwm[0]][pwm[1]][pwm[2]][pwm[3]] +
 * [speed[0..3]]，共20字节
 *         - 前4字节存储4个PWM值（int8_t类型）
 *         - 后16字节存储4个float类型的速度值（每个float占4字节）
 *         - 使用memcpy直接复制float的二进制表示，确保字节序与接收端一致
 *         - 先清空整个载荷缓冲区，再填充数据
 *         - 若传入空指针，函数直接返回
 */
void RemoteProtocol_EncodeFeedback(
    const RemoteProtocolFeedbackPacket_t *packet,
    uint8_t payload[REMOTE_PROTOCOL_PAYLOAD_SIZE]) {
  if ((packet == 0) || (payload == 0)) {
    return;
  }

  /* 清空整个载荷缓冲区 */
  memset(payload, 0, REMOTE_PROTOCOL_PAYLOAD_SIZE);

  /* 前4字节：存储4个PWM值（int8_t） */
  for (uint8_t i = 0U; i < 4U; i++) {
    payload[i] = (uint8_t)packet->pwm[i];
    /* 后16字节：存储4个float速度值（每个占4字节），使用memcpy保证二进制兼容 */
    memcpy(&payload[4U + (i * sizeof(float))], &packet->speed[i],
           sizeof(float));
  }
}

/**
 * @brief 将字节流载荷解码为反馈数据包结构体
 * @param payload 指向载荷缓冲区的指针（输入，大小为
 * REMOTE_PROTOCOL_PAYLOAD_SIZE）
 * @param packet  指向反馈数据包结构体的指针（输出）
 * @retval 无
 * @note   - 载荷格式与编码函数保持一致
 *         - 从载荷中提取4个PWM值（int8_t）和4个速度值（float）
 *         - 使用memcpy将字节流还原为float类型，确保二进制兼容
 *         - 若传入空指针，函数直接返回
 */
void RemoteProtocol_DecodeFeedback(
    const uint8_t payload[REMOTE_PROTOCOL_PAYLOAD_SIZE],
    RemoteProtocolFeedbackPacket_t *packet) {
  if ((payload == 0) || (packet == 0)) {
    return;
  }

  /* 提取4个PWM值（前4字节） */
  for (uint8_t i = 0U; i < 4U; i++) {
    packet->pwm[i] = (int8_t)payload[i];
    /* 提取4个速度值（从偏移4开始，每个float占4字节） */
    memcpy(&packet->speed[i], &payload[4U + (i * sizeof(float))],
           sizeof(float));
  }
}