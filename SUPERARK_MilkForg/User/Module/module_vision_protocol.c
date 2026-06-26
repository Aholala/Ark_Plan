/**
 * @file module_vision_protocol.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 视觉协议解析模块实现
 * @version 1.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块提供视觉传感器数据帧的校验和解析功能，
 *          支持颜色帧的CRC校验及颜色信息提取。
 */

#include "module_vision_protocol.h"

/**
 * @brief 计算数据块的CRC8校验值（异或校验）
 *
 * @param data 待校验的数据缓冲区指针（不可为空）
 * @param len  数据长度（字节数）
 * @return uint8_t 计算得到的CRC8校验值
 * @retval 0 当data为空指针时返回0
 *
 * @note 校验算法为简单异或（XOR）累加，对所有字节依次异或。
 */
uint8_t VisionProtocol_CalcCrc8(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0U;

  /* 空指针检查 */
  if (data == 0) {
    return 0U;
  }

  /* 逐字节异或累加 */
  for (uint16_t i = 0U; i < len; i++) {
    crc ^= data[i];
  }

  return crc;
}

/**
 * @brief 解析视觉颜色帧数据
 *
 * @param data  原始数据帧缓冲区指针（至少 VISION_PROTOCOL_FRAME_SIZE 字节）
 * @param len   数据缓冲区长度（字节）
 * @param frame 输出参数，用于存储解析出的颜色帧信息（不可为空）
 * @return uint8_t 解析结果
 * @retval 1 解析成功，frame中填充有效数据
 * @retval 0 解析失败（参数错误、帧头不匹配、颜色枚举无效或CRC校验错误）
 *
 * @note 帧格式约定：
 *       - 字节0: SOF1 (帧头标识1)
 *       - 字节1: SOF2 (帧头标识2)
 *       - 字节2: base_color (底色枚举)
 *       - 字节3: core_color (芯色枚举)
 *       - 字节4: CRC8 (校验字节，覆盖字节0~3)
 *       - 帧总长度固定为 VISION_PROTOCOL_FRAME_SIZE
 */
uint8_t VisionProtocol_ParseColorFrame(const uint8_t *data, uint16_t len,
                                       VisionColorFrame_t *frame) {
  uint8_t crc;

  /* 1. 参数有效性检查 */
  if ((data == 0) || (frame == 0) || (len < VISION_PROTOCOL_FRAME_SIZE)) {
    return 0U;
  }

  /* 2. 帧头校验 */
  if ((data[0] != VISION_PROTOCOL_SOF1) || (data[1] != VISION_PROTOCOL_SOF2)) {
    return 0U;
  }

  /* 3. 颜色枚举值合法性检查（防止越界） */
  if ((data[2] >= (uint8_t)VISION_COLOR_COUNT) ||
      (data[3] >= (uint8_t)VISION_COLOR_COUNT)) {
    return 0U;
  }

  /* 4. CRC校验（不包括最后一个校验字节本身） */
  crc = VisionProtocol_CalcCrc8(data, VISION_PROTOCOL_FRAME_SIZE - 1U);
  if (crc != data[VISION_PROTOCOL_FRAME_SIZE - 1U]) {
    return 0U;
  }

  /* 5. 解析并填充颜色信息 */
  frame->base_color = (VisionColor_t)data[2];
  frame->core_color = (VisionColor_t)data[3];
  return 1U;
}