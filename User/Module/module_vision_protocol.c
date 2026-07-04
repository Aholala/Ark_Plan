/**
 * @file module_vision_protocol.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 视觉协议解析模块实现
 * @version 2.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块提供视觉传感器数据帧的校验和解析功能，
 *          支持5字节固定帧格式（SOF1, SOF2, base, core, CRC），
 *          解析结果以错误码形式返回，便于上层处理。
 *          test:A5 5A 01 02 FC
 */

#include "module_vision_protocol.h"
#include <stddef.h>

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

  if (data == NULL) {
    return 0U;
  }

  for (uint16_t i = 0U; i < len; i++) {
    crc ^= data[i];
  }

  return crc;
}

/**
 * @brief 解析视觉颜色帧数据（5字节格式）
 *
 * @param data  原始数据帧缓冲区指针（至少 VISION_PROTOCOL_FRAME_SIZE 字节）
 * @param len   数据缓冲区长度（字节）
 * @param frame 输出参数，用于存储解析出的颜色帧信息（不可为空）
 * @return VisionParseError_t 解析结果错误码
 *
 * @note 帧格式约定（5字节）：
 *       - 字节0: SOF1 (0xA5)
 *       - 字节1: SOF2 (0x5A)
 *       - 字节2: base_color 基础框颜色
 *       - 字节3: core_color 核心框颜色
 *       - 字节4: CRC8 (校验字节，覆盖字节0~3)
 */
VisionParseError_t VisionProtocol_ParseColorFrame(const uint8_t *data,
                                                  uint16_t len,
                                                  VisionColorFrame_t *frame) {
  uint8_t crc;

  /* 1. 参数有效性检查 */
  if ((data == NULL) || (frame == NULL)) {
    return VISION_ERR_NULL_PTR;
  }
  if (len < VISION_PROTOCOL_FRAME_SIZE) {
    return VISION_ERR_LEN_TOO_SHORT;
  }

  /* 2. 帧头校验 */
  if ((data[0] != VISION_PROTOCOL_SOF1) || (data[1] != VISION_PROTOCOL_SOF2)) {
    return VISION_ERR_SOF_MISMATCH;
  }

  /* 3. 颜色枚举值合法性检查（防止越界） */
  if ((data[2] >= (uint8_t)VISION_COLOR_COUNT) ||
      (data[3] >= (uint8_t)VISION_COLOR_COUNT)) {
    return VISION_ERR_INVALID_COLOR;
  }

  /* 4. CRC校验（仅校验前4字节，即SOF1, SOF2, base, core） */
  crc = VisionProtocol_CalcCrc8(data, VISION_PROTOCOL_FRAME_SIZE - 1U);
  if (crc != data[VISION_PROTOCOL_FRAME_SIZE - 1U]) {
    return VISION_ERR_CRC_FAIL;
  }

  /* 5. 解析并填充颜色信息 */
  frame->base_color = (VisionColor_t)data[2];
  frame->core_color = (VisionColor_t)data[3];

  return VISION_PARSE_OK;
}