/**
 * @file module_vision_protocol.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 视觉协议解析模块实现
 * @version 3.0
 * @date 2026-07-22
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块提供视觉传感器数据帧的校验和解析功能，
 *          支持9字节固定帧格式（SOF1, SOF2, source, base/core/ball*3, CRC），
 *          解析结果以错误码形式返回，便于上层处理。
 *          test(自家二维码): A5 5A 00 01 06 00 00 00 F8
 *          test(球框二维码): A5 5A 01 00 00 01 04 06 FD
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
 * @brief 解析视觉帧数据（9字节格式）
 *
 * @param data  原始数据帧缓冲区指针（至少 VISION_PROTOCOL_FRAME_SIZE 字节）
 * @param len   数据缓冲区长度（字节）
 * @param frame 输出参数，用于存储解析出的帧信息（不可为空）
 * @return VisionParseError_t 解析结果错误码
 *
 * @note 帧格式约定（9字节）：
 *       - 字节0: SOF1 (0xA5)
 *       - 字节1: SOF2 (0x5A)
 *       - 字节2: data_source (0=自家二维码, 1=球框二维码)
 *       - 字节3: base_color (仅 data_source=0 时有效)
 *       - 字节4: core_color (仅 data_source=0 时有效)
 *       - 字节5: ball_color1 (仅 data_source=1 时有效)
 *       - 字节6: ball_color2 (仅 data_source=1 时有效)
 *       - 字节7: ball_color3 (仅 data_source=1 时有效)
 *       - 字节8: CRC8 (校验字节，覆盖字节0~7)
 */
VisionParseError_t VisionProtocol_ParseFrame(const uint8_t *data,
                                              uint16_t len,
                                              VisionFrame_t *frame) {
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

  /* 3. 数据源合法性检查 */
  if (data[2] > 1U) {
    return VISION_ERR_INVALID_COLOR;
  }

  /* 4. 根据数据源校验颜色字段 */
  if (data[2] == (uint8_t)VISION_SOURCE_OWN_QR) {
    /* 自家二维码：校验底色和芯色 */
    if ((data[3] >= (uint8_t)VISION_COLOR_COUNT) ||
        (data[4] >= (uint8_t)VISION_COLOR_COUNT)) {
      return VISION_ERR_INVALID_COLOR;
    }
  } else {
    /* 球框二维码：校验三个球框颜色 */
    if ((data[5] >= (uint8_t)VISION_COLOR_COUNT) ||
        (data[6] >= (uint8_t)VISION_COLOR_COUNT) ||
        (data[7] >= (uint8_t)VISION_COLOR_COUNT)) {
      return VISION_ERR_INVALID_COLOR;
    }
  }

  /* 5. CRC校验（校验前8字节，即SOF1~ball_color3） */
  crc = VisionProtocol_CalcCrc8(data, VISION_PROTOCOL_FRAME_SIZE - 1U);
  if (crc != data[VISION_PROTOCOL_FRAME_SIZE - 1U]) {
    return VISION_ERR_CRC_FAIL;
  }

  /* 6. 解析并填充帧信息 */
  frame->data_source = (VisionDataSource_t)data[2];
  if (frame->data_source == VISION_SOURCE_OWN_QR) {
    frame->base_color = (VisionColor_t)data[3];
    frame->core_color = (VisionColor_t)data[4];
  } else {
    frame->ball_color1 = (VisionColor_t)data[5];
    frame->ball_color2 = (VisionColor_t)data[6];
    frame->ball_color3 = (VisionColor_t)data[7];
  }

  return VISION_PARSE_OK;
}