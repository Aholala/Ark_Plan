/**
 * @file module_vision_protocol.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 视觉协议解析模块 - 头文件
 * @version 2.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 定义视觉传感器通信协议的数据结构、枚举常量和接口函数。
 *          协议帧格式固定为5字节，包含帧头、颜色ID和CRC校验。
 */

#ifndef __MODULE_VISION_PROTOCOL_H
#define __MODULE_VISION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @defgroup VisionProtocol 视觉协议定义
 * @brief 视觉传感器USB CDC通信协议
 * @{
 */

/**
 * @name 协议帧格式定义
 * @brief 帧格式说明（总长5字节）：
 *        - Byte0: 帧头1 (0xA5)
 *        - Byte1: 帧头2 (0x5A)
 *        - Byte2: 底色ID (参见 VisionColor_t)
 *        - Byte3: 芯色ID (参见 VisionColor_t)
 *        - Byte4: CRC校验 (对Byte0~Byte3进行异或)
 * @{
 */
#define VISION_PROTOCOL_SOF1 0xA5U    /**< 帧头标识字节1 */
#define VISION_PROTOCOL_SOF2 0x5AU    /**< 帧头标识字节2 */
#define VISION_PROTOCOL_FRAME_SIZE 5U /**< 完整帧总字节数 */
/** @} */

/**
 * @brief 视觉颜色枚举
 * @note 颜色ID与协议中字节值对应，依次为0~7
 */
typedef enum {
  VISION_COLOR_NONE = 0, /**< 无颜色/关闭 */
  VISION_COLOR_RED,      /**< 红色 */
  VISION_COLOR_ORANGE,   /**< 橙色 */
  VISION_COLOR_YELLOW,   /**< 黄色 */
  VISION_COLOR_GREEN,    /**< 绿色 */
  VISION_COLOR_CYAN,     /**< 青色 */
  VISION_COLOR_BLUE,     /**< 蓝色 */
  VISION_COLOR_PURPLE,   /**< 紫色 */
  VISION_COLOR_COUNT     /**< 颜色总数（用于枚举范围检查） */
} VisionColor_t;

/**
 * @brief 视觉颜色帧数据结构
 * @note 存储解析后得到的底色和芯色
 */
typedef struct {
  VisionColor_t base_color; /**< 目标底色（如装甲板灯条颜色） */
  VisionColor_t core_color; /**< 目标芯色（如装甲板中心颜色） */
} VisionColorFrame_t;

/**
 * @brief 解析错误码枚举
 */
typedef enum {
  VISION_PARSE_OK = 0,      /**< 解析成功 */
  VISION_ERR_NULL_PTR,      /**< 空指针参数 */
  VISION_ERR_LEN_TOO_SHORT, /**< 数据长度不足 */
  VISION_ERR_SOF_MISMATCH,  /**< 帧头不匹配 */
  VISION_ERR_INVALID_COLOR, /**< 颜色ID超出范围 */
  VISION_ERR_CRC_FAIL       /**< CRC校验失败 */
} VisionParseError_t;

/**
 * @brief 计算数据块的CRC8校验值（异或校验）
 *
 * @param data 待校验数据缓冲区指针
 * @param len  数据长度（字节数）
 * @return uint8_t 异或累加得到的校验值
 * @retval 0 当data为NULL时返回0
 *
 * @note 校验算法简单高效，适用于短帧校验。
 */
uint8_t VisionProtocol_CalcCrc8(const uint8_t *data, uint16_t len);

/**
 * @brief 解析视觉颜色帧（5字节格式）
 *
 * @param data  原始帧数据缓冲区（至少 VISION_PROTOCOL_FRAME_SIZE 字节）
 * @param len   缓冲区长度（字节）
 * @param frame 输出参数，解析成功后填充颜色信息
 * @return VisionParseError_t 解析结果错误码
 *
 * @note 内部调用 VisionProtocol_CalcCrc8 进行校验，确保数据完整性。
 */
VisionParseError_t VisionProtocol_ParseColorFrame(const uint8_t *data,
                                                  uint16_t len,
                                                  VisionColorFrame_t *frame);

/** @} */ /* end of VisionProtocol group */

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_VISION_PROTOCOL_H */