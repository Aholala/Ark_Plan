/**
 * @file module_vision_protocol.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 视觉协议解析模块 - 头文件
 * @version 3.0
 * @date 2026-07-22
 *
 * @copyright Copyright (c) 2026
 *
 * @details 定义视觉传感器通信协议的数据结构、枚举常量和接口函数。
 *          协议帧格式固定为9字节，包含帧头、数据源、颜色ID和CRC校验。
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
 * @brief 帧格式说明（总长9字节）：
 *        - Byte0: 帧头1 (0xA5)
 *        - Byte1: 帧头2 (0x5A)
 *        - Byte2: 数据源 (0=自家二维码, 1=球框二维码)
 *        - Byte3: 底色ID (仅数据源=0时有效，参见 VisionColor_t)
 *        - Byte4: 芯色ID (仅数据源=0时有效，参见 VisionColor_t)
 *        - Byte5: 球框颜色1 (仅数据源=1时有效，参见 VisionColor_t)
 *        - Byte6: 球框颜色2 (仅数据源=1时有效，参见 VisionColor_t)
 *        - Byte7: 球框颜色3 (仅数据源=1时有效，参见 VisionColor_t)
 *        - Byte8: CRC校验 (对Byte0~Byte7进行异或)
 * @{
 */
#define VISION_PROTOCOL_SOF1 0xA5U    /**< 帧头标识字节1 */
#define VISION_PROTOCOL_SOF2 0x5AU    /**< 帧头标识字节2 */
#define VISION_PROTOCOL_FRAME_SIZE 9U /**< 完整帧总字节数 */
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
 * @brief 视觉数据源枚举
 */
typedef enum {
  VISION_SOURCE_OWN_QR = 0, /**< 自家二维码 */
  VISION_SOURCE_BALL_QR = 1, /**< 球框二维码 */
} VisionDataSource_t;

/**
 * @brief 视觉帧数据结构
 * @note 存储解析后的数据源和颜色信息，字段有效性取决于 data_source
 */
typedef struct {
  VisionDataSource_t data_source; /**< 数据源类型 */
  VisionColor_t base_color;       /**< 目标底色（data_source=0时有效） */
  VisionColor_t core_color;       /**< 目标芯色（data_source=0时有效） */
  VisionColor_t ball_color1;      /**< 球框颜色1（data_source=1时有效） */
  VisionColor_t ball_color2;      /**< 球框颜色2（data_source=1时有效） */
  VisionColor_t ball_color3;      /**< 球框颜色3（data_source=1时有效） */
} VisionFrame_t;

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
 * @brief 解析视觉帧数据（9字节格式）
 *
 * @param data  原始帧数据缓冲区（至少 VISION_PROTOCOL_FRAME_SIZE 字节）
 * @param len   缓冲区长度（字节）
 * @param frame 输出参数，解析成功后填充数据
 * @return VisionParseError_t 解析结果错误码
 *
 * @note 内部调用 VisionProtocol_CalcCrc8 进行校验，确保数据完整性。
 */
VisionParseError_t VisionProtocol_ParseFrame(const uint8_t *data,
                                              uint16_t len,
                                              VisionFrame_t *frame);

/** @} */ /* end of VisionProtocol group */

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_VISION_PROTOCOL_H */