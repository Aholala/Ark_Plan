/**
 * @file bsp_usb_cdc.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包 - USB CDC虚拟串口驱动头文件
 * @version 1.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 定义USB CDC通信接口，包括接收回调、数据取出和发送函数。
 *          接收缓冲区大小可配置，支持非阻塞轮询式读取。
 */

#ifndef __BSP_USB_CDC_H
#define __BSP_USB_CDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @defgroup BSP_USB_CDC USB CDC驱动
 * @brief USB虚拟串口驱动接口
 * @{
 */

/**
 * @name 配置宏定义
 * @{
 */
#define BSP_USB_CDC_RX_BUFFER_SIZE 64U /**< USB CDC接收缓冲区大小（字节） */
/** @} */

/**
 * @brief USB CDC接收回调函数（由USB中断调用）
 *
 * @param data 接收到的数据缓冲区指针
 * @param len  接收到的数据长度（字节）
 *
 * @note 该函数运行在中断上下文，应尽量简短。
 * @note 数据会被拷贝到内部缓冲区，并置位接收标志。
 * @note 同时会将数据回传给主机（环回模式），便于调试。
 */
void Bsp_UsbCdc_OnRx(const uint8_t *data, uint16_t len);

/**
 * @brief 从接收缓冲区取出一帧数据（非阻塞）
 *
 * @param data      输出缓冲区指针，用于存放取出的数据
 * @param data_size 输出缓冲区大小（字节）
 * @param len       输出参数，返回实际拷贝的数据长度（字节）
 * @return uint8_t 操作结果
 * @retval 1 成功取到数据
 * @retval 0 无新数据或参数无效
 *
 * @note 调用后会清除接收有效标志，每次只能取一帧。
 * @note 若内部缓冲数据长度大于 data_size，则截断，仅拷贝前 data_size 字节。
 * @note 建议应用层轮询调用，以检查并获取新数据。
 */
uint8_t Bsp_UsbCdc_TakeRx(uint8_t *data, uint16_t data_size, uint16_t *len);

/**
 * @brief 通过USB CDC发送数据
 *
 * @param data 待发送数据缓冲区指针
 * @param len  待发送数据长度（字节）
 * @return uint8_t 发送结果
 * @retval 1 发送请求成功（底层异步发送）
 * @retval 0 发送失败（如设备未就绪或参数无效）
 *
 * @note 非阻塞发送，实际发送在USB中断中完成。
 * @note 数据发送完成后会触发相应回调（由USB库处理）。
 */
uint8_t Bsp_UsbCdc_Transmit(uint8_t *data, uint16_t len);

/** @} */ /* end of BSP_USB_CDC group */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USB_CDC_H */