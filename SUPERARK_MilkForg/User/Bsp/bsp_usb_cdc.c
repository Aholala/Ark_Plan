/**
 * @file bsp_usb_cdc.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包 - USB CDC虚拟串口驱动
 * @version 1.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块封装USB CDC类通信接口，提供：
 *          1. 接收回调函数（由USB中断调用），将数据存入内部缓冲区；
 *          2. 接收数据取出接口（非阻塞，供应用层轮询调用）；
 *          3. 数据发送接口（直接调用底层CDC发送函数）。
 *          @note 接收缓冲区为单帧缓冲，新数据会覆盖旧数据。
 */

#include "bsp_usb_cdc.h"
#include "usbd_cdc_if.h"

/*==================== 静态变量 ====================*/

/** @brief 当前接收数据长度（字节） */
static volatile uint16_t bsp_usb_cdc_rx_len;

/** @brief 接收数据有效标志（1=有数据待读取，0=无新数据） */
static volatile uint8_t bsp_usb_cdc_has_rx;

/** @brief 接收数据缓冲区（单帧缓冲，大小由 BSP_USB_CDC_RX_BUFFER_SIZE 定义） */
static uint8_t bsp_usb_cdc_rx_buffer[BSP_USB_CDC_RX_BUFFER_SIZE];

/*==================== 函数实现 ====================*/

/**
 * @brief USB CDC接收回调函数（由USB中断服务程序调用）
 *
 * @param data 接收到的数据缓冲区指针
 * @param len  接收到的数据长度（字节）
 *
 * @note 本函数运行在中断上下文，应快速执行，避免耗时操作。
 * @note 若数据长度超过缓冲区容量，则只复制前 N 个字节（N为缓冲区大小）。
 * @note 新数据会覆盖旧数据，应用层需及时调用 Bsp_UsbCdc_TakeRx() 取走数据。
 * @note 该函数内部将接收到的数据回传给主机（回环测试），便于调试。
 *
 * @warning data 指针必须有效，否则直接返回。
 */
void Bsp_UsbCdc_OnRx(const uint8_t *data, uint16_t len) {
  uint16_t copy_len = len;

  /* 空指针检查 */
  if (data == 0) {
    return;
  }

  /* 长度截断，防止缓冲区溢出 */
  if (copy_len > BSP_USB_CDC_RX_BUFFER_SIZE) {
    copy_len = BSP_USB_CDC_RX_BUFFER_SIZE;
  }

  /* 拷贝数据到内部缓冲区 */
  for (uint16_t i = 0U; i < copy_len; i++) {
    bsp_usb_cdc_rx_buffer[i] = data[i];
  }

  /* 更新接收状态 */
  bsp_usb_cdc_rx_len = copy_len;
  bsp_usb_cdc_has_rx = 1U;

  /* 回传数据（环回模式，便于上位机调试） */
  (void)CDC_Transmit_FS(bsp_usb_cdc_rx_buffer, copy_len);
}

/**
 * @brief 从接收缓冲区取出一帧数据（非阻塞）
 *
 * @param data      输出缓冲区指针，用于存储取出的数据
 * @param data_size 输出缓冲区大小（字节）
 * @param len       输出参数，返回实际取出的数据长度（字节）
 * @return uint8_t 操作结果
 * @retval 1 成功取到数据
 * @retval 0 无新数据或参数无效
 *
 * @note 调用后会清除接收有效标志，下一次调用前需等待新数据到达。
 * @note 若 data_size 小于实际接收长度，则只拷贝前 data_size 个字节，
 *       但 len 返回实际长度（截断前长度），调用者需注意。
 * @note 该函数可被应用层轮询调用，以检测并获取新数据。
 */
uint8_t Bsp_UsbCdc_TakeRx(uint8_t *data, uint16_t data_size, uint16_t *len) {
  uint16_t copy_len;

  /* 参数有效性检查 */
  if ((data == 0) || (len == 0) || (bsp_usb_cdc_has_rx == 0U)) {
    return 0U;
  }

  /* 计算实际拷贝长度（不超过输出缓冲区大小） */
  copy_len = bsp_usb_cdc_rx_len;
  if (copy_len > data_size) {
    copy_len = data_size;
  }

  /* 拷贝数据到用户缓冲区 */
  for (uint16_t i = 0U; i < copy_len; i++) {
    data[i] = bsp_usb_cdc_rx_buffer[i];
  }

  /* 返回实际数据长度（即使截断，也返回原始长度供上层判断） */
  *len = copy_len; /* 注：若需返回原始长度，可改为
                      bsp_usb_cdc_rx_len，但此处为安全仅返回拷贝长度 */

  /* 清除接收标志，表示数据已被取走 */
  bsp_usb_cdc_has_rx = 0U;

  return 1U;
}

/**
 * @brief 通过USB CDC发送数据
 *
 * @param data 待发送数据缓冲区指针
 * @param len  待发送数据长度（字节）
 * @return uint8_t 发送结果
 * @retval 1 发送请求成功（实际发送由USB中断完成）
 * @retval 0 发送失败（如设备未就绪或参数无效）
 *
 * @note 该函数直接调用底层 CDC_Transmit_FS，为非阻塞发送。
 * @note 数据发送完成时会触发回调函数（由USB库处理）。
 * @note 若设备未连接或正在发送中，可能返回失败。
 */
uint8_t Bsp_UsbCdc_Transmit(uint8_t *data, uint16_t len) {
  return CDC_Transmit_FS(data, len);
}