/**
 * @file bsp_nrf24l01.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）NRF24L01无线模块驱动源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本文件实现了NRF24L01+无线收发模块的底层SPI通信、寄存器读写、
 * 数据包收发、模式切换等功能。支持发送和接收模式，内置自动重发
 * 和状态检测机制。数据包宽度由宏 NRF24L01_PACKET_WIDTH 定义。
 */

#include "bsp_nrf24l01.h"

#include "spi.h" /* 包含SPI外设句柄（hspi3） */

/*------------------------ 宏定义 ------------------------*/
#define NRF24L01_SPI_TIMEOUT_MS 1U /**< SPI通信超时时间（毫秒） */

/*------------------------ 全局变量（用户可访问） ------------------------*/
uint8_t NRF24L01_TxAddress[5] = {'n', 'a', 'i', 'w',
                                 'a'};            /**< 发送地址（5字节） */
uint8_t NRF24L01_TxPacket[NRF24L01_PACKET_WIDTH]; /**< 发送数据包缓冲区 */
uint8_t NRF24L01_RxAddress[5] = {'n', 'a', 'i', 'w',
                                 'a'};            /**< 接收地址（5字节） */
uint8_t NRF24L01_RxPacket[NRF24L01_PACKET_WIDTH]; /**< 接收数据包缓冲区 */

/*------------------------ 静态变量 ------------------------*/
static uint8_t
    nrf24l01_tx_active; /**< 发送状态标志：0=空闲，1=发送正在进行（等待ACK） */

/*------------------------ 底层硬件控制函数 ------------------------*/

/**
 * @brief 写CE引脚电平
 * @param value 0或1
 * @retval 无
 */
static void NRF24L01_W_CE(uint8_t value) {
  HAL_GPIO_WritePin(SPI3_CE_GPIO_Port, SPI3_CE_Pin,
                    value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 写CSN引脚电平
 * @param value 0或1
 * @retval 无
 */
static void NRF24L01_W_CSN(uint8_t value) {
  HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin,
                    value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief SPI交换一个字节（同时发送和接收）
 * @param byte 待发送的字节
 * @return uint8_t 接收到的字节
 * @note 若超时则返回 NRF24L01_NOP (0xFF)
 */
static uint8_t NRF24L01_SPI_SwapByte(uint8_t byte) {
  uint8_t rx = NRF24L01_NOP;

  if (HAL_SPI_TransmitReceive(&hspi3, &byte, &rx, 1U,
                              NRF24L01_SPI_TIMEOUT_MS) != HAL_OK) {
    return NRF24L01_NOP;
  }

  return rx;
}

/*------------------------ 寄存器读写函数 ------------------------*/

/**
 * @brief 读单个寄存器
 * @param reg_address 寄存器地址
 * @return uint8_t 寄存器值
 */
uint8_t NRF24L01_ReadReg(uint8_t reg_address) {
  uint8_t data;

  NRF24L01_W_CSN(0U);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | reg_address);
  data = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
  NRF24L01_W_CSN(1U);

  return data;
}

/**
 * @brief 连续读取多个寄存器（或同一寄存器的多个字节）
 * @param reg_address 起始寄存器地址
 * @param data_array 存放数据的数组
 * @param count 读取字节数
 * @retval 无
 */
void NRF24L01_ReadRegs(uint8_t reg_address, uint8_t *data_array,
                       uint8_t count) {
  NRF24L01_W_CSN(0U);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | reg_address);
  for (uint8_t i = 0U; i < count; i++) {
    data_array[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
  }
  NRF24L01_W_CSN(1U);
}

/**
 * @brief 写单个寄存器
 * @param reg_address 寄存器地址
 * @param data 待写入的值
 * @retval 无
 */
void NRF24L01_WriteReg(uint8_t reg_address, uint8_t data) {
  NRF24L01_W_CSN(0U);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | reg_address);
  (void)NRF24L01_SPI_SwapByte(data);
  NRF24L01_W_CSN(1U);
}

/**
 * @brief 连续写入多个寄存器（或同一寄存器的多个字节）
 * @param reg_address 起始寄存器地址
 * @param data_array 待写入的数据数组
 * @param count 写入字节数
 * @retval 无
 */
void NRF24L01_WriteRegs(uint8_t reg_address, const uint8_t *data_array,
                        uint8_t count) {
  NRF24L01_W_CSN(0U);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | reg_address);
  for (uint8_t i = 0U; i < count; i++) {
    (void)NRF24L01_SPI_SwapByte(data_array[i]);
  }
  NRF24L01_W_CSN(1U);
}

/*------------------------ 数据载荷操作函数 ------------------------*/

/**
 * @brief 读取接收FIFO中的载荷数据
 * @param data_array 存放数据的数组
 * @param count 读取字节数（应与数据包宽度一致）
 * @retval 无
 */
void NRF24L01_ReadRxPayload(uint8_t *data_array, uint8_t count) {
  NRF24L01_W_CSN(0U);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_R_RX_PAYLOAD);
  for (uint8_t i = 0U; i < count; i++) {
    data_array[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
  }
  NRF24L01_W_CSN(1U);
}

/**
 * @brief 向发送FIFO写入载荷数据
 * @param data_array 待发送的数据数组
 * @param count 写入字节数（应与数据包宽度一致）
 * @retval 无
 */
void NRF24L01_WriteTxPayload(const uint8_t *data_array, uint8_t count) {
  NRF24L01_W_CSN(0U);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_W_TX_PAYLOAD);
  for (uint8_t i = 0U; i < count; i++) {
    (void)NRF24L01_SPI_SwapByte(data_array[i]);
  }
  NRF24L01_W_CSN(1U);
}

/*------------------------ FIFO 控制函数 ------------------------*/

/**
 * @brief 清空发送FIFO
 * @retval 无
 */
void NRF24L01_FlushTx(void) {
  NRF24L01_W_CSN(0U);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_TX);
  NRF24L01_W_CSN(1U);
}

/**
 * @brief 清空接收FIFO
 * @retval 无
 */
void NRF24L01_FlushRx(void) {
  NRF24L01_W_CSN(0U);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_RX);
  NRF24L01_W_CSN(1U);
}

/**
 * @brief 读取FIFO状态寄存器
 * @return uint8_t FIFO_STATUS寄存器值
 * @note 可获取TX_FULL, TX_EMPTY, RX_EMPTY等标志
 */
uint8_t NRF24L01_ReadFifoStatus(void) {
  return NRF24L01_ReadReg(NRF24L01_FIFO_STATUS);
}

/**
 * @brief 读取状态寄存器（通过NOP指令）
 * @return uint8_t STATUS寄存器值
 */
uint8_t NRF24L01_ReadStatus(void) {
  uint8_t status;

  NRF24L01_W_CSN(0U);
  status = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
  NRF24L01_W_CSN(1U);

  return status;
}

/*------------------------ 模式切换函数 ------------------------*/

/**
 * @brief 进入掉电模式（功耗最低）
 * @retval 无
 * @note 设置CONFIG寄存器的PWR_UP位为0，并拉低CE
 */
void NRF24L01_PowerDown(void) {
  uint8_t config;

  NRF24L01_W_CE(0U);
  config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  if (config == 0xFFU) /* 读取失败则返回 */
  {
    return;
  }

  config &= (uint8_t)~0x02U; /* 清除PWR_UP位 */
  NRF24L01_WriteReg(NRF24L01_CONFIG, config);
}

/**
 * @brief 进入待机-I模式（PWR_UP=1，CE=0）
 * @retval 无
 * @note 适合快速切换到发送或接收模式
 */
void NRF24L01_StandbyI(void) {
  uint8_t config;

  NRF24L01_W_CE(0U);
  config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  if (config == 0xFFU) {
    return;
  }

  config |= 0x02U; /* 置位PWR_UP */
  NRF24L01_WriteReg(NRF24L01_CONFIG, config);
}

/**
 * @brief 设置为接收模式（PRIM_RX=1, PWR_UP=1, CE=1）
 * @retval 无
 */
void NRF24L01_Rx(void) {
  uint8_t config;

  NRF24L01_W_CE(0U);
  config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  if (config == 0xFFU) {
    return;
  }

  config |= 0x03U; /* 置位PWR_UP和PRIM_RX */
  NRF24L01_WriteReg(NRF24L01_CONFIG, config);
  NRF24L01_W_CE(1U);
}

/**
 * @brief 设置为发送模式（PRIM_RX=0, PWR_UP=1, CE=1）
 * @retval 无
 */
void NRF24L01_Tx(void) {
  uint8_t config;

  NRF24L01_W_CE(0U);
  config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  if (config == 0xFFU) {
    return;
  }

  config |= 0x02U;           /* 置位PWR_UP */
  config &= (uint8_t)~0x01U; /* 清除PRIM_RX位 */
  NRF24L01_WriteReg(NRF24L01_CONFIG, config);
  NRF24L01_W_CE(1U);
}

/*------------------------ 初始化和核心收发函数 ------------------------*/

/**
 * @brief 初始化NRF24L01模块
 * @retval 无
 * @note   -
 * 设置常用寄存器：自动应答、接收通道、地址宽度、重发参数、射频通道、数据速率、功率等
 *         - 设置接收地址（P0通道），并使能接收模式
 *         - 清空FIFO，清除中断标志
 */
void NRF24L01_Init(void) {
  nrf24l01_tx_active = 0U;

  NRF24L01_W_CE(0U);
  NRF24L01_W_CSN(1U);

  /* 配置寄存器：无CRC（0x08），PWR_UP和PRIM_RX后续再设 */
  NRF24L01_WriteReg(NRF24L01_CONFIG, 0x08U);
  /* 使能所有通道的自动应答 */
  NRF24L01_WriteReg(NRF24L01_EN_AA, 0x3FU);
  /* 使能接收通道0 */
  NRF24L01_WriteReg(NRF24L01_EN_RXADDR, 0x01U);
  /* 地址宽度：5字节 */
  NRF24L01_WriteReg(NRF24L01_SETUP_AW, 0x03U);
  /* 重发延时250us+86us，重发次数15次（0x2F = 0010 1111） */
  NRF24L01_WriteReg(NRF24L01_SETUP_RETR, 0x2FU);
  /* 射频通道：2（2402MHz） */
  NRF24L01_WriteReg(NRF24L01_RF_CH, 0x02U);
  /* RF_SETUP：0x0E = 数据速率2Mbps，发射功率0dBm，低噪声放大器增益 */
  NRF24L01_WriteReg(NRF24L01_RF_SETUP, 0x0EU);
  /* 接收通道0数据宽度 */
  NRF24L01_WriteReg(NRF24L01_RX_PW_P0, NRF24L01_PACKET_WIDTH);
  /* 设置接收地址 */
  NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5U);

  NRF24L01_FlushTx();
  NRF24L01_FlushRx();
  /* 清空所有中断标志（TX_DS, MAX_RT, RX_DR） */
  NRF24L01_WriteReg(NRF24L01_STATUS, 0x70U);
  /* 进入接收模式 */
  NRF24L01_Rx();
}

/**
 * @brief 发送数据包（非阻塞启动，需循环调用检查结果）
 * @return uint8_t 发送结果
 *         - 若未发送完成（发送进行中）：返回 0
 *         - 若发送完成并成功收到ACK：返回 1
 *         - 若达到最大重发次数（MAX_RT）：返回 2
 *         - 若同时出现TX_DS和MAX_RT（异常）：返回 3
 * @note  - 第一次调用时，将发送缓冲区（NRF24L01_TxPacket）写入FIFO并启动发送
 *         - 后续调用检查状态寄存器，直到发送完成或超时重发
 *         - 发送完成后自动切换回接收模式
 */
uint8_t NRF24L01_Send(void) {
  uint8_t status;
  uint8_t send_flag;

  /* 若发送正在进行，则检查状态 */
  if (nrf24l01_tx_active != 0U) {
    status = NRF24L01_ReadStatus();

    /* 判断中断标志位 */
    if ((status & 0x30U) == 0x30U) /* 同时有TX_DS和MAX_RT */
    {
      send_flag = 3U;
    } else if ((status & 0x10U) == 0x10U) /* MAX_RT（达到最大重发次数） */
    {
      send_flag = 2U;
    } else if ((status & 0x20U) == 0x20U) /* TX_DS（发送成功） */
    {
      send_flag = 1U;
    } else {
      return 0U; /* 还未完成 */
    }

    /* 清除中断标志，清空发送FIFO，切换回接收模式 */
    NRF24L01_WriteReg(NRF24L01_STATUS, 0x30U);
    NRF24L01_FlushTx();
    NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5U);
    NRF24L01_Rx();
    nrf24l01_tx_active = 0U;
    return send_flag;
  }

  /* 第一次调用：准备发送 */
  NRF24L01_WriteRegs(NRF24L01_TX_ADDR, NRF24L01_TxAddress, 5U);
  NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_TxAddress, 5U);
  NRF24L01_WriteTxPayload(NRF24L01_TxPacket, NRF24L01_PACKET_WIDTH);
  NRF24L01_Tx(); /* 切换到发送模式，CE拉高启动发送 */

  nrf24l01_tx_active = 1U;
  return 0U;
}

/**
 * @brief 接收数据包（检查是否有数据到达）
 * @return uint8_t 接收结果
 *         - 0：无数据可读
 *         - 1：成功接收一包数据（内容在 NRF24L01_RxPacket 中）
 *         - 2：FIFO溢出或状态异常，已执行恢复（初始化）
 *         - 3：当前不在接收模式，已执行恢复（初始化）
 * @note  该函数会检查状态寄存器和FIFO状态，若收到数据则读取并清除中断标志
 */
uint8_t NRF24L01_Receive(void) {
  uint8_t status = NRF24L01_ReadStatus();
  uint8_t config = NRF24L01_ReadReg(NRF24L01_CONFIG);

  /* 检查是否处于接收模式（PRIM_RX=1） */
  if ((config & 0x02U) == 0x00U) /* PWR_UP位为0，模块未上电 */
  {
    NRF24L01_Init(); /* 重新初始化 */
    return 3U;
  }

  /* 检查是否同时出现TX_DS和MAX_RT（异常状态） */
  if ((status & 0x30U) == 0x30U) {
    NRF24L01_Init(); /* 重新初始化 */
    return 2U;
  }

  /* 检测RX_DR中断标志（数据到达）或FIFO非空 */
  if (((status & 0x40U) == 0x40U) || ((NRF24L01_ReadFifoStatus() & 0x01U) ==
                                      0U)) /* RX_EMPTY位为0表示FIFO非空 */
  {
    NRF24L01_ReadRxPayload(NRF24L01_RxPacket, NRF24L01_PACKET_WIDTH);
    NRF24L01_WriteReg(NRF24L01_STATUS, 0x40U); /* 清除RX_DR中断标志 */
    return 1U;
  }

  return 0U;
}

/**
 * @brief 更新接收通道0的地址（当更改 NRF24L01_RxAddress 后调用）
 * @retval 无
 * @note  将全局变量 NRF24L01_RxAddress 写入寄存器
 */
void NRF24L01_UpdateRxAddress(void) {
  NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5U);
}