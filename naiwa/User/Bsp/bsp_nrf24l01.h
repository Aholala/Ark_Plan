/**
 * @file bsp_nrf24l01.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）NRF24L01无线模块驱动头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件声明了NRF24L01+无线收发模块的底层驱动接口。
 * 提供寄存器读写、数据包收发、模式切换、初始化等完整功能。
 * 数据包宽度固定为 NRF24L01_PACKET_WIDTH（32字节）。
 */

#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "main.h"                   /* HAL库及硬件定义 */
#include "bsp_nrf24l01_define.h"    /* 寄存器地址及指令宏定义 */

/**
 * @brief 数据包宽度（固定32字节，符合NRF24L01+最大载荷长度）
 */
#define NRF24L01_PACKET_WIDTH 32U

/*------------------------ 外部全局变量（用户可操作） ------------------------*/

/**
 * @brief 发送地址（5字节）
 * @note  默认地址为 {'n','a','i','w','a'}，用户可修改
 */
extern uint8_t NRF24L01_TxAddress[5];

/**
 * @brief 发送数据包缓冲区（长度为 NRF24L01_PACKET_WIDTH）
 * @note  发送前将待发送数据填入该缓冲区，然后调用 NRF24L01_Send()
 */
extern uint8_t NRF24L01_TxPacket[NRF24L01_PACKET_WIDTH];

/**
 * @brief 接收地址（5字节）
 * @note  默认地址为 {'n','a','i','w','a'}，用户可修改
 *        修改后需调用 NRF24L01_UpdateRxAddress() 使生效
 */
extern uint8_t NRF24L01_RxAddress[5];

/**
 * @brief 接收数据包缓冲区（长度为 NRF24L01_PACKET_WIDTH）
 * @note  调用 NRF24L01_Receive() 成功接收后，数据存放于此
 */
extern uint8_t NRF24L01_RxPacket[NRF24L01_PACKET_WIDTH];

/*------------------------ 寄存器读写函数 ------------------------*/

/**
 * @brief 读单个寄存器
 * @param reg_address 寄存器地址（如 NRF24L01_CONFIG）
 * @return uint8_t 寄存器值
 */
uint8_t NRF24L01_ReadReg(uint8_t reg_address);

/**
 * @brief 连续读取多个寄存器（或同一寄存器的多个字节）
 * @param reg_address 起始寄存器地址
 * @param data_array 存放数据的数组
 * @param count 读取字节数
 */
void NRF24L01_ReadRegs(uint8_t reg_address, uint8_t *data_array,
                       uint8_t count);

/**
 * @brief 写单个寄存器
 * @param reg_address 寄存器地址（如 NRF24L01_CONFIG）
 * @param data 待写入的值
 */
void NRF24L01_WriteReg(uint8_t reg_address, uint8_t data);

/**
 * @brief 连续写入多个寄存器（或同一寄存器的多个字节）
 * @param reg_address 起始寄存器地址
 * @param data_array 待写入的数据数组
 * @param count 写入字节数
 */
void NRF24L01_WriteRegs(uint8_t reg_address, const uint8_t *data_array,
                        uint8_t count);

/*------------------------ FIFO 操作函数 ------------------------*/

/**
 * @brief 读取接收FIFO中的载荷数据
 * @param data_array 存放数据的数组
 * @param count 读取字节数（通常为 NRF24L01_PACKET_WIDTH）
 */
void NRF24L01_ReadRxPayload(uint8_t *data_array, uint8_t count);

/**
 * @brief 向发送FIFO写入载荷数据
 * @param data_array 待发送的数据数组
 * @param count 写入字节数（通常为 NRF24L01_PACKET_WIDTH）
 */
void NRF24L01_WriteTxPayload(const uint8_t *data_array, uint8_t count);

/**
 * @brief 清空发送FIFO
 */
void NRF24L01_FlushTx(void);

/**
 * @brief 清空接收FIFO
 */
void NRF24L01_FlushRx(void);

/**
 * @brief 读取FIFO状态寄存器
 * @return uint8_t FIFO_STATUS寄存器值
 * @note  可获取 TX_FULL, TX_EMPTY, RX_EMPTY 等标志位
 */
uint8_t NRF24L01_ReadFifoStatus(void);

/**
 * @brief 读取状态寄存器（通过NOP指令）
 * @return uint8_t STATUS寄存器值
 * @note  包含 TX_DS, MAX_RT, RX_DR 等中断标志
 */
uint8_t NRF24L01_ReadStatus(void);

/*------------------------ 模式切换函数 ------------------------*/

/**
 * @brief 进入掉电模式（功耗最低，PWR_UP=0，CE=0）
 */
void NRF24L01_PowerDown(void);

/**
 * @brief 进入待机-I模式（PWR_UP=1，CE=0）
 * @note  适合快速切换到发送或接收模式
 */
void NRF24L01_StandbyI(void);

/**
 * @brief 设置为接收模式（PRIM_RX=1, PWR_UP=1, CE=1）
 */
void NRF24L01_Rx(void);

/**
 * @brief 设置为发送模式（PRIM_RX=0, PWR_UP=1, CE=1）
 */
void NRF24L01_Tx(void);

/*------------------------ 核心收发函数 ------------------------*/

/**
 * @brief 初始化NRF24L01模块
 * @note  配置寄存器、设置地址、清空FIFO、进入接收模式
 */
void NRF24L01_Init(void);

/**
 * @brief 发送数据包（非阻塞启动，需循环调用检查结果）
 * @return uint8_t 发送结果
 *         - 0：发送进行中（未完成）
 *         - 1：发送成功并收到ACK
 *         - 2：达到最大重发次数（MAX_RT）
 *         - 3：同时出现TX_DS和MAX_RT（异常）
 * @note  第一次调用时启动发送，后续调用检查状态直到完成。
 *        发送完成后自动切换回接收模式。
 */
uint8_t NRF24L01_Send(void);

/**
 * @brief 接收数据包（检查是否有数据到达）
 * @return uint8_t 接收结果
 *         - 0：无数据可读
 *         - 1：成功接收一包数据（内容在 NRF24L01_RxPacket 中）
 *         - 2：FIFO溢出或状态异常，已执行恢复（重新初始化）
 *         - 3：当前不在接收模式，已执行恢复（重新初始化）
 */
uint8_t NRF24L01_Receive(void);

/**
 * @brief 更新接收通道0的地址
 * @note  当修改 NRF24L01_RxAddress 后调用此函数使地址生效
 */
void NRF24L01_UpdateRxAddress(void);

#endif /* __NRF24L01_H */