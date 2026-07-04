/**
 * @file bsp_dwt.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包 - DWT周期计数器定时辅助头文件
 * @version 1.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块基于ARM Cortex-M内核的DWT（Data Watchpoint and Trace）单元，
 *          提供高精度的时间测量、延时和超时判断功能。
 *          所有函数均自动确保DWT已使能，无需用户显式初始化。
 *          适用于需要微秒级精度的场景，如协议超时、精确延时等。
 */

#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief 时间点结构体
 * @note 存储某一时刻的DWT周期计数值，用于时间差计算
 */
typedef struct {
  uint32_t cycles; /**< 周期计数（DWT->CYCCNT的值） */
} BspDwtTimePoint_t;

/**
 * @brief 初始化DWT周期计数器
 * @note 使能DWT跟踪单元和周期计数器，并清零计数。
 *       通常无需手动调用，模块内部函数会自动使能。
 */
void Bsp_Dwt_Init(void);

/**
 * @brief 检查DWT周期计数器是否已使能
 * @return uint8_t 使能状态
 * @retval 1 已使能
 * @retval 0 未使能
 */
uint8_t Bsp_Dwt_IsEnabled(void);

/**
 * @brief 获取CPU内核时钟频率（Hz）
 * @return uint32_t 系统内核时钟频率（SystemCoreClock）
 */
uint32_t Bsp_Dwt_GetCoreClockHz(void);

/**
 * @brief 获取每微秒对应的CPU周期数
 * @return uint32_t 每微秒的周期数（最小为1）
 * @note 通过 SystemCoreClock / 1,000,000 计算，若结果为0则强制为1。
 */
uint32_t Bsp_Dwt_GetCyclesPerUs(void);

/**
 * @brief 复位DWT周期计数器（清零）
 * @note 若计数器未使能，会自动初始化。
 */
void Bsp_Dwt_Reset(void);

/**
 * @brief 获取当前DWT周期计数值
 * @return uint32_t 当前周期计数（32位，溢出后归零）
 * @note 若计数器未使能，会自动初始化。
 */
uint32_t Bsp_Dwt_GetCycle(void);

/**
 * @brief 捕获当前时刻作为时间点
 * @return BspDwtTimePoint_t 包含当前周期数的时间点结构体
 * @note 该时间点可用于后续 Bsp_Dwt_ElapsedFrom() 等函数进行时间差计算。
 */
BspDwtTimePoint_t Bsp_Dwt_Now(void);

/**
 * @brief 计算从起始周期到当前时刻的已过周期数
 * @param start_cycle 起始周期计数
 * @return uint32_t 已过周期数（当前周期 - 起始周期）
 * @note 即使计数器溢出，由于无符号整数运算特性，结果依然正确。
 */
uint32_t Bsp_Dwt_ElapsedCycles(uint32_t start_cycle);

/**
 * @brief 计算从指定时间点到当前时刻的已过周期数
 * @param start 起始时间点（由 Bsp_Dwt_Now() 获取）
 * @return uint32_t 已过周期数
 */
uint32_t Bsp_Dwt_ElapsedFrom(BspDwtTimePoint_t start);

/**
 * @brief 将周期数转换为微秒数
 * @param cycles 周期数
 * @return uint32_t 对应的微秒数（向下取整）
 */
uint32_t Bsp_Dwt_CyclesToUs(uint32_t cycles);

/**
 * @brief 将微秒数转换为周期数
 * @param us 微秒数
 * @return uint32_t 对应的周期数
 * @note 若计算结果超出 uint32_t 范围，则饱和返回 UINT32_MAX。
 */
uint32_t Bsp_Dwt_UsToCycles(uint32_t us);

/**
 * @brief 微秒级阻塞延时
 * @param us 延时微秒数
 * @note 忙等待实现，占用CPU，适合短延时（毫秒级以下）。
 *       延时精度受CPU周期数限制，实际可能略有偏差。
 */
void Bsp_Dwt_DelayUs(uint32_t us);

/**
 * @brief 毫秒级阻塞延时
 * @param ms 延时毫秒数
 * @note 通过循环调用 Bsp_Dwt_DelayUs(1000) 实现，适合中短延时。
 *       长时间延时建议使用RTOS延时函数。
 */
void Bsp_Dwt_DelayMs(uint32_t ms);

/**
 * @brief 判断从起始时间点开始是否已过指定的微秒数
 * @param start 起始时间点
 * @param us    目标微秒数
 * @return uint8_t 判断结果
 * @retval 1 已超时（已过时间 >= 目标微秒）
 * @retval 0 尚未超时
 */
uint8_t Bsp_Dwt_HasElapsedUs(BspDwtTimePoint_t start, uint32_t us);

/**
 * @brief 判断从起始时间点开始是否已过指定的毫秒数
 * @param start 起始时间点
 * @param ms    目标毫秒数
 * @return uint8_t 判断结果
 * @retval 1 已超时（已过时间 >= 目标毫秒）
 * @retval 0 尚未超时
 * @note 内部将毫秒转换为微秒，若转换溢出则取 UINT32_MAX。
 */
uint8_t Bsp_Dwt_HasElapsedMs(BspDwtTimePoint_t start, uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DWT_H */