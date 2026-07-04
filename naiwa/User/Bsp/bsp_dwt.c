/**
 * @file bsp_dwt.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包 - DWT周期计数器定时辅助函数
 * @version 1.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块基于ARM Cortex-M内核的DWT（Data Watchpoint and Trace）单元，
 *          提供高精度的时间测量和延时功能。
 *          支持获取CPU周期数、微秒/毫秒延时、时间点记录及时间差判断。
 *          所有函数内部均会自动确保DWT已使能，无需用户显式初始化。
 *
 * @note DWT计数器在调试模式下可能受断点影响，实际产品中需注意。
 */

#include "bsp_dwt.h"

/**
 * @brief 获取CPU内核时钟频率（Hz）
 *
 * @return uint32_t 系统内核时钟频率（通常为 SystemCoreClock 变量）
 * @note 该值由系统时钟配置决定，在启动时由CMSIS设置。
 */
uint32_t Bsp_Dwt_GetCoreClockHz(void) { return SystemCoreClock; }

/**
 * @brief 获取每微秒对应的CPU周期数
 *
 * @return uint32_t 每微秒的周期数（最小为1）
 * @note 通过 SystemCoreClock / 1,000,000 计算，若为0则强制为1，避免除零错误。
 */
uint32_t Bsp_Dwt_GetCyclesPerUs(void) {
  uint32_t cycles_per_us = SystemCoreClock / 1000000U;

  if (cycles_per_us == 0U) {
    cycles_per_us = 1U;
  }

  return cycles_per_us;
}

/**
 * @brief 确保DWT计数器已使能（内部函数）
 * @note 若未使能则调用 Bsp_Dwt_Init() 初始化。
 */
static void Bsp_Dwt_EnsureEnabled(void) {
  if (Bsp_Dwt_IsEnabled() == 0U) {
    Bsp_Dwt_Init();
  }
}

/**
 * @brief 初始化DWT周期计数器
 * @note 使能DWT跟踪单元，清零周期计数器，并使能计数器。
 * @warning 该函数会直接操作CoreDebug和DWT寄存器，需在特权模式下调用。
 */
void Bsp_Dwt_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 使能DWT跟踪单元 */
  DWT->CYCCNT = 0U;                               /* 清零计数器 */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            /* 使能周期计数 */
}

/**
 * @brief 检查DWT计数器是否已使能
 *
 * @return uint8_t 使能状态
 * @retval 1 已使能
 * @retval 0 未使能
 */
uint8_t Bsp_Dwt_IsEnabled(void) {
  return ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;
}

/**
 * @brief 复位DWT周期计数器（清零）
 * @note 若计数器未使能，会自动初始化。
 */
void Bsp_Dwt_Reset(void) {
  Bsp_Dwt_EnsureEnabled();
  DWT->CYCCNT = 0U;
}

/**
 * @brief 获取当前DWT周期计数值
 *
 * @return uint32_t 当前周期计数
 * @note 若计数器未使能，会自动初始化。
 * @note 计数器最大值为2^32-1，溢出后归零，使用时需考虑溢出。
 */
uint32_t Bsp_Dwt_GetCycle(void) {
  Bsp_Dwt_EnsureEnabled();
  return DWT->CYCCNT;
}

/**
 * @brief 获取当前时间点（周期计数封装）
 *
 * @return BspDwtTimePoint_t 包含当前周期数的时间点结构体
 * @note 该结构体可用于后续的时间差计算。
 */
BspDwtTimePoint_t Bsp_Dwt_Now(void) {
  BspDwtTimePoint_t now;

  now.cycles = Bsp_Dwt_GetCycle();
  return now;
}

/**
 * @brief 计算从起始周期到当前时刻的已过周期数
 *
 * @param start_cycle 起始周期计数
 * @return uint32_t 已过周期数（当前周期 - 起始周期）
 * @note 若计数器发生溢出，计算结果仍然正确（因为是无符号整数溢出特性）。
 */
uint32_t Bsp_Dwt_ElapsedCycles(uint32_t start_cycle) {
  return (uint32_t)(Bsp_Dwt_GetCycle() - start_cycle);
}

/**
 * @brief 计算从指定时间点开始到当前时刻的已过周期数
 *
 * @param start 起始时间点（由 Bsp_Dwt_Now() 获得）
 * @return uint32_t 已过周期数
 */
uint32_t Bsp_Dwt_ElapsedFrom(BspDwtTimePoint_t start) {
  return Bsp_Dwt_ElapsedCycles(start.cycles);
}

/**
 * @brief 将周期数转换为微秒数
 *
 * @param cycles 周期数
 * @return uint32_t 对应的微秒数（向下取整）
 * @note 通过除以每微秒周期数得到。
 */
uint32_t Bsp_Dwt_CyclesToUs(uint32_t cycles) {
  return cycles / Bsp_Dwt_GetCyclesPerUs();
}

/**
 * @brief 将微秒数转换为周期数
 *
 * @param us 微秒数
 * @return uint32_t 对应的周期数
 * @note 若计算结果超出 uint32_t 范围，则返回 UINT32_MAX。
 */
uint32_t Bsp_Dwt_UsToCycles(uint32_t us) {
  uint64_t cycles = (uint64_t)us * (uint64_t)Bsp_Dwt_GetCyclesPerUs();

  if (cycles > UINT32_MAX) {
    return UINT32_MAX;
  }

  return (uint32_t)cycles;
}

/**
 * @brief 微秒级阻塞延时
 *
 * @param us 延时微秒数
 * @note 忙等待实现，延时精度受内核周期数限制。
 * @note 延时过程中CPU被占用，不适合长时间延时。
 */
void Bsp_Dwt_DelayUs(uint32_t us) {
  uint32_t start = Bsp_Dwt_GetCycle();
  uint32_t ticks = Bsp_Dwt_UsToCycles(us);

  while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
    /* 忙等待 */
  }
}

/**
 * @brief 毫秒级阻塞延时
 *
 * @param ms 延时毫秒数
 * @note 通过循环调用 Bsp_Dwt_DelayUs(1000) 实现，适合中短延时。
 * @note 若需长时间延时，建议使用RTOS的延时函数以节省CPU资源。
 */
void Bsp_Dwt_DelayMs(uint32_t ms) {
  while (ms > 0U) {
    Bsp_Dwt_DelayUs(1000U);
    ms--;
  }
}

/**
 * @brief 判断从起始时间点开始是否已过指定的微秒数
 *
 * @param start 起始时间点
 * @param us    目标微秒数
 * @return uint8_t 判断结果
 * @retval 1 已超时（已过时间 >= 目标微秒）
 * @retval 0 尚未超时
 */
uint8_t Bsp_Dwt_HasElapsedUs(BspDwtTimePoint_t start, uint32_t us) {
  return (Bsp_Dwt_ElapsedFrom(start) >= Bsp_Dwt_UsToCycles(us)) ? 1U : 0U;
}

/**
 * @brief 判断从起始时间点开始是否已过指定的毫秒数
 *
 * @param start 起始时间点
 * @param ms    目标毫秒数
 * @return uint8_t 判断结果
 * @retval 1 已超时（已过时间 >= 目标毫秒）
 * @retval 0 尚未超时
 * @note 内部将毫秒转换为微秒，若转换后溢出则取 UINT32_MAX。
 */
uint8_t Bsp_Dwt_HasElapsedMs(BspDwtTimePoint_t start, uint32_t ms) {
  uint64_t us = (uint64_t)ms * 1000ULL;

  if (us > UINT32_MAX) {
    us = UINT32_MAX;
  }

  return Bsp_Dwt_HasElapsedUs(start, (uint32_t)us);
}