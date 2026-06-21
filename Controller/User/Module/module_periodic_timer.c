/**
 * @file module_periodic_timer.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块化周期定时器组件源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本模块实现了一个简单的软件周期性定时器，可用于任务调度、状态轮询等场景。
 * 定时器通过累加毫秒计数（elapsed_ms）与设定的周期（period_ms）比较，
 * 当达到周期值时返回触发标志，并自动重置计数器。
 * 该模块不依赖具体硬件，需外部提供1ms的时间基准驱动（由上层调用 Tick 函数）。
 */

#include "module_periodic_timer.h"

/**
 * @brief 初始化周期定时器
 * @param timer      指向定时器结构体的指针（不能为NULL）
 * @param period_ms  定时周期（毫秒）
 * @retval 无
 * @note   - 设置周期值，并将已运行时间清零
 *         - 若 timer 为空指针，函数直接返回不做任何操作
 */
void Module_PeriodicTimer_Init(ModulePeriodicTimer_t *timer,
                               uint16_t period_ms) {
  if (timer == 0) {
    return;
  }

  timer->period_ms = period_ms;
  timer->elapsed_ms = 0U;
}

/**
 * @brief 动态修改定时器的周期值
 * @param timer      指向定时器结构体的指针（不能为NULL）
 * @param period_ms  新的定时周期（毫秒）
 * @retval 无
 * @note   -
 * 若修改后的周期小于或等于当前已运行时间，则将已运行时间清零，防止立即超时
 *         - 若 timer 为空指针，函数直接返回
 */
void Module_PeriodicTimer_SetPeriod(ModulePeriodicTimer_t *timer,
                                    uint16_t period_ms) {
  if (timer == 0) {
    return;
  }

  timer->period_ms = period_ms;
  /* 防止已运行时间超过新周期导致立即触发 */
  if (timer->elapsed_ms >= period_ms) {
    timer->elapsed_ms = 0U;
  }
}

/**
 * @brief 定时器滴答驱动函数（需以1ms周期调用）
 * @param timer 指向定时器结构体的指针（不能为NULL）
 * @return uint8_t 定时是否到达
 *         - 1：定时到达（周期已满），计数器自动重置
 *         - 0：定时尚未到达，或 timer 无效/周期为0
 * @note   - 该函数每次调用使 elapsed_ms 递增1
 *         - 当 elapsed_ms 达到 period_ms 时，返回1并自动清零，开始下一轮计时
 *         - 若 period_ms 为0，定时器无效，始终返回0
 *         -
 * 典型用法：在1ms硬件定时器中断或循环中调用此函数，根据返回值执行周期性任务
 */
uint8_t Module_PeriodicTimer_Tick(ModulePeriodicTimer_t *timer) {
  if ((timer == 0) || (timer->period_ms == 0U)) {
    return 0U;
  }

  timer->elapsed_ms++;
  if (timer->elapsed_ms < timer->period_ms) {
    return 0U;
  }

  timer->elapsed_ms = 0U;
  return 1U;
}