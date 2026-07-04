/**
 * @file module_periodic_timer.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块化周期定时器组件头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本模块实现了一个轻量级的软件周期性定时器，适用于任务调度、状态轮询等场景。
 * 定时器通过累加毫秒计数并与设定周期比较，达到周期时触发并自动重置。
 * 该模块不依赖硬件定时器，需外部提供1ms的时间基准（通过 Tick 函数驱动）。
 */

#ifndef __MODULE_PERIODIC_TIMER_H
#define __MODULE_PERIODIC_TIMER_H

#include <stdint.h> /* 标准整数类型定义 */

/**
 * @brief 周期定时器结构体
 * @note  存储定时器的周期和已运行时间
 */
typedef struct {
  uint16_t period_ms;  /**< 定时周期（毫秒） */
  uint16_t elapsed_ms; /**< 从上次触发以来经过的毫秒数（递增至周期值后清零） */
} ModulePeriodicTimer_t;

/**
 * @brief 初始化周期定时器
 * @param timer      指向定时器结构体的指针（不能为NULL）
 * @param period_ms  定时周期（毫秒）
 * @retval 无
 * @note  设置周期值并将已运行时间清零
 */
void Module_PeriodicTimer_Init(ModulePeriodicTimer_t *timer,
                               uint16_t period_ms);

/**
 * @brief 动态修改定时器的周期值
 * @param timer      指向定时器结构体的指针（不能为NULL）
 * @param period_ms  新的定时周期（毫秒）
 * @retval 无
 * @note
 * 若修改后的周期小于或等于当前已运行时间，则将已运行时间清零，防止立即超时
 */
void Module_PeriodicTimer_SetPeriod(ModulePeriodicTimer_t *timer,
                                    uint16_t period_ms);

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
uint8_t Module_PeriodicTimer_Tick(ModulePeriodicTimer_t *timer);

#endif /* __MODULE_PERIODIC_TIMER_H */