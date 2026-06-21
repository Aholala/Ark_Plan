/**
 * @file bsp_key.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）按键驱动头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件定义了按键编号、事件标志及外部接口函数。
 * 支持最多12个按键的单击、双击、长按、重复触发等事件检测。
 */

#ifndef __KEY_H
#define __KEY_H

#include "main.h"   /* 包含HAL库及硬件引脚定义 */

/*------------------------ 按键数量及编号定义 ------------------------*/
#define KEY_COUNT    12    /* 按键总数（支持0~11号共12个按键） */

/* 按键编号（用于指定操作哪个按键） */
#define KEY_1        0
#define KEY_2        1
#define KEY_3        2
#define KEY_4        3
#define KEY_5        4
#define KEY_6        5
#define KEY_7        6
#define KEY_8        7
#define KEY_9        8
#define KEY_10       9
#define KEY_11       10
#define KEY_12       11

/*------------------------ 按键事件标志位（可组合） ------------------------*/
#define KEY_HOLD     0x01   /* 按键当前处于按下状态（持续标志，不会自动清除） */
#define KEY_DOWN     0x02   /* 按键按下事件（边沿触发，检查后自动清除） */
#define KEY_UP       0x04   /* 按键释放事件（边沿触发，检查后自动清除） */
#define KEY_SINGLE   0x08   /* 按键单击事件（检查后自动清除） */
#define KEY_DOUBLE   0x10   /* 按键双击事件（检查后自动清除） */
#define KEY_LONG     0x20   /* 按键长按事件（长按时间到触发一次，检查后自动清除） */
#define KEY_REPEAT   0x40   /* 长按后重复触发事件（每隔一段间隔触发，检查后自动清除） */

/*------------------------ 外部接口函数声明 ------------------------*/

/**
 * @brief 按键模块初始化
 * @retval 无
 * @note  内部调用 Key_Clear() 清空所有状态，通常在主循环前调用一次
 */
void Key_Init(void);

/**
 * @brief 检查指定按键是否发生了特定事件
 * @param n    按键编号（0~KEY_COUNT-1，使用 KEY_1 ~ KEY_12 宏）
 * @param Flag 要检查的事件标志（如 KEY_DOWN、KEY_SINGLE 等，可以是组合）
 * @return uint8_t 若该事件已发生则返回1，否则返回0
 * @note  除 KEY_HOLD 外，其他事件标志在检查后会自动清除（避免重复处理）
 *        使用示例：if (Key_Check(KEY_1, KEY_DOWN)) 
 */
uint8_t Key_Check(uint8_t n, uint8_t Flag);

/**
 * @brief 清空所有按键的内部状态和事件标志
 * @retval 无
 * @note  在需要重置按键状态时调用，例如系统复位或模式切换
 */
void Key_Clear(void);

/**
 * @brief 按键状态机周期处理函数（需周期性调用）
 * @retval 无
 * @note  建议每20ms调用一次，内部进行按键采样、消抖、状态更新和事件触发。
 *        该函数必须在主循环或定时器中断中周期调用，以保证实时性。
 */
void Key_Tick(void);

#endif /* __KEY_H */