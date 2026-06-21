/**
 * @file app_remote.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 遥控器应用层模块头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件声明了遥控器应用的核心接口函数，包括初始化、主任务循环和1ms周期回调。
 * 应用层负责协调ADC、按键、NRF24L01和OLED等底层驱动，实现遥控器功能。
 */

#ifndef APP_REMOTE_H
#define APP_REMOTE_H

#include <stdint.h>

/**
 * @brief 遥控器应用初始化
 * @retval 无
 * @note   - 初始化所有硬件外设（OLED、按键、ADC、NRF24L01、定时器）
 *         - 清空应用上下文，显示欢迎界面
 *         - 进入等待启动状态（按KEY_10启动）
 */
void App_Remote_Init(void);

/**
 * @brief 遥控器应用主任务（需在主循环中周期调用）
 * @retval 无
 * @note   - 检测启动按键，进入运行状态
 *         - 扫描摇杆和按键输入
 *         - 按周期发送控制数据包（通过NRF24L01）
 *         - 更新OLED显示（摇杆值/反馈信息/信号强度等）
 */
void App_Remote_Task(void);

/**
 * @brief 1ms周期回调函数（由定时器中断调用）
 * @retval 无
 * @note   - 调用按键状态机 Key_Tick() 进行按键消抖和事件检测
 *         - 累加发送周期计数器，每100ms置位发送标志，触发数据发送
 *         - 该函数需在1ms定时器中断服务函数中调用
 */
void App_Remote_Tick1ms(void);

#endif /* APP_REMOTE_H */