/**
 * @file bsp_timer.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）定时器驱动源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本文件实现了定时器的初始化，用于提供系统时钟基准或周期性中断。
 * 当前使用TIM1的更新中断，可用于按键扫描、LED闪烁等周期性任务。
 */

#include "bsp_timer.h"
#include "tim.h" /* 包含HAL库生成的定时器句柄（htim1） */

/**
 * @brief 初始化定时器，启动其更新中断
 * @retval 无
 * @note   - 调用HAL_TIM_Base_Start_IT启动TIM1的更新中断
 *         - 定时器的预分频器和自动重载值在CubeMX或ioc中配置
 *         - 中断服务函数 HAL_TIM_PeriodElapsedCallback
 * 需在外部实现，用于处理周期性任务
 *         - 若启动失败，可在外部添加错误处理（本函数未包含）
 */
void Timer_Init(void)
{
  HAL_TIM_Base_Start_IT(&htim1);
}