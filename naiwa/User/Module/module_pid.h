/**
 * @file module_pid.h
 * @author 梁源锋
 * @brief 双环PID控制器模块 - 头文件
 * @version 1.0
 * @date 2026-07-10
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块提供标准位置式PID控制器，支持：
 *          - 比例/积分/微分三路增益独立配置
 *          - 积分分离 + 积分限幅（抗积分饱和）
 *          - 微分项低通滤波（抑制高频噪声）
 *          - 输出限幅（防止执行器饱和）
 *          - 适用于位置环（外环）和速度环（内环）的双环级联控制
 *
 * @note  所有浮点运算运行在STM32F4的硬件FPU上，5ms周期下性能无忧。
 */

#ifndef __MODULE_PID_H
#define __MODULE_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief PID控制器结构体
 * @note  每个控制环（位置/速度）各需一个实例
 */
typedef struct {
  /* ---- 用户可调参数 ---- */
  float kp;                 /**< 比例增益 */
  float ki;                 /**< 积分增益 */
  float kd;                 /**< 微分增益 */
  float integral_limit;     /**< 积分项输出限幅（绝对值） */
  float output_limit;       /**< 总输出限幅（绝对值，如PWM上限95.0） */
  float deriv_filter_alpha; /**< 微分低通滤波系数（0~1，0=无滤波，1=完全滤波） */

  /* ---- 内部状态（只读） ---- */
  float integral;       /**< 积分累加值 */
  float prev_error;     /**< 上一次误差（用于微分计算） */
  float prev_derivative; /**< 上一次微分值（低通滤波用） */
  float prev_output;    /**< 上一次输出 */
  float dt;             /**< 采样周期（秒） */
} ModulePid_t;

/**
 * @brief 初始化PID控制器
 *
 * @param pid               PID结构体指针
 * @param kp                比例增益
 * @param ki                积分增益
 * @param kd                微分增益
 * @param integral_limit    积分项限幅（>0），如 20.0f 表示积分最多贡献±20的输出
 * @param output_limit      总输出限幅（>0），如 95.0f 表示PWM占空比上限
 * @param dt                采样周期（秒），如 0.005f 对应200Hz
 * @param deriv_filter_alpha 微分滤波系数（0~1），建议 0.1~0.3
 * @note  所有内部状态清零
 */
void Module_Pid_Init(ModulePid_t *pid, float kp, float ki, float kd,
                     float integral_limit, float output_limit, float dt,
                     float deriv_filter_alpha);

/**
 * @brief 重置PID控制器（清零积分和误差缓存）
 *
 * @param pid PID结构体指针
 * @note  适用于模式切换、目标突变或电机停止时，防止历史积分干扰
 */
void Module_Pid_Reset(ModulePid_t *pid);

/**
 * @brief PID单步更新（核心计算）
 *
 * @param pid          PID结构体指针
 * @param setpoint     目标值（期望值）
 * @param measurement  实际测量值（反馈值）
 * @return float 控制输出（已限幅）
 *
 * @note  计算顺序：
 *        1. 误差 = setpoint - measurement
 *        2. 比例项 = Kp × 误差
 *        3. 积分项 = 上次积分 + Ki × 误差 × dt（带限幅和抗饱和）
 *        4. 微分项 = Kd × (误差 - 上次误差) / dt（带低通滤波）
 *        5. 输出 = clamp(P + I + D, ±output_limit)
 *        6. 抗饱和：输出饱和时不累加同向积分
 */
float Module_Pid_Update(ModulePid_t *pid, float setpoint, float measurement);

/**
 * @brief 在线修改PID增益（不重置积分）
 *
 * @param pid PID结构体指针
 * @param kp  新的比例增益
 * @param ki  新的积分增益
 * @param kd  新的微分增益
 * @note  仅更新增益系数，不清零内部状态。适合在线调参。
 */
void Module_Pid_SetGains(ModulePid_t *pid, float kp, float ki, float kd);

/**
 * @brief 获取PID上一次输出值
 *
 * @param pid PID结构体指针
 * @return float 上一次输出值
 */
float Module_Pid_GetPrevOutput(const ModulePid_t *pid);

/**
 * @brief 获取PID积分项当前值
 *
 * @param pid PID结构体指针
 * @return float 当前积分累加值
 */
float Module_Pid_GetIntegral(const ModulePid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_PID_H */
