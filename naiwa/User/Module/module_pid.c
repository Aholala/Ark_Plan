/**
 * @file module_pid.c
 * @author 梁源锋
 * @brief 双环PID控制器模块 - 实现
 * @version 1.0
 * @date 2026-07-10
 *
 * @copyright Copyright (c) 2026
 *
 * @details 标准位置式PID算法实现，包含抗积分饱和与微分低通滤波。
 *          适用于位置环（外环）→速度环（内环）的双环级联控制架构。
 */

#include "module_pid.h"
#include <math.h>  /* fabsf */

/*==================== 全局API函数 ====================*/

/**
 * @brief 初始化PID控制器
 */
void Module_Pid_Init(ModulePid_t *pid, float kp, float ki, float kd,
                     float integral_limit, float output_limit, float dt,
                     float deriv_filter_alpha) {
  if (pid == 0) {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral_limit = integral_limit;
  pid->output_limit = output_limit;
  pid->deriv_filter_alpha = deriv_filter_alpha;
  pid->dt = dt;

  /* 内部状态清零 */
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->prev_derivative = 0.0f;
  pid->prev_output = 0.0f;
}

/**
 * @brief 重置PID控制器内部状态
 */
void Module_Pid_Reset(ModulePid_t *pid) {
  if (pid == 0) {
    return;
  }

  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->prev_derivative = 0.0f;
  pid->prev_output = 0.0f;
}

/**
 * @brief PID单步更新（核心计算）
 *
 * @note 算法特点：
 *       - 比例项：直接比例放大误差，提供基本响应速度
 *       - 积分项：梯形积分（误差平均），带限幅抗饱和。
 *                 当输出已饱和且误差与输出同号时，停止积分累加。
 *       - 微分项：差分计算 + 一阶低通滤波，抑制编码器量化噪声
 *       - 输出限幅：最终输出钳位在 ±output_limit 内
 */
float Module_Pid_Update(ModulePid_t *pid, float setpoint, float measurement) {
  float error;
  float p_term, i_term, d_term;
  float raw_derivative;
  float output;

  if (pid == 0) {
    return 0.0f;
  }

  /* 1. 计算误差 */
  error = setpoint - measurement;

  /* 2. 比例项 */
  p_term = pid->kp * error;

  /* 3. 积分项（梯形积分 + 限幅 + 抗饱和）*/
  if (pid->ki != 0.0f && pid->dt > 0.0f) {
    /* 抗饱和：仅当上一轮输出未饱和 或 误差方向有助于退出饱和时才积分 */
    float prev_abs = (pid->prev_output > 0.0f) ? pid->prev_output
                                                : -pid->prev_output;
    uint8_t saturated = (prev_abs >= pid->output_limit - 0.01f);
    uint8_t same_sign =
        ((error > 0.0f) && (pid->prev_output > 0.0f)) ||
        ((error < 0.0f) && (pid->prev_output < 0.0f));

    if (!saturated || !same_sign) {
      /* 梯形积分：(error + prev_error) / 2 * dt */
      pid->integral += pid->ki * (error + pid->prev_error) * 0.5f * pid->dt;
    }

    /* 积分限幅 */
    if (pid->integral > pid->integral_limit) {
      pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
      pid->integral = -pid->integral_limit;
    }
  }
  i_term = pid->integral;

  /* 4. 微分项（带低通滤波）*/
  if (pid->kd != 0.0f && pid->dt > 0.0f) {
    /* 原始微分 = (error - prev_error) / dt */
    raw_derivative = (error - pid->prev_error) / pid->dt;

    /* 一阶低通滤波：filtered = alpha * raw + (1-alpha) * prev_filtered */
    d_term = pid->deriv_filter_alpha * raw_derivative +
             (1.0f - pid->deriv_filter_alpha) * pid->prev_derivative;
    pid->prev_derivative = d_term;

    d_term *= pid->kd;
  } else {
    d_term = 0.0f;
  }

  /* 5. 合并输出并限幅 */
  output = p_term + i_term + d_term;

  if (output > pid->output_limit) {
    output = pid->output_limit;
  } else if (output < -pid->output_limit) {
    output = -pid->output_limit;
  }

  /* 6. 保存状态用于下一周期 */
  pid->prev_error = error;
  pid->prev_output = output;

  return output;
}

/**
 * @brief 在线修改PID增益
 */
void Module_Pid_SetGains(ModulePid_t *pid, float kp, float ki, float kd) {
  if (pid == 0) {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

/**
 * @brief 获取上一次PID输出
 */
float Module_Pid_GetPrevOutput(const ModulePid_t *pid) {
  if (pid == 0) {
    return 0.0f;
  }

  return pid->prev_output;
}

/**
 * @brief 获取积分项当前值
 */
float Module_Pid_GetIntegral(const ModulePid_t *pid) {
  if (pid == 0) {
    return 0.0f;
  }

  return pid->integral;
}
