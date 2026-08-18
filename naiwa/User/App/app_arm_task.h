/**
 * @file app_arm_task.h
 * @author 梁源锋
 * @brief 双轴连杆机械臂控制任务 - 头文件（四杆机构版本）
 * @version 2.0
 * @date 2026-07-10
 *
 * @copyright Copyright (c) 2026
 *
 * @details 机械结构：
 *          M5 → 直驱 → 大臂(380mm) → 肘关节 → 小臂(270mm) → 末端
 *          M6 → 直驱 → 曲柄(90mm) → 连杆(380mm) → 小臂附接点(距肘90mm)
 *
 *          这是一个平面四杆机构驱动的串联机械臂。
 *          M5 控制大臂角度 θ₅（绝对，从Y轴），
 *          M6 控制曲柄角度 θ_crank（绝对，从Y轴），
 *          小臂相对角度 θ_rel 由四杆机构非线性确定。
 *
 *          角度定义（侧视图，Y轴向上，X轴向前）：
 *          - θ₅ = 0°：大臂竖直向上
 *            正方向(+)：大臂向前放下（逆时针）
 *          - θ_rel = 0°：小臂与大臂共线（直臂，总长650mm）
 *            正方向(+)：小臂向前展开
 *          - θ_crank：M6曲柄的绝对角度（从Y轴）
 *            θ_rel=0° 且 θ₅=0° 时，θ_crank=0°
 *
 *          机械限位：
 *          - M5大臂：θ₅ ∈ [-17.41°, 57.35°]
 *          - M6小臂绝对角：θ₅ + θ_rel ≤ 58.22°（固定限位，Y轴顺时针最大）
 */

#ifndef __APP_ARM_TASK_H
#define __APP_ARM_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "module_motor.h"
#include <stdint.h>

/**
 * @brief 机械臂状态信息（监控/调试用）
 */
typedef struct {
  float m5_angle_deg;   /**< 大臂当前角度θ₅（度，从Y轴） */
  float m6_angle_deg;   /**< 小臂当前相对角度θ_rel（度，相对大臂） */
  float m5_target_deg;  /**< 大臂目标角度θ₅（度） */
  float m6_target_deg;  /**< 小臂目标相对角度θ_rel（度） */
  float target_x_mm;    /**< 目标末端X坐标（mm） */
  float target_z_mm;    /**< 目标末端Z坐标（mm） */
  float actual_x_mm;    /**< 当前末端X坐标（mm，正运动学计算） */
  float actual_z_mm;    /**< 当前末端Z坐标（mm，正运动学计算） */
  int8_t m5_duty;       /**< M5当前输出PWM占空比 */
  int8_t m6_duty;       /**< M6当前输出PWM占空比 */
  uint8_t ik_valid;     /**< 逆运动学是否可达 */
} AppArmState_t;

void App_Arm_Init(void);
void App_Arm_Task(void);
const AppArmState_t *App_Arm_GetState(void);
void App_Arm_EmergencyStop(void);
void StartarmTask(void *argument);
uint8_t App_Arm_IsControlActive(void);

/**
 * @brief 校准编码器零位
 * @param theta5_known    当前大臂实际角度θ₅（度，从Y轴）
 * @param theta6_rel_known 当前小臂相对角度θ_rel（度）
 * @note  将机械臂手动摆到已知位姿后调用。
 *        例如竖直位：App_Arm_Calibrate(0.0f, 62.9f)
 */
void App_Arm_Calibrate(float theta5_known, float theta6_rel_known);

/**
 * @brief 设置编码器计数方向
 * @param m5_dir +1.0f=count++对应角度++（默认）, -1.0f=反转
 * @param m6_dir 同上
 */
void App_Arm_SetEncoderDir(float m5_dir, float m6_dir);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ARM_TASK_H */
