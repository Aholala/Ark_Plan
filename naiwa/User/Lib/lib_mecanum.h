/**
 * @file lib_mecanum.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 麦克纳姆轮底盘开环运动学混控库 - 头文件
 * @version 1.0
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块提供四轮麦克纳姆轮底盘的运动学解算接口，
 *          将平移（X/Y）和旋转（Yaw）指令映射为四个电机的目标占空比。
 *          输出结果经过等比例缩放防止饱和，确保各轮输出不超限。
 */

#ifndef __LIB_MECANUM_H
#define __LIB_MECANUM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 麦克纳姆轮混控输出结构体
 * @note 包含四个电机的目标占空比，范围通常为 -max_output ~ +max_output
 */
typedef struct {
  int8_t m1; /**< 电机1（左前）占空比 */
  int8_t m2; /**< 电机2（右前）占空比 */
  int8_t m3; /**< 电机3（左后）占空比 */
  int8_t m4; /**< 电机4（右后）占空比 */
} LibMecanumOutput_t;

/**
 * @brief 麦克纳姆轮运动学混控函数
 *
 * @param x           横向平移分量（-100~100，正值为右移）
 * @param y           纵向平移分量（-100~100，正值为前进）
 * @param yaw         旋转分量（-100~100，正值为顺时针旋转）
 * @param max_output  输出占空比限幅值（绝对值，0~100，负值会取反）
 * @param output      输出结构体指针，存储四个电机的目标占空比
 *
 * @note 输入值均视为有符号占空比，建议调用方预先进行死区处理。
 * @note 若混控值最大绝对值超过100，则将所有轮子等比例缩放至 max_output 以内；
 *       否则直接按 max_output 进行比例映射。
 * @note 电机符号约定（底盘俯视图，前进方向为+Y）：
 *       - 前进：M1(-), M2(+), M3(-), M4(+)
 *       - 右平移：M1(+), M2(+), M3(+), M4(+)
 *       - 顺时针旋转：M1(-), M2(-), M3(+), M4(+)
 * @warning 若 output 指针为 NULL，函数直接返回。
 */
void Lib_Mecanum_Mix(int8_t x, int8_t y, int8_t yaw, int8_t max_output,
                     LibMecanumOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif /* __LIB_MECANUM_H */