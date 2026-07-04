/**
 * @file lib_mecanum.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 麦克纳姆轮底盘开环运动学混控库
 * @version 1.0
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本模块实现四轮麦克纳姆轮底盘的运动学解算，将遥控器或控制器的
 *          平移（X/Y）和旋转（Yaw）指令映射为四个电机的目标占空比。
 *          采用开环混控，支持输出限幅和等比例缩放，确保各轮输出不超限。
 *          电机符号约定如下（底盘俯视图，前进方向为+ Y）：
 *          - 前进：M1(-), M2(+), M3(-), M4(+)
 *          - 右平移：M1(+), M2(+), M3(+), M4(+)
 *          - 顺时针旋转：M1(-), M2(-), M3(+), M4(+)
 */

#include "lib_mecanum.h"

/**
 * @brief 获取16位有符号整数的绝对值
 *
 * @param value 输入值
 * @return int16_t 绝对值
 */
static int16_t Lib_Mecanum_Abs16(int16_t value) {
  return (value < 0) ? (int16_t)-value : value;
}

/**
 * @brief 获取四个16位数中绝对值最大的值
 *
 * @param a,b,c,d 四个输入值
 * @return int16_t 最大绝对值
 */
static int16_t Lib_Mecanum_MaxAbs4(int16_t a, int16_t b, int16_t c, int16_t d) {
  int16_t max = Lib_Mecanum_Abs16(a);
  int16_t value = Lib_Mecanum_Abs16(b);

  if (value > max) {
    max = value;
  }
  value = Lib_Mecanum_Abs16(c);
  if (value > max) {
    max = value;
  }
  value = Lib_Mecanum_Abs16(d);
  if (value > max) {
    max = value;
  }

  return max;
}

/**
 * @brief 将原始运动学值缩放到目标占空比范围
 *
 * @param value       原始混控值（可能超出 ±100）
 * @param max_abs     所有轮子中的最大绝对值（用于归一化）
 * @param max_output  输出占空比的最大值（如 98）
 * @return int8_t     缩放后的占空比（-max_output ~ +max_output）
 * @note 若 max_abs <= 0，则返回0（避免除零）
 */
static int8_t Lib_Mecanum_ScaleToDuty(int16_t value, int16_t max_abs,
                                      int8_t max_output) {
  if (max_abs <= 0) {
    return 0;
  }

  return (int8_t)((value * max_output) / max_abs);
}

/**
 * @brief 麦克纳姆轮运动学混控函数
 *
 * @param x           横向平移分量（-100~100，正值为右移）
 * @param y           纵向平移分量（-100~100，正值为前进）
 * @param yaw         旋转分量（-100~100，正值为顺时针旋转）
 * @param max_output  输出占空比限幅值（绝对值，0~100，负值会取反）
 * @param output      输出结构体指针，存储四个电机的目标占空比
 *
 * @note 输入值均会被视为有符号占空比，内部会进行死区处理（需调用方预先处理）。
 * @note 若混控值最大绝对值超过100，则将所有轮子等比例缩放至 max_output 以内；
 *       否则直接按 max_output 进行比例映射。
 * @note 电机符号约定已在注释中说明，适配具体底盘机械安装方向。
 * @warning 若 output 指针为 NULL，函数直接返回。
 */
void Lib_Mecanum_Mix(int8_t x, int8_t y, int8_t yaw, int8_t max_output,
                     LibMecanumOutput_t *output) {
  int16_t m1;
  int16_t m2;
  int16_t m3;
  int16_t m4;
  int16_t max_abs;

  if (output == 0) {
    return;
  }

  /* 最大输出限幅处理：取绝对值并限制在100以内 */
  if (max_output < 0) {
    max_output = (int8_t)-max_output;
  }
  if (max_output > 100) {
    max_output = 100;
  }

  /*
   * 电机符号约定（底盘俯视图，前进方向为+Y）：
   *   前进：      M1 -, M2 +, M3 -, M4 +
   *   右平移：    M1 +, M2 +, M3 +, M4 +
   *   顺时针旋转：M1 -, M2 -, M3 +, M4 +
   *
   * 其中 M1=左前，M2=右前，M3=左后，M4=右后（假设标准X型布置）
   */
  m1 = (int16_t)x - (int16_t)y - (int16_t)yaw;
  m2 = (int16_t)x + (int16_t)y - (int16_t)yaw;
  m3 = (int16_t)x - (int16_t)y + (int16_t)yaw;
  m4 = (int16_t)x + (int16_t)y + (int16_t)yaw;

  /* 计算最大绝对值，用于等比例缩放防止饱和 */
  max_abs = Lib_Mecanum_MaxAbs4(m1, m2, m3, m4);

  if (max_abs > 100) {
    /* 若超出输入范围，则按最大绝对值归一化后缩放至 max_output */
    output->m1 = Lib_Mecanum_ScaleToDuty(m1, max_abs, max_output);
    output->m2 = Lib_Mecanum_ScaleToDuty(m2, max_abs, max_output);
    output->m3 = Lib_Mecanum_ScaleToDuty(m3, max_abs, max_output);
    output->m4 = Lib_Mecanum_ScaleToDuty(m4, max_abs, max_output);
  } else {
    /* 若在范围内，直接以100为分母缩放至 max_output */
    output->m1 = Lib_Mecanum_ScaleToDuty(m1, 100, max_output);
    output->m2 = Lib_Mecanum_ScaleToDuty(m2, 100, max_output);
    output->m3 = Lib_Mecanum_ScaleToDuty(m3, 100, max_output);
    output->m4 = Lib_Mecanum_ScaleToDuty(m4, 100, max_output);
  }
}