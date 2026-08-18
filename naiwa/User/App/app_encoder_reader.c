/**
 * @file app_encoder_reader.c
 * @brief M5/M6 编码器调试读取 — 存入全局变量供 Ozone 查看
 *
 * 在 StartDefaultTask 循环中调用 App_EncoderReader_Update() 即可。
 * Ozone 中观察 g_enc 结构体。
 */

#include "app_encoder_reader.h"
#include "module_motor.h"

#define ENC_CPR           (48.0f * 139.0f)        /* 6672 */
#define ENC_DEG_PER_COUNT (360.0f / ENC_CPR)      /* 0.05396 */

AppEncoderData_t g_enc;

void App_EncoderReader_Update(void) {
  g_enc.m5_raw = Module_Motor_GetEncoderCount(MODULE_MOTOR_5);
  g_enc.m6_raw = Module_Motor_GetEncoderCount(MODULE_MOTOR_6);
  g_enc.m5_deg = (float)g_enc.m5_raw * ENC_DEG_PER_COUNT;
  g_enc.m6_deg = (float)g_enc.m6_raw * ENC_DEG_PER_COUNT;
}
