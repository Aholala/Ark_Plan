#ifndef __APP_ENCODER_READER_H
#define __APP_ENCODER_READER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Ozone 中直接查看 g_enc 即可 */
typedef struct {
  int32_t m5_raw;   /**< M5 编码器 raw */
  int32_t m6_raw;   /**< M6 编码器 raw */
  float   m5_deg;   /**< M5 换算角度(°) */
  float   m6_deg;   /**< M6 换算角度(°) */
} AppEncoderData_t;

extern AppEncoderData_t g_enc;

/** 放在 StartDefaultTask 循环中周期调用 */
void App_EncoderReader_Update(void);

#ifdef __cplusplus
}
#endif

#endif
