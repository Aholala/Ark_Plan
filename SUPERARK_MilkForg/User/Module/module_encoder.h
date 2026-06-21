/**
 * @file module_encoder.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-06-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __MODULE_ENCODER_H
#define __MODULE_ENCODER_H

#include <stdint.h>

#define MODULE_ENCODER_COUNT 2U

typedef enum
{
  MODULE_ENCODER_M5 = 0,
  MODULE_ENCODER_M6,
} ModuleEncoderId_t;

typedef struct
{
  void *context;
  int32_t (*read_count)(void *context, uint8_t encoder);
  void (*reset_count)(void *context, uint8_t encoder);
} ModuleEncoderOps_t;

typedef struct
{
  const ModuleEncoderOps_t *ops;
  int32_t last_count[MODULE_ENCODER_COUNT];
} ModuleEncoder_t;

void Module_Encoder_Init(ModuleEncoder_t *encoder, const ModuleEncoderOps_t *ops);
int32_t Module_Encoder_GetCount(ModuleEncoder_t *encoder, ModuleEncoderId_t id);
int32_t Module_Encoder_GetDelta(ModuleEncoder_t *encoder, ModuleEncoderId_t id);
void Module_Encoder_Reset(ModuleEncoder_t *encoder, ModuleEncoderId_t id);
void Module_Encoder_ResetAll(ModuleEncoder_t *encoder);

#endif
