/**
 * @file module_encoder.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-06-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "module_encoder.h"

static uint8_t Module_Encoder_IsValid(ModuleEncoder_t *encoder,
                                      ModuleEncoderId_t id)
{
  return (encoder != 0) &&
         (encoder->ops != 0) &&
         ((uint8_t)id < MODULE_ENCODER_COUNT);
}

void Module_Encoder_Init(ModuleEncoder_t *encoder, const ModuleEncoderOps_t *ops)
{
  if (encoder == 0)
  {
    return;
  }

  encoder->ops = ops;
  for (uint8_t i = 0U; i < MODULE_ENCODER_COUNT; i++)
  {
    encoder->last_count[i] = 0;
  }
}

int32_t Module_Encoder_GetCount(ModuleEncoder_t *encoder, ModuleEncoderId_t id)
{
  if (!Module_Encoder_IsValid(encoder, id) || (encoder->ops->read_count == 0))
  {
    return 0;
  }

  return encoder->ops->read_count(encoder->ops->context, (uint8_t)id);
}

int32_t Module_Encoder_GetDelta(ModuleEncoder_t *encoder, ModuleEncoderId_t id)
{
  int32_t current;
  int32_t delta;

  if (!Module_Encoder_IsValid(encoder, id))
  {
    return 0;
  }

  current = Module_Encoder_GetCount(encoder, id);
  delta = current - encoder->last_count[id];
  encoder->last_count[id] = current;

  return delta;
}

void Module_Encoder_Reset(ModuleEncoder_t *encoder, ModuleEncoderId_t id)
{
  if (!Module_Encoder_IsValid(encoder, id))
  {
    return;
  }

  if (encoder->ops->reset_count != 0)
  {
    encoder->ops->reset_count(encoder->ops->context, (uint8_t)id);
  }
  encoder->last_count[id] = 0;
}

void Module_Encoder_ResetAll(ModuleEncoder_t *encoder)
{
  for (uint8_t i = 0U; i < MODULE_ENCODER_COUNT; i++)
  {
    Module_Encoder_Reset(encoder, (ModuleEncoderId_t)i);
  }
}
