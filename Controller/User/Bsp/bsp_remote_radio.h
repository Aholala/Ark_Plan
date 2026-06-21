#ifndef __BSP_REMOTE_RADIO_H
#define __BSP_REMOTE_RADIO_H

#include <stdint.h>

typedef struct
{
  uint8_t mode;
  int8_t lh;
  int8_t lv;
  int8_t rh;
  int8_t rv;
  uint8_t key;
  uint8_t key_seq;
} BspRemoteRadioControlPacket_t;

typedef struct
{
  uint8_t speed_level;
  int8_t pwm_l;
  int8_t pwm_r;
  float angle;
  float speed_left;
  float speed_right;
} BspRemoteRadioFeedbackPacket_t;

void Bsp_RemoteRadio_Init(void);
uint8_t Bsp_RemoteRadio_SendControl(const BspRemoteRadioControlPacket_t *packet);
uint8_t Bsp_RemoteRadio_ReceiveFeedback(BspRemoteRadioFeedbackPacket_t *packet);

#endif
