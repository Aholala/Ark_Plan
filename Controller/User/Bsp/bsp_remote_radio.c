#include "bsp_remote_radio.h"

#include "bsp_nrf24l01.h"

#include <string.h>

void Bsp_RemoteRadio_Init(void)
{
  NRF24L01_Init();
}

uint8_t Bsp_RemoteRadio_SendControl(const BspRemoteRadioControlPacket_t *packet)
{
  if (packet == 0)
  {
    return 0U;
  }

  NRF24L01_TxPacket[0] = packet->mode;
  NRF24L01_TxPacket[1] = (uint8_t)packet->lh;
  NRF24L01_TxPacket[2] = (uint8_t)packet->lv;
  NRF24L01_TxPacket[3] = (uint8_t)packet->rh;
  NRF24L01_TxPacket[4] = (uint8_t)packet->rv;
  NRF24L01_TxPacket[5] = packet->key & 0x3FU;
  NRF24L01_TxPacket[6] = packet->key_seq;

  return NRF24L01_Send();
}

uint8_t Bsp_RemoteRadio_ReceiveFeedback(BspRemoteRadioFeedbackPacket_t *packet)
{
  if (NRF24L01_Receive() != 1U)
  {
    return 0U;
  }

  if (packet != 0)
  {
    packet->speed_level = NRF24L01_RxPacket[1];
    packet->pwm_l = (int8_t)NRF24L01_RxPacket[2];
    packet->pwm_r = (int8_t)NRF24L01_RxPacket[3];
    memcpy(&packet->angle, &NRF24L01_RxPacket[4], sizeof(packet->angle));
    memcpy(&packet->speed_left, &NRF24L01_RxPacket[8], sizeof(packet->speed_left));
    memcpy(&packet->speed_right, &NRF24L01_RxPacket[12], sizeof(packet->speed_right));
  }

  return 1U;
}
