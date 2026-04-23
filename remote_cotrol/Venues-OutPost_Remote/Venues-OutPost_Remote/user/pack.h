/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pack.h
  * @brief   This file contains all the function prototypes for
  *          the pack.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PACK_H__
#define __PACK_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/
#include <string.h>

typedef struct __attribute__((packed)) {
    uint8_t length;
} ReceivePacket_t;
#define PACKET_SIZE sizeof(ReceivePacket_t)

typedef struct __attribute__((packed)) {
    uint16_t Vx;
    uint16_t Vy;
    uint16_t Vw;
    int shoulder;
    int elbow;
    uint8_t VELkey;
    uint8_t PAWkey1;
    uint8_t PAWkey2;
    uint8_t ARMkey;
    uint8_t RESET;
} SendPacket_t;
#define SEND_PACKET_SIZE sizeof(SendPacket_t)

void send_packet(UART_HandleTypeDef *huart, const SendPacket_t *payload);
void process_ring_buffer();
float get_transmit_vx(void);


#ifdef __cplusplus
}
#endif

#endif /* __PACK_H__ */
