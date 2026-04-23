#include "pack.h"
#include <stdbool.h>

#define PACKET_MAX_SIZE 32
#define RX_BUF_SIZE 256
#define ADC_MAX_VALUE 4095.0f
#define JOYSTICK_VX_CENTER 1940.0f
#define JOYSTICK_VY_CENTER 2053.0f

extern uint8_t usart1_rx_buf[1]; // 临时接收 1 字节
extern uint8_t ring_buffer[];
extern volatile uint16_t write_index;
ReceivePacket_t packet;

static float normalize_joystick_axis(uint16_t raw, float center)
{
    float value = 0.0f;

    if ((float)raw >= center)
    {
        value = ((float)raw - center) / (ADC_MAX_VALUE - center);
    }
    else
    {
        value = ((float)raw - center) / center;
    }

    if (value > 1.0f)
    {
        return 1.0f;
    }

    if (value < -1.0f)
    {
        return -1.0f;
    }

    return value;
}

void process_ring_buffer()
{
    static uint16_t read_index = 0;
    const uint16_t total_size = 1 + PACKET_SIZE + 1; // 包头 + 数据 + 包尾

    while (((write_index + RX_BUF_SIZE) - read_index) % RX_BUF_SIZE >= total_size)
    {
        // 查找包头
        if (ring_buffer[read_index] == 0xA5)
        {
            // 提取出包尾索引位置
            uint16_t end_index = (read_index + total_size - 1) % RX_BUF_SIZE;

            // 检查包尾是否为 0x5A
            if (ring_buffer[end_index] == 0x5A)
            {
                // 拷贝出数据部分（不含包头和包尾）
                uint8_t temp[PACKET_SIZE];
                for (int i = 0; i < PACKET_SIZE; i++)
                {
                    temp[i] = ring_buffer[(read_index + 1 + i) % RX_BUF_SIZE];
                }

                // 反序列化成结构体
                memcpy(&packet, temp, PACKET_SIZE);

                // ✅ 成功解析一个完整数据包：packet.pitch、packet.yaw 可用

                // 移动读指针，跳过整个包
                read_index = (read_index + total_size) % RX_BUF_SIZE;
                continue;
            }
        }

        // 若不是包头或包尾不对，继续找下一个字节
        read_index = (read_index + 1) % RX_BUF_SIZE;
    }
}

#pragma pack(push, 1)
    typedef struct
    {
        uint8_t Frame_Header;
        bool K1;
        bool K2;
        bool K3;
        bool LB;
        bool RB;
        float Vx;
        float Vy;
        uint8_t Frame_Tail;
    } Serial_TX_Frame_t;
    #pragma pack(pop)

typedef union
{
    Serial_TX_Frame_t Data;
    uint8_t Raw[sizeof(Serial_TX_Frame_t)];
} Serial_TX_Frame_u;

Serial_TX_Frame_u Transmit_Union;

float get_transmit_vx(void)
{
    return Transmit_Union.Data.Vx;
}

void send_packet(UART_HandleTypeDef *huart, const SendPacket_t *payload)
{
    

    Transmit_Union.Data.Frame_Header = 0xA5; // 包头
    Transmit_Union.Data.K1 = payload->ARMkey;
    Transmit_Union.Data.K2 = payload->RESET;
    Transmit_Union.Data.K3 = payload->PAWkey1;
    Transmit_Union.Data.LB = payload->PAWkey2;
    Transmit_Union.Data.RB = payload->VELkey;

    Transmit_Union.Data.Vx = normalize_joystick_axis(payload->Vx, JOYSTICK_VX_CENTER);
    Transmit_Union.Data.Vy = normalize_joystick_axis(payload->Vy, JOYSTICK_VY_CENTER);
    
    Transmit_Union.Data.Frame_Tail = 0x5A; // 包尾

    // uint8_t buffer[1 + SEND_PACKET_SIZE + 1];

    // buffer[0] = 0xA5; // 包头
    // memcpy(&buffer[1], payload, SEND_PACKET_SIZE); // 数据体
    // buffer[1 + SEND_PACKET_SIZE] = 0x5A; // 包尾

    HAL_UART_Transmit_DMA(huart, Transmit_Union.Raw, sizeof(Transmit_Union));
}


