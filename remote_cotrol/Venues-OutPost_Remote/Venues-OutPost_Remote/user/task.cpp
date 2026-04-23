#include "task.hpp"
#include "DWT.h"
#include "OLED/joled.h"
#include "pack.h"

BSP_ADC* rocker_adc = nullptr;
Encoder* shoulder_encode = nullptr; // 用于摇杆编码器
Encoder* elbow_encode = nullptr;    // 用于摇杆编码器
uint16_t adcBuffer[4]; //右摇杆x：0 y：4  左摇杆x：3  y：2
int armBuffer[2]; //机械臂编码器数据 shoulder:0 elbow:1
SendPacket_t sendPacket;
extern struct Bkeys bkeys[];
uint8_t PAWkey1;
uint8_t PAWkey2;

uint8_t key;
uint8_t g_k1_flag = 1;
uint8_t g_k2_flag = 1;
uint8_t g_k3_flag = 1;
uint8_t g_lb_flag = 1;
uint8_t g_rb_flag = 1;

#define STATUS_DISPLAY_TICKS_PER_REFRESH 10u
#define LOOP_PERIOD_DISPLAY_MAX_US 99999u
#define OLED_DIRECTION_SCREEN_W 128u
#define OLED_DIRECTION_SCREEN_H 64u
#define OLED_DIRECTION_EDGE_W 4u
#define OLED_STATUS_TEXT_CLEAR "               "

static uint32_t g_loop_period_us = 0u;
static uint64_t g_last_loop_timestamp_us = 0u;
static uint8_t g_loop_period_valid = 0u;
static volatile uint8_t g_status_display_pending = 0u;
static uint8_t g_status_display_tick_count = 0u;

static void draw_direction_pixel(int16_t x, int16_t y, OLED_ColorMode color)
{
    if (x < 0 || y < 0) {
        return;
    }

    if (x >= (int16_t)OLED_DIRECTION_SCREEN_W || y >= (int16_t)OLED_DIRECTION_SCREEN_H) {
        return;
    }

    OLED_SetPixel((uint8_t)x, (uint8_t)y, color);
}

static void clear_direction_edge()
{
    for (uint8_t x = 0u; x < OLED_DIRECTION_SCREEN_W; x++) {
        for (uint8_t y = 0u; y < OLED_DIRECTION_EDGE_W; y++) {
            OLED_SetPixel(x, y, OLED_COLOR_REVERSED);
            OLED_SetPixel(x, (uint8_t)(OLED_DIRECTION_SCREEN_H - 1u - y), OLED_COLOR_REVERSED);
        }
    }

    for (uint8_t y = OLED_DIRECTION_EDGE_W; y < (OLED_DIRECTION_SCREEN_H - OLED_DIRECTION_EDGE_W); y++) {
        for (uint8_t x = 0u; x < OLED_DIRECTION_EDGE_W; x++) {
            OLED_SetPixel(x, y, OLED_COLOR_REVERSED);
            OLED_SetPixel((uint8_t)(OLED_DIRECTION_SCREEN_W - 1u - x), y, OLED_COLOR_REVERSED);
        }
    }
}

static void clear_status_text_area()
{
    JOLED_ShowString(2, 2, (char *)OLED_STATUS_TEXT_CLEAR);
    JOLED_ShowString(3, 2, (char *)OLED_STATUS_TEXT_CLEAR);
}

static void draw_arrow_right(uint8_t x, uint8_t y)
{
    for (int16_t dx = -4; dx <= 0; dx++) {
        draw_direction_pixel((int16_t)x + dx, y, OLED_COLOR_NORMAL);
    }
    draw_direction_pixel((int16_t)x + 1, y, OLED_COLOR_NORMAL);
    draw_direction_pixel(x, (int16_t)y - 1, OLED_COLOR_NORMAL);
    draw_direction_pixel(x, (int16_t)y + 1, OLED_COLOR_NORMAL);
}

static void draw_arrow_down(uint8_t x, uint8_t y)
{
    for (int16_t dy = -4; dy <= 0; dy++) {
        draw_direction_pixel(x, (int16_t)y + dy, OLED_COLOR_NORMAL);
    }
    draw_direction_pixel(x, (int16_t)y + 1, OLED_COLOR_NORMAL);
    draw_direction_pixel((int16_t)x - 1, y, OLED_COLOR_NORMAL);
    draw_direction_pixel((int16_t)x + 1, y, OLED_COLOR_NORMAL);
}

static void draw_arrow_left(uint8_t x, uint8_t y)
{
    for (int16_t dx = 0; dx <= 4; dx++) {
        draw_direction_pixel((int16_t)x + dx, y, OLED_COLOR_NORMAL);
    }
    draw_direction_pixel((int16_t)x - 1, y, OLED_COLOR_NORMAL);
    draw_direction_pixel(x, (int16_t)y - 1, OLED_COLOR_NORMAL);
    draw_direction_pixel(x, (int16_t)y + 1, OLED_COLOR_NORMAL);
}

static void draw_arrow_up(uint8_t x, uint8_t y)
{
    for (int16_t dy = 0; dy <= 4; dy++) {
        draw_direction_pixel(x, (int16_t)y + dy, OLED_COLOR_NORMAL);
    }
    draw_direction_pixel(x, (int16_t)y - 1, OLED_COLOR_NORMAL);
    draw_direction_pixel((int16_t)x - 1, y, OLED_COLOR_NORMAL);
    draw_direction_pixel((int16_t)x + 1, y, OLED_COLOR_NORMAL);
}

static void update_loop_period()
{
    uint64_t now_us = DWT_GetUs();

    if (g_loop_period_valid != 0u) {
        uint64_t period_us = now_us - g_last_loop_timestamp_us;
        g_loop_period_us = period_us > 0xFFFFFFFFULL ? 0xFFFFFFFFu : (uint32_t)period_us;
    }
    else {
        g_loop_period_valid = 1u;
    }

    g_last_loop_timestamp_us = now_us;
}

static void init_status_display()
{
    JOLED_SetAutoRefresh(0);
    JOLED_Clear();
    // JOLED_ShowString(1, 1, (char *)"K1:  K2:  K3:");
    // JOLED_ShowString(2, 1, (char *)"LB:  RB:");
    // JOLED_ShowString(3, 1, (char *)"Vx:     Vy:");
    
    //JOLED_ShowString(4, 1, (char *)"T:00000us");
    JOLED_Refresh();
}

static void display_clockwise()
{
    JOLED_SetAutoRefresh(0);
    clear_direction_edge();

    draw_arrow_right(16u, 2u);
    draw_arrow_right(40u, 2u);
    draw_arrow_right(64u, 2u);
    draw_arrow_right(88u, 2u);
    draw_arrow_right(112u, 2u);

    draw_arrow_down(125u, 14u);
    draw_arrow_down(125u, 26u);
    draw_arrow_down(125u, 38u);
    draw_arrow_down(125u, 50u);

    draw_arrow_left(112u, 61u);
    draw_arrow_left(88u, 61u);
    draw_arrow_left(64u, 61u);
    draw_arrow_left(40u, 61u);
    draw_arrow_left(16u, 61u);

    draw_arrow_up(2u, 50u);
    draw_arrow_up(2u, 38u);
    draw_arrow_up(2u, 26u);
    draw_arrow_up(2u, 14u);
}

static void display_anticlockwise()
{
    JOLED_SetAutoRefresh(0);
    clear_direction_edge();

    draw_arrow_left(16u, 2u);
    draw_arrow_left(40u, 2u);
    draw_arrow_left(64u, 2u);
    draw_arrow_left(88u, 2u);
    draw_arrow_left(112u, 2u);

    draw_arrow_up(125u, 14u);
    draw_arrow_up(125u, 26u);
    draw_arrow_up(125u, 38u);
    draw_arrow_up(125u, 50u);

    draw_arrow_right(16u, 61u);
    draw_arrow_right(40u, 61u);
    draw_arrow_right(64u, 61u);
    draw_arrow_right(88u, 61u);
    draw_arrow_right(112u, 61u);
    
    draw_arrow_down(2u, 14u);
    draw_arrow_down(2u, 26u);
    draw_arrow_down(2u, 38u);
    draw_arrow_down(2u, 50u);
}

static void update_status_display()
{
    JOLED_SetAutoRefresh(0);
    clear_status_text_area();
    clear_direction_edge();

    if (sendPacket.PAWkey1 == 0)
    {
        JOLED_ShowString(2, 2, (char *)"  RemoteMode  ");
        if(get_transmit_vx() >= 0.2)
        {
            display_clockwise();
            JOLED_ShowString(3, 2, (char *)"    >>>>>>    ");
        }
        else if(get_transmit_vx() <= -0.2)
        {
            display_anticlockwise();
            JOLED_ShowString(3, 2, (char *)"    <<<<<<    ");
        }
    }
    else
    {
        JOLED_ShowString(2, 2, (char *)"   AutoMode    ");

        if (sendPacket.ARMkey == 1)
        {
            JOLED_ShowString(3, 2, (char *)"    Running     ");

            if (sendPacket.RESET == 1)
            {
                display_anticlockwise();
            }
            else
            {
                display_clockwise();
            }
        }
        else
        {
            JOLED_ShowString(3, 2, (char *)" Stop Running  ");
        }
    }

    // JOLED_ShowNum(1, 4, sendPacket.ARMkey, 1);
    // JOLED_ShowNum(1, 9, sendPacket.RESET, 1);
    // JOLED_ShowNum(1, 14, sendPacket.PAWkey1, 1);
    // JOLED_ShowNum(2, 4, sendPacket.PAWkey2, 1);
    // JOLED_ShowNum(2, 9, sendPacket.VELkey, 1);
    // JOLED_ShowNum(3, 4, sendPacket.Vx, 4);
    // JOLED_ShowNum(3, 12, sendPacket.Vy, 4);
    JOLED_Refresh();
}

static void service_status_display()
{
    if (g_status_display_pending == 0u) {
        return;
    }

    g_status_display_pending = 0u;
    update_status_display();
}

void myinit()
{
    DWT_Init();
    JOLED_Init();
    //OLED_Init();
    rocker_adc = new BSP_ADC(&hadc1);
    shoulder_encode = new Encoder(&htim1, 65535); // 65535是16位计数器的最大值
    elbow_encode = new Encoder(&htim2, 65535);    // 65535是16位计数器的最大值
    shoulder_encode->start();
    elbow_encode->start(); // 启动编码器

    sendPacket.ARMkey = 0;
    sendPacket.RESET = 0;
    sendPacket.PAWkey1 = 0;
    sendPacket.PAWkey2 = 0;
    sendPacket.VELkey = 0;

    init_status_display();
    update_status_display();
}

void myloop()
{
    update_loop_period();
    //JOLED_ShowChar(1, 1, 'A');
    rocker_adc->init(adcBuffer, 4); // 启动 DMA 采集
    armBuffer[0]=shoulder_encode->getPos(); // 获取机械臂编码器数据
    armBuffer[1]=-elbow_encode->getPos(); // 获取机械臂编码器数据
    process_ring_buffer();
    sendPacket.Vx = adcBuffer[2]; // 获取右摇杆X轴数据
    sendPacket.Vy = -adcBuffer[1]; // 获取右摇杆Y轴数据
    sendPacket.Vw = adcBuffer[0]; // 获取左摇杆X轴数据
    sendPacket.shoulder=armBuffer[0];// 获取肩电机数据
    sendPacket.elbow=armBuffer[1]; // 获取肘电机数据
    key_loop();
    send_packet(&huart1, &sendPacket); // 发送数据包
    service_status_display();
}

void key_loop()
{
    //按下1 不按0的逻辑 已注释
    // PAWkey1=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1); // 读取按键状态
    // PAWkey2=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12); // 读取按键状态
    // if (PAWkey1) {
    //     sendPacket.PAWkey1=1;
    //     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET); // 切换LED状态
    // }
    // else {
    //     sendPacket.PAWkey1=0;
    //     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET); // 切换LED状态
    // }

    // if (PAWkey2) {
    //     sendPacket.PAWkey2=0;
    //     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET); // 切换LED状态
    // }
    // else {
    //     sendPacket.PAWkey2=1;
    //     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET); // 切换LED状态
    // }

    //短按切换

    //PAWkey2 / LB
    if (bkeys[1].short_flag==1) {
        if (g_lb_flag == 0u) {
            sendPacket.PAWkey2 = 0;
            g_lb_flag = 1u;
        }
        else {
            sendPacket.PAWkey2 = 1;
            g_lb_flag = 0u;
        }
        bkeys[1].short_flag=0;
    }

    //PAWkey1 / K3
    if (bkeys[5].short_flag==1) {
        if (g_k3_flag == 0u) {
            sendPacket.PAWkey1 = 0;
            g_k3_flag = 1u;
        }
        else {
            sendPacket.PAWkey1 = 1;
            g_k3_flag = 0u;
        }
        bkeys[5].short_flag=0;
    }

    //ARMkey / K1
    if (bkeys[4].short_flag==1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
        if (g_k1_flag == 0u) {
            sendPacket.ARMkey = 0;
            g_k1_flag = 1u;
        }
        else {
            sendPacket.ARMkey = 1;
            g_k1_flag = 0u;
        }
        bkeys[4].short_flag=0;
    }

    //RESET / K2
    if (bkeys[3].short_flag==1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
        if (g_k2_flag == 0u) {
            sendPacket.RESET = 0;
            g_k2_flag = 1u;
        }
        else {
            sendPacket.RESET = 1;
            g_k2_flag = 0u;
        }
        bkeys[3].short_flag=0;
    }

    //VELkey / RB
    if (bkeys[2].short_flag==1) {
        if (g_rb_flag == 0u) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
            sendPacket.VELkey = 0;
            g_rb_flag = 1u;
        }
        else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
            sendPacket.VELkey = 1;
            g_rb_flag = 0u;
        }
        bkeys[2].short_flag=0;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM4)
    {
        key_serv();
        //update_status_display();
      //	key_serv_long();
            //	key_serv_long_2();
       // key_serv_double();
    }   
    if(htim->Instance==TIM3)
    {
        g_status_display_tick_count++;
        if (g_status_display_tick_count >= STATUS_DISPLAY_TICKS_PER_REFRESH) {
            g_status_display_tick_count = 0u;
            g_status_display_pending = 1u;
        }
    }
}

