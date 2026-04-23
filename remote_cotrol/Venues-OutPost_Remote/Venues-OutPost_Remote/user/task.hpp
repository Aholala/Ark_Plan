#ifndef __TASK_HPP
#define __TASK_HPP

#include "Rocker.hpp"
#include "bsp_encode.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "adc.h"
#include "pack.h"
#include "usart.h"
#include "key.h"
#include "OLED/joled.h"

    void myinit();
    void myloop();
    void key_loop();
    extern uint8_t g_k1_flag;
    extern uint8_t g_k2_flag;
    extern uint8_t g_k3_flag;
    extern uint8_t g_lb_flag;
    extern uint8_t g_rb_flag;

#ifdef __cplusplus
}
#endif

#endif




