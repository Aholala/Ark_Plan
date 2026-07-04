/**
 * @file bsp_key.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）按键驱动源文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本文件实现了按键的硬件读取、消抖处理、事件检测（单击、双击、长按、重复触发等）。
 * 采用状态机方式管理每个按键，通过周期性调用 Key_Tick() 更新状态。
 */

#include "bsp_key.h"
#include "main.h"

#include <string.h>

/* 按键电平定义 */
#define KEY_PRESSED 1   /* 按键按下（低电平有效） */
#define KEY_UNPRESSED 0 /* 按键释放 */

/* 时间参数（单位：毫秒，基于 Key_Tick 的调用周期 20ms） */
#define KEY_TIME_DOUBLE                                                        \
  0 /* 双击检测窗口时间（此处设为0，表示不检测双击，实际可在回调中处理） */
#define KEY_TIME_LONG 1000  /* 长按判定时间（1000ms） */
#define KEY_TIME_REPEAT 100 /* 长按后重复触发间隔（100ms） */

/* 全局事件标志数组，每位代表一种事件，供外部检查 */
uint8_t Key_Flag[KEY_COUNT];

/* 静态变量：状态机内部使用 */
static uint8_t
    Key_Count; /* 周期计数器，每20ms递增，达到20次（即400ms）进行一次状态更新 */
static uint8_t
    Key_CurrState[KEY_COUNT]; /* 当前采样到的按键物理状态（按下/释放） */
static uint8_t Key_PrevState[KEY_COUNT]; /* 上一次采样到的按键物理状态 */
static uint8_t Key_State[KEY_COUNT];     /* 按键状态机状态（0~4） */
static uint16_t Key_Time[KEY_COUNT];     /* 状态机计时器（递减计数） */

/**
 * @brief 按键初始化，清空所有内部状态
 * @retval 无
 */
void Key_Init(void) { Key_Clear(); }

/**
 * @brief 读取指定GPIO引脚的电平，转换为按键状态（按下/释放）
 * @param GPIOx    GPIO端口
 * @param GPIO_Pin GPIO引脚
 * @return uint8_t 返回 KEY_PRESSED 或 KEY_UNPRESSED
 * @note 假设按键按下时引脚为低电平（GPIO_PIN_RESET）
 */
static uint8_t Key_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  return (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) ? KEY_PRESSED
                                                               : KEY_UNPRESSED;
}

/**
 * @brief 根据按键编号获取当前物理状态
 * @param n 按键编号（0~KEY_COUNT-1，宏定义如 KEY_1, KEY_2...）
 * @return uint8_t 返回 KEY_PRESSED 或 KEY_UNPRESSED
 * @note 该函数直接读取硬件，未做消抖，仅供内部状态机调用
 */
uint8_t Key_GetState(uint8_t n) {
  if (n == KEY_1) {
    return Key_Read(KEY1_GPIO_Port, KEY1_Pin);
  }
  if (n == KEY_2) {
    return Key_Read(KEY2_GPIO_Port, KEY2_Pin);
  }
  if (n == KEY_3) {
    return Key_Read(KEY3_GPIO_Port, KEY3_Pin);
  }
  if (n == KEY_4) {
    return Key_Read(KEY4_GPIO_Port, KEY4_Pin);
  }
  if (n == KEY_5) {
    return Key_Read(KEY5_GPIO_Port, KEY5_Pin);
  }
  if (n == KEY_6) {
    return Key_Read(KEY6_GPIO_Port, KEY6_Pin);
  }
  if (n == KEY_7) {
    return Key_Read(KEY7_GPIO_Port, KEY7_Pin);
  }
  if (n == KEY_8) {
    return Key_Read(KEY8_GPIO_Port, KEY8_Pin);
  }
  if (n == KEY_9) {
    return Key_Read(KEY9_GPIO_Port, KEY9_Pin);
  }
  if (n == KEY_10) {
    return Key_Read(KEY10_GPIO_Port, KEY10_Pin);
  }
  if (n == KEY_11) {
    return Key_Read(KEY11_GPIO_Port, KEY11_Pin);
  }
  if (n == KEY_12) {
    return Key_Read(KEY12_GPIO_Port, KEY12_Pin);
  }
  return KEY_UNPRESSED;
}

/**
 * @brief 检查指定按键的事件标志
 * @param n    按键编号
 * @param Flag 要检查的事件标志（如 KEY_DOWN, KEY_UP, KEY_SINGLE, KEY_DOUBLE,
 * KEY_LONG, KEY_REPEAT, KEY_HOLD）
 * @return uint8_t 若该事件发生则返回1，否则返回0
 * @note 除 KEY_HOLD 外，其他事件标志在检查后会自动清除（便于事件驱动处理）
 *       KEY_HOLD 是持续状态，不清除，用于检测按键当前是否处于按下状态
 */
uint8_t Key_Check(uint8_t n, uint8_t Flag) {
  if (Key_Flag[n] & Flag) {
    if (Flag != KEY_HOLD) /* HOLD 标志不清除，其他标志一次性使用后清除 */
    {
      Key_Flag[n] &= ~Flag;
    }
    return 1;
  }
  return 0;
}

/**
 * @brief 清空所有按键的内部状态和事件标志
 * @retval 无
 * @note 通常在初始化或需要重置时调用
 */
void Key_Clear(void) {
  memset(Key_Flag, 0, sizeof(Key_Flag));
  memset(Key_CurrState, 0, sizeof(Key_CurrState));
  memset(Key_PrevState, 0, sizeof(Key_PrevState));
  memset(Key_State, 0, sizeof(Key_State));
  memset(Key_Time, 0, sizeof(Key_Time));
  Key_Count = 0U;
}

/**
 * @brief 按键状态机周期处理函数（建议每20ms调用一次）
 * @retval 无
 * @note 该函数执行消抖、事件检测（按下/释放/单击/双击/长按/重复）
 *       - 每20ms调用一次，内部以20次（400ms）为一个周期对按键进行采样和状态更新
 *       - 状态机流程：
 *         状态0（空闲）：检测到按下 -> 进入状态1，启动长按计时
 *         状态1（等待释放或长按）：若释放 ->
 * 进入状态2（等待双击）；若长按计时到 -> 触发长按，进入状态4（重复触发）
 *         状态2（等待双击）：若再次按下 -> 触发双击，进入状态3；若超时 ->
 * 触发单击，回到状态0 状态3（双击后等待释放）：释放后回到状态0
 *         状态4（长按重复）：若释放 -> 回到状态0；若重复计时到 ->
 * 触发重复事件，重置重复计时
 *       - 事件标志位：KEY_DOWN, KEY_UP, KEY_SINGLE, KEY_DOUBLE, KEY_LONG,
 * KEY_REPEAT, KEY_HOLD
 */
void Key_Tick(void) {
  uint8_t i;

  /* 所有按键的计时器递减（以20ms为单位） */
  for (i = 0; i < KEY_COUNT; i++) {
    if (Key_Time[i] > 0) {
      Key_Time[i]--;
    }
  }

  /* 每20次调用（即 20*20ms = 400ms）进行一次按键采样和状态更新 */
  Key_Count++;
  if (Key_Count >= 20) {
    Key_Count = 0U;

    for (i = 0; i < KEY_COUNT; i++) {
      /* 更新当前和上一次采样状态 */
      Key_PrevState[i] = Key_CurrState[i];
      Key_CurrState[i] = Key_GetState(i);

      /* 设置/清除 HOLD 标志（表示当前是否按下） */
      if (Key_CurrState[i] == KEY_PRESSED) {
        Key_Flag[i] |= KEY_HOLD;
      } else {
        Key_Flag[i] &= ~KEY_HOLD;
      }

      /* 检测按下和释放的边沿事件 */
      if (Key_CurrState[i] == KEY_PRESSED &&
          Key_PrevState[i] == KEY_UNPRESSED) {
        Key_Flag[i] |= KEY_DOWN;
      }
      if (Key_CurrState[i] == KEY_UNPRESSED &&
          Key_PrevState[i] == KEY_PRESSED) {
        Key_Flag[i] |= KEY_UP;
      }

      /* 状态机处理 */
      if (Key_State[i] == 0) /* 空闲态 */
      {
        if (Key_CurrState[i] == KEY_PRESSED) /* 检测到按下 */
        {
          Key_Time[i] = KEY_TIME_LONG; /* 启动长按计时（1000ms） */
          Key_State[i] = 1;            /* 进入状态1 */
        }
      } else if (Key_State[i] == 1) /* 等待释放或长按超时 */
      {
        if (Key_CurrState[i] == KEY_UNPRESSED) /* 提前释放 */
        {
          Key_Time[i] = KEY_TIME_DOUBLE; /* 设置双击等待时间（此处为0，即不等待双击，直接触发单击）
                                          */
          Key_State[i] = 2;          /* 进入状态2（双击检测） */
        } else if (Key_Time[i] == 0) /* 长按时间到 */
        {
          Key_Time[i] = KEY_TIME_REPEAT; /* 设置重复触发间隔 */
          Key_Flag[i] |= KEY_LONG;       /* 触发长按事件 */
          Key_State[i] = 4;              /* 进入状态4（长按重复） */
        }
      } else if (Key_State[i] ==
                 2) /* 等待双击（若再次按下则双击，否则超时触发单击） */
      {
        if (Key_CurrState[i] == KEY_PRESSED) /* 又按下了 */
        {
          Key_Flag[i] |= KEY_DOUBLE; /* 触发双击事件 */
          Key_State[i] = 3;          /* 进入状态3（等待释放） */
        } else if (Key_Time[i] == 0) /* 超时未双击 */
        {
          Key_Flag[i] |= KEY_SINGLE; /* 触发单击事件 */
          Key_State[i] = 0;          /* 回到空闲态 */
        }
      } else if (Key_State[i] == 3) /* 双击后等待释放 */
      {
        if (Key_CurrState[i] == KEY_UNPRESSED) {
          Key_State[i] = 0;
        }                           /* 释放后回到空闲 */
      } else if (Key_State[i] == 4) /* 长按重复状态 */
      {
        if (Key_CurrState[i] == KEY_UNPRESSED) /* 释放按键 */
        {
          Key_State[i] = 0;          /* 回到空闲 */
        } else if (Key_Time[i] == 0) /* 重复间隔到 */
        {
          Key_Time[i] = KEY_TIME_REPEAT; /* 重置重复计时 */
          Key_Flag[i] |= KEY_REPEAT;     /* 触发重复事件 */
        }
      }
    }
  }
}