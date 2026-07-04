/**
 * @file app_remote.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 遥控器应用层模块（发送端增强版）
 * @version 2.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本模块实现了遥控器发送端的核心功能：
 * - 读取ADC摇杆值（左右摇杆的水平和垂直分量）
 * - 扫描按键事件，支持快速重复发送（按键持续按下时连续发送）
 * - 通过NRF24L01无线发送控制数据包（发送周期：无按键时50ms，有按键时5ms）
 * - 接收并显示来自接收端（如机器人底盘）的反馈数据（角度、PWM、速度等）
 * - OLED屏幕显示实时状态（摇杆值、按键提示、信号强度等）
 * - 支持两种模式：发送模式（显示摇杆）和接收模式（显示反馈）
 *
 * 相较于V1.0版本的改进：
 * - 增加按键序列号（key_seq），用于接收端去重
 * - 按键发送周期缩短至5ms，响应更迅速
 * - 增加重复发送计数（key_repeat_remaining），保证按键在成功发送10次后自动清除
 * - 使用 Bsp_RadioLink 链路层替代直接操作 NRF24L01
 * - 使用 Bsp_Time_GetMs() 替代 HAL_GetTick()，统一时间接口
 * - 摇杆通道使用宏定义（AD_CHANNEL_LH等），提高可读性
 */

#include "app_remote.h"

#include "bsp_ad.h"
#include "bsp_key.h"
#include "bsp_oled.h"
#include "bsp_radio_link.h"
#include "bsp_time.h"
#include "module_periodic_timer.h"

#include <string.h>

/*------------------------ 应用参数宏定义 ------------------------*/
#define APP_AD_DEAD_ZONE 100     /* ADC摇杆死区范围（中心值±100） */
#define APP_SEND_PERIOD_MS 50    /* 数据发送周期（毫秒）- 无按键时 */
#define APP_KEY_SEND_PERIOD_MS 5 /* 有按键时的数据发送周期（毫秒），快速响应 \
                                  */
#define APP_KEY_REPEAT_SUCCESS_COUNT                                           \
  10 /* 按键重复发送成功次数，达到后自动清除待发送状态 */
#define APP_KEY_BANNER_TIME_MS 100 /* 按键提示显示时间（毫秒） */

/*------------------------ 应用状态机枚举 ------------------------*/
typedef enum {
  APP_REMOTE_WAIT_START = 0, /* 等待启动状态（显示欢迎界面） */
  APP_REMOTE_RUN,            /* 正常运行状态 */
} AppRemoteState;

/*------------------------ 应用上下文结构体 ------------------------*/
typedef struct {
  /* ADC原始值 */
  uint16_t ad_lh; /* 左摇杆水平原始值（0~4095） */
  uint16_t ad_lv; /* 左摇杆垂直原始值 */
  uint16_t ad_rh; /* 右摇杆水平原始值 */
  uint16_t ad_rv; /* 右摇杆垂直原始值 */

  /* 处理后的摇杆值（带死区，映射到 -100 ~ +100） */
  int8_t lh; /* 左摇杆水平分量（有符号） */
  int8_t lv; /* 左摇杆垂直分量 */
  int8_t rh; /* 右摇杆水平分量 */
  int8_t rv; /* 右摇杆垂直分量 */

  /* 按键相关 */
  uint8_t key_pending;          /* 待发送的按键编号（0表示无按键） */
  uint8_t key_num;              /* 当前按下的按键编号（用于显示） */
  uint8_t key_seq;              /* 按键序列号（每次按键递增，用于接收端去重） */
  uint8_t key_pending_seq;      /* 待发送按键的序列号 */
  uint8_t key_repeat_remaining; /* 按键重复发送剩余次数（成功发送后递减） */
  uint8_t banner_key_num;       /* 用于显示按键提示的编号 */
  uint32_t key_banner_until;    /* 按键提示显示的截止时间（系统tick） */

  /* 无线通信相关 */
  uint8_t send_flag;         /* 上次数据发送结果（1成功，0失败） */
  uint8_t success_ratio;     /* 最近10次发送成功率（0~10） */
  volatile uint8_t send_due; /* 发送定时标志（由定时器置位，主循环清除） */

  uint8_t mode; /* 工作模式：0=发送模式（显示摇杆），1=接收模式（显示反馈） */
  AppRemoteState state; /* 应用状态 */
} AppRemoteContext;

static AppRemoteContext app_remote; /* 全局应用上下文 */
static ModulePeriodicTimer_t app_send_timer; /* 发送周期软件定时器 */

/*------------------------ 内部函数声明 ------------------------*/
static int8_t App_Remote_ProcessAd(uint16_t value); /* ADC值处理（死区+映射） */
static uint8_t App_Remote_CalculateSuccessRatio(
    uint8_t send_flag);                        /* 计算最近10次发送成功率 */
static uint8_t App_Remote_GetPressedKey(void); /* 获取当前按下的按键编号 */
static void
App_Remote_ScanInputs(AppRemoteContext *ctx); /* 扫描所有输入（ADC+按键） */
static void
App_Remote_SendPacket(AppRemoteContext *ctx); /* 打包并发送控制数据包 */
static void App_Remote_UpdateDisplay(AppRemoteContext *ctx); /* 更新OLED显示 */
static void App_Remote_ShowWelcome(void);                    /* 显示欢迎界面 */

/**
 * @brief 遥控器应用初始化
 * @retval 无
 * @note   - 清空上下文，初始化所有硬件外设（OLED、按键、ADC、无线模块、定时器）
 *         - 显示欢迎界面，等待用户按KEY_10启动
 */
void App_Remote_Init(void) {
  memset(&app_remote, 0, sizeof(app_remote));
  Module_PeriodicTimer_Init(&app_send_timer, APP_SEND_PERIOD_MS);
  app_remote.state = APP_REMOTE_WAIT_START;

  OLED_Init();
  Key_Init();
  AD_Init();
  Bsp_RadioLink_Init();
  Bsp_Time_InitTick();

  App_Remote_ShowWelcome();
}

/**
 * @brief 遥控器应用主任务（在主循环中周期调用）
 * @retval 无
 * @note   - 若处于等待启动状态，检测KEY_10按下后清屏进入运行状态
 *         - 在运行状态下，执行输入扫描、发送数据包、更新显示
 */
void App_Remote_Task(void) {
  AppRemoteContext *ctx = &app_remote;

  /* 等待启动状态：仅检测启动按键 */
  if (ctx->state == APP_REMOTE_WAIT_START) {
    if (Key_Check(KEY_10, KEY_DOWN)) {
      Key_Clear();
      OLED_Clear();
      ctx->state = APP_REMOTE_RUN;
    }
    return;
  }

  /* 正常运行状态 */
  App_Remote_ScanInputs(ctx); /* 扫描摇杆和按键输入 */

  /* 若发送定时到，则发送数据包 */
  if (ctx->send_due) {
    ctx->send_due = 0U;
    App_Remote_SendPacket(ctx);
  }

  App_Remote_UpdateDisplay(ctx); /* 更新OLED显示 */
}

/**
 * @brief 1ms周期回调（由定时器中断调用）
 * @retval 无
 * @note   - 调用Key_Tick处理按键状态机
 *         - 根据是否有按键待发送，动态调整发送周期：
 *           无按键时按 APP_SEND_PERIOD_MS（50ms）发送
 *           有按键时按 APP_KEY_SEND_PERIOD_MS（5ms）快速发送
 */
void App_Remote_Tick1ms(void) {
  Key_Tick();

  Module_PeriodicTimer_SetPeriod(
      &app_send_timer,
      (app_remote.key_pending != 0U) ? APP_KEY_SEND_PERIOD_MS
                                     : APP_SEND_PERIOD_MS);
  if (Module_PeriodicTimer_Tick(&app_send_timer) != 0U) {
    app_remote.send_due = 1U;
  }
}

/**
 * @brief 显示欢迎界面
 * @retval 无
 * @note  显示团队名称、版本信息和启动提示（K10>表示按KEY_10启动）
 */
static void App_Remote_ShowWelcome(void) {
  OLED_Clear();
  OLED_ShowString(0, 0, " Milkforg Team   ", OLED_8X16);
  OLED_ShowString(4, 16, " Remote Tester ", OLED_8X16);
  OLED_ShowString(0, 32, "      V1.0      ", OLED_8X16);
  OLED_ShowString(0, 48, "            K10>", OLED_8X16);
  OLED_Update();
}

/**
 * @brief 扫描所有输入（ADC摇杆值和按键）
 * @param ctx 应用上下文指针
 * @retval 无
 * @note   - 读取四个ADC通道并处理为有符号摇杆值（-100~+100）
 *         - 检测12个按键的按下事件，更新key_num和key_pending
 *         - 每次按键按下时递增key_seq序列号（用于接收端去重）
 *         - 特殊按键KEY_9（编号9）用于切换工作模式（发送/接收）
 */
static void App_Remote_ScanInputs(AppRemoteContext *ctx) {
  /* 获取当前按下的按键编号 */
  ctx->key_num = App_Remote_GetPressedKey();

  /* 读取ADC原始值（使用宏定义的通道） */
  ctx->ad_lh = AD_GetValue(AD_CHANNEL_LH);
  ctx->ad_lv = AD_GetValue(AD_CHANNEL_LV);
  ctx->ad_rh = AD_GetValue(AD_CHANNEL_RH);
  ctx->ad_rv = AD_GetValue(AD_CHANNEL_RV);

  /* 转换为有符号摇杆值 */
  ctx->lh = App_Remote_ProcessAd(ctx->ad_lh);
  ctx->lv = App_Remote_ProcessAd(ctx->ad_lv);
  ctx->rh = App_Remote_ProcessAd(ctx->ad_rh);
  ctx->rv = App_Remote_ProcessAd(ctx->ad_rv);

  /* 若有按键按下，则更新待发送按键号和序列号 */
  if (ctx->key_num != 0U) {
    ctx->key_pending = ctx->key_num;
    ctx->key_seq++;         /* 序列号递增 */
    if (ctx->key_seq == 0U) /* 防止溢出归零 */
    {
      ctx->key_seq = 1U;
    }
    ctx->key_pending_seq = ctx->key_seq;
    ctx->key_repeat_remaining =
        APP_KEY_REPEAT_SUCCESS_COUNT; /* 重置重复发送计数 */
    ctx->banner_key_num = ctx->key_num;
    ctx->key_banner_until = Bsp_Time_GetMs() + APP_KEY_BANNER_TIME_MS;
    ctx->send_due = 1U; /* 立即触发发送 */
  }

  /* 特殊按键：KEY_9（编号9）用于切换工作模式 */
  if (ctx->key_num == 9U) {
    ctx->mode = !ctx->mode; /* 0->1 或 1->0 */
    OLED_Clear();           /* 切换模式时清屏，以便显示新界面 */
  }
}

/**
 * @brief 打包并发送控制数据包
 * @param ctx 应用上下文指针
 * @retval 无
 * @note   - 数据包通过 Bsp_RadioLink_SendControl() 发送
 *         - 若发送成功，递减 key_repeat_remaining
 *         - 当 key_repeat_remaining
 * 减至0时，清除待发送按键（表示按键已可靠发送多次）
 *         - 更新发送成功率
 */
static void App_Remote_SendPacket(AppRemoteContext *ctx) {
  BspRadioLinkControlPacket_t packet;

  /* 填充数据包 */
  packet.mode = ctx->mode;
  packet.lh = ctx->lh;
  packet.lv = ctx->lv;
  packet.rh = ctx->rh;
  packet.rv = ctx->rv;
  packet.key = ctx->key_pending;
  packet.key_seq = ctx->key_pending_seq;

  /* 发送控制数据包 */
  ctx->send_flag = Bsp_RadioLink_SendControl(&packet);
  if (ctx->send_flag == 1U) /* 发送成功 */
  {
    if (ctx->key_repeat_remaining > 0U) {
      ctx->key_repeat_remaining--;
    }

    /* 按键已成功发送指定次数，清除待发送状态 */
    if (ctx->key_repeat_remaining == 0U) {
      ctx->key_pending = 0U;
    }
  }

  /* 更新发送成功率 */
  ctx->success_ratio = App_Remote_CalculateSuccessRatio(ctx->send_flag);
}

/**
 * @brief 获取当前按下的按键编号
 * @return uint8_t 按键编号（1~12），0表示无按键按下
 * @note  使用 Key_Check(KEY_DOWN) 检测按键按下事件（边沿触发，检查后自动清除）
 */
static uint8_t App_Remote_GetPressedKey(void) {
  if (Key_Check(KEY_1, KEY_DOWN)) {
    return 1U;
  }
  if (Key_Check(KEY_2, KEY_DOWN)) {
    return 2U;
  }
  if (Key_Check(KEY_3, KEY_DOWN)) {
    return 3U;
  }
  if (Key_Check(KEY_4, KEY_DOWN)) {
    return 4U;
  }
  if (Key_Check(KEY_5, KEY_DOWN)) {
    return 5U;
  }
  if (Key_Check(KEY_6, KEY_DOWN)) {
    return 6U;
  }
  if (Key_Check(KEY_7, KEY_DOWN)) {
    return 7U;
  }
  if (Key_Check(KEY_8, KEY_DOWN)) {
    return 8U;
  }
  if (Key_Check(KEY_9, KEY_DOWN)) {
    return 9U;
  }
  if (Key_Check(KEY_10, KEY_DOWN)) {
    return 10U;
  }
  if (Key_Check(KEY_11, KEY_DOWN)) {
    return 11U;
  }
  if (Key_Check(KEY_12, KEY_DOWN)) {
    return 12U;
  }
  return 0U;
}

/**
 * @brief 更新OLED显示内容
 * @param ctx 应用上下文指针
 * @retval 无
 * @note   - 根据工作模式显示不同内容：
 *           模式0：显示摇杆值（LH/LV/RH/RV）和按键提示（右上角）
 *           模式1：尝试接收反馈数据包，显示角度、PWM值、速度等信息
 *         - 左上角显示信号强度图标（基于成功率）
 */
static void App_Remote_UpdateDisplay(AppRemoteContext *ctx) {
  /* 右上角显示按键提示（若有按键按下且未超时） */
  if (ctx->key_banner_until > Bsp_Time_GetMs()) {
    OLED_Printf(104, 0, OLED_8X16, "K%d", ctx->banner_key_num);
  } else {
    OLED_ClearArea(104, 0, 24, 16); /* 超时则清除该区域 */
  }

  /* 模式0：发送模式，显示摇杆值 */
  if (ctx->mode == 0U) {
    OLED_Printf(0, 16, OLED_8X16, "LH:%+04d", ctx->lh);
    OLED_Printf(0, 32, OLED_8X16, "LV:%+04d", ctx->lv);
    OLED_Printf(72, 16, OLED_8X16, "RH:%+04d", ctx->rh);
    OLED_Printf(72, 32, OLED_8X16, "RV:%+04d", ctx->rv);
  }
  /* 模式1：接收模式，尝试接收反馈数据并显示 */
  else if (ctx->mode == 1U) {
    BspRadioLinkFeedbackPacket_t packet;

    if (Bsp_RadioLink_ReceiveFeedback(&packet) == 1U) /* 接收到反馈数据包 */
    {
      OLED_Clear(); /* 接收模式下清屏显示完整反馈信息 */
      OLED_Printf(0, 16, OLED_6X8, "M1 P:%+04d S:%+05.1f", packet.pwm[0],
                  packet.speed[0]);
      OLED_Printf(0, 26, OLED_6X8, "M2 P:%+04d S:%+05.1f", packet.pwm[1],
                  packet.speed[1]);
      OLED_Printf(0, 40, OLED_6X8, "M3 P:%+04d S:%+05.1f", packet.pwm[2],
                  packet.speed[2]);
      OLED_Printf(0, 50, OLED_6X8, "M4 P:%+04d S:%+05.1f", packet.pwm[3],
                  packet.speed[3]);
    }
    /* 若未接收到数据，则保留上次显示内容，不额外处理 */
  }

  /* 左上角显示信号强度图标（基于最近10次发送成功率） */
  if (ctx->success_ratio >= 9U) {
    OLED_ShowImage(0, 0, 16, 16, Signal_3);
  } else if (ctx->success_ratio >= 5U) {
    OLED_ShowImage(0, 0, 16, 16, Signal_2);
  } else if (ctx->success_ratio >= 1U) {
    OLED_ShowImage(0, 0, 16, 16, Signal_1);
  } else {
    OLED_ShowImage(0, 0, 16, 16, Signal_0);
  }

  OLED_Update(); /* 将显存内容刷新到屏幕 */
}

/**
 * @brief 处理ADC原始值，加入死区并映射到有符号范围
 * @param value ADC原始值（0~4095）
 * @return int8_t 处理后摇杆值（范围 -100 ~ +100）
 * @note   - 以2048为中心，死区范围为 ±APP_AD_DEAD_ZONE
 *         - 超出死区后线性映射到 -100 ~ +100
 *         - 返回值可直接用于控制指令
 */
static int8_t App_Remote_ProcessAd(uint16_t value) {
  int32_t centered = (int32_t)value - 2048; /* 中心偏移 */

  /* 死区处理 */
  if (centered > APP_AD_DEAD_ZONE) {
    centered -= APP_AD_DEAD_ZONE;
  } else if (centered < -APP_AD_DEAD_ZONE) {
    centered += APP_AD_DEAD_ZONE;
  } else {
    centered = 0;
  }

  /* 映射到 -100 ~ +100 */
  centered = centered * 101 / (2049 - APP_AD_DEAD_ZONE);
  return (int8_t)centered;
}

/**
 * @brief 计算最近10次发送成功率
 * @param send_flag 本次发送结果（1成功，0失败）
 * @return uint8_t 成功次数（0~10）
 * @note   使用静态环形缓冲区保存最近10次发送结果
 */
static uint8_t App_Remote_CalculateSuccessRatio(uint8_t send_flag) {
  static uint8_t send_flags[10]; /* 环形缓冲区 */
  static uint8_t index;          /* 当前写入位置 */
  uint8_t success_count = 0U;

  /* 存入本次结果 */
  send_flags[index] = send_flag;
  index = (uint8_t)((index + 1U) % 10U);

  /* 统计成功次数 */
  for (uint8_t i = 0U; i < 10U; i++) {
    if (send_flags[i] == 1U) {
      success_count++;
    }
  }

  return success_count;
}
