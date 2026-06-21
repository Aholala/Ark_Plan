#include "app_remote.h"

#include "bsp_ad.h"
#include "bsp_key.h"
#include "bsp_oled.h"
#include "bsp_remote_radio.h"
#include "bsp_timer.h"
#include "bsp_time.h"

#include <string.h>

#define APP_AD_DEAD_ZONE 100
#define APP_SEND_PERIOD_MS 50
#define APP_KEY_SEND_PERIOD_MS 5
#define APP_KEY_REPEAT_SUCCESS_COUNT 10
#define APP_KEY_BANNER_TIME_MS 100

typedef enum
{
  APP_REMOTE_WAIT_START = 0,
  APP_REMOTE_RUN,
} AppRemoteState;

typedef struct
{
  uint16_t ad_lh;
  uint16_t ad_lv;
  uint16_t ad_rh;
  uint16_t ad_rv;
  int8_t lh;
  int8_t lv;
  int8_t rh;
  int8_t rv;
  uint8_t key_pending;
  uint8_t key_num;
  uint8_t key_seq;
  uint8_t key_pending_seq;
  uint8_t key_repeat_remaining;
  uint8_t banner_key_num;
  uint8_t send_flag;
  uint8_t success_ratio;
  uint8_t mode;
  volatile uint8_t send_due;
  uint32_t key_banner_until;
  AppRemoteState state;
} AppRemoteContext;

static AppRemoteContext app_remote;
static uint16_t app_send_count;

static int8_t App_Remote_ProcessAd(uint16_t value);
static uint8_t App_Remote_CalculateSuccessRatio(uint8_t send_flag);
static uint8_t App_Remote_GetPressedKey(void);
static void App_Remote_ScanInputs(AppRemoteContext *ctx);
static void App_Remote_SendPacket(AppRemoteContext *ctx);
static void App_Remote_UpdateDisplay(AppRemoteContext *ctx);
static void App_Remote_ShowWelcome(void);

void App_Remote_Init(void)
{
  memset(&app_remote, 0, sizeof(app_remote));
  app_send_count = 0U;
  app_remote.state = APP_REMOTE_WAIT_START;

  OLED_Init();
  Key_Init();
  AD_Init();
  Bsp_RemoteRadio_Init();
  Timer_Init();

  App_Remote_ShowWelcome();
}

void App_Remote_Task(void)
{
  AppRemoteContext *ctx = &app_remote;

  if (ctx->state == APP_REMOTE_WAIT_START)
  {
    if (Key_Check(KEY_10, KEY_DOWN))
    {
      Key_Clear();
      OLED_Clear();
      ctx->state = APP_REMOTE_RUN;
    }
    return;
  }

  App_Remote_ScanInputs(ctx);

  if (ctx->send_due)
  {
    ctx->send_due = 0U;
    App_Remote_SendPacket(ctx);
  }

  App_Remote_UpdateDisplay(ctx);
}

void App_Remote_Tick1ms(void)
{
  Key_Tick();

  app_send_count++;
  if (app_send_count >= ((app_remote.key_pending != 0U) ? APP_KEY_SEND_PERIOD_MS : APP_SEND_PERIOD_MS))
  {
    app_send_count = 0U;
    app_remote.send_due = 1U;
  }
}

static void App_Remote_ShowWelcome(void)
{
  OLED_Clear();
  OLED_ShowString(0, 0, " Milkforg Team   ", OLED_8X16);
  OLED_ShowString(4, 16, " Remote Tester ", OLED_8X16);
  OLED_ShowString(0, 32, "      V1.0      ", OLED_8X16);
  OLED_ShowString(0, 48, "            K10>", OLED_8X16);
  OLED_Update();
}

static void App_Remote_ScanInputs(AppRemoteContext *ctx)
{
  ctx->key_num = App_Remote_GetPressedKey();

  ctx->ad_lh = AD_GetValue(AD_CHANNEL_LH);
  ctx->ad_lv = AD_GetValue(AD_CHANNEL_LV);
  ctx->ad_rh = AD_GetValue(AD_CHANNEL_RH);
  ctx->ad_rv = AD_GetValue(AD_CHANNEL_RV);

  ctx->lh = App_Remote_ProcessAd(ctx->ad_lh);
  ctx->lv = App_Remote_ProcessAd(ctx->ad_lv);
  ctx->rh = App_Remote_ProcessAd(ctx->ad_rh);
  ctx->rv = App_Remote_ProcessAd(ctx->ad_rv);

  if (ctx->key_num != 0U)
  {
    ctx->key_pending = ctx->key_num;
    ctx->key_seq++;
    if (ctx->key_seq == 0U)
    {
      ctx->key_seq = 1U;
    }
    ctx->key_pending_seq = ctx->key_seq;
    ctx->key_repeat_remaining = APP_KEY_REPEAT_SUCCESS_COUNT;
    ctx->banner_key_num = ctx->key_num;
    ctx->key_banner_until = Bsp_Time_GetMs() + APP_KEY_BANNER_TIME_MS;
    ctx->send_due = 1U;
  }

  if (ctx->key_num == 9U)
  {
    ctx->mode = !ctx->mode;
    OLED_Clear();
  }
}

static void App_Remote_SendPacket(AppRemoteContext *ctx)
{
  BspRemoteRadioControlPacket_t packet;

  packet.mode = ctx->mode;
  packet.lh = ctx->lh;
  packet.lv = ctx->lv;
  packet.rh = ctx->rh;
  packet.rv = ctx->rv;
  packet.key = ctx->key_pending;
  packet.key_seq = ctx->key_pending_seq;

  ctx->send_flag = Bsp_RemoteRadio_SendControl(&packet);
  if (ctx->send_flag == 1U)
  {
    if (ctx->key_repeat_remaining > 0U)
    {
      ctx->key_repeat_remaining--;
    }

    if (ctx->key_repeat_remaining == 0U)
    {
      ctx->key_pending = 0U;
    }
  }

  ctx->success_ratio = App_Remote_CalculateSuccessRatio(ctx->send_flag);
}

static uint8_t App_Remote_GetPressedKey(void)
{
  if (Key_Check(KEY_1, KEY_DOWN))
  {
    return 1U;
  }
  if (Key_Check(KEY_2, KEY_DOWN))
  {
    return 2U;
  }
  if (Key_Check(KEY_3, KEY_DOWN))
  {
    return 3U;
  }
  if (Key_Check(KEY_4, KEY_DOWN))
  {
    return 4U;
  }
  if (Key_Check(KEY_5, KEY_DOWN))
  {
    return 5U;
  }
  if (Key_Check(KEY_6, KEY_DOWN))
  {
    return 6U;
  }
  if (Key_Check(KEY_7, KEY_DOWN))
  {
    return 7U;
  }
  if (Key_Check(KEY_8, KEY_DOWN))
  {
    return 8U;
  }
  if (Key_Check(KEY_9, KEY_DOWN))
  {
    return 9U;
  }
  if (Key_Check(KEY_10, KEY_DOWN))
  {
    return 10U;
  }
  if (Key_Check(KEY_11, KEY_DOWN))
  {
    return 11U;
  }
  if (Key_Check(KEY_12, KEY_DOWN))
  {
    return 12U;
  }
  return 0U;
}

static void App_Remote_UpdateDisplay(AppRemoteContext *ctx)
{
  if (ctx->key_banner_until > Bsp_Time_GetMs())
  {
    OLED_Printf(104, 0, OLED_8X16, "K%d", ctx->banner_key_num);
  }
  else
  {
    OLED_ClearArea(104, 0, 24, 16);
  }

  if (ctx->mode == 0U)
  {
    OLED_Printf(0, 16, OLED_8X16, "LH:%+04d", ctx->lh);
    OLED_Printf(0, 32, OLED_8X16, "LV:%+04d", ctx->lv);
    OLED_Printf(72, 16, OLED_8X16, "RH:%+04d", ctx->rh);
    OLED_Printf(72, 32, OLED_8X16, "RV:%+04d", ctx->rv);
  }
  else if (ctx->mode == 1U)
  {
    BspRemoteRadioFeedbackPacket_t packet;

    if (Bsp_RemoteRadio_ReceiveFeedback(&packet) == 1U)
    {
      OLED_Clear();
      OLED_Printf(0, 21, OLED_6X8, "Angle:%+06.1f", packet.angle);
      OLED_DrawLine(0, 36, 127, 36);
      OLED_Printf(0, 40, OLED_6X8, "PWML:%+05d PWMR:%+05d", packet.pwm_l,
                  packet.pwm_r);
      OLED_Printf(0, 48, OLED_6X8, "SpdL:%+05.1f SpdR:%+05.1f",
                  packet.speed_left, packet.speed_right);
      OLED_Printf(0, 56, OLED_6X8, "SpdLevel:%1d", packet.speed_level);
    }
  }

  if (ctx->success_ratio >= 9U)
  {
    OLED_ShowImage(0, 0, 16, 16, Signal_3);
  }
  else if (ctx->success_ratio >= 5U)
  {
    OLED_ShowImage(0, 0, 16, 16, Signal_2);
  }
  else if (ctx->success_ratio >= 1U)
  {
    OLED_ShowImage(0, 0, 16, 16, Signal_1);
  }
  else
  {
    OLED_ShowImage(0, 0, 16, 16, Signal_0);
  }

  OLED_Update();
}

static int8_t App_Remote_ProcessAd(uint16_t value)
{
  int32_t centered = (int32_t)value - 2048;

  if (centered > APP_AD_DEAD_ZONE)
  {
    centered -= APP_AD_DEAD_ZONE;
  }
  else if (centered < -APP_AD_DEAD_ZONE)
  {
    centered += APP_AD_DEAD_ZONE;
  }
  else
  {
    centered = 0;
  }

  centered = centered * 101 / (2049 - APP_AD_DEAD_ZONE);
  return (int8_t)centered;
}

static uint8_t App_Remote_CalculateSuccessRatio(uint8_t send_flag)
{
  static uint8_t send_flags[10];
  static uint8_t index;
  uint8_t success_count = 0U;

  send_flags[index] = send_flag;
  index = (uint8_t)((index + 1U) % 10U);

  for (uint8_t i = 0U; i < 10U; i++)
  {
    if (send_flags[i] == 1U)
    {
      success_count++;
    }
  }

  return success_count;
}
