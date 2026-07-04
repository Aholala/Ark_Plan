/**
 * @file app_chassis_task.c
 * @brief Chassis open-loop motor task.
 */

#include "app_chassis_task.h"

#include "cmsis_os.h"

#define CHASSIS_MOTOR_DUTY_MAX 95
#define CHASSIS_MOTOR_DUTY_MIN 5

#define CHASSIS_MOTOR_REVERSE_BRAKE_MS 80U

#define CHASSIS_MOTOR_RAMP_PERIOD_MS 5U
#define CHASSIS_MOTOR_RAMP_STEP 3

#define APP_CHASSIS_DEBUG_ENABLE 0U
#define APP_CHASSIS_DEBUG_MOTOR MODULE_MOTOR_1
#define APP_CHASSIS_DEBUG_DUTY 10

typedef struct {
  ModuleMotorId_t motor;
  int8_t target_duty;
  int8_t output_duty;
  uint8_t reverse_braking;
  uint32_t brake_start_tick;
} AppChassisMotorState_t;

static AppChassisMotorState_t app_chassis_motors[] = {
    {MODULE_MOTOR_1, 0, 0, 0U, 0U},
    {MODULE_MOTOR_2, 0, 0, 0U, 0U},
    {MODULE_MOTOR_3, 0, 0, 0U, 0U},
    {MODULE_MOTOR_4, 0, 0, 0U, 0U},
};

static uint32_t App_Chassis_MsToTicks(uint32_t milliseconds) {
  uint32_t tick_freq = osKernelGetTickFreq();

  return (uint32_t)(((uint64_t)milliseconds * tick_freq + 999ULL) / 1000ULL);
}

static uint8_t App_Chassis_IsMotorUsed(ModuleMotorId_t motor) {
  return (motor >= MODULE_MOTOR_1) && (motor <= MODULE_MOTOR_4);
}

static int8_t App_Chassis_FilterDuty(int8_t duty_percent) {
  if (duty_percent > CHASSIS_MOTOR_DUTY_MAX) {
    return CHASSIS_MOTOR_DUTY_MAX;
  }
  if (duty_percent < -CHASSIS_MOTOR_DUTY_MAX) {
    return -CHASSIS_MOTOR_DUTY_MAX;
  }
  if ((duty_percent > -CHASSIS_MOTOR_DUTY_MIN) &&
      (duty_percent < CHASSIS_MOTOR_DUTY_MIN)) {
    return 0;
  }

  return duty_percent;
}

static uint8_t App_Chassis_IsReverse(int8_t from_duty, int8_t to_duty) {
  return ((from_duty > 0) && (to_duty < 0)) ||
         ((from_duty < 0) && (to_duty > 0));
}

static int8_t App_Chassis_RampToward(int8_t current, int8_t target) {
  if (current < target) {
    current = (int8_t)(current + CHASSIS_MOTOR_RAMP_STEP);
    if (current > target) {
      current = target;
    }
  } else if (current > target) {
    current = (int8_t)(current - CHASSIS_MOTOR_RAMP_STEP);
    if (current < target) {
      current = target;
    }
  }

  return current;
}

static void App_Chassis_OutputMotor(AppChassisMotorState_t *state) {
  Module_Motor_SetDuty(state->motor, App_Chassis_FilterDuty(state->output_duty));
}

static void App_Chassis_UpdateMotor(AppChassisMotorState_t *state,
                                    uint32_t now) {
  if (state->reverse_braking != 0U) {
    state->output_duty = 0;
    Module_Motor_Stop(state->motor);

    if (state->target_duty == 0) {
      state->reverse_braking = 0U;
      return;
    }

    if ((uint32_t)(now - state->brake_start_tick) >=
        App_Chassis_MsToTicks(CHASSIS_MOTOR_REVERSE_BRAKE_MS)) {
      state->reverse_braking = 0U;
    } else {
      return;
    }
  }

  if (App_Chassis_IsReverse(state->output_duty, state->target_duty)) {
    state->output_duty = App_Chassis_RampToward(state->output_duty, 0);
    App_Chassis_OutputMotor(state);

    if (state->output_duty == 0) {
      state->reverse_braking = 1U;
      state->brake_start_tick = now;
      Module_Motor_Stop(state->motor);
    }
    return;
  }

  state->output_duty =
      App_Chassis_RampToward(state->output_duty, state->target_duty);
  App_Chassis_OutputMotor(state);
}

void App_Chassis_Init(void) {
  for (uint8_t i = 0U; i < (uint8_t)(sizeof(app_chassis_motors) /
                                     sizeof(app_chassis_motors[0]));
       i++) {
    app_chassis_motors[i].target_duty = 0;
    app_chassis_motors[i].output_duty = 0;
    app_chassis_motors[i].reverse_braking = 0U;
    app_chassis_motors[i].brake_start_tick = 0U;
    Module_Motor_Stop(app_chassis_motors[i].motor);
  }
}

void App_Chassis_SetMotorDuty(ModuleMotorId_t motor, int8_t duty_percent) {
  if (!App_Chassis_IsMotorUsed(motor)) {
    return;
  }

  app_chassis_motors[(uint8_t)motor].target_duty =
      App_Chassis_FilterDuty(duty_percent);
}

void App_Chassis_StopMotor(ModuleMotorId_t motor) {
  App_Chassis_SetMotorDuty(motor, 0);
}

void App_Chassis_StopAllMotors(void) {
  for (uint8_t i = 0U; i < (uint8_t)(sizeof(app_chassis_motors) /
                                     sizeof(app_chassis_motors[0]));
       i++) {
    app_chassis_motors[i].target_duty = 0;
  }
}

void App_Chassis_Task(void) {
  uint32_t now = osKernelGetTickCount();

  for (uint8_t i = 0U; i < (uint8_t)(sizeof(app_chassis_motors) /
                                     sizeof(app_chassis_motors[0]));
       i++) {
    App_Chassis_UpdateMotor(&app_chassis_motors[i], now);
  }
}

void StartChassicTask(void *argument) {
  (void)argument;

  App_Chassis_Init();

#if APP_CHASSIS_DEBUG_ENABLE
  App_Chassis_SetMotorDuty(APP_CHASSIS_DEBUG_MOTOR, APP_CHASSIS_DEBUG_DUTY);
#endif

  for (;;) {
    App_Chassis_Task();
    osDelay(App_Chassis_MsToTicks(CHASSIS_MOTOR_RAMP_PERIOD_MS));
  }
}

void StartChassisTask(void *argument) {
  StartChassicTask(argument);
}
