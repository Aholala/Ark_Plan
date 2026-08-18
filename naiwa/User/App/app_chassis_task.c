/**
 * @file app_chassis_task.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 底盘开环控制任务 - 支持遥控器控制、占空比缓启缓停及反向刹车
 * @version 1.0
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本任务实现四轮底盘的开环速度控制，主要功能包括：
 *          1.
 * 通过遥控器数据（通道值）进行麦克纳姆轮运动学解算，输出各电机目标占空比；
 *          2. 对每个电机进行占空比斜率限制（缓启动/缓停止），避免电流冲击；
 *          3. 方向切换时插入短暂的零占空比刹车（反向刹车），保护驱动器和电机；
 *          4. 提供单路/全部电机停止接口，支持外部控制。
 *          任务周期由宏 CHASSIS_MOTOR_RAMP_PERIOD_MS 控制，默认5ms。
 */

#include "app_chassis_task.h"
#include "app_remote_task.h"
#include "app_arm_task.h"
#include "cmsis_os.h"
#include "lib_mecanum.h"

/*==================== 宏定义 ====================*/

/** @brief 电机最大占空比（%），限制输出幅度 */
#define CHASSIS_MOTOR_DUTY_MAX 98

/** @brief 电机最小有效占空比（%），小于此值视为0，避免死区 */
#define CHASSIS_MOTOR_DUTY_MIN 1

/** @brief 方向反转时的刹车时间（毫秒） */
#define CHASSIS_MOTOR_REVERSE_BRAKE_MS 5U

/** @brief 占空比斜坡更新的周期（毫秒） */
#define CHASSIS_MOTOR_RAMP_PERIOD_MS 5U

/** @brief 占空比斜坡单步变化量（%），每周期递增/递减此值 */
#define CHASSIS_MOTOR_RAMP_STEP 5

/** @brief 遥控器摇杆死区（通道值），小于此值视为0 */
#define CHASSIS_REMOTE_DEADBAND 5

/*==================== 类型定义 ====================*/

/**
 * @brief 底盘电机状态结构体
 * @note 记录每个电机的目标占空比、当前输出占空比、反向刹车状态及时间戳
 */
typedef struct {
  ModuleMotorId_t motor;     /**< 电机编号（MODULE_MOTOR_1~4） */
  int8_t target_duty;        /**< 目标占空比（由遥控器或外部接口设定） */
  int8_t output_duty;        /**< 当前实际输出占空比（经过斜坡处理后） */
  uint8_t reverse_braking;   /**< 反向刹车标志（1=正在刹车，0=正常） */
  uint32_t brake_start_tick; /**< 刹车开始时刻的系统Tick值 */
} AppChassisMotorState_t;

/*==================== 静态变量 ====================*/

/**
 * @brief 底盘4个电机的状态数组
 * @note 顺序与 MODULE_MOTOR_1~4 对应，索引即电机编号
 */
static AppChassisMotorState_t app_chassis_motors[] = {
    {MODULE_MOTOR_1, 0, 0, 0U, 0U},
    {MODULE_MOTOR_2, 0, 0, 0U, 0U},
    {MODULE_MOTOR_3, 0, 0, 0U, 0U},
    {MODULE_MOTOR_4, 0, 0, 0U, 0U},
};

/*==================== 静态函数 ====================*/

/**
 * @brief 将毫秒转换为系统Tick数（向上取整）
 *
 * @param milliseconds 毫秒数
 * @return uint32_t 对应的Tick数
 * @note 使用 osKernelGetTickFreq() 获取系统Tick频率，进行精确转换。
 */
static uint32_t App_Chassis_MsToTicks(uint32_t milliseconds) {
  uint32_t tick_freq = osKernelGetTickFreq();

  return (uint32_t)(((uint64_t)milliseconds * tick_freq + 999ULL) / 1000ULL);
}

/**
 * @brief 检查电机编号是否属于底盘使用范围（1~4）
 *
 * @param motor 电机编号
 * @return uint8_t 是否有效
 * @retval 1 是底盘电机
 * @retval 0 不是底盘电机（如M5/M6）
 */
static uint8_t App_Chassis_IsMotorUsed(ModuleMotorId_t motor) {
  return (motor >= MODULE_MOTOR_1) && (motor <= MODULE_MOTOR_4);
}

/**
 * @brief 对占空比进行限幅和死区处理
 *
 * @param duty_percent 原始占空比
 * @return int8_t 滤波后的占空比
 * @note 限制在 ±CHASSIS_MOTOR_DUTY_MAX 内，
 *       且小于 CHASSIS_MOTOR_DUTY_MIN 的非零值被归零。
 */
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

/**
 * @brief 判断从当前占空比切换到目标占空比是否发生方向反转
 *
 * @param from_duty 当前占空比
 * @param to_duty   目标占空比
 * @return uint8_t 是否反转
 * @retval 1 发生反转（一正一负）
 * @retval 0 未反转（同向或其中一方为零）
 */
static uint8_t App_Chassis_IsReverse(int8_t from_duty, int8_t to_duty) {
  return ((from_duty > 0) && (to_duty < 0)) ||
         ((from_duty < 0) && (to_duty > 0));
}

/**
 * @brief 占空比斜坡处理：逐步逼近目标值
 *
 * @param current 当前值
 * @param target  目标值
 * @return int8_t 更新后的当前值（向目标靠近 CHASSIS_MOTOR_RAMP_STEP）
 * @note 若差值小于步长，直接设置为目标值。
 */
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

/**
 * @brief 对遥控器通道值应用死区
 *
 * @param value 原始通道值
 * @return int8_t 死区处理后值（小于死区范围则归零）
 */
static int8_t App_Chassis_ApplyDeadband(int8_t value) {
  if ((value > -CHASSIS_REMOTE_DEADBAND) && (value < CHASSIS_REMOTE_DEADBAND)) {
    return 0;
  }

  return value;
}

/**
 * @brief 通过遥控器数据更新电机目标占空比
 * @note 若遥控器未连接，则停止所有电机。
 *       否则进行麦克纳姆轮运动学解算，将摇杆值映射为四轮占空比。
 */
static void App_Chassis_UpdateRemoteControl(void) {
  AppRemoteData_t remote;
  App_Remote_GetSnapshot(&remote);
  LibMecanumOutput_t output;

  if (remote.connected == 0U) {
    App_Chassis_StopAllMotors();
    return;
  }

  /* 运动学解算 */
  if (App_Arm_IsControlActive() != 0U) {
    /* 机械臂模式：左摇杆归机械臂，右摇杆保留底盘旋转 */
    {
      int8_t yaw_raw = App_Chassis_ApplyDeadband(remote.rh);
      int8_t yaw = (int8_t)(((int16_t)yaw_raw * 6) / 10);
      Lib_Mecanum_Mix(0, 0, yaw, CHASSIS_MOTOR_DUTY_MAX, &output);
    }
  } else {
    /* 底盘模式：全摇杆控制 */
    Lib_Mecanum_Mix(-App_Chassis_ApplyDeadband(remote.lh),
                    App_Chassis_ApplyDeadband(remote.lv),
                    App_Chassis_ApplyDeadband(remote.rh), CHASSIS_MOTOR_DUTY_MAX,
                    &output);
  }

  /* 设置四个电机的目标占空比 */
  App_Chassis_SetMotorDuty(MODULE_MOTOR_1, output.m1);
  App_Chassis_SetMotorDuty(MODULE_MOTOR_2, output.m2);
  App_Chassis_SetMotorDuty(MODULE_MOTOR_3, output.m3);
  App_Chassis_SetMotorDuty(MODULE_MOTOR_4, output.m4);
}

/**
 * @brief 将电机状态中的输出占空比实际写入硬件
 *
 * @param state 电机状态指针
 * @note 内部调用 Module_Motor_SetDuty，并应用滤波。
 */
static void App_Chassis_OutputMotor(AppChassisMotorState_t *state) {
  Module_Motor_SetDuty(state->motor,
                       App_Chassis_FilterDuty(state->output_duty));
}

/**
 * @brief 更新单个电机的状态（包括斜坡、刹车逻辑）
 *
 * @param state 电机状态指针
 * @param now   当前系统Tick值
 * @note 若处于反向刹车状态，则维持输出0并等待刹车时间结束；
 *       否则判断是否需要进入反向刹车，或执行正常斜坡逼近。
 */
static void App_Chassis_UpdateMotor(AppChassisMotorState_t *state,
                                    uint32_t now) {
  /* 阶段1：反向刹车中 */
  if (state->reverse_braking != 0U) {
    state->output_duty = 0;
    Module_Motor_Stop(state->motor);

    /* 若目标已变为0，提前退出刹车状态 */
    if (state->target_duty == 0) {
      state->reverse_braking = 0U;
      return;
    }

    /* 检查刹车时间是否已到 */
    if ((uint32_t)(now - state->brake_start_tick) >=
        App_Chassis_MsToTicks(CHASSIS_MOTOR_REVERSE_BRAKE_MS)) {
      state->reverse_braking = 0U;
    } else {
      return; /* 刹车未结束，本次不更新输出 */
    }
  }

  /* 阶段2：判断是否即将发生方向反转 */
  if (App_Chassis_IsReverse(state->output_duty, state->target_duty)) {
    /* 先将输出占空比斜坡降至0，触发刹车 */
    state->output_duty = App_Chassis_RampToward(state->output_duty, 0);
    App_Chassis_OutputMotor(state);

    /* 一旦达到0，立即进入刹车状态并记录起始Tick */
    if (state->output_duty == 0) {
      state->reverse_braking = 1U;
      state->brake_start_tick = now;
      Module_Motor_Stop(state->motor);
    }
    return;
  }

  /* 阶段3：正常斜坡逼近目标 */
  state->output_duty =
      App_Chassis_RampToward(state->output_duty, state->target_duty);
  App_Chassis_OutputMotor(state);
}

/*==================== 全局API函数 ====================*/

/**
 * @brief 初始化底盘电机状态（停止所有电机，清除内部状态）
 * @note 应在任务启动前调用，确保电机处于安全状态。
 */
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

/**
 * @brief 设置底盘电机的目标占空比（外部调用接口）
 *
 * @param motor         电机编号（必须为 MODULE_MOTOR_1~4）
 * @param duty_percent  目标占空比（-100~100），内部会滤波限幅
 * @note 若电机编号不在底盘范围内，函数直接返回。
 */
void App_Chassis_SetMotorDuty(ModuleMotorId_t motor, int8_t duty_percent) {
  if (!App_Chassis_IsMotorUsed(motor)) {
    return;
  }

  app_chassis_motors[(uint8_t)motor].target_duty =
      App_Chassis_FilterDuty(duty_percent);
}

/**
 * @brief 停止指定底盘电机（目标占空比设为0）
 *
 * @param motor 电机编号
 */
void App_Chassis_StopMotor(ModuleMotorId_t motor) {
  App_Chassis_SetMotorDuty(motor, 0);
}

/**
 * @brief 停止所有底盘电机（目标占空比归零）
 * @note 不会立即停止输出，而是通过斜坡逐渐降至0。
 */
void App_Chassis_StopAllMotors(void) {
  for (uint8_t i = 0U; i < (uint8_t)(sizeof(app_chassis_motors) /
                                     sizeof(app_chassis_motors[0]));
       i++) {
    app_chassis_motors[i].target_duty = 0;
  }
}

/**
 * @brief 底盘任务主函数（周期调用）
 * @note 此函数需以固定周期（CHASSIS_MOTOR_RAMP_PERIOD_MS）被调用。
 *       其工作流程：
 *       1. 读取遥控器数据并更新目标占空比；
 *       2. 遍历所有电机，执行状态更新（斜坡/刹车/输出）。
 */
void App_Chassis_Task(void) {
  uint32_t now = osKernelGetTickCount();

  /* 更新遥控器目标值 */
  App_Chassis_UpdateRemoteControl();

  /* 更新每个电机的输出 */
  for (uint8_t i = 0U; i < (uint8_t)(sizeof(app_chassis_motors) /
                                     sizeof(app_chassis_motors[0]));
       i++) {
    App_Chassis_UpdateMotor(&app_chassis_motors[i], now);
  }
}

/**
 * @brief 底盘任务入口函数（FreeRTOS任务）
 *
 * @param argument 任务参数（未使用）
 * @note 任务初始化后周期性调用 App_Chassis_Task。
 * @warning 函数名拼写为 StartChassicTask（历史遗留），为保持兼容，
 *          另外提供了 StartChassisTask 作为别名。
 */
void StartChassicTask(void *argument) {
  (void)argument;

  App_Chassis_Init();

  for (;;) {
    App_Chassis_Task();
    osDelay(App_Chassis_MsToTicks(CHASSIS_MOTOR_RAMP_PERIOD_MS));
  }
}

/**
 * @brief 底盘任务入口函数（正确拼写，供外部调用）
 *
 * @param argument 任务参数（未使用）
 * @note 直接调用 StartChassicTask 实现，以兼容两种命名。
 */
void StartChassisTask(void *argument) { StartChassicTask(argument); }