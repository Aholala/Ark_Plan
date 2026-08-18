/**
 * @file app_arm_task.c
 * @author 梁源锋
 * @brief 双轴连杆机械臂控制任务 - 四杆机构运动学实现
 * @version 3.0
 * @date 2026-07-10
 *
 * @copyright Copyright (c) 2026
 *
 * @details 机械结构（平面串联臂+四杆驱动）：
 *
 *          底座（肩关节）:
 *            M5 ──直驱──→ 大臂(380mm) ──→ 肘关节 ──→ 小臂(270mm) ──→ 末端
 *            M6 ──直驱──→ 曲柄(90mm) ──→ 连杆(380mm) ──→ 小臂上距肘90mm处
 *
 *          运动学链路：
 *          ┌── 2连杆IK:  (x,z) → (θ₅_tgt, θ_rel_tgt)
 *          └── 4杆 IK:   (θ₅_tgt, θ_rel_tgt) → θ_crank_tgt
 *          ┌── 4杆 FK:   (θ₅, θ_crank) → θ_rel
 *          └── 2连杆FK:  (θ₅, θ_rel) → (x,z)
 *
 *          角度定义（侧视图，Y轴竖直向上，X轴水平向前）：
 *          θ₅     = 大臂相对Y轴角度，0°=竖直向上，正=前倾（顺时针）
 *          θ_rel  = 小臂相对大臂角度，0°=与大臂共线，正=前展
 *          θ_crank= M6曲柄相对Y轴角度
 *
 *          机械限位（实测FK输出，0°=与大臂共线，顺时针为正）：
 *          M5大臂：θ₅ ∈ [-20°, 70°]（0°=竖直向上）
 *          M6肘关节：θ_rel ∈ [34°, 146°]
 *              146°=折叠，34°=展开
 */

#include "app_arm_task.h"
#include "app_remote_task.h"
#include "cmsis_os.h"
#include <math.h>
#include "module_motor.h"
#include "module_pid.h"
#include "tim.h"
#include "stm32f4xx_hal_flash.h"
#include "gpio.h"

/*==================== Flash 校准持久化 ====================*/
#define ARM_FLASH_ADDR        0x080E0000u  /* Sector 11 起始地址（128KB空闲） */
#define ARM_FLASH_SECTOR      FLASH_SECTOR_11
#define ARM_FLASH_MAGIC       0x41524D05u  /* "ARM\5" */

/*==================== 机械臂物理参数 ====================*/

#define ARM_L1_MM       380.0f /**< 大臂长度（肩→肘，mm） */
#define ARM_L2_MM       270.0f /**< 小臂长度（肘→末端，mm） */
#define ARM_D_CRANK      90.0f /**< M6曲柄长度（mm） */
#define ARM_D_ROD        380.0f /**< 连杆长度（曲柄→小臂附接点，mm） */
#define ARM_D_ATTACH     90.0f /**< 附接点距肘距离（mm，沿小臂方向） */
#define ARM_GEAR_RATIO   139.0f /**< 减速比 139:1 */
#define ARM_ENCODER_CPR   48.0f /**< 编码器每转计数（4倍频后） */

/** 编码器每关节圈计数 = 48*139 = 6672 */
#define ARM_COUNTS_PER_JOINT_REV (ARM_ENCODER_CPR * ARM_GEAR_RATIO)

/** 编码器计数→度 = 360/6672 = 0.05396 */
#define ARM_DEG_PER_COUNT (360.0f / ARM_COUNTS_PER_JOINT_REV)

/*==================== 机械限位（度） ====================*/

/** M5 参考系：0°=竖直向上（Y轴），正=前倾，负=后仰 */
#define ARM_M5_ANGLE_MIN_DEG      -18.0f  /* 后仰下限位 */
#define ARM_M5_ANGLE_MAX_DEG       70.0f  /* 前倾上限位 */
/** M5 内部偏移：直接Y轴角，无需转换 */
#define ARM_M5_REF_OFFSET_DEG     0.0f

/** M6肘关节夹角：θ_rel ∈ [34°, 146°]（0°=与大臂共线，顺时针为正，实测FK输出）
 *  146°=折叠，34°=展开 */
#define ARM_M6_REL_ANGLE_MIN_DEG   34.0f
#define ARM_M6_REL_ANGLE_MAX_DEG  165.0f

/*==================== 控制参数 ====================*/

#define ARM_PID_DT_S               0.005f /**< PID周期（200Hz） */
#define ARM_TASK_PERIOD_MS         5U     /**< RTOS任务周期 */
#define ARM_REMOTE_DEADBAND        8      /**< 摇杆死区 */
#define ARM_JOYSTICK_GAIN_MM_PER_S 120.0f /**< 摇杆→目标速度映射 */
#define ARM_AXIS_LOCK_RATIO          1.5f /**< 主轴明显占优时锁定另一轴 */
#define ARM_CART_TRACK_KP            5.0f /**< 笛卡尔位置误差修正增益(1/s) */
#define ARM_CART_SPEED_LIMIT_MM_S  300.0f /**< 笛卡尔合成速度限幅(mm/s) */
#define ARM_JOINT_FF_LIMIT_DPS      300.0f /**< 关节速度前馈限幅(deg/s) */
#define ARM_TARGET_X_MIN_MM        50.0f  /* 折叠 x≈88 */
#define ARM_TARGET_X_MAX_MM       650.0f  /* 展开 x≈619 */
#define ARM_TARGET_Z_MIN_MM         0.0f  /* 最低 */
#define ARM_TARGET_Z_MAX_MM       650.0f  /* 最高 z≈619 */
#define ARM_HOME_X_MM             90.0f  /* 归位点 X */
#define ARM_HOME_Z_MM             195.0f  /* 归位点 Z */
#define ARM_HOME_SPEED_MM_PER_S    200.0f  /* 归位速度（mm/s） */
#define ARM_HOME_TOLERANCE_MM      5.0f   /* 到点阈值 */

#define ARM_GRAB_X_MM             500.0f  /* 抓取点 X */
#define ARM_GRAB_Z_MM             29.0f  /* 抓取点 Z */
#define ARM_GRAB_SPEED_MM_PER_S  200.0f   /* 抓取移动速度 */
#define ARM_THROW_X_MM            175.0f  /* 投球点 X */
#define ARM_THROW_Z_MM            120.0f  /* 投球点 Z */
#define ARM_THROW_SPEED_MM_PER_S 200.0f   /* 投球移动速度 */
#define ARM_POS_PID_OUTPUT_LIMIT   400.0f /**< 位置PID输出限幅(deg/s) */
#define ARM_VEL_PID_OUTPUT_LIMIT   95.0f  /**< 速度PID输出限幅(%) */
#define ARM_POS_PID_INTEGRAL_LIMIT 40.0f
#define ARM_VEL_PID_INTEGRAL_LIMIT 30.0f
/* M5 大臂 PID */
#define ARM_M5_POS_KP_DEFAULT      23.0f
#define ARM_M5_POS_KI_DEFAULT      0.6f
#define ARM_M5_POS_KD_DEFAULT      0.0f
#define ARM_M5_VEL_KP_DEFAULT      1.5f
#define ARM_M5_VEL_KI_DEFAULT      0.8f
#define ARM_M5_VEL_KD_DEFAULT      0.0f

/* M6 小臂 PID */
#define ARM_M6_POS_KP_DEFAULT      23.0f
#define ARM_M6_POS_KI_DEFAULT      0.6f
#define ARM_M6_POS_KD_DEFAULT      0.0f
#define ARM_M6_VEL_KP_DEFAULT      1.5f
#define ARM_M6_VEL_KI_DEFAULT      0.8f
#define ARM_M6_VEL_KD_DEFAULT      0.0f
#define ARM_DERIV_FILTER_ALPHA     0.15f
#define ARM_IK_TOLERANCE_MM        1.0f
#define ARM_REMOTE_LOST_LIMIT      100U  /**< 断连保护阈值(周期数) */

/** 调试模式：1=跳过遥控器检测，Ozone 直接改 arm_target_x_mm/z_mm 调 */
#define ARM_DEBUG_NO_REMOTE        0
/** 位置环速度前馈系数：目标变化率 × kff 直接叠加到速度指令，绕过PID延迟 */
float arm_debug_pos_kff = 0.0f;
/** 调试模式：非零=绕过PID，直接发此duty(%)给M5。设0走正常PID */
float arm_debug_raw_duty_m5 = 0.0f;
/** 调试模式：非零=绕过PID，直接发此duty(%)给M6。设0走正常PID */
float arm_debug_raw_duty_m6 = 0.0f;
/** 调试模式：1=在线校准零位（掰到折叠限位后设1，自动归0） */
float arm_debug_do_calibrate = 0.0f;
/** 调试模式：非零=固定目标角度（下限位=0°），设 0 则用方波模式 */
float arm_debug_m5_step = 10.0f;
/** 调试模式：1=启用M5位置控制 */
float arm_debug_enable_m5 = 0.0f;
/** 调试模式：1=启用M6，非零raw_duty走裸duty，否则走PID */
float arm_debug_enable_m6 = 0.0f;
/** 方波低位目标（下限位=0°） */
float arm_debug_square_lo = 10.0f;
/** 方波高位目标（下限位=0°） */
float arm_debug_square_hi = 40.0f;
/** 方波半周期（任务周期数，5ms/周期，200 = 1秒） */
uint16_t arm_debug_square_period = 200U;
/** Ozone 直接控制：1=用 arm_debug_tgt_x_mm/z_mm 覆盖摇杆目标 */
float arm_debug_override_target = 0.0f;
float arm_debug_tgt_x_mm = 350.0f; /**< Ozone 写入：目标 X (mm) */
float arm_debug_tgt_z_mm = 280.0f; /**< Ozone 写入：目标 Z (mm) */
/** 调试只读：当前编码器原始值（counts），直接读，不经任何换算 */
int32_t arm_debug_enc_raw_m5 = 0;  /**< M5 编码器 raw */
int32_t arm_debug_enc_raw_m6 = 0;  /**< M6 编码器 raw */
/** 调试只读：编码器换算角度（度），带校准方向+偏移 */
float arm_debug_enc_deg_m5 = 0.0f;  /**< M5 大臂角度 θ₅（下限位=0°） */
float arm_debug_enc_deg_m6 = 0.0f;  /**< M6 曲柄角度 θ_crank */
/** 调试只读：末端目标/实际坐标（mm），Ozone 看跟随情况 */
volatile float arm_debug_target_x_mm = 0.0f;   /**< 目标 X */
volatile float arm_debug_target_z_mm = 0.0f;   /**< 目标 Z */
volatile float arm_debug_actual_x_mm = 0.0f;   /**< 实际 X（FK） */
volatile float arm_debug_actual_z_mm = 0.0f;   /**< 实际 Z（FK） */
/** 调试只读：任务心跳，每5ms+1，Ozone看这个确认任务在跑 */
volatile uint32_t arm_debug_heartbeat = 0U;
/** 调试只读：TIM2/TIM3硬件计数器直读（用__HAL_TIM_GET_COUNTER），绕过所有中间层 */
volatile int32_t arm_debug_tim2_cnt = 0;  /**< TIM2 CNT寄存器（M5编码器硬件值） */
volatile int32_t arm_debug_tim3_cnt = 0;  /**< TIM3 CNT寄存器（M6编码器硬件值） */
/** 调试只读：遥控器数据镜像，Ozone 直接看遥控器是否收到值 */
volatile int8_t  arm_debug_rh = 0;       /**< 遥控器右摇杆水平 */
volatile int8_t  arm_debug_rv = 0;       /**< 遥控器右摇杆垂直 */
volatile int8_t  arm_debug_lh = 0;       /**< 遥控器左摇杆水平 */
volatile int8_t  arm_debug_lv = 0;       /**< 遥控器左摇杆垂直 */
volatile uint8_t arm_debug_remote_connected = 0U; /**< 遥控器连接状态 */
volatile uint8_t arm_debug_remote_key = 0U;       /**< 遥控器当前按键 */
volatile uint8_t arm_debug_relay_state = 1U;       /**< 电磁阀状态（PC3：1=低电平吸气，0=高电平放气） */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD2DEG (180.0f / M_PI)
#define DEG2RAD (M_PI / 180.0f)

/*==================== 四杆预计算常量 ====================*/

/** FK分母: 2*d_attach = 180 */
#define FK4_DENOM (2.0f * ARM_D_ATTACH)
/** IK分母: 2*d_crank = 180 */
#define IK4_DENOM (2.0f * ARM_D_CRANK)
/** FK分子常数: L_rod² - d_attach² = 136300 */
#define FK4_CONST (ARM_D_ROD * ARM_D_ROD - ARM_D_ATTACH * ARM_D_ATTACH)
/** IK分子常数: d_crank² - L_rod² = -136300 */
#define IK4_CONST (ARM_D_CRANK * ARM_D_CRANK - ARM_D_ROD * ARM_D_ROD)

/*==================== 类型定义 ====================*/

/** 四杆IK结果 */
typedef struct {
  float theta_crank_deg; /**< 曲柄角度（度） */
  uint8_t valid;         /**< 是否有效（cos在[-1,1]内） */
} Arm4BarIKResult_t;

/** 四杆FK结果 */
typedef struct {
  float theta_rel_deg;   /**< 小臂相对角（度） */
  uint8_t valid;         /**< 是否有效 */
} Arm4BarFKResult_t;

/** 双环PID电机控制器 */
typedef struct {
  ModulePid_t pos_pid;     /**< 位置环PID */
  ModulePid_t vel_pid;     /**< 速度环PID */
  float angle_target;      /**< 目标角度（度，斜坡平滑后） */
  float angle_current;     /**< 当前角度（度） */
  float vel_current;       /**< 当前速度（deg/s） */
  float vel_filtered;      /**< 速度低通滤波值 */
  int32_t enc_last;        /**< 上次编码器值 */
} AppArmMotorCtrl_t;

/*==================== 静态变量 ====================*/

static AppArmMotorCtrl_t arm_m5_ctrl;  /**< M5：大臂角度θ₅ */
static AppArmMotorCtrl_t arm_m6_ctrl;  /**< M6：曲柄角度θ_crank */
static float arm_target_x_mm = 350.0f;
static float arm_target_z_mm = 280.0f;
static float arm_cart_prev_x_mm = 350.0f;
static float arm_cart_prev_z_mm = 280.0f;
static uint8_t arm_cart_prev_valid = 0U;
static uint8_t arm_ik_valid = 0U;
static AppArmState_t arm_state;
static uint32_t arm_remote_lost_count = 0U;
static volatile uint8_t arm_control_active = 0U; /**< 1=机械臂遥控模式（跨任务共享） */
static uint8_t  arm_homing_active = 0U;       /**< 1=正在归位中 */
static uint8_t  arm_grab_active = 0U;         /**< 1=正在抓取移动中 */
static uint8_t  arm_throw_active = 0U;        /**< 1=正在投球移动中 */
static uint8_t  arm_disconnect_homing = 0U;   /**< 1=失联触发的归位，完成后切回底盘 */

/** 编码器校准：编码器raw→物理角度的变换 = raw * DEG_PER_COUNT * dir + offset */
static float arm_enc_offset_m5_deg = 0.0f; /**< M5编码器零位偏移（度） */
static float arm_enc_offset_m6_deg = 0.0f; /**< M6编码器零位偏移（度） */
static float arm_enc_dir_m5 = -1.0f;       /**< M5方向（+1或-1） */
static float arm_enc_dir_m6 = -1.0f;       /**< M6方向（+1或-1） */

/*==================== 四杆机构运动学 ====================*/

/**
 * @brief 四杆机构正向运动学（FK）：(θ₅, θ_crank) → θ_rel
 *
 * @param theta5_deg     大臂角度（度，从Y轴）
 * @param theta_crank_deg 曲柄角度（度，从Y轴）
 * @return Arm4BarFKResult_t 结果（含有效标志）
 *
 * @note  推导：
 *        附接点 D(θ₅,θ_rel):  D = 肘 + d_attach·(sin(θ₅+θ_rel), cos(θ₅+θ_rel))
 *        曲柄末端 C(θ_crank): C = d_crank·(sinθ_crank, cosθ_crank)
 *        约束 |D-C|² = L_rod²
 *        令 V = 肘 - C,  则 |V + d_attach·u|² = L_rod²
 *        → cos(θ₅+θ_rel - φ_v) = (L_rod² - |V|² - d_attach²) / (2·d_attach·|V|)
 *        → θ_rel = φ_v - θ₅ ± acos(K/|V|)
 *
 *        两个解支对应四杆的两种装配模式。根据当前曲柄位置选择正确分支。
 */
static Arm4BarFKResult_t App_Arm_4Bar_FK(float theta5_deg,
                                          float theta_crank_deg) {
  Arm4BarFKResult_t res = {0.0f, 0U};
  /* θ₅入参为下限位参考系，trig需转回Y轴角 */
  float t5  = (theta5_deg - ARM_M5_REF_OFFSET_DEG) * DEG2RAD;
  float tcr = theta_crank_deg * DEG2RAD;

  /* 肘关节 (大臂末端) */
  float Ex = ARM_L1_MM * sinf(t5);
  float Ey = ARM_L1_MM * cosf(t5);

  /* 曲柄末端 C */
  float Cx = ARM_D_CRANK * sinf(tcr);
  float Cy = ARM_D_CRANK * cosf(tcr);

  /* V = E - C */
  float Vx = Ex - Cx;
  float Vy = Ey - Cy;
  float V_sq = Vx * Vx + Vy * Vy;
  float V_mag = sqrtf(V_sq);

  /* 除零保护：肘与曲柄末端重合时无法确定方向 */
  if (V_mag < 1e-6f) {
    res.valid = 0U;
    res.theta_rel_deg = arm_state.m6_angle_deg; /* 保持上次值 */
    return res;
  }

  /* K = (L_rod² - |V|² - d_attach²) / (2*d_attach) */
  float K = (FK4_CONST - V_sq) / FK4_DENOM;

  float cos_val = K / V_mag;
  if (cos_val > 1.0f || cos_val < -1.0f) {
    /* 不可达：四杆无法装配，钳位后返回无效标志 */
    res.valid = 0U;
    if (cos_val > 1.0f)  cos_val = 1.0f;
    if (cos_val < -1.0f) cos_val = -1.0f;
  } else {
    res.valid = 1U;
  }

  float phi_v = atan2f(Vx, Vy);   /* V方向角（从Y轴） */
  float delta = acosf(cos_val);   /* 正值，范围[0, π] */

  /* 两个解支: θ_rel = φ_v - θ₅ ± delta */
  float sol_minus = phi_v - t5 - delta;  /* "-" 解 */
  float sol_plus  = phi_v - t5 + delta;  /* "+" 解 */

  /* 选择最接近[62.9°, 121.12°]的解（取范围内，否则取最近的） */
  float d_minus = sol_minus * RAD2DEG;
  float d_plus  = sol_plus * RAD2DEG;

  uint8_t minus_ok = (d_minus >= ARM_M6_REL_ANGLE_MIN_DEG &&
                      d_minus <= ARM_M6_REL_ANGLE_MAX_DEG);
  uint8_t plus_ok  = (d_plus >= ARM_M6_REL_ANGLE_MIN_DEG &&
                      d_plus <= ARM_M6_REL_ANGLE_MAX_DEG);

  if (minus_ok && !plus_ok) {
    res.theta_rel_deg = d_minus;
  } else if (plus_ok && !minus_ok) {
    res.theta_rel_deg = d_plus;
  } else if (minus_ok && plus_ok) {
    /* 两个都在范围内：选距离上次读值更近的（保持连续性，含360°环绕） */
    float prev_rel = arm_state.m6_angle_deg;
    float diff_m = fabsf(d_minus - prev_rel);
    float diff_p = fabsf(d_plus - prev_rel);
    if (diff_m > 180.0f) diff_m = 360.0f - diff_m;
    if (diff_p > 180.0f) diff_p = 360.0f - diff_p;
    res.theta_rel_deg = (diff_m <= diff_p) ? d_minus : d_plus;
  } else {
    /* 都不在范围内：取误差较小的（四杆可能已到达极限之外） */
    float err_m = (d_minus < ARM_M6_REL_ANGLE_MIN_DEG)
                   ? (ARM_M6_REL_ANGLE_MIN_DEG - d_minus)
                   : (d_minus - ARM_M6_REL_ANGLE_MAX_DEG);
    float err_p = (d_plus < ARM_M6_REL_ANGLE_MIN_DEG)
                   ? (ARM_M6_REL_ANGLE_MIN_DEG - d_plus)
                   : (d_plus - ARM_M6_REL_ANGLE_MAX_DEG);
    res.theta_rel_deg = (err_m <= err_p) ? d_minus : d_plus;
    res.valid = 0U;
  }

  return res;
}

/**
 * @brief 四杆机构逆运动学（IK）：(θ₅, θ_rel) → θ_crank
 *
 * @param theta5_deg   大臂角度（度）
 * @param theta_rel_deg 小臂相对角（度）
 * @return Arm4BarIKResult_t 结果
 *
 * @note  已知θ₅和θ_rel，求解θ_crank。
 *        从约束反解：
 *        D = 肘 + d_attach·(sin(θ₅+θ_rel), cos(θ₅+θ_rel))
 *        |D-C|² = L_rod² (C = 曲柄末端)
 *        → cos(φ_d - θ_crank) = (|D|² + d_crank² - L_rod²) / (2·d_crank·|D|)
 *        → θ_crank = φ_d ± acos(K/|D|)
 *
 *        选择最接近当前曲柄角度的解（保持连续性）。
 */
static Arm4BarIKResult_t App_Arm_4Bar_IK(float theta5_deg,
                                          float theta_rel_deg) {
  Arm4BarIKResult_t res = {0.0f, 0U};
  /* θ₅入参为下限位参考系，trig需转回Y轴角 */
  float t5   = (theta5_deg - ARM_M5_REF_OFFSET_DEG) * DEG2RAD;
  float trel = theta_rel_deg * DEG2RAD;

  /* 附接点D */
  float Dx = ARM_L1_MM * sinf(t5) + ARM_D_ATTACH * sinf(t5 + trel);
  float Dy = ARM_L1_MM * cosf(t5) + ARM_D_ATTACH * cosf(t5 + trel);
  float D_sq = Dx * Dx + Dy * Dy;
  float D_mag = sqrtf(D_sq);

  /* 除零保护：附接点与原点重合时无法确定方向 */
  if (D_mag < 1e-6f) {
    res.valid = 0U;
    res.theta_crank_deg = arm_m6_ctrl.angle_current; /* 保持上次值 */
    return res;
  }

  float phi_d = atan2f(Dx, Dy);  /* D方向角（从Y轴） */

  /* K = (|D|² + d_crank² - L_rod²) / (2*d_crank) = (|D|² + IK4_CONST) / IK4_DENOM */
  float K = (D_sq + IK4_CONST) / IK4_DENOM;

  float cos_val = K / D_mag;
  if (cos_val > 1.0f || cos_val < -1.0f) {
    res.valid = 0U;
    if (cos_val > 1.0f)  cos_val = 1.0f;
    if (cos_val < -1.0f) cos_val = -1.0f;
  } else {
    res.valid = 1U;
  }

  float delta = acosf(cos_val);

  /* 两个解: θ_crank = φ_d ± delta */
  float sol_minus = (phi_d - delta) * RAD2DEG;
  float sol_plus  = (phi_d + delta) * RAD2DEG;

  /* 选择最接近当前曲柄角度的解 */
  float prev_crank = arm_m6_ctrl.angle_current;
  float diff_m = fabsf(sol_minus - prev_crank);
  float diff_p = fabsf(sol_plus - prev_crank);

  /* 同时考虑180°环绕 */
  if (diff_m > 180.0f) diff_m = 360.0f - diff_m;
  if (diff_p > 180.0f) diff_p = 360.0f - diff_p;

  res.theta_crank_deg = (diff_m <= diff_p) ? sol_minus : sol_plus;
  return res;
}

/*==================== 2连杆运动学 ====================*/

/**
 * @brief 2连杆逆运动学：(x,z) → (θ₅, θ_rel)
 *
 * @param x_mm,z_mm      末端坐标（mm）
 * @param theta5_deg     输出：大臂角度（度）
 * @param theta_rel_deg  输出：小臂相对角（度）
 * @return uint8_t 1=可达, 0=不可达
 *
 * @note  余弦定理:
 *        D² = L1² + L2² + 2·L1·L2·cos(θ_rel)
 *        → cos(θ_rel) = (D² - L1² - L2²) / (2·L1·L2)
 *        θ₅ = atan2(x,z) - atan2(L2·sinθ_rel, L1 + L2·cosθ_rel)
 */
static uint8_t App_Arm_2Link_IK(float x_mm, float z_mm,
                                float *theta5_deg, float *theta_rel_deg) {
  float d_sq = x_mm * x_mm + z_mm * z_mm;
  float d = sqrtf(d_sq);

  /* 可达性：|L1-L2| ≤ D ≤ L1+L2 */
  if (d > (ARM_L1_MM + ARM_L2_MM + ARM_IK_TOLERANCE_MM)) return 0U;
  if (d < fabsf(ARM_L1_MM - ARM_L2_MM) - ARM_IK_TOLERANCE_MM) return 0U;

  /* cos(θ_rel) = (D² - L1² - L2²) / (2·L1·L2) */
  float cos_rel = (d_sq - ARM_L1_MM * ARM_L1_MM - ARM_L2_MM * ARM_L2_MM)
                / (2.0f * ARM_L1_MM * ARM_L2_MM);
  if (cos_rel > 1.0f) cos_rel = 1.0f;
  if (cos_rel < -1.0f) cos_rel = -1.0f;

  float trel_rad = acosf(cos_rel);
  float t5_rad   = atan2f(x_mm, z_mm)
                 - atan2f(ARM_L2_MM * sinf(trel_rad),
                          ARM_L1_MM + ARM_L2_MM * cosf(trel_rad));

  *theta5_deg    = t5_rad * RAD2DEG + ARM_M5_REF_OFFSET_DEG;
  *theta_rel_deg = trel_rad * RAD2DEG;

  /* 归一化 M5 到 [-180,180] */
  while (*theta5_deg > 180.0f)  *theta5_deg -= 360.0f;
  while (*theta5_deg < -180.0f) *theta5_deg += 360.0f;

  return 1U;
}

/**
 * @brief 2连杆正向运动学：(θ₅, θ_rel) → (x, z)
 */
static void App_Arm_2Link_FK(float theta5_deg, float theta_rel_deg,
                              float *x_mm, float *z_mm) {
  /* θ₅入参为下限位参考系，trig需转回Y轴角 */
  float t5   = (theta5_deg - ARM_M5_REF_OFFSET_DEG) * DEG2RAD;
  float trel = theta_rel_deg * DEG2RAD;
  float tsum = t5 + trel;

  *x_mm = ARM_L1_MM * sinf(t5) + ARM_L2_MM * sinf(tsum);
  *z_mm = ARM_L1_MM * cosf(t5) + ARM_L2_MM * cosf(tsum);
}

/*==================== 编码器校准 ====================*/

/**
 * @brief 读取编码器并转换为物理角度（带校准）
 *
 * @param theta5_deg      输出：大臂角度θ₅（度）
 * @param theta_crank_deg 输出：曲柄角度θ_crank（度）
 *
 * @note  公式: θ = raw * DEG_PER_COUNT * dir + offset
 *        校准流程：机械臂置于已知位姿，调用 App_Arm_Calibrate()
 */
static void App_Arm_ReadEncoders(float *theta5_deg, float *theta_crank_deg) {
  int32_t raw5 = Module_Motor_GetEncoderCount(MODULE_MOTOR_5);
  int32_t raw6 = Module_Motor_GetEncoderCount(MODULE_MOTOR_6);

  arm_debug_enc_raw_m5 = raw5;
  arm_debug_enc_raw_m6 = raw6;

  /* 直读硬件TIM计数器，绕过所有中间层，用于排查 */
  arm_debug_tim2_cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  arm_debug_tim3_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

  /* 先算Y轴角，再加偏移转到下限位=0参考系 */
  *theta5_deg = (float)raw5 * ARM_DEG_PER_COUNT * arm_enc_dir_m5
              + arm_enc_offset_m5_deg + ARM_M5_REF_OFFSET_DEG;
  *theta_crank_deg = (float)raw6 * ARM_DEG_PER_COUNT * arm_enc_dir_m6 + arm_enc_offset_m6_deg;

  /* 归一化到 [-180, 180] */
  while (*theta5_deg > 180.0f)       *theta5_deg -= 360.0f;
  while (*theta5_deg < -180.0f)      *theta5_deg += 360.0f;
  while (*theta_crank_deg > 180.0f)  *theta_crank_deg -= 360.0f;
  while (*theta_crank_deg < -180.0f) *theta_crank_deg += 360.0f;

  arm_debug_enc_deg_m5 = *theta5_deg;
  arm_debug_enc_deg_m6 = *theta_crank_deg;
}

/*==================== 速度计算 ====================*/

static void App_Arm_CalcVelocity(AppArmMotorCtrl_t *ctrl, int32_t enc_now,
                                  float *vel_out, float enc_dir) {
  int32_t delta = enc_now - ctrl->enc_last;
  ctrl->enc_last = enc_now;
  *vel_out = (float)delta * ARM_DEG_PER_COUNT * enc_dir / ARM_PID_DT_S;
}

/*==================== 辅助函数 ====================*/

static float App_Arm_Clamp(float v, float lo, float hi) {
  if (v > hi) return hi;
  if (v < lo) return lo;
  return v;
}

/*==================== 遥控器目标更新 ====================*/

static void App_Arm_UpdateRemoteTarget(void) {
  AppRemoteData_t remote;
  App_Remote_GetSnapshot(&remote);

#if ARM_DEBUG_NO_REMOTE
  (void)remote.lh;
#else
  if (remote.connected == 0U) {
    arm_remote_lost_count++;
    if (arm_remote_lost_count > ARM_REMOTE_LOST_LIMIT) {
      App_Arm_EmergencyStop();
    }
    return;
  }
  arm_remote_lost_count = 0U;

  int8_t lh = remote.lh, lv = remote.lv;
  if (lh > -ARM_REMOTE_DEADBAND && lh < ARM_REMOTE_DEADBAND) lh = 0;
  if (lv > -ARM_REMOTE_DEADBAND && lv < ARM_REMOTE_DEADBAND) lv = 0;

  /* 主轴明显占优时抑制另一轴的机械串扰；斜推时仍允许二维运动。 */
  {
    int16_t abs_lh = (lh < 0) ? -(int16_t)lh : (int16_t)lh;
    int16_t abs_lv = (lv < 0) ? -(int16_t)lv : (int16_t)lv;

    if ((float)abs_lv >= (float)abs_lh * ARM_AXIS_LOCK_RATIO) {
      lh = 0;
    } else if ((float)abs_lh >= (float)abs_lv * ARM_AXIS_LOCK_RATIO) {
      lv = 0;
    }
  }

  float gain = ARM_JOYSTICK_GAIN_MM_PER_S / 100.0f;
  arm_target_x_mm += (float)lh * gain * ARM_PID_DT_S;
  arm_target_z_mm += (float)(-lv) * gain * ARM_PID_DT_S;
#endif

  arm_target_x_mm = App_Arm_Clamp(arm_target_x_mm,
                                   ARM_TARGET_X_MIN_MM, ARM_TARGET_X_MAX_MM);
  arm_target_z_mm = App_Arm_Clamp(arm_target_z_mm,
                                   ARM_TARGET_Z_MIN_MM, ARM_TARGET_Z_MAX_MM);
}

/*==================== 雅可比逆：笛卡尔速度 → 关节速度 ====================*/

/**
 * @brief 数值雅可比逆，保证末端运动方向与指令一致
 * @param theta5     大臂角度（Y轴参考系）
 * @param theta_rel  小臂相对角
 * @param vx_mm_s    期望末端X速度（mm/s）
 * @param vz_mm_s    期望末端Z速度（mm/s）
 * @param w5_dps     输出：M5角速度（deg/s）
 * @param wrel_dps   输出：小臂相对角速度（deg/s）
 * @note  FK扰动 θ₅/θ_rel 各 +0.5°，得 J₂ₓ₂，反解 J⁻¹×v_cart = ω_joint。
 *        绕过关节位置环的滞后差异，末端瞬时方向由雅可比几何保证。
 */
static void App_Arm_ResolvedRate(float theta5, float theta_rel,
                                  float vx_mm_s, float vz_mm_s,
                                  float *w5_dps, float *wrel_dps) {
  float x0, z0;
  App_Arm_2Link_FK(theta5, theta_rel, &x0, &z0);

  float eps = 0.5f; /* 0.5° 角度扰动 */

  float x5, z5;
  App_Arm_2Link_FK(theta5 + eps, theta_rel, &x5, &z5);   /* 扰动θ₅ */

  float xr, zr;
  App_Arm_2Link_FK(theta5, theta_rel + eps, &xr, &zr);   /* 扰动θ_rel */

  /* J = [∂x/∂θ₅  ∂x/∂θ_rel;  ∂z/∂θ₅  ∂z/∂θ_rel]  (mm/deg) */
  float j11 = (x5 - x0) / eps;
  float j21 = (z5 - z0) / eps;
  float j12 = (xr - x0) / eps;
  float j22 = (zr - z0) / eps;

  float det = j11 * j22 - j12 * j21;
  if (fabsf(det) < 1e-6f) {
    *w5_dps = 0.0f; *wrel_dps = 0.0f;
    return;
  }

  /* J⁻¹ = 1/det × [j22, -j12; -j21, j11],  ω = J⁻¹ × v */
  *w5_dps  = ( j22 * vx_mm_s - j12 * vz_mm_s) / det;
  *wrel_dps = (-j21 * vx_mm_s + j11 * vz_mm_s) / det;
}

/*==================== 单电机双环PID ====================*/

static void App_Arm_ControlMotor(AppArmMotorCtrl_t *ctrl,
                                  ModuleMotorId_t motor, float target_deg,
                                  float vel_ff_dps) {
  int32_t enc_now = Module_Motor_GetEncoderCount(motor);
  uint8_t is_m5 = (motor == MODULE_MOTOR_5);
  float enc_dir = is_m5 ? arm_enc_dir_m5 : arm_enc_dir_m6;
  float enc_off = is_m5 ? arm_enc_offset_m5_deg : arm_enc_offset_m6_deg;
  float angle_now = (float)enc_now * ARM_DEG_PER_COUNT * enc_dir + enc_off;

  /* M5: Y轴角 → 下限位参考系（M6不动，保持Y轴） */
  if (is_m5) {
    angle_now += ARM_M5_REF_OFFSET_DEG;
  }

  /* 角度归一化到 [-180, 180] */
  while (angle_now > 180.0f)  angle_now -= 360.0f;
  while (angle_now < -180.0f) angle_now += 360.0f;

  ctrl->angle_current = angle_now;
  App_Arm_CalcVelocity(ctrl, enc_now, &ctrl->vel_current, enc_dir);

  /* 速度EMA低通滤波 */
  ctrl->vel_filtered = 0.15f * ctrl->vel_current + 0.85f * ctrl->vel_filtered;
  ctrl->vel_current = ctrl->vel_filtered;

  /* ── 位置环：target 已由调用方比例同步，直接接收 ── */
  {
    float ref_prev = ctrl->angle_target;

    ctrl->angle_target = target_deg;

    /* 前馈：轨迹速度 × kff */
    float vel_ref = (ctrl->angle_target - ref_prev) / ARM_PID_DT_S;

    /* 反馈：位置误差 → PID */
    float pos_err = ctrl->angle_target - ctrl->angle_current;
    while (pos_err > 180.0f)  pos_err -= 360.0f;
    while (pos_err < -180.0f) pos_err += 360.0f;

    float vel_fb = Module_Pid_Update(&ctrl->pos_pid,
                                      ctrl->angle_current + pos_err,
                                      ctrl->angle_current);

    /* 合成：反馈 + 前馈 + 雅可比笛卡尔速度前馈 */
    float vel_sp = vel_fb + arm_debug_pos_kff * vel_ref + vel_ff_dps;
    vel_sp = App_Arm_Clamp(vel_sp, -ARM_POS_PID_OUTPUT_LIMIT, ARM_POS_PID_OUTPUT_LIMIT);

    /* 速度环 */
    float duty = Module_Pid_Update(&ctrl->vel_pid, vel_sp, ctrl->vel_current);
    duty = App_Arm_Clamp(duty, -ARM_VEL_PID_OUTPUT_LIMIT, ARM_VEL_PID_OUTPUT_LIMIT);

    /* M5: PWM极性翻转（dir=-1只翻转了编码器，电机物理极性需单独翻转） */
    if (is_m5) {
      duty = -duty;
    }
    Module_Motor_SetDuty(motor, (int8_t)duty);
  }
}

/*==================== Flash 校准存储 ====================*/

static void App_Arm_FlashSave(void) {
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  FLASH_Erase_Sector(ARM_FLASH_SECTOR, FLASH_VOLTAGE_RANGE_3);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);

  uint32_t *addr = (uint32_t *)ARM_FLASH_ADDR;
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&addr[0], ARM_FLASH_MAGIC);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&addr[1],
                    *(uint32_t *)&arm_enc_offset_m5_deg);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&addr[2],
                    *(uint32_t *)&arm_enc_offset_m6_deg);

  HAL_FLASH_Lock();
}

static uint8_t App_Arm_FlashLoad(void) {
  uint32_t *addr = (uint32_t *)ARM_FLASH_ADDR;
  if (addr[0] != ARM_FLASH_MAGIC) return 0U;
  arm_enc_offset_m5_deg = *(float *)&addr[1];
  arm_enc_offset_m6_deg = *(float *)&addr[2];
  return 1U;
}

/*==================== 全局API ====================*/

/**
 * @brief 初始化机械臂控制模块
 * @note  优先从Flash加载校准值，无效则需在折叠位上电自动校准并保存
 */
void App_Arm_Init(void) {
  /* ── 电磁阀初始化为放气（PC3高电平） ── */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  arm_debug_relay_state = 1U;

  /* ── 优先从Flash加载校准值 ── */
  if (App_Arm_FlashLoad() == 0U) {
    /* Flash无效 → 假设机械臂在折叠限位（θ₅≈-8°, θ_rel=117°）并保存 */
    int32_t raw5_init = Module_Motor_GetEncoderCount(MODULE_MOTOR_5);
    int32_t raw6_init = Module_Motor_GetEncoderCount(MODULE_MOTOR_6);

    /* M5: 折叠位 θ₅=-8°（后仰），offset = θ_known - raw*DEG*dir */
    float theta5_fold = -18.0f; /* 折叠位=下限位 */
    arm_enc_offset_m5_deg = theta5_fold - (float)raw5_init * ARM_DEG_PER_COUNT * arm_enc_dir_m5;

    /* M6: 实测折叠限位 FK输出 θ_rel=146°，4杆IK反算θ_crank → 计算offset */
    float theta_rel_fold = 146.0f;
    Arm4BarIKResult_t ik = App_Arm_4Bar_IK(theta5_fold, theta_rel_fold);
    float crank_ref = ik.theta_crank_deg;
    arm_enc_offset_m6_deg = crank_ref - (float)raw6_init * ARM_DEG_PER_COUNT * arm_enc_dir_m6;

    /* 首次写入Flash，之后不再需要折叠位上电 */
    App_Arm_FlashSave();
  }

  float theta5_now, theta_crank_now;
  App_Arm_ReadEncoders(&theta5_now, &theta_crank_now);

  /* M5 PID: 控制大臂角度θ₅ */
  Module_Pid_Init(&arm_m5_ctrl.pos_pid, ARM_M5_POS_KP_DEFAULT, ARM_M5_POS_KI_DEFAULT,
                  ARM_M5_POS_KD_DEFAULT, ARM_POS_PID_INTEGRAL_LIMIT,
                  ARM_POS_PID_OUTPUT_LIMIT, ARM_PID_DT_S, ARM_DERIV_FILTER_ALPHA);
  Module_Pid_Init(&arm_m5_ctrl.vel_pid, ARM_M5_VEL_KP_DEFAULT, ARM_M5_VEL_KI_DEFAULT,
                  ARM_M5_VEL_KD_DEFAULT, ARM_VEL_PID_INTEGRAL_LIMIT,
                  ARM_VEL_PID_OUTPUT_LIMIT, ARM_PID_DT_S, ARM_DERIV_FILTER_ALPHA);
  arm_m5_ctrl.angle_current = theta5_now;
  arm_m5_ctrl.angle_target  = theta5_now;
  arm_m5_ctrl.vel_current   = 0.0f;
  arm_m5_ctrl.vel_filtered  = 0.0f;
  arm_m5_ctrl.enc_last      = Module_Motor_GetEncoderCount(MODULE_MOTOR_5);

  /* M6 PID: 控制曲柄角度θ_crank */
  Module_Pid_Init(&arm_m6_ctrl.pos_pid, ARM_M6_POS_KP_DEFAULT, ARM_M6_POS_KI_DEFAULT,
                  ARM_M6_POS_KD_DEFAULT, ARM_POS_PID_INTEGRAL_LIMIT,
                  ARM_POS_PID_OUTPUT_LIMIT, ARM_PID_DT_S, ARM_DERIV_FILTER_ALPHA);
  Module_Pid_Init(&arm_m6_ctrl.vel_pid, ARM_M6_VEL_KP_DEFAULT, ARM_M6_VEL_KI_DEFAULT,
                  ARM_M6_VEL_KD_DEFAULT, ARM_VEL_PID_INTEGRAL_LIMIT,
                  ARM_VEL_PID_OUTPUT_LIMIT, ARM_PID_DT_S, ARM_DERIV_FILTER_ALPHA);
  arm_m6_ctrl.angle_current = theta_crank_now;
  arm_m6_ctrl.angle_target  = theta_crank_now;
  arm_m6_ctrl.vel_current   = 0.0f;
  arm_m6_ctrl.vel_filtered  = 0.0f;
  arm_m6_ctrl.enc_last      = Module_Motor_GetEncoderCount(MODULE_MOTOR_6);

  /* 初始目标 = 当前位置 */
  Arm4BarFKResult_t fk_res = App_Arm_4Bar_FK(theta5_now, theta_crank_now);
  float theta_rel_now = fk_res.theta_rel_deg;
  App_Arm_2Link_FK(theta5_now, theta_rel_now,
                    &arm_target_x_mm, &arm_target_z_mm);
  arm_cart_prev_x_mm = arm_target_x_mm;
  arm_cart_prev_z_mm = arm_target_z_mm;
  arm_cart_prev_valid = 1U;

  arm_ik_valid = fk_res.valid;
  arm_remote_lost_count = 0U;
  Module_Motor_Stop(MODULE_MOTOR_5);
  Module_Motor_Stop(MODULE_MOTOR_6);
}

/**
 * @brief 校准编码器零位
 *
 * @param theta5_known    当前大臂角度（度，下限位=0参考系）
 * @param theta6_rel_known 当前小臂相对角（度）
 * @note  将机械臂摆到已知位姿（如下限位处θ₅=0°），调用本函数锁定编码器偏移。
 */
void App_Arm_Calibrate(float theta5_known, float theta6_rel_known) {
  float raw5 = (float)Module_Motor_GetEncoderCount(MODULE_MOTOR_5);
  float raw6 = (float)Module_Motor_GetEncoderCount(MODULE_MOTOR_6);

  /* M5: 直驱，编码器线性对应。offset存Y轴偏置，已知角扣REF_OFFSET得Y轴角 */
  arm_enc_offset_m5_deg = theta5_known - raw5 * ARM_DEG_PER_COUNT * arm_enc_dir_m5
                          - ARM_M5_REF_OFFSET_DEG;

  /* M6: 需要先通过四杆IK算出曲柄角已知值 */
  float theta5_cur_ya = raw5 * ARM_DEG_PER_COUNT * arm_enc_dir_m5 + arm_enc_offset_m5_deg;
  float theta5_cur_ll = theta5_cur_ya + ARM_M5_REF_OFFSET_DEG;
  Arm4BarIKResult_t ik = App_Arm_4Bar_IK(theta5_cur_ll, theta6_rel_known);
  float crank_known = ik.theta_crank_deg;

  arm_enc_offset_m6_deg = crank_known - raw6 * ARM_DEG_PER_COUNT * arm_enc_dir_m6;

  /* 重置PID目标 */
  arm_m5_ctrl.angle_target = theta5_known;
  arm_m5_ctrl.angle_current = theta5_known;
  arm_m6_ctrl.angle_target = crank_known;
  arm_m6_ctrl.angle_current = crank_known;

  /* 更新目标坐标 */
  App_Arm_2Link_FK(theta5_known, theta6_rel_known,
                    &arm_target_x_mm, &arm_target_z_mm);
}

/**
 * @brief 设置编码器方向
 *
 * @param m5_dir +1.0f 或 -1.0f（默认+1：count++对应角度++）
 * @param m6_dir +1.0f 或 -1.0f
 */
void App_Arm_SetEncoderDir(float m5_dir, float m6_dir) {
  arm_enc_dir_m5 = (m5_dir >= 0.0f) ? 1.0f : -1.0f;
  arm_enc_dir_m6 = (m6_dir >= 0.0f) ? 1.0f : -1.0f;
}

/**
 * @brief 机械臂控制主任务（5ms周期）
 */
void App_Arm_Task(void) {
  arm_debug_heartbeat++;  /* 心跳：每5ms+1，Ozone看这个确认任务在跑 */

  /* 0. 镜像遥控器数据 → Ozone 调试看 */
  {
    AppRemoteData_t r;
    App_Remote_GetSnapshot(&r);
    arm_debug_rh        = r.rh;
    arm_debug_rv        = r.rv;
    arm_debug_lh        = r.lh;
    arm_debug_lv        = r.lv;
    arm_debug_remote_connected = r.connected;
    arm_debug_remote_key       = r.key;
  }

  /* 1. 读取编码器 → 物理角度 */
  float theta5, theta_crank;
  App_Arm_ReadEncoders(&theta5, &theta_crank);

  /* 2. 四杆FK：计算当前小臂相对角 */
  Arm4BarFKResult_t fk_res = App_Arm_4Bar_FK(theta5, theta_crank);
  float theta_rel = fk_res.theta_rel_deg;

  /* 3. 2连杆FK：计算当前末端位置 */
  App_Arm_2Link_FK(theta5, theta_rel,
                    &arm_state.actual_x_mm, &arm_state.actual_z_mm);

  arm_state.m5_angle_deg = theta5;
  arm_state.m6_angle_deg = theta_rel;
  arm_state.target_x_mm  = arm_target_x_mm;
  arm_state.target_z_mm  = arm_target_z_mm;
  arm_state.ik_valid = fk_res.valid;

  /* Ozone 调试镜像 */
  arm_debug_target_x_mm = arm_target_x_mm;
  arm_debug_target_z_mm = arm_target_z_mm;
  arm_debug_actual_x_mm = arm_state.actual_x_mm;
  arm_debug_actual_z_mm = arm_state.actual_z_mm;

  /* ── 在线校准：大臂+小臂掰到折叠限位 → Ozone 设 arm_debug_do_calibrate=1 ── */
  if (fabsf(arm_debug_do_calibrate) > 0.5f) {
    int32_t raw5 = Module_Motor_GetEncoderCount(MODULE_MOTOR_5);
    int32_t raw6 = Module_Motor_GetEncoderCount(MODULE_MOTOR_6);

    /* M5: 折叠位 θ₅=-8°（后仰） */
    float theta5_at_cal = -18.0f;             /* 校准位：θ₅=-18°（下限位） */
    arm_enc_offset_m5_deg = theta5_at_cal - (float)raw5 * ARM_DEG_PER_COUNT * arm_enc_dir_m5;

    /* M6: 实测折叠限位 FK输出 θ_rel=146°，通过4杆IK反算曲柄角 */
    float theta_rel_cal = 146.0f;            /* 折叠位 θ_rel=146° */
    Arm4BarIKResult_t ik = App_Arm_4Bar_IK(theta5_at_cal, theta_rel_cal);
    float crank_ref = ik.theta_crank_deg;
    arm_enc_offset_m6_deg = crank_ref - (float)raw6 * ARM_DEG_PER_COUNT * arm_enc_dir_m6;

    /* 更新目标坐标到校准位 */
    App_Arm_2Link_FK(theta5_at_cal, theta_rel_cal,
                     &arm_target_x_mm, &arm_target_z_mm);

    /* 重置PID目标和当前角度 */
    arm_m5_ctrl.angle_target = theta5_at_cal;
    arm_m5_ctrl.angle_current = theta5_at_cal;
    arm_m6_ctrl.angle_target = crank_ref;
    arm_m6_ctrl.angle_current = crank_ref;
    Module_Pid_Reset(&arm_m5_ctrl.pos_pid);
    Module_Pid_Reset(&arm_m5_ctrl.vel_pid);
    Module_Pid_Reset(&arm_m6_ctrl.pos_pid);
    Module_Pid_Reset(&arm_m6_ctrl.vel_pid);

    App_Arm_FlashSave();              /* 写入Flash，下次上电自动恢复 */
    arm_debug_do_calibrate = 0.0f;    /* 自动归零，只执行一次 */
  }

  /* ══════════════════ 控制模式选择 ══════════════════ */
  if (fabsf(arm_debug_enable_m5) > 0.5f || fabsf(arm_debug_enable_m6) > 0.5f) {

    /* ── 调试模式：M5 大臂 ── */
    if (fabsf(arm_debug_enable_m5) > 0.5f) {
      /* M5 裸duty测试 */
      if (fabsf(arm_debug_raw_duty_m5) > 0.1f) {
        /* 限位保护：读到超限立即停，自动退出 */
        if (theta5 < ARM_M5_ANGLE_MIN_DEG - 5.0f ||
            theta5 > ARM_M5_ANGLE_MAX_DEG + 5.0f) {
          Module_Motor_Stop(MODULE_MOTOR_5);
          arm_state.m5_duty = 0;
          arm_debug_enable_m5 = 0.0f;
          arm_debug_raw_duty_m5 = 0.0f;
        } else {
          float d = App_Arm_Clamp(arm_debug_raw_duty_m5,
                                   -ARM_VEL_PID_OUTPUT_LIMIT, ARM_VEL_PID_OUTPUT_LIMIT);
          Module_Motor_SetDuty(MODULE_MOTOR_5, (int8_t)d);
          arm_state.m5_duty = (int8_t)d;
        }
        arm_state.m5_target_deg = arm_debug_enc_deg_m5;
        /* 不在此 return，允许 M6 继续独立控制 */
      } else {
        float tgt_ll;

        /* M5 急停保护 */
        if (theta5 < ARM_M5_ANGLE_MIN_DEG - 5.0f ||
            theta5 > ARM_M5_ANGLE_MAX_DEG + 5.0f) {
          Module_Motor_Stop(MODULE_MOTOR_5);
          Module_Pid_Reset(&arm_m5_ctrl.pos_pid);
          Module_Pid_Reset(&arm_m5_ctrl.vel_pid);
          arm_m5_ctrl.angle_target = arm_m5_ctrl.angle_current;
          arm_state.m5_duty = 0;
          arm_debug_enable_m5 = 0.0f;
          /* 不 return，M6 可能还在跑 */
        } else {
          if (fabsf(arm_debug_m5_step) > 0.05f) {
            tgt_ll = arm_debug_m5_step;
          } else {
            static uint16_t square_cnt = 0U;
            static uint8_t  square_hi  = 0U;
            square_cnt++;
            if (square_cnt >= arm_debug_square_period) {
              square_cnt = 0U;
              square_hi  = (uint8_t)(!square_hi);
            }
            tgt_ll = square_hi ? arm_debug_square_hi : arm_debug_square_lo;
          }
          tgt_ll = App_Arm_Clamp(tgt_ll, ARM_M5_ANGLE_MIN_DEG, ARM_M5_ANGLE_MAX_DEG);
          App_Arm_ControlMotor(&arm_m5_ctrl, MODULE_MOTOR_5, tgt_ll, 0.0f);
          arm_state.m5_target_deg = tgt_ll;
          arm_state.m5_duty = (int8_t)Module_Pid_GetPrevOutput(&arm_m5_ctrl.vel_pid);
          arm_state.ik_valid = 1U;
        }
      }
    } else {
      Module_Motor_Stop(MODULE_MOTOR_5);
      arm_state.m5_duty = 0;
    }

    /* ── 调试模式：M6 小臂 ── */
    if (fabsf(arm_debug_enable_m6) > 0.5f) {
      /* M6 裸duty测试 */
      if (fabsf(arm_debug_raw_duty_m6) > 0.1f) {
        float d = App_Arm_Clamp(arm_debug_raw_duty_m6,
                                 -ARM_VEL_PID_OUTPUT_LIMIT, ARM_VEL_PID_OUTPUT_LIMIT);
        Module_Motor_SetDuty(MODULE_MOTOR_6, (int8_t)d);
        arm_state.m6_duty = (int8_t)d;
        arm_state.m6_target_deg = arm_debug_enc_deg_m6;
      } else {
        Module_Motor_Stop(MODULE_MOTOR_6);
        arm_state.m6_duty = 0;
      }
    } else {
      Module_Motor_Stop(MODULE_MOTOR_6);
      arm_state.m6_duty = 0;
    }

  } else {
    /* ── 正常模式：K9 按一下→机械臂控制，再按一下→底盘控制 ── */

    AppRemoteData_t remote;
    App_Remote_GetSnapshot(&remote);
    uint8_t cur_key = (remote.connected != 0U) ? remote.key : 0U;
    uint8_t connected = (remote.connected != 0U) ? 1U : 0U;

    /* 遥控失联检测：失联→触发归位，重连→恢复正常 */
    {
      static uint8_t was_connected = 0U;
      if (connected == 0U && was_connected != 0U) {
        /* 刚失联：停止电机 + 触发归位（完成后切回底盘） */
        Module_Motor_Stop(MODULE_MOTOR_5);
        Module_Motor_Stop(MODULE_MOTOR_6);
        Module_Pid_Reset(&arm_m5_ctrl.pos_pid);
        Module_Pid_Reset(&arm_m5_ctrl.vel_pid);
        Module_Pid_Reset(&arm_m6_ctrl.pos_pid);
        Module_Pid_Reset(&arm_m6_ctrl.vel_pid);
        arm_homing_active = 1U;
        arm_grab_active = 0U;
        arm_throw_active = 0U;
        arm_control_active = 1U;    /* 保持机械臂模式以执行归位 */
        arm_disconnect_homing = 1U; /* 标记为失联触发的归位 */
        arm_target_x_mm = arm_state.actual_x_mm;
        arm_target_z_mm = arm_state.actual_z_mm;
      }
      if (connected != 0U && was_connected == 0U) {
        /* 刚重连：恢复正常 */
        arm_homing_active = 0U;
        arm_grab_active = 0U;
        arm_throw_active = 0U;
        arm_disconnect_homing = 0U;
      }
      was_connected = connected;
    }

    /* K8 / K9 按键事件检测（基于 key_event_count，不依赖 raw key 值） */
    {
      static uint32_t last_event_count = 0U;
      uint32_t cur_events = remote.key_event_count;

      if (cur_events != last_event_count) {
        uint8_t evt_key = remote.key;

        /* K9 事件 → 切换控制模式（纯翻转，不受其他按键影响） */
        if (evt_key == 9U) {
          arm_control_active = (uint8_t)(!arm_control_active);
          arm_homing_active = 0U;
          arm_grab_active = 0U;
          arm_throw_active = 0U;
          arm_disconnect_homing = 0U;
          if (arm_control_active != 0U) {
            arm_target_x_mm = arm_state.actual_x_mm;
            arm_target_z_mm = arm_state.actual_z_mm;
          }
        }

        /* K8 事件 → 激活归位（任意模式有效，不切换控制模式，底盘保持全摇杆） */
        if (evt_key == 8U) {
          arm_homing_active = 1U;
          arm_grab_active = 0U;
          arm_throw_active = 0U;
          arm_disconnect_homing = 0U;
          arm_target_x_mm = arm_state.actual_x_mm;
          arm_target_z_mm = arm_state.actual_z_mm;
        }

        /* K5 事件 → 抓取地面目标（任意模式有效，不切换控制模式，底盘保持全摇杆） */
        if (evt_key == 5U) {
          arm_grab_active = 1U;
          arm_throw_active = 0U;
          arm_homing_active = 0U;
          arm_disconnect_homing = 0U;
          arm_target_x_mm = arm_state.actual_x_mm;
          arm_target_z_mm = arm_state.actual_z_mm;
        }

        /* K6 事件 → 投球（任意模式有效，不切换控制模式，底盘保持全摇杆） */
        if (evt_key == 6U) {
          arm_throw_active = 1U;
          arm_grab_active = 0U;
          arm_homing_active = 0U;
          arm_disconnect_homing = 0U;
          arm_target_x_mm = arm_state.actual_x_mm;
          arm_target_z_mm = arm_state.actual_z_mm;
        }

        last_event_count = cur_events;
      }

      /* K10 上升沿 → 翻转吸/放气 */
      {
        static uint8_t prev_k10 = 0U;
        if (cur_key == 10U && prev_k10 != 10U) {
          arm_debug_relay_state = (uint8_t)(!arm_debug_relay_state);
          if (arm_debug_relay_state != 0U) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
          } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
          }
        }
        prev_k10 = cur_key;
      }
    }

    if (arm_control_active != 0U || arm_grab_active != 0U
        || arm_throw_active != 0U || arm_homing_active != 0U) {
      /* ── 抓取模式：缓速插值到抓取点，摇杆一动就取消 ── */
      if (arm_grab_active != 0U) {
        float dx = ARM_GRAB_X_MM - arm_target_x_mm;
        float dz = ARM_GRAB_Z_MM - arm_target_z_mm;
        float dist = sqrtf(dx*dx + dz*dz);
        if (dist < ARM_HOME_TOLERANCE_MM) {
          arm_grab_active = 0U;
        } else {
          float step = ARM_GRAB_SPEED_MM_PER_S * ARM_PID_DT_S;
          if (step > dist) step = dist;
          arm_target_x_mm += dx / dist * step;
          arm_target_z_mm += dz / dist * step;
          if (remote.lh > ARM_REMOTE_DEADBAND || remote.lh < -ARM_REMOTE_DEADBAND ||
               remote.lv > ARM_REMOTE_DEADBAND || remote.lv < -ARM_REMOTE_DEADBAND) {
            arm_grab_active = 0U;
          }
        }
      /* ── 投球模式：缓速插值到投球点，摇杆一动就取消 ── */
      } else if (arm_throw_active != 0U) {
        float dx = ARM_THROW_X_MM - arm_target_x_mm;
        float dz = ARM_THROW_Z_MM - arm_target_z_mm;
        float dist = sqrtf(dx*dx + dz*dz);
        if (dist < ARM_HOME_TOLERANCE_MM) {
          arm_throw_active = 0U;
        } else {
          float step = ARM_THROW_SPEED_MM_PER_S * ARM_PID_DT_S;
          if (step > dist) step = dist;
          arm_target_x_mm += dx / dist * step;
          arm_target_z_mm += dz / dist * step;
          if (remote.lh > ARM_REMOTE_DEADBAND || remote.lh < -ARM_REMOTE_DEADBAND ||
               remote.lv > ARM_REMOTE_DEADBAND || remote.lv < -ARM_REMOTE_DEADBAND) {
            arm_throw_active = 0U;
          }
        }
      /* ── 归位模式：缓速插值到 home，摇杆一动就取消 ── */
      } else if (arm_homing_active != 0U) {
        float dx = ARM_HOME_X_MM - arm_target_x_mm;
        float dz = ARM_HOME_Z_MM - arm_target_z_mm;
        float dist = sqrtf(dx*dx + dz*dz);
        if (dist < ARM_HOME_TOLERANCE_MM) {
          arm_homing_active = 0U;
          if (arm_disconnect_homing != 0U) {
            arm_control_active = 0U;   /* 失联归位完成 → 切回底盘模式 */
            arm_disconnect_homing = 0U;
          }
        } else {
          float step = ARM_HOME_SPEED_MM_PER_S * ARM_PID_DT_S;
          if (step > dist) step = dist;
          arm_target_x_mm += dx / dist * step;
          arm_target_z_mm += dz / dist * step;
          if (remote.lh > ARM_REMOTE_DEADBAND || remote.lh < -ARM_REMOTE_DEADBAND ||
               remote.lv > ARM_REMOTE_DEADBAND || remote.lv < -ARM_REMOTE_DEADBAND) {
            arm_homing_active = 0U;
          }
        }
      } else {
        /* 机械臂模式 → 读摇杆更新目标 */
        App_Arm_UpdateRemoteTarget();
      }

      /* 失联时 non-homing/grab → 停住不动，等重连 */
      if (connected == 0U && arm_homing_active == 0U && arm_grab_active == 0U && arm_throw_active == 0U) {
        Module_Motor_Stop(MODULE_MOTOR_5);
        Module_Motor_Stop(MODULE_MOTOR_6);
        arm_state.m5_duty = 0;
        arm_state.m6_duty = 0;
        /* 跳过 IK/PID，直接继续（motors stopped） */
      } else {

      /* Ozone 覆盖：设 arm_debug_override_target=1 → 直接用 Ozone 写入的 X/Z */
      if (fabsf(arm_debug_override_target) > 0.5f) {
        arm_target_x_mm = arm_debug_tgt_x_mm;
        arm_target_z_mm = arm_debug_tgt_z_mm;
        arm_target_x_mm = App_Arm_Clamp(arm_target_x_mm,
                                         ARM_TARGET_X_MIN_MM, ARM_TARGET_X_MAX_MM);
        arm_target_z_mm = App_Arm_Clamp(arm_target_z_mm,
                                         ARM_TARGET_Z_MIN_MM, ARM_TARGET_Z_MAX_MM);
        arm_homing_active = 0U;
      }

      /* 任务空间闭环补偿：target + K×(target - actual) → 消稳态误差 */
      float x_err = arm_target_x_mm - arm_state.actual_x_mm;
      float z_err = arm_target_z_mm - arm_state.actual_z_mm;
      float x_ik = arm_target_x_mm + arm_debug_pos_kff * x_err;
      float z_ik = arm_target_z_mm + arm_debug_pos_kff * z_err;
      x_ik = App_Arm_Clamp(x_ik, ARM_TARGET_X_MIN_MM, ARM_TARGET_X_MAX_MM);
      z_ik = App_Arm_Clamp(z_ik, ARM_TARGET_Z_MIN_MM, ARM_TARGET_Z_MAX_MM);

      /* 2连杆IK：(x,z) → (θ₅, θ_rel) */
      float t5_tgt, trel_tgt;
      uint8_t ik2_ok = App_Arm_2Link_IK(x_ik, z_ik, &t5_tgt, &trel_tgt);
      if (ik2_ok) {
        /* 机械限位钳位 */
        t5_tgt   = App_Arm_Clamp(t5_tgt,   ARM_M5_ANGLE_MIN_DEG, ARM_M5_ANGLE_MAX_DEG);
        trel_tgt = App_Arm_Clamp(trel_tgt, ARM_M6_REL_ANGLE_MIN_DEG, ARM_M6_REL_ANGLE_MAX_DEG);

        /* 急停保护：M5大臂 + M6小臂同时检测 */
        if (theta5 < ARM_M5_ANGLE_MIN_DEG - 5.0f ||
            theta5 > ARM_M5_ANGLE_MAX_DEG + 5.0f ||
            theta_rel < ARM_M6_REL_ANGLE_MIN_DEG - 5.0f ||
            theta_rel > ARM_M6_REL_ANGLE_MAX_DEG + 5.0f) {
          Module_Motor_Stop(MODULE_MOTOR_5);
          Module_Motor_Stop(MODULE_MOTOR_6);
          arm_m5_ctrl.angle_target = arm_m5_ctrl.angle_current;
          arm_m6_ctrl.angle_target = arm_m6_ctrl.angle_current;
          arm_state.m5_duty = 0;
          arm_state.m6_duty = 0;
          arm_state.ik_valid = 0U;
        } else {
          /* 4杆IK：(θ₅, θ_rel) → θ_crank */
          Arm4BarIKResult_t ik_res = App_Arm_4Bar_IK(t5_tgt, trel_tgt);
          if (ik_res.valid) {
            /*
             * X/Z 目标已经逐周期生成直线轨迹。这里直接跟踪每个轨迹点，
             * 避免再次进行关节空间比例插值而把末端路径变成弧线。
             */

            /* 双环PID控制 M5 + M6（笛卡尔速度经雅可比转换为关节速度前馈） */

            /* ── 雅可比逆：笛卡尔误差 → 关节速度前馈 ── */
            float w5_ff = 0.0f, wc_ff = 0.0f;
            {
              float x_err = arm_target_x_mm - arm_state.actual_x_mm;
              float z_err = arm_target_z_mm - arm_state.actual_z_mm;
              float vx_ref = 0.0f;
              float vz_ref = 0.0f;

              /*
               * 轨迹点增量提供期望末端速度，位置误差只负责纠偏。
               * 这样两个关节无需等待误差产生就会同步开始运动。
               */
              if (arm_cart_prev_valid != 0U) {
                vx_ref = (arm_target_x_mm - arm_cart_prev_x_mm) / ARM_PID_DT_S;
                vz_ref = (arm_target_z_mm - arm_cart_prev_z_mm) / ARM_PID_DT_S;
              }
              arm_cart_prev_x_mm = arm_target_x_mm;
              arm_cart_prev_z_mm = arm_target_z_mm;
              arm_cart_prev_valid = 1U;

              float vx = vx_ref + ARM_CART_TRACK_KP * x_err;
              float vz = vz_ref + ARM_CART_TRACK_KP * z_err;
              vx = App_Arm_Clamp(vx, -ARM_CART_SPEED_LIMIT_MM_S,
                                  ARM_CART_SPEED_LIMIT_MM_S);
              vz = App_Arm_Clamp(vz, -ARM_CART_SPEED_LIMIT_MM_S,
                                  ARM_CART_SPEED_LIMIT_MM_S);

              /* 数值雅可比 → θ₅, θ_rel 速度 */
              float w5_dps, wr_dps;
              App_Arm_ResolvedRate(theta5, theta_rel, vx, vz, &w5_dps, &wr_dps);

              w5_dps = App_Arm_Clamp(w5_dps, -ARM_JOINT_FF_LIMIT_DPS,
                                     ARM_JOINT_FF_LIMIT_DPS);
              wr_dps = App_Arm_Clamp(wr_dps, -ARM_JOINT_FF_LIMIT_DPS,
                                     ARM_JOINT_FF_LIMIT_DPS);

              w5_ff = w5_dps;

              /* θ_rel 速度 → θ_crank 速度（4杆数值微分） */
              float eps_c = 0.1f;
              Arm4BarFKResult_t fk_a = App_Arm_4Bar_FK(theta5, theta_crank);
              Arm4BarFKResult_t fk_b = App_Arm_4Bar_FK(theta5, theta_crank + eps_c);
              float dr_dc = (fk_b.theta_rel_deg - fk_a.theta_rel_deg) / eps_c;
              if (fabsf(dr_dc) > 1e-6f) {
                wc_ff = wr_dps / dr_dc;
              }
              wc_ff = App_Arm_Clamp(wc_ff, -ARM_JOINT_FF_LIMIT_DPS,
                                    ARM_JOINT_FF_LIMIT_DPS);
            }

            App_Arm_ControlMotor(&arm_m5_ctrl, MODULE_MOTOR_5, t5_tgt, w5_ff);
            App_Arm_ControlMotor(&arm_m6_ctrl, MODULE_MOTOR_6,
                                 ik_res.theta_crank_deg, wc_ff);
            arm_state.m5_target_deg = t5_tgt;
            arm_state.m6_target_deg = trel_tgt;
            arm_state.m5_duty = (int8_t)Module_Pid_GetPrevOutput(&arm_m5_ctrl.vel_pid);
            arm_state.m6_duty = (int8_t)Module_Pid_GetPrevOutput(&arm_m6_ctrl.vel_pid);
            arm_state.ik_valid = 1U;
          } else {
            /* 4杆IK不可达：停住保持位置 + 拉回target + 复位PID */
            Module_Motor_Stop(MODULE_MOTOR_5);
            Module_Motor_Stop(MODULE_MOTOR_6);
            Module_Pid_Reset(&arm_m5_ctrl.pos_pid);
            Module_Pid_Reset(&arm_m5_ctrl.vel_pid);
            Module_Pid_Reset(&arm_m6_ctrl.pos_pid);
            Module_Pid_Reset(&arm_m6_ctrl.vel_pid);
            arm_m5_ctrl.angle_target = theta5;
            arm_m6_ctrl.angle_target = theta_crank;
            arm_target_x_mm = arm_state.actual_x_mm;
            arm_target_z_mm = arm_state.actual_z_mm;
            arm_state.m5_duty = 0;
            arm_state.m6_duty = 0;
            arm_state.ik_valid = 0U;
          }
        }
      } else {
        /* 2连杆IK不可达：停住保持位置 + 拉回target + 复位PID */
        Module_Motor_Stop(MODULE_MOTOR_5);
        Module_Motor_Stop(MODULE_MOTOR_6);
        Module_Pid_Reset(&arm_m5_ctrl.pos_pid);
        Module_Pid_Reset(&arm_m5_ctrl.vel_pid);
        Module_Pid_Reset(&arm_m6_ctrl.pos_pid);
        Module_Pid_Reset(&arm_m6_ctrl.vel_pid);
        arm_m5_ctrl.angle_target = theta5;
        arm_m6_ctrl.angle_target = theta_crank;
        arm_target_x_mm = arm_state.actual_x_mm;
        arm_target_z_mm = arm_state.actual_z_mm;
        arm_state.m5_duty = 0;
        arm_state.m6_duty = 0;
        arm_state.ik_valid = 0U;
      }
      }  /* 关闭 connected==0 pause 的 else 块 */
    } else {
      /* 底盘模式 → 停止机械臂电机，不跑PID */
      arm_cart_prev_valid = 0U;
      Module_Motor_Stop(MODULE_MOTOR_5);
      Module_Motor_Stop(MODULE_MOTOR_6);
      Module_Pid_Reset(&arm_m5_ctrl.pos_pid);
      Module_Pid_Reset(&arm_m5_ctrl.vel_pid);
      Module_Pid_Reset(&arm_m6_ctrl.pos_pid);
      Module_Pid_Reset(&arm_m6_ctrl.vel_pid);
      arm_state.m5_duty = 0;
      arm_state.m6_duty = 0;
      arm_state.ik_valid = 0U;
    }
  }
}
const AppArmState_t *App_Arm_GetState(void) { return &arm_state; }

uint8_t App_Arm_IsControlActive(void) { return arm_control_active; }

void App_Arm_EmergencyStop(void) {
  Module_Motor_Stop(MODULE_MOTOR_5);
  Module_Motor_Stop(MODULE_MOTOR_6);
  Module_Pid_Reset(&arm_m5_ctrl.pos_pid);
  Module_Pid_Reset(&arm_m5_ctrl.vel_pid);
  Module_Pid_Reset(&arm_m6_ctrl.pos_pid);
  Module_Pid_Reset(&arm_m6_ctrl.vel_pid);
  arm_m5_ctrl.angle_target = arm_m5_ctrl.angle_current;
  arm_m6_ctrl.angle_target = arm_m6_ctrl.angle_current;
  arm_ik_valid = 0U;
}

void StartarmTask(void *argument) {
  (void)argument;
  App_Arm_Init();
  uint32_t tick = osKernelGetTickCount();
  for (;;) {
    App_Arm_Task();
    tick += ARM_TASK_PERIOD_MS;
    osDelayUntil(tick);
  }
}
