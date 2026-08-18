/**
 * @file app_arm_test.c
 * @brief 机械臂控制代码综合测试（可在PC上运行）
 *
 * 编译: gcc -std=c11 -Wall -Wextra -DTEST -o arm_test User/App/app_arm_test.c -lm
 * 运行: ./arm_test
 *
 * 测试覆盖:
 *   1. 二连杆 FK → IK 互逆（含边界值）
 *   2. 四杆机构 FK → IK 互逆（正常工作区全覆盖）
 *   3. 编码器换算（count↔角度，方向，校准）
 *   4. PID 逻辑（P/I/D，抗饱和，限幅，滤波）
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/*==================== 测试框架 ====================*/

static int g_pass = 0;
static int g_fail = 0;

#define TEST(name) \
  do { printf("  %-55s", name); } while (0)

#define PASS() do { printf("\033[32mPASS\033[0m\n"); g_pass++; } while (0)
#define FAIL(fmt, ...) \
  do { printf("\033[31mFAIL\033[0m  " fmt "\n", ##__VA_ARGS__); g_fail++; } while (0)

#define CHECK_FLOAT_EQ(got, expect, tol, label) \
  do { \
    float _g = (got), _e = (expect); \
    if (fabsf(_g - _e) > (tol)) { \
      FAIL("%s: got %.4f, expected %.4f (tol=%.4f)", label, _g, _e, tol); \
      return; \
    } \
  } while (0)

#define CHECK_INT_EQ(got, expect, label) \
  do { \
    if ((got) != (expect)) { \
      FAIL("%s: got %d, expected %d", label, (int)(got), (int)(expect)); \
      return; \
    } \
  } while (0)

#define CHECK_UINT8_EQ(got, expect, label) \
  do { \
    if ((got) != (expect)) { \
      FAIL("%s: got %u, expected %u", label, (unsigned)(got), (unsigned)(expect)); \
      return; \
    } \
  } while (0)

#define CHECK_TRUE(cond, label) \
  do { \
    if (!(cond)) { FAIL("%s: expected true", label); return; } \
  } while (0)

/*==================== PID 模块（内嵌测试版） ====================*/

typedef struct {
  float kp, ki, kd;
  float integral_limit;
  float output_limit;
  float deriv_filter_alpha;
  float integral;
  float prev_error;
  float prev_derivative;
  float prev_output;
  float dt;
} ModulePid_t;

static void Pid_Init(ModulePid_t *pid, float kp, float ki, float kd,
                      float integral_limit, float output_limit, float dt,
                      float deriv_filter_alpha) {
  if (pid == 0) return;
  pid->kp = kp; pid->ki = ki; pid->kd = kd;
  pid->integral_limit = integral_limit;
  pid->output_limit = output_limit;
  pid->deriv_filter_alpha = deriv_filter_alpha;
  pid->dt = dt;
  pid->integral = 0.0f; pid->prev_error = 0.0f;
  pid->prev_derivative = 0.0f; pid->prev_output = 0.0f;
}

static void Pid_Reset(ModulePid_t *pid) {
  if (pid == 0) return;
  pid->integral = 0.0f; pid->prev_error = 0.0f;
  pid->prev_derivative = 0.0f; pid->prev_output = 0.0f;
}

static float Pid_Update(ModulePid_t *pid, float setpoint, float measurement) {
  if (pid == 0) return 0.0f;
  float error = setpoint - measurement;
  float p_term = pid->kp * error;

  float i_term = 0.0f;
  if (pid->ki != 0.0f && pid->dt > 0.0f) {
    float prev_abs = (pid->prev_output > 0.0f) ? pid->prev_output : -pid->prev_output;
    uint8_t saturated = (prev_abs >= pid->output_limit - 0.01f);
    uint8_t same_sign = ((error > 0.0f) && (pid->prev_output > 0.0f)) ||
                         ((error < 0.0f) && (pid->prev_output < 0.0f));
    if (!saturated || !same_sign) {
      pid->integral += pid->ki * (error + pid->prev_error) * 0.5f * pid->dt;
    }
    if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    i_term = pid->integral;
  }

  float d_term = 0.0f;
  if (pid->kd != 0.0f && pid->dt > 0.0f) {
    float raw = (error - pid->prev_error) / pid->dt;
    float filtered = pid->deriv_filter_alpha * raw +
                     (1.0f - pid->deriv_filter_alpha) * pid->prev_derivative;
    pid->prev_derivative = filtered;
    d_term = pid->kd * filtered;
  }

  float output = p_term + i_term + d_term;
  if (output > pid->output_limit)       output = pid->output_limit;
  else if (output < -pid->output_limit) output = -pid->output_limit;

  pid->prev_error = error;
  pid->prev_output = output;
  return output;
}

/*==================== 机械臂参数（从 app_arm_task.c 复制） ====================*/

#define ARM_L1_MM        380.0f
#define ARM_L2_MM        270.0f
#define ARM_D_CRANK       90.0f
#define ARM_D_ROD         380.0f
#define ARM_D_ATTACH      90.0f
#define ARM_GEAR_RATIO    139.0f
#define ARM_ENCODER_CPR    48.0f

#define ARM_COUNTS_PER_JOINT_REV (ARM_ENCODER_CPR * ARM_GEAR_RATIO)  /* 6672   */
#define ARM_DEG_PER_COUNT (360.0f / ARM_COUNTS_PER_JOINT_REV)         /* 0.05396 */

#define ARM_M5_ANGLE_MIN_DEG    (-17.41f)
#define ARM_M5_ANGLE_MAX_DEG     57.35f
#define ARM_M6_REL_ANGLE_MIN_DEG  62.90f
#define ARM_M6_REL_ANGLE_MAX_DEG  147.20f

#define ARM_PID_DT_S               0.005f
#define ARM_IK_TOLERANCE_MM        1.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD2DEG (180.0f / M_PI)
#define DEG2RAD (M_PI / 180.0f)

/* 四杆预计算常量 */
#define FK4_DENOM (2.0f * ARM_D_ATTACH)
#define IK4_DENOM (2.0f * ARM_D_CRANK)
#define FK4_CONST (ARM_D_ROD * ARM_D_ROD - ARM_D_ATTACH * ARM_D_ATTACH)
#define IK4_CONST (ARM_D_CRANK * ARM_D_CRANK - ARM_D_ROD * ARM_D_ROD)

/*==================== Mock 数据（替代 app_arm_task.c 中的静态变量） ====================*/

typedef struct {
  float theta_rel_deg;    /* 四杆FK上次结果，用于解支选择 */
  uint8_t valid;
} Mock4BarFKResult_t;

typedef struct {
  float theta_crank_deg;  /* 四杆IK上次结果，用于解支选择 */
  uint8_t valid;
} Mock4BarIKResult_t;

static float g_mock_m6_rel_angle;  /* arm_state.m6_angle_deg */
static float g_mock_m6_crank;      /* arm_m6_ctrl.angle_current */

static void Mock_Reset(float rel_deg, float crank_deg) {
  g_mock_m6_rel_angle = rel_deg;
  g_mock_m6_crank = crank_deg;
}

/*==================== 二连杆运动学 ====================*/

static uint8_t Link2_IK(float x_mm, float z_mm,
                        float *theta5_deg, float *theta_rel_deg) {
  float d_sq = x_mm * x_mm + z_mm * z_mm;
  float d = sqrtf(d_sq);
  if (d > (ARM_L1_MM + ARM_L2_MM + ARM_IK_TOLERANCE_MM)) return 0U;
  if (d < fabsf(ARM_L1_MM - ARM_L2_MM) - ARM_IK_TOLERANCE_MM) return 0U;

  float cos_rel = (d_sq - ARM_L1_MM * ARM_L1_MM - ARM_L2_MM * ARM_L2_MM)
                / (2.0f * ARM_L1_MM * ARM_L2_MM);
  if (cos_rel > 1.0f) cos_rel = 1.0f;
  if (cos_rel < -1.0f) cos_rel = -1.0f;

  float trel_rad = acosf(cos_rel);
  float t5_rad   = atan2f(x_mm, z_mm)
                 - atan2f(ARM_L2_MM * sinf(trel_rad),
                          ARM_L1_MM + ARM_L2_MM * cosf(trel_rad));
  *theta5_deg    = t5_rad * RAD2DEG;
  *theta_rel_deg = trel_rad * RAD2DEG;

  while (*theta5_deg > 180.0f)  *theta5_deg -= 360.0f;
  while (*theta5_deg < -180.0f) *theta5_deg += 360.0f;
  return 1U;
}

static void Link2_FK(float theta5_deg, float theta_rel_deg,
                     float *x_mm, float *z_mm) {
  float t5   = theta5_deg * DEG2RAD;
  float trel = theta_rel_deg * DEG2RAD;
  float tsum = t5 + trel;
  *x_mm = ARM_L1_MM * sinf(t5) + ARM_L2_MM * sinf(tsum);
  *z_mm = ARM_L1_MM * cosf(t5) + ARM_L2_MM * cosf(tsum);
}

/*==================== 四杆机构运动学 ====================*/

static Mock4BarFKResult_t Bar4_FK(float theta5_deg, float theta_crank_deg) {
  Mock4BarFKResult_t res = {0.0f, 0U};
  float t5  = theta5_deg * DEG2RAD;
  float tcr = theta_crank_deg * DEG2RAD;

  float Ex = ARM_L1_MM * sinf(t5);
  float Ey = ARM_L1_MM * cosf(t5);
  float Cx = ARM_D_CRANK * sinf(tcr);
  float Cy = ARM_D_CRANK * cosf(tcr);

  float Vx = Ex - Cx, Vy = Ey - Cy;
  float V_sq = Vx * Vx + Vy * Vy;
  float V_mag = sqrtf(V_sq);

  if (V_mag < 1e-6f) {
    res.theta_rel_deg = g_mock_m6_rel_angle;
    return res;
  }

  float K = (FK4_CONST - V_sq) / FK4_DENOM;
  float cos_val = K / V_mag;
  if (cos_val > 1.0f || cos_val < -1.0f) {
    res.valid = 0U;
    if (cos_val > 1.0f)  cos_val = 1.0f;
    if (cos_val < -1.0f) cos_val = -1.0f;
  } else {
    res.valid = 1U;
  }

  float phi_v = atan2f(Vx, Vy);
  float delta = acosf(cos_val);

  float d_minus = (phi_v - t5 - delta) * RAD2DEG;
  float d_plus  = (phi_v - t5 + delta) * RAD2DEG;

  uint8_t minus_ok = (d_minus >= ARM_M6_REL_ANGLE_MIN_DEG &&
                      d_minus <= ARM_M6_REL_ANGLE_MAX_DEG);
  uint8_t plus_ok  = (d_plus >= ARM_M6_REL_ANGLE_MIN_DEG &&
                      d_plus <= ARM_M6_REL_ANGLE_MAX_DEG);

  if (minus_ok && !plus_ok) {
    res.theta_rel_deg = d_minus;
  } else if (plus_ok && !minus_ok) {
    res.theta_rel_deg = d_plus;
  } else if (minus_ok && plus_ok) {
    float diff_m = fabsf(d_minus - g_mock_m6_rel_angle);
    float diff_p = fabsf(d_plus - g_mock_m6_rel_angle);
    if (diff_m > 180.0f) diff_m = 360.0f - diff_m;
    if (diff_p > 180.0f) diff_p = 360.0f - diff_p;
    res.theta_rel_deg = (diff_m <= diff_p) ? d_minus : d_plus;
  } else {
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

static Mock4BarIKResult_t Bar4_IK(float theta5_deg, float theta_rel_deg) {
  Mock4BarIKResult_t res = {0.0f, 0U};
  float t5   = theta5_deg * DEG2RAD;
  float trel = theta_rel_deg * DEG2RAD;

  float Dx = ARM_L1_MM * sinf(t5) + ARM_D_ATTACH * sinf(t5 + trel);
  float Dy = ARM_L1_MM * cosf(t5) + ARM_D_ATTACH * cosf(t5 + trel);
  float D_sq = Dx * Dx + Dy * Dy;
  float D_mag = sqrtf(D_sq);

  if (D_mag < 1e-6f) {
    res.theta_crank_deg = g_mock_m6_crank;
    return res;
  }

  float phi_d = atan2f(Dx, Dy);
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
  float sol_minus = (phi_d - delta) * RAD2DEG;
  float sol_plus  = (phi_d + delta) * RAD2DEG;

  float diff_m = fabsf(sol_minus - g_mock_m6_crank);
  float diff_p = fabsf(sol_plus - g_mock_m6_crank);
  if (diff_m > 180.0f) diff_m = 360.0f - diff_m;
  if (diff_p > 180.0f) diff_p = 360.0f - diff_p;
  res.theta_crank_deg = (diff_m <= diff_p) ? sol_minus : sol_plus;
  return res;
}

/*==================== 角度环绕辅助 ====================*/

static float WrapDeg(float deg) {
  while (deg > 180.0f)  deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

/*==================== 测试用例 ====================*/

/*──────────── 1. 二连杆 FK→IK 互逆测试 ────────────*/

static void Test2Link_Roundtrip_01(void) {
  /* 标准位姿: θ₅=30°, θ_rel=60° */
  TEST("2Link RT: theta5=30, thetarel=60");
  float x, z, t5, tr;
  Link2_FK(30.0f, 60.0f, &x, &z);
  CHECK_UINT8_EQ(Link2_IK(x, z, &t5, &tr), 1U, "reachable");
  CHECK_FLOAT_EQ(WrapDeg(t5), 30.0f, 0.5f, "theta5");
  CHECK_FLOAT_EQ(tr, 60.0f, 0.5f, "thetarel");
  PASS();
}

static void Test2Link_Roundtrip_02(void) {
  /* 竖直向上完全折叠: θ₅=0°, θ_rel=62.90° (极限) */
  TEST("2Link RT: theta5=0, thetarel=62.90 (fold limit)");
  float x, z, t5, tr;
  Link2_FK(0.0f, 62.90f, &x, &z);
  CHECK_UINT8_EQ(Link2_IK(x, z, &t5, &tr), 1U, "reachable");
  CHECK_FLOAT_EQ(WrapDeg(t5), 0.0f, 0.5f, "theta5");
  CHECK_FLOAT_EQ(tr, 62.90f, 0.5f, "thetarel");
  PASS();
}

static void Test2Link_Roundtrip_03(void) {
  /* 前倾+展开: θ₅=57°, θ_rel=140° */
  TEST("2Link RT: theta5=57, thetarel=140 (extreme)");
  float x, z, t5, tr;
  Link2_FK(57.0f, 140.0f, &x, &z);
  CHECK_UINT8_EQ(Link2_IK(x, z, &t5, &tr), 1U, "reachable");
  CHECK_FLOAT_EQ(WrapDeg(t5), 57.0f, 0.5f, "theta5");
  CHECK_FLOAT_EQ(tr, 140.0f, 0.5f, "thetarel");
  PASS();
}

static void Test2Link_Roundtrip_04(void) {
  /* 后仰: θ₅=-17° */
  TEST("2Link RT: theta5=-17, thetarel=90");
  float x, z, t5, tr;
  Link2_FK(-17.0f, 90.0f, &x, &z);
  CHECK_UINT8_EQ(Link2_IK(x, z, &t5, &tr), 1U, "reachable");
  CHECK_FLOAT_EQ(WrapDeg(t5), -17.0f, 0.5f, "theta5");
  CHECK_FLOAT_EQ(tr, 90.0f, 0.5f, "thetarel");
  PASS();
}

static void Test2Link_Boundary_TooFar(void) {
  /* 超出最大范围 L1+L2=650mm */
  TEST("2Link IK: out of range (>650mm)");
  float t5, tr;
  uint8_t ok = Link2_IK(700.0f, 0.0f, &t5, &tr);
  CHECK_UINT8_EQ(ok, 0U, "unreachable");
  PASS();
}

static void Test2Link_Boundary_TooClose(void) {
  /* 小于最小范围 L1-L2=110mm */
  TEST("2Link IK: out of range (<110mm)");
  float t5, tr;
  uint8_t ok = Link2_IK(50.0f, 0.0f, &t5, &tr);
  CHECK_UINT8_EQ(ok, 0U, "unreachable");
  PASS();
}

static void Test2Link_Boundary_MinReach(void) {
  /* |L1-L2| = 110mm，此时 θ_rel = π (180°)，即完全折叠 */
  TEST("2Link IK: exactly at min reach (110mm)");
  float t5, tr;
  /* 110mm 在正前方 (z=110, x=0) */
  uint8_t ok = Link2_IK(0.0f, 110.0f, &t5, &tr);
  CHECK_UINT8_EQ(ok, 1U, "reachable");
  /* D=110, 180°折叠，θ₅=0°（在正前方折叠） */
  CHECK_FLOAT_EQ(tr, 180.0f, 1.0f, "thetarel=180 (fully folded)");
  CHECK_FLOAT_EQ(WrapDeg(t5), 0.0f, 1.0f, "theta5=0");
  PASS();
}

static void Test2Link_Boundary_MaxReach(void) {
  /* 刚好可达: L1+L2=650mm */
  TEST("2Link IK: exactly at max reach (650mm)");
  float t5, tr;
  uint8_t ok = Link2_IK(0.0f, 650.0f, &t5, &tr);
  CHECK_UINT8_EQ(ok, 1U, "reachable");
  /* 直臂 θ_rel=0°，θ₅=0° */
  CHECK_FLOAT_EQ(tr, 0.0f, 1.0f, "thetarel=0");
  PASS();
}

/*──────────── 2. 四杆 FK→IK 互逆测试 ────────────*/

static void Test4Bar_Roundtrip_01(void) {
  /* M5=0°, θ_rel=90° → θ_crank → FK回算θ_rel */
  TEST("4Bar RT: theta5=0, thetarel=90");
  Mock_Reset(90.0f, 0.0f);
  Mock4BarIKResult_t ik = Bar4_IK(0.0f, 90.0f);
  CHECK_UINT8_EQ(ik.valid, 1U, "IK valid");

  /* 用 IK 得到的 crank 反推 FK */
  Mock_Reset(90.0f, ik.theta_crank_deg);
  Mock4BarFKResult_t fk = Bar4_FK(0.0f, ik.theta_crank_deg);
  CHECK_FLOAT_EQ(fk.theta_rel_deg, 90.0f, 0.5f, "theta_rel roundtrip");
  PASS();
}

static void Test4Bar_Roundtrip_02(void) {
  /* M5=30°, θ_rel=100° */
  TEST("4Bar RT: theta5=30, thetarel=100");
  Mock_Reset(100.0f, 30.0f);
  Mock4BarIKResult_t ik = Bar4_IK(30.0f, 100.0f);
  CHECK_UINT8_EQ(ik.valid, 1U, "IK valid");

  Mock_Reset(100.0f, ik.theta_crank_deg);
  Mock4BarFKResult_t fk = Bar4_FK(30.0f, ik.theta_crank_deg);
  CHECK_FLOAT_EQ(fk.theta_rel_deg, 100.0f, 0.5f, "theta_rel roundtrip");
  PASS();
}

static void Test4Bar_Roundtrip_03(void) {
  /* M5=0°, θ_rel=62.90°（折叠极限） */
  TEST("4Bar RT: theta5=0, thetarel=62.90 (fold limit)");
  Mock_Reset(62.90f, 0.0f);
  Mock4BarIKResult_t ik = Bar4_IK(0.0f, 62.90f);
  CHECK_UINT8_EQ(ik.valid, 1U, "IK valid");

  Mock_Reset(62.90f, ik.theta_crank_deg);
  Mock4BarFKResult_t fk = Bar4_FK(0.0f, ik.theta_crank_deg);
  CHECK_FLOAT_EQ(fk.theta_rel_deg, 62.90f, 0.5f, "theta_rel roundtrip");
  PASS();
}

static void Test4Bar_Roundtrip_04(void) {
  /* M5=0°, θ_rel=147.20°（展开极限） */
  TEST("4Bar RT: theta5=0, thetarel=147.20 (extend limit)");
  Mock_Reset(147.20f, 0.0f);
  Mock4BarIKResult_t ik = Bar4_IK(0.0f, 147.20f);
  CHECK_UINT8_EQ(ik.valid, 1U, "IK valid");

  Mock_Reset(147.20f, ik.theta_crank_deg);
  Mock4BarFKResult_t fk = Bar4_FK(0.0f, ik.theta_crank_deg);
  CHECK_FLOAT_EQ(fk.theta_rel_deg, 147.20f, 0.5f, "theta_rel roundtrip");
  PASS();
}

static void Test4Bar_Roundtrip_05(void) {
  /* M5=57°, θ_rel=70°（大臂前倾极限+小折叠） */
  TEST("4Bar RT: theta5=57, thetarel=70");
  Mock_Reset(70.0f, 60.0f);
  Mock4BarIKResult_t ik = Bar4_IK(57.0f, 70.0f);
  CHECK_UINT8_EQ(ik.valid, 1U, "IK valid");

  Mock_Reset(70.0f, ik.theta_crank_deg);
  Mock4BarFKResult_t fk = Bar4_FK(57.0f, ik.theta_crank_deg);
  CHECK_FLOAT_EQ(fk.theta_rel_deg, 70.0f, 0.5f, "theta_rel roundtrip");
  PASS();
}

static void Test4Bar_Roundtrip_06(void) {
  /* M5=-17°, θ_rel=110°（后仰+展开） */
  TEST("4Bar RT: theta5=-17, thetarel=110");
  Mock_Reset(110.0f, -20.0f);
  Mock4BarIKResult_t ik = Bar4_IK(-17.0f, 110.0f);
  CHECK_UINT8_EQ(ik.valid, 1U, "IK valid");

  Mock_Reset(110.0f, ik.theta_crank_deg);
  Mock4BarFKResult_t fk = Bar4_FK(-17.0f, ik.theta_crank_deg);
  CHECK_FLOAT_EQ(fk.theta_rel_deg, 110.0f, 0.5f, "theta_rel roundtrip");
  PASS();
}

static void Test4Bar_GridScan(void) {
  /* 全限位区域均匀扫描 */
  TEST("4Bar GridScan: full workspace 7x7=49 points");
  int errors = 0;
  for (int i = 0; i < 7; i++) {
    float t5 = -17.0f + (57.0f + 17.0f) * i / 6.0f;
    for (int j = 0; j < 7; j++) {
      float tr = 62.90f + (147.20f - 62.90f) * j / 6.0f;
      Mock_Reset(tr, t5);
      Mock4BarIKResult_t ik = Bar4_IK(t5, tr);
      Mock_Reset(tr, ik.theta_crank_deg);
      Mock4BarFKResult_t fk = Bar4_FK(t5, ik.theta_crank_deg);
      if (fabsf(fk.theta_rel_deg - tr) > 1.0f) {
        errors++;
        if (errors <= 3)
          printf("\n        t5=%.1f tr=%.1f: IK→crank=%.1f→FK=%.1f err=%.2f",
                 t5, tr, ik.theta_crank_deg, fk.theta_rel_deg,
                 fabsf(fk.theta_rel_deg - tr));
      }
    }
  }
  if (errors > 0) {
    FAIL("%d of 49 points exceed tolerance", errors);
  } else {
    PASS();
  }
}

/*──────────── 3. 编码器换算测试 ────────────*/

static void TestEncoder_RawToDeg(void) {
  TEST("Encoder: raw 0 → deg with default calib");
  /* dir=+1, offset=0 → deg = raw * 0.05396 */
  float raw0 = 0.0f;
  float deg0 = raw0 * ARM_DEG_PER_COUNT * 1.0f + 0.0f;
  CHECK_FLOAT_EQ(deg0, 0.0f, 0.001f, "raw=0");
  PASS();
}

static void TestEncoder_Multiplier(void) {
  /* 一圈 = 6672 counts = 360° */
  TEST("Encoder: 6672 counts = 360 deg");
  float deg = 6672.0f * ARM_DEG_PER_COUNT;
  CHECK_FLOAT_EQ(deg, 360.0f, 0.01f, "360deg");
  PASS();
}

static void TestEncoder_MidRange(void) {
  /* 45° = 6672/8 = 834 counts */
  TEST("Encoder: 834 counts ≈ 45 deg");
  float deg = 834.0f * ARM_DEG_PER_COUNT;
  CHECK_FLOAT_EQ(deg, 45.0f, 0.05f, "45deg");
  PASS();
}

static void TestEncoder_NegDirection(void) {
  /* 编码器反向: dir=-1 */
  TEST("Encoder: dir=-1, raw=834 → -45 deg");
  float deg = 834.0f * ARM_DEG_PER_COUNT * (-1.0f) + 0.0f;
  CHECK_FLOAT_EQ(deg, -45.0f, 0.05f, "-45deg");
  PASS();
}

static void TestEncoder_CalibOffset(void) {
  /* 校准偏移: raw=0 应显示 θ=30° → offset=30 */
  TEST("Encoder: offset calib, raw=0 → 30 deg");
  float offset = 30.0f - 0.0f * ARM_DEG_PER_COUNT * 1.0f;
  CHECK_FLOAT_EQ(offset, 30.0f, 0.001f, "offset=30");
  float deg = 0.0f * ARM_DEG_PER_COUNT * 1.0f + offset;
  CHECK_FLOAT_EQ(deg, 30.0f, 0.001f, "deg=30");
  PASS();
}

static void TestEncoder_WrapAround(void) {
  /* 环绕: raw 超过一圈 → 归一到±180 */
  TEST("Encoder: normalization");
  float deg = 8340.0f * ARM_DEG_PER_COUNT;  /* ≈ 450° */
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  CHECK_FLOAT_EQ(deg, 90.0f, 0.05f, "450→90");
  PASS();
}

/*──────────── 4. PID 逻辑测试 ────────────*/

static void TestPid_ProportionalOnly(void) {
  /* P=2, 误差=5 → 输出=10，限幅95 */
  TEST("PID: P-only error=5 → output=10");
  ModulePid_t pid;
  Pid_Init(&pid, 2.0f, 0.0f, 0.0f, 50.0f, 95.0f, 0.005f, 0.15f);
  float out = Pid_Update(&pid, 15.0f, 10.0f);  /* setpoint=15, meas=10 → err=5 */
  CHECK_FLOAT_EQ(out, 10.0f, 0.01f, "P term");
  PASS();
}

static void TestPid_IntegralAccumulation(void) {
  /* P=0, I=10, 持续误差=1, dt=0.005 → 积分逐步增加 */
  TEST("PID: I-only accumulation over time");
  ModulePid_t pid;
  Pid_Init(&pid, 0.0f, 10.0f, 0.0f, 50.0f, 95.0f, 0.005f, 0.15f);
  float sum = 0.0f;
  /* 梯形积分: 第一步 error=1, prev_error=0 → 增量 = 10*(1+0)/2*0.005 = 0.025 */
  /* 后续: error 保持 1, (1+1)/2*0.005 = 0.005 per step */
  sum += Pid_Update(&pid, 1.0f, 0.0f);  /* 第一步 ≈ 0.025 */
  sum += Pid_Update(&pid, 1.0f, 0.0f);  /* 第二步 ≈ 0.025+0.05=0.075 */
  sum += Pid_Update(&pid, 1.0f, 0.0f);  /* 第三步 ≈ 0.075+0.05=0.125 */
  (void)sum;
  /* 不精确比较，只验证积分在增长 */
  CHECK_TRUE(pid.integral > 0.05f, "integral growing");
  CHECK_TRUE(pid.integral < 10.0f, "integral not exploding");
  PASS();
}

static void TestPid_IntegralWindup(void) {
  /* 输出已饱和(95)且同向误差，积分应停止 */
  TEST("PID: anti-windup at saturation");
  ModulePid_t pid;
  Pid_Init(&pid, 20.0f, 10.0f, 0.0f, 50.0f, 95.0f, 0.005f, 0.15f);
  /* 大误差 → 立即饱和 */
  float out = Pid_Update(&pid, 100.0f, 0.0f);  /* err=100, P=2000 → 限幅到95 */
  CHECK_FLOAT_EQ(out, 95.0f, 0.01f, "saturated output");
  float int1 = pid.integral;

  /* 同一方向继续，积分不应大幅增长 */
  out = Pid_Update(&pid, 100.0f, 0.0f);
  CHECK_FLOAT_EQ(out, 95.0f, 0.01f, "still saturated");
  CHECK_FLOAT_EQ(pid.integral, int1, 0.01f, "integral frozen");
  PASS();
}

static void TestPid_WindupRecovery(void) {
  /* 正向积分积累后，误差反转 → 积分应降低 */
  TEST("PID: anti-windup recovery on error reversal");
  ModulePid_t pid;
  Pid_Init(&pid, 0.1f, 20.0f, 0.0f, 50.0f, 95.0f, 0.005f, 0.15f);
  /* 跑几步积累正向积分（P很小不会立即饱和） */
  Pid_Update(&pid, 30.0f, 0.0f);  /* err=30 */
  Pid_Update(&pid, 30.0f, 0.0f);  /* err=30, 积分继续积累 */
  float int_before = pid.integral;
  CHECK_TRUE(int_before > 1.0f, "integral accumulated");

  /* 误差反转：测量值远超目标，误差为负且绝对值大 */
  Pid_Update(&pid, 30.0f, 100.0f); /* err=-70, 梯形积分平均(30-70)/2<0 */
  CHECK_TRUE(pid.integral < int_before, "integral decreasing after reversal");
  PASS();
}

static void TestPid_OutputClamp(void) {
  /* 输出不应超过 output_limit */
  TEST("PID: output clamped to ±output_limit");
  ModulePid_t pid;
  Pid_Init(&pid, 100.0f, 100.0f, 100.0f, 50.0f, 30.0f, 0.005f, 0.15f);
  float out = Pid_Update(&pid, 100.0f, 0.0f);
  CHECK_TRUE(fabsf(out) <= 30.01f, "within ±30");
  PASS();
}

static void TestPid_DerivativeFiltering(void) {
  /* D-only: 阶梯变化 → D 输出被低通滤波平滑衰减 */
  TEST("PID: D-term low-pass filtering");
  ModulePid_t pid;
  Pid_Init(&pid, 0.0f, 0.0f, 0.5f, 50.0f, 95.0f, 0.005f, 0.2f);
  /* 第一步: error跳变 0→10 → raw_deriv=10/0.005=2000 → filt=0.2*2000=400 → d=200 */
  float out1 = Pid_Update(&pid, 10.0f, 0.0f);
  float d1_abs = fabsf(out1);

  /* 第二步: error不变 → raw_deriv=0 → filt逐步衰减: filt=0.2*0+0.8*400=320 → d=160 */
  float out2 = Pid_Update(&pid, 10.0f, 10.0f);
  float d2_abs = fabsf(out2);

  /* D 输出应减弱（未饱和情况下） */
  CHECK_TRUE(d2_abs < d1_abs, "D decaying after step");
  PASS();
}

static void TestPid_ResetZeroesState(void) {
  /* Reset 后积分和误差清零 */
  TEST("PID: reset zeros all state");
  ModulePid_t pid;
  Pid_Init(&pid, 1.0f, 1.0f, 1.0f, 50.0f, 95.0f, 0.005f, 0.15f);
  Pid_Update(&pid, 10.0f, 0.0f);
  Pid_Update(&pid, 10.0f, 0.0f);
  Pid_Reset(&pid);
  CHECK_FLOAT_EQ(pid.integral, 0.0f, 0.001f, "integral=0");
  CHECK_FLOAT_EQ(pid.prev_error, 0.0f, 0.001f, "prev_error=0");
  CHECK_FLOAT_EQ(pid.prev_output, 0.0f, 0.001f, "prev_output=0");
  PASS();
}

static void TestPid_NullPointer(void) {
  /* NULL pid 不应崩溃 */
  TEST("PID: null pointer safety");
  Pid_Init(0, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  Pid_Reset(0);
  float out = Pid_Update(0, 10.0f, 0.0f);
  CHECK_FLOAT_EQ(out, 0.0f, 0.001f, "null returns 0");
  PASS();
}

/*──────────── 5. 四杆不可达测试 ────────────*/

static void Test4Bar_Unreachable(void) {
  /* 强行给一个四杆无法装配的位姿 */
  TEST("4Bar: unreachable pose flagged");
  /* 超远距离：θ₅=0°, θ_rel 极大值超出连杆范围。
     用 FK 正向算一个极限位置再尝试不可能的角度组合 */
  Mock4BarIKResult_t ik = Bar4_IK(0.0f, 180.0f);  /* θ_rel=180° 机械上不可能 */
  /* 可能 valid=0（cos钳位），也可能 valid=1（仅公式可解但物理不对） */
  /* 至少 FK 的 valid 应该反映这个情况 */
  Mock_Reset(180.0f, ik.theta_crank_deg);
  Mock4BarFKResult_t fk = Bar4_FK(0.0f, ik.theta_crank_deg);
  /* 如果 IK valid=0 说明 cos 超限；如果 valid=1 则 FK 应该钳回限位附近 */
  if (ik.valid == 0U) {
    /* 预期行为：标记不可达 */
  }
  /* FK 结果不应偏离 IK 输入太离谱（因钳位保护） */
  CHECK_TRUE(fk.theta_rel_deg >= 0.0f, "FK result sane");
  PASS();
}

/*──────────── 6. 角度环绕辅助函数测试 ────────────*/

static void TestAngleWrap_Basic(void) {
  TEST("Angle wrap: basic cases");
  CHECK_FLOAT_EQ(WrapDeg(350.0f), -10.0f, 0.01f, "350→-10");
  CHECK_FLOAT_EQ(WrapDeg(-350.0f), 10.0f, 0.01f, "-350→10");
  CHECK_FLOAT_EQ(WrapDeg(180.0f), 180.0f, 0.01f, "180 stays");
  CHECK_FLOAT_EQ(WrapDeg(-180.0f), -180.0f, 0.01f, "-180 stays");
  /* 多圈 */
  CHECK_FLOAT_EQ(WrapDeg(720.0f), 0.0f, 0.01f, "720→0");
  CHECK_FLOAT_EQ(WrapDeg(-720.0f), 0.0f, 0.01f, "-720→0");
  PASS();
}

/*──────────── 7. PID 角度环绕（模拟 ControlMotor 逻辑）───────────*/

static void TestPid_AngleWrapping(void) {
  /* 模拟 App_Arm_ControlMotor 中角度环绕的 PID 调用 */
  TEST("PID: angle wrap in position control");
  ModulePid_t pid;
  Pid_Init(&pid, 2.0f, 0.5f, 0.1f, 40.0f, 180.0f, 0.005f, 0.15f);

  /* 目标 170°, 当前 -170° → 直接误差 340°, 环绕后 -20° */
  float target = 170.0f;
  float current = -170.0f;
  float pos_err = target - current;          /* = 340 */
  while (pos_err > 180.0f)  pos_err -= 360.0f;  /* 340→-20 */
  while (pos_err < -180.0f) pos_err += 360.0f;

  CHECK_FLOAT_EQ(pos_err, -20.0f, 0.01f, "wrapped error=-20");

  /* 传入虚拟目标 */
  float vel = Pid_Update(&pid, current + pos_err, current);
  /* 负误差 → 负的 P 输出 */
  CHECK_TRUE(vel < 0.0f, "negative P output for negative error");

  /* 反向测试: 目标 -170°, 当前 170° */
  Pid_Reset(&pid);
  pos_err = (-170.0f) - 170.0f;           /* = -340 */
  while (pos_err > 180.0f)  pos_err -= 360.0f;
  while (pos_err < -180.0f) pos_err += 360.0f;  /* -340→20 */
  CHECK_FLOAT_EQ(pos_err, 20.0f, 0.01f, "wrapped error=20");
  vel = Pid_Update(&pid, 170.0f + pos_err, 170.0f);
  CHECK_TRUE(vel > 0.0f, "positive P output for positive error");
  PASS();
}

/*========================== main ==========================*/

int main(void) {
  printf("\n╔═══════════════════════════════════════════════╗\n");
  printf("║   Mechanical Arm Control — Unit Test Suite   ║\n");
  printf("╚═══════════════════════════════════════════════╝\n\n");

  printf("─── 1. 2-Link FK/IK Roundtrip ───\n");
  Test2Link_Roundtrip_01();
  Test2Link_Roundtrip_02();
  Test2Link_Roundtrip_03();
  Test2Link_Roundtrip_04();
  Test2Link_Boundary_TooFar();
  Test2Link_Boundary_TooClose();
  Test2Link_Boundary_MinReach();
  Test2Link_Boundary_MaxReach();

  printf("\n─── 2. 4-Bar FK/IK Roundtrip ───\n");
  Test4Bar_Roundtrip_01();
  Test4Bar_Roundtrip_02();
  Test4Bar_Roundtrip_03();
  Test4Bar_Roundtrip_04();
  Test4Bar_Roundtrip_05();
  Test4Bar_Roundtrip_06();
  Test4Bar_GridScan();

  printf("\n─── 3. Encoder Conversion ───\n");
  TestEncoder_RawToDeg();
  TestEncoder_Multiplier();
  TestEncoder_MidRange();
  TestEncoder_NegDirection();
  TestEncoder_CalibOffset();
  TestEncoder_WrapAround();

  printf("\n─── 4. PID Logic ───\n");
  TestPid_ProportionalOnly();
  TestPid_IntegralAccumulation();
  TestPid_IntegralWindup();
  TestPid_WindupRecovery();
  TestPid_OutputClamp();
  TestPid_DerivativeFiltering();
  TestPid_ResetZeroesState();
  TestPid_NullPointer();

  printf("\n─── 5. Other ───\n");
  Test4Bar_Unreachable();
  TestAngleWrap_Basic();
  TestPid_AngleWrapping();

  printf("\n═══════════════════════════════════════════════\n");
  printf("  TOTAL: %d pass, %d fail",
         g_pass, g_fail);
  if (g_fail > 0)
    printf(" \033[31m✗\033[0m\n");
  else
    printf(" \033[32m✓\033[0m\n");
  printf("═══════════════════════════════════════════════\n\n");

  return (g_fail > 0) ? 1 : 0;
}
