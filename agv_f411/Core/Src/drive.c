#include "drive.h"
#include "agv_config.h"
#include "imu.h"
#include "main.h"
#include "wheel.h"

static uint8_t active, hold, holding;
static int16_t cmd_vx, cmd_vy, cmd_wz;
static float heading, target;           /* do */
static float bias;                      /* lech goc gyro Z, don vi tho */
static int8_t gsign = GYRO_Z_SIGN;
static float hkp = HEADING_KP, hkd = HEADING_KD, hki = HEADING_KI;
static float hint;                      /* tich phan loi huong, do x giay */

void Drive_Init(void)
{
  int16_t a[3], g[3];
  int32_t sum = 0, n = 0;
  for (int i = 0; i < 100; i++) {               /* ~0.3 s, xe phai dung yen */
    if (IMU_Read(a, g)) { sum += g[2]; n++; }
    HAL_Delay(3);
  }
  bias = n ? (float)sum / (float)n : 0.0f;
  heading = target = 0.0f;
}

void Drive_Set(int16_t vx, int16_t vy, int16_t wz, uint8_t h)
{
  if (!active) holding = 0;
  active = 1;
  cmd_vx = vx; cmd_vy = vy; cmd_wz = wz;
  hold = h;
}

void Drive_Off(void) { active = 0; holding = 0; }

void Drive_Ctrl(void)
{
  const float dt = CTRL_PERIOD_MS / 1000.0f;
  static uint8_t imu_bad;                       /* MPU loi: thu lai moi 1 s de khong chan vong lap */
  static uint16_t retry;
  int16_t a[3], g[3];
  float rate = 0.0f;                            /* do/s, + = xoay trai */
  uint8_t ok = 0;
  if (!imu_bad || ++retry >= 1000U / CTRL_PERIOD_MS) {
    retry = 0;
    ok = IMU_Read(a, g);
    imu_bad = !ok;
  }
  if (ok) {
    float raw = (float)g[2];
    uint8_t still = !active || (cmd_vx == 0 && cmd_vy == 0 && cmd_wz == 0);
    if (still && Wheel_Speed(0) == 0 && Wheel_Speed(1) == 0 && Wheel_Speed(2) == 0 && Wheel_Speed(3) == 0)
      bias += 0.002f * (raw - bias);            /* dung yen: cap nhat lech goc tu tu */
    rate = gsign * (raw - bias) / GYRO_LSB_PER_DPS;
    heading += rate * dt;
  }
  if (!active) return;

  float w = (float)cmd_wz;
  if (hold && cmd_wz == 0 && (cmd_vx != 0 || cmd_vy != 0)) {
    if (!holding) { target = heading; holding = 1; hint = 0.0f; }
    float err = heading - target;               /* lech trai = + */
    hint += err * dt;
    float lim = HEADING_MAX_CPS / (hki > 0.0f ? hki : 1.0f);   /* chong bao hoa tich phan */
    if (hint > lim) hint = lim;
    if (hint < -lim) hint = -lim;
    w -= hkp * err + hki * hint + hkd * rate;
    if (w > HEADING_MAX_CPS) w = HEADING_MAX_CPS;
    if (w < -HEADING_MAX_CPS) w = -HEADING_MAX_CPS;
  } else {
    holding = 0;
  }
  /* mecanum, con lan chu X nhin tu tren. Dau xe = phia M3, M4.
   * Cach cam tu 2026-11-20: M4 truoc-trai, M3 truoc-phai, M2 sau-trai, M1 sau-phai */
  float vx = cmd_vx, vy = cmd_vy;
  float fl = vx - vy - w, rl = vx + vy - w, rr = vx - vy + w, fr = vx + vy + w;
  int16_t cps[4] = { (int16_t)rr, (int16_t)rl, (int16_t)fr, (int16_t)fl };
  Wheel_SetSpeed(cps);
}

float   Drive_Heading(void) { return heading; }
uint8_t Drive_Active(void)  { return active; }

void Drive_SetGyro(int8_t sign, float kp, float kd)
{
  gsign = sign < 0 ? -1 : 1;
  hkp = kp; hkd = kd;
}

void  Drive_SetHeadingKi(float ki) { hki = ki; }
float Drive_GetHeadingKi(void)     { return hki; }

void Drive_GetGyro(int8_t *sign, float *kp, float *kd, float *b)
{
  *sign = gsign; *kp = hkp; *kd = hkd; *b = bias;
}
