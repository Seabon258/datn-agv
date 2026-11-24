#include "wheel.h"
#include "agv_config.h"
#include "motor.h"

static const int8_t motor_sign[4] = MOTOR_SIGN;
static const int8_t enc_sign[4]   = ENC_SIGN;

static uint8_t mode;
static int32_t last_cnt[4];
static float speed[4];          /* xung/s, da loc */
static float sp[4], integ[4];   /* sp = toc do dat dang tang dan toi sp_target */
static float sp_target[4];
static float pwm_cur[4], pwm_target[4];
static float kf = WHEEL_KF, kp = WHEEL_KP, ki = WHEEL_KI, ks = WHEEL_KS;   /* doi duoc qua lenh "k" */

static float approach(float x, float target, float step)
{
  if (x < target - step) return x + step;
  if (x > target + step) return x - step;
  return target;
}

void Wheel_Init(void)
{
  for (int i = 0; i < 4; i++) last_cnt[i] = Wheel_Count(i);
  Wheel_Stop();
}

void Wheel_Stop(void)
{
  mode = WHEEL_OFF;
  for (int i = 0; i < 4; i++) {
    sp[i] = sp_target[i] = 0.0f;
    pwm_cur[i] = pwm_target[i] = 0.0f;
    integ[i] = 0.0f;
    Motor_Set(i, 0);
  }
}

void Wheel_SetPwm(const int16_t duty[4])      /* PWM tang dan theo WHEEL_PWM_SLEW trong Wheel_Ctrl */
{
  if (mode != WHEEL_PWM)
    for (int i = 0; i < 4; i++) pwm_cur[i] = 0.0f;
  mode = WHEEL_PWM;
  for (int i = 0; i < 4; i++) pwm_target[i] = (float)duty[i];
}

void Wheel_SetSpeed(const int16_t cps[4])     /* toc do dat tang dan theo WHEEL_ACCEL_CPS2 */
{
  if (mode != WHEEL_SPEED)
    for (int i = 0; i < 4; i++) {
      integ[i] = 0.0f;
      sp[i] = speed[i];                         /* bat dau tu toc do hien tai, khong giat */
    }
  mode = WHEEL_SPEED;
  for (int i = 0; i < 4; i++) sp_target[i] = (float)cps[i];
}

void Wheel_Ctrl(void)
{
  const float dt = CTRL_PERIOD_MS / 1000.0f;
  for (int i = 0; i < 4; i++) {
    int32_t now = Wheel_Count(i);
    float raw = (float)(now - last_cnt[i]) / dt;
    last_cnt[i] = now;
    speed[i] += WHEEL_SPEED_LPF * (raw - speed[i]);

    if (mode == WHEEL_PWM) {
      pwm_cur[i] = approach(pwm_cur[i], pwm_target[i], WHEEL_PWM_SLEW * dt);
      Motor_Set(i, motor_sign[i] * (int32_t)pwm_cur[i]);
      continue;
    }
    if (mode != WHEEL_SPEED) continue;
    sp[i] = approach(sp[i], sp_target[i], WHEEL_ACCEL_CPS2 * dt);
    if (sp[i] == 0.0f) {                      /* dat 0 -> tha troi, khong de PID rung quanh 0 */
      integ[i] = 0.0f;
      Motor_Set(i, 0);
      continue;
    }
    float err = sp[i] - speed[i];
    float out = kf * sp[i] + (sp[i] > 0.0f ? ks : -ks) + kp * err + ki * (integ[i] + err * dt);
    if (out > MOTOR_PWM_TOP)       out = MOTOR_PWM_TOP;
    else if (out < -MOTOR_PWM_TOP) out = -MOTOR_PWM_TOP;
    else integ[i] += err * dt;                /* chong bao hoa tich phan: chi cong khi chua bao hoa */
    Motor_Set(i, motor_sign[i] * (int32_t)out);
  }
}

int16_t Wheel_Speed(int ch)
{
  float s = speed[ch];
  if (s > 32767.0f) s = 32767.0f;
  if (s < -32768.0f) s = -32768.0f;
  return (int16_t)s;
}

int32_t Wheel_Count(int ch) { return enc_sign[ch] * Enc_Get(ch); }
uint8_t Wheel_Mode(void)    { return mode; }

void Wheel_SetGains(float f, float p, float i, float s)
{
  kf = f; kp = p; ki = i; ks = s;
}

void Wheel_GetGains(float *f, float *p, float *i, float *s)
{
  *f = kf; *p = kp; *i = ki; *s = ks;
}
