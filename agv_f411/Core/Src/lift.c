#include "lift.h"
#include "agv_config.h"
#include "main.h"
#include "tim.h"
#include <math.h>

#define TICK_HZ   1000000.0f   /* TIM10: 100 MHz / (99 + 1) */
#define V_START   200.0f       /* toc do khoi dong / dung, buoc/s */

static volatile int32_t pos;
static volatile int32_t target;
static volatile int8_t  dir;
static volatile uint8_t busy;
static volatile uint8_t homing, homed;
static float v, v_max, accel;

static void set_rate(float sps)
{
  uint32_t arr = (uint32_t)(TICK_HZ / sps) - 1U;
  if (arr > 0xFFFFU) arr = 0xFFFFU;
  if (arr < 20U) arr = 20U;           /* toi da 50 kHz */
  __HAL_TIM_SET_AUTORELOAD(&htim10, arr);
}

/* cong tac dong xuong GND -> muc thap = cham */
static uint8_t limit_hit(int8_t d)
{
  if (d == LIFT_DIR_UP)
    return HAL_GPIO_ReadPin(LIMIT_TOP_GPIO_Port, LIMIT_TOP_Pin) == GPIO_PIN_RESET;
  return HAL_GPIO_ReadPin(LIMIT_BOT_GPIO_Port, LIMIT_BOT_Pin) == GPIO_PIN_RESET;
}

void Lift_Init(void)
{
  Lift_Enable(0);
  TIM10->CR1 |= TIM_CR1_ARPE;         /* doi ARR o cuoi chu ky, khong bi nhay xung */
  pos = target = 0;
  busy = 0;
}

void Lift_Enable(uint8_t on)
{
  /* EN muc thap = driver chay */
  HAL_GPIO_WritePin(STEP_EN_GPIO_Port, STEP_EN_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint8_t Lift_MoveSteps(int32_t steps, float max_sps, float accel_sps2)
{
  if (steps == 0) return 0;
  int8_t d = (steps > 0) ? 1 : -1;
  if (limit_hit(d)) return 0;          /* dang cham cong tac phia muon di */
  HAL_TIM_Base_Stop_IT(&htim10);
  dir = d;
  HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, (dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  target = pos + steps;
  v_max = max_sps;
  accel = accel_sps2;
  v = V_START;
  set_rate(v);
  __HAL_TIM_SET_COUNTER(&htim10, 0);
  TIM10->EGR = TIM_EGR_UG;            /* nap ARR ngay */
  __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE);
  busy = 1;
  HAL_TIM_Base_Start_IT(&htim10);
  return 1;
}

void Lift_Stop(void)
{
  HAL_TIM_Base_Stop_IT(&htim10);
  target = pos;
  busy = 0;
  homing = 0;
}

/* ve goc: di xuong toi cong tac BOT, dat vi tri = 0 */
void Lift_Home(float sps, float accel_sps2)
{
  int8_t down = (int8_t)-LIFT_DIR_UP;
  homed = 0;
  if (limit_hit(down)) {                      /* dang nam o goc san */
    Lift_Stop();
    pos = target = 0;
    homed = 1;
    return;
  }
  Lift_MoveSteps(down * LIFT_HOME_MAX_STEPS, sps, accel_sps2);
  homing = 1;
}

uint8_t Lift_Homed(void)  { return homed; }
uint8_t Lift_Homing(void) { return homing; }

uint8_t Lift_Busy(void) { return busy; }
int32_t Lift_Pos(void)  { return pos * LIFT_DIR_UP; }   /* + = len */

void Lift_Tick1ms(void)
{
  if (!busy) {
    homing = 0;                               /* het buoc ma chua cham BOT -> ve goc that bai */
    return;
  }
  if (limit_hit(dir)) {
    uint8_t at_home = homing && dir == -LIFT_DIR_UP;
    Lift_Stop();
    if (at_home) {
      pos = target = 0;
      homed = 1;
    }
    return;
  }
  int32_t remain = target - pos;
  if (remain < 0) remain = -remain;
  /* giam toc kip dung: v^2 = 2 a s */
  float v_stop = sqrtf(2.0f * accel * (float)remain);
  v += accel * 0.001f;
  if (v > v_max)  v = v_max;
  if (v > v_stop) v = v_stop;
  if (v < V_START) v = V_START;
  set_rate(v);
}

void Lift_StepISR(void)
{
  if (pos == target) {
    HAL_TIM_Base_Stop_IT(&htim10);
    busy = 0;
    return;
  }
  STEP_STEP_GPIO_Port->BSRR = STEP_STEP_Pin;
  for (volatile int i = 0; i < 20; i++) { }   /* xung cao ~1 us, TMC2209 can > 100 ns */
  STEP_STEP_GPIO_Port->BSRR = (uint32_t)STEP_STEP_Pin << 16U;
  pos += dir;
}
