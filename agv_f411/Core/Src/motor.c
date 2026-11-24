#include "motor.h"
#include "agv_config.h"
#include "main.h"
#include "tim.h"

static const uint32_t pwm_ch[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
static GPIO_TypeDef *const dir_port[4] = { M1_DIR_IN1_GPIO_Port, M2_DIR_IN1_GPIO_Port,
                                           M3_DIR_IN1_GPIO_Port, M4_DIR_IN1_GPIO_Port };
static const uint16_t dir_pin[4] = { M1_DIR_IN1_Pin, M2_DIR_IN1_Pin, M3_DIR_IN1_Pin, M4_DIR_IN1_Pin };
static TIM_HandleTypeDef *const enc_tim[4] = { &htim2, &htim3, &htim4, &htim5 };

static int32_t rev_off = MOTOR_REV_OFFSET;   /* bu vung chet lui, doi duoc qua lenh "c" */
static float rev_gain = MOTOR_REV_GAIN;
static uint32_t enc_last[4];
static volatile int32_t enc_total[4];

void Motor_Init(void)
{
  for (int ch = 0; ch < 4; ch++) {
    Motor_Set(ch, 0);
    HAL_TIM_PWM_Start(&htim1, pwm_ch[ch]);
    HAL_TIM_Encoder_Start(enc_tim[ch], TIM_CHANNEL_ALL);
    enc_last[ch] = __HAL_TIM_GET_COUNTER(enc_tim[ch]);
    enc_total[ch] = 0;
  }
}

/* DRV8871: chi IN2 co PWM nen khi IN1 = 1 phai dao duty
 *   duty > 0: IN1 = 1, IN2 muc thap = chay  -> OUT1 H, OUT2 L
 *   duty < 0: IN1 = 0, IN2 muc cao = chay   -> OUT1 L, OUT2 H
 *   duty = 0: IN1 = 0, IN2 = 0              -> tha troi
 * Chieu lui duoc bu vung chet (MOTOR_REV_OFFSET, MOTOR_REV_GAIN) de 2 chieu ra toc do gan bang nhau. */
void Motor_Set(int ch, int32_t duty)
{
  if (ch < 0 || ch > 3) return;
  if (duty > MOTOR_PWM_TOP)  duty = MOTOR_PWM_TOP;
  if (duty < -MOTOR_PWM_TOP) duty = -MOTOR_PWM_TOP;

  if (duty > 0) {
    HAL_GPIO_WritePin(dir_port[ch], dir_pin[ch], GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, pwm_ch[ch], MOTOR_PWM_TOP - duty);
  } else if (duty < 0) {
    int32_t d = rev_off + (int32_t)(rev_gain * (float)(-duty));   /* bu vung chet lui */
    if (d > MOTOR_PWM_TOP) d = MOTOR_PWM_TOP;
    HAL_GPIO_WritePin(dir_port[ch], dir_pin[ch], GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, pwm_ch[ch], d);
  } else {
    HAL_GPIO_WritePin(dir_port[ch], dir_pin[ch], GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, pwm_ch[ch], 0);
  }
}

void Motor_SetRevComp(int32_t offset, float gain)
{
  rev_off = offset;
  rev_gain = gain;
}

void Motor_GetRevComp(int32_t *offset, float *gain)
{
  *offset = rev_off;
  *gain = rev_gain;
}

void Motor_StopAll(void)
{
  for (int ch = 0; ch < 4; ch++) Motor_Set(ch, 0);
}

void Motor_EmergencyOff(void)
{
  TIM1->BDTR &= ~TIM_BDTR_MOE;               /* tat ngo ra PWM, dien tro 10k keo IN2 xuong */
  GPIOB->BSRR = (uint32_t)(M1_DIR_IN1_Pin | M2_DIR_IN1_Pin | M3_DIR_IN1_Pin | M4_DIR_IN1_Pin) << 16U;
  STEP_EN_GPIO_Port->BSRR = STEP_EN_Pin;     /* EN = 1: tat TMC2209 */
}

void Enc_Update(void)
{
  for (int ch = 0; ch < 4; ch++) {
    uint32_t now = __HAL_TIM_GET_COUNTER(enc_tim[ch]);
    int32_t d = (int32_t)(now - enc_last[ch]);
    if (enc_tim[ch]->Instance == TIM3 || enc_tim[ch]->Instance == TIM4)
      d = (int16_t)d;                        /* TIM3, TIM4 la 16 bit */
    enc_last[ch] = now;
    enc_total[ch] += d;
  }
}

int32_t Enc_Get(int ch) { return enc_total[ch]; }
