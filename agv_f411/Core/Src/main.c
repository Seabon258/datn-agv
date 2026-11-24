/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "agv_config.h"
#include "batt.h"
#include "console.h"
#include "drive.h"
#include "imu.h"
#include "lift.h"
#include "link.h"
#include "motor.h"
#include "params.h"
#include "wheel.h"
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USART2 (C11), 460800 8N1:
 *  - Pi gui khung nhi phan (link.h) -> chay that, telemetry nhi phan 50 Hz
 *  - Go lenh chu (CH340) -> console de thu mach, xem print_help() */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t imu_who;
static uint8_t telem_on = 1;          /* in trang thai chu */
static uint32_t telem_ms = TELEM_PERIOD_MS;   /* doi bang lenh "r" */
static uint32_t test_t0;              /* lenh chu "m"/"v" dang chay: tu dung sau MOTOR_TEST_MS */
static uint8_t test_run;
static uint32_t test_len = MOTOR_TEST_MS;
static int32_t drive_counts;          /* lenh "d": so xung trung binh can di (>0), 0 = khong chay */
static int32_t enc_start[4];
static uint32_t link_last_ms;         /* lan cuoi nhan khung hop le */
static uint8_t link_seen, link_timeout, wdg_reset;
static uint8_t lift_was_busy;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void iwdg_start(void)
{
  IWDG->KR = 0xCCCC;                  /* bat truoc: LSI chi chay sau lenh nay, SR moi xoa duoc */
  IWDG->KR = 0x5555;                  /* mo khoa PR, RLR */
  IWDG->PR = 3;                       /* LSI 32 kHz / 32 = 1 kHz -> 1 don vi = 1 ms */
  IWDG->RLR = IWDG_TIMEOUT_MS;
  while (IWDG->SR) { }
  IWDG->KR = 0xAAAA;
}

static void print_help(void)
{
  /* in tung dong: bo dem Con_Printf chi 192 ky tu */
  Con_Printf("\r\n=== AGV F411 ===\r\n");
  Con_Printf("m <1-4> <-100..100>  PWM tho 1 dong co %lu ms (0 = ca 4)\r\n", (unsigned long)MOTOR_TEST_MS);
  Con_Printf("v <s1> <s2> <s3> <s4> PID toc do (xung/s) %lu ms\r\n", (unsigned long)MOTOR_TEST_MS);
  Con_Printf("d <so xung> <%%>       4 banh chay toi khi du so xung roi dung\r\n");
  Con_Printf("k <KF> <KP> <KI> <KS>  doi he so PID (phan nghin), r <ms> chu ky in\r\n");
  Con_Printf("b vx vy wz giu | g dau kp kd | j ki | c bu_lui he_so | p in | w luu | z xoa luu\r\n");
  Con_Printf("l <so buoc>          nang (+) / ha (-), 1600 buoc = 1 vong\r\n");
  Con_Printf("h                    ve goc (xuong toi cong tac BOT)\r\n");
  Con_Printf("s                    dung tat ca\r\n");
  Con_Printf("t / i / ?            bat-tat in trang thai / in 1 lan / tro giup\r\n");
}

static void print_status(void)
{
  uint32_t mv = Batt_ReadMv();
  Con_Printf("ENC %ld %ld %ld %ld | V %d %d %d %d | PIN %lu.%02lu V | TOP %s BOT %s | LIFT %ld%s%s",
             (long)Wheel_Count(0), (long)Wheel_Count(1), (long)Wheel_Count(2), (long)Wheel_Count(3),
             Wheel_Speed(0), Wheel_Speed(1), Wheel_Speed(2), Wheel_Speed(3),
             (unsigned long)(mv / 1000U), (unsigned long)(mv % 1000U / 10U),
             HAL_GPIO_ReadPin(LIMIT_TOP_GPIO_Port, LIMIT_TOP_Pin) == GPIO_PIN_RESET ? "CHAM" : "-",
             HAL_GPIO_ReadPin(LIMIT_BOT_GPIO_Port, LIMIT_BOT_Pin) == GPIO_PIN_RESET ? "CHAM" : "-",
             (long)Lift_Pos(), Lift_Busy() ? " (chay)" : "", Lift_Homed() ? " goc-OK" : "");
  Con_Printf(" | H %ld", (long)(Drive_Heading() * 10.0f));   /* huong, phan muoi do, + = trai */
  int16_t a[3], g[3];
  if (!imu_who) imu_who = IMU_Init();      /* MPU6050 khoi dong cham hon F411: thu lai */
  if (imu_who && IMU_Read(a, g))
    Con_Printf(" | A %d %d %d G %d %d %d\r\n", a[0], a[1], a[2], g[0], g[1], g[2]);
  else
    Con_Printf(" | IMU loi I2C 0x%02lX\r\n", (unsigned long)HAL_I2C_GetError(&hi2c1));
}

static void stop_all(void)
{
  Drive_Off();
  Wheel_Stop();
  test_run = 0;
  drive_counts = 0;
  Lift_Stop();
  Lift_Enable(0);
}

static void lift_move_up(int32_t steps_up, float sps)
{
  Lift_Enable(1);
  HAL_Delay(2);
  if (!Lift_MoveSteps(steps_up * LIFT_DIR_UP, sps, LIFT_TEST_ACCEL) && !link_seen)
    Con_Printf("khong chay: 0 buoc hoac dang cham cong tac phia do\r\n");
}

static void start_test(void)
{
  test_len = MOTOR_TEST_MS;
  Drive_Off();                                /* lenh banh rieng le */
  drive_counts = 0;                           /* "d" dat lai sau khi goi ham nay */
  for (int ch = 0; ch < 4; ch++) enc_start[ch] = Wheel_Count(ch);
  test_run = 1;
  test_t0 = HAL_GetTick();
}

/* ---------- lenh chu (CH340) ---------- */
static void handle_cmd(char *cmd)
{
  /* Lenh chat: 1 chu cai, sau do chi la so nguyen cach nhau bang dau cach.
   * Sai so luong tham so hoac co ky tu la -> khong lam gi (tranh rac thanh lenh). */
  static const struct { char c; int n; } fmt[] = {
    { 'm', 2 }, { 'v', 4 }, { 'd', 2 }, { 'k', 4 }, { 'r', 1 }, { 'b', 4 }, { 'g', 3 }, { 'c', 2 }, { 'j', 1 }, { 'w', 0 }, { 'p', 0 }, { 'z', 0 }, { 'l', 1 }, { 'h', 0 }, { 's', 0 }, { 't', 0 }, { 'i', 0 }, { '?', 0 },
  };
  char c = cmd[0];
  if (cmd[1] != '\0' && cmd[1] != ' ') c = 0;
  int need = -1;
  for (unsigned k = 0; k < sizeof fmt / sizeof fmt[0]; k++)
    if (fmt[k].c == c) need = fmt[k].n;
  long a[4] = { 0, 0, 0, 0 };
  int got = 0;
  char *p = cmd + 1;
  while (need >= 0) {
    while (*p == ' ') p++;
    if (*p == '\0') break;
    char *e;
    long v = strtol(p, &e, 10);
    if (e == p || got >= 4 || (*e != ' ' && *e != '\0')) { need = -2; break; }
    a[got++] = v;
    p = e;
  }
  if (need < 0 || got != need) {
    if (c == 's') stop_all();                 /* "s" kem rac van coi la dung, cho an toan */
    Con_Printf("bo qua lenh: \"%s\" (go ? de xem lenh)\r\n", cmd);
    return;
  }
  if (c == 'l' && (a[0] > LIFT_MAX_MOVE_STEPS || a[0] < -LIFT_MAX_MOVE_STEPS)) {
    Con_Printf("sai: l toi da +-%ld buoc moi lenh\r\n", (long)LIFT_MAX_MOVE_STEPS);
    return;
  }

  switch (c) {
  case 'm': {
    if (a[0] < 0 || a[0] > 4 || a[1] < -100 || a[1] > 100) {
      Con_Printf("sai: m <0-4> <-100..100>\r\n");
      break;
    }
    Wheel_Stop();                             /* PWM tho, bo qua MOTOR_SIGN de thu tung kenh */
    int32_t duty = (int32_t)(a[1] * MOTOR_PWM_TOP / 100);
    for (int ch = 0; ch < 4; ch++)
      if (a[0] == 0 || a[0] == ch + 1) Motor_Set(ch, duty);
    start_test();
    Con_Printf("chay M%ld %ld%% ...\r\n", a[0], a[1]);
    break;
  }
  case 'd': {                                 /* d <so xung> <%>: 4 banh cung chieu toi khi du xung */
    if (a[0] <= 0 || a[0] > 200000 || a[1] < -100 || a[1] > 100 || a[1] == 0) {
      Con_Printf("sai: d <so xung 1..200000> <-100..100 khac 0>\r\n");
      break;
    }
    int16_t duty = (int16_t)(a[1] * MOTOR_PWM_TOP / 100);
    int16_t d4[4] = { duty, duty, duty, duty };
    Wheel_Stop();
    Wheel_SetPwm(d4);                           /* dau tung banh + tang dan: trong wheel.c */
    start_test();
    drive_counts = (int32_t)a[0];
    Con_Printf("di %ld xung, %ld%% ...\r\n", a[0], a[1]);
    break;
  }
  case 'k': {                                 /* k <kf> <kp> <ki> <ks>: phan nghin, vd k 730 250 2500 600000 */
    Wheel_SetGains(a[0] / 1000.0f, a[1] / 1000.0f, a[2] / 1000.0f, a[3] / 1000.0f);
    float f, pp, ii, ss;
    Wheel_GetGains(&f, &pp, &ii, &ss);
    Con_Printf("PID: KF %ld KP %ld KI %ld KS %ld (phan nghin)\r\n",
               (long)(f * 1000), (long)(pp * 1000), (long)(ii * 1000), (long)(ss * 1000));
    break;
  }
  case 'r':                                   /* r <ms>: chu ky in trang thai 20..2000 ms */
    if (a[0] < 20 || a[0] > 2000) {
      Con_Printf("sai: r <20..2000 ms>\r\n");
      break;
    }
    telem_ms = (uint32_t)a[0];
    telem_on = 1;
    break;
  case 'b': {                                 /* b <vx> <vy> <wz> <giu huong 0/1>: ca xe, xung/s */
    start_test();
    test_len = DRIVE_TEST_MS;
    Drive_Set((int16_t)a[0], (int16_t)a[1], (int16_t)a[2], a[3] != 0);
    Con_Printf("xe vx %ld vy %ld wz %ld giu huong %ld, huong %ld.%01ld do ...\r\n", a[0], a[1], a[2], a[3],
               (long)Drive_Heading(), (long)((Drive_Heading() < 0 ? -Drive_Heading() : Drive_Heading()) * 10) % 10);
    break;
  }
  case 'g': {                                 /* g <dau gyro +-1> <kp*1000> <kd*1000> */
    Drive_SetGyro((int8_t)a[0], a[1] / 1000.0f, a[2] / 1000.0f);
    int8_t sg; float kp_, kd_, b_;
    Drive_GetGyro(&sg, &kp_, &kd_, &b_);
    Con_Printf("gyro dau %d KP %ld KD %ld (phan nghin), lech goc %ld\r\n", sg, (long)(kp_ * 1000), (long)(kd_ * 1000), (long)b_);
    break;
  }
  case 'w':                                   /* luu thong so vao flash (dung xe truoc) */
    stop_all();
    Con_Printf("dang luu ...\r\n");
    Con_Printf(Params_Save() ? "DA LUU thong so vao flash\r\n" : "LUU LOI\r\n");
    break;
  case 'p':
    Params_Print();
    break;
  case 'z':                                   /* xoa ban luu, lan khoi dong sau dung mac dinh */
    stop_all();
    Con_Printf(Params_Clear() ? "da xoa ban luu, reset de dung mac dinh\r\n" : "XOA LOI\r\n");
    break;
  case 'j':                                   /* j <ki*1000>: tich phan giu huong */
    Drive_SetHeadingKi(a[0] / 1000.0f);
    Con_Printf("giu huong KI %ld (phan nghin)\r\n", a[0]);
    break;
  case 'c':                                   /* c <bu lui PWM 0..2500> <he so*1000> */
    if (a[0] < 0 || a[0] > 2500 || a[1] < 300 || a[1] > 1500) {
      Con_Printf("sai: c <0..2500> <300..1500>\r\n");
      break;
    }
    Motor_SetRevComp((int32_t)a[0], a[1] / 1000.0f);
    Con_Printf("bu lui: %ld + %ld/1000 x lenh\r\n", a[0], a[1]);
    break;
  case 'v': {
    int16_t cps[4];
    for (int i = 0; i < 4; i++) cps[i] = (int16_t)a[i];
    Wheel_SetSpeed(cps);
    start_test();
    Con_Printf("PID %ld %ld %ld %ld xung/s ...\r\n", a[0], a[1], a[2], a[3]);
    break;
  }
  case 's':
    stop_all();
    Con_Printf("da dung\r\n");
    break;
  case 'l':
    lift_move_up((int32_t)a[0], LIFT_TEST_SPS);
    Con_Printf("nang ha %ld buoc\r\n", a[0]);
    break;
  case 'h':
    Lift_Enable(1);
    HAL_Delay(2);
    Lift_Home(LIFT_HOME_SPS, LIFT_TEST_ACCEL);
    Con_Printf("ve goc ...\r\n");
    break;
  case 't':
    telem_on = !telem_on;
    break;
  case 'i':
    print_status();
    break;
  default:
    print_help();
    break;
  }
}

/* ---------- khung nhi phan tu Pi ---------- */
static int16_t rd16(const uint8_t *b) { return (int16_t)(b[0] | (b[1] << 8)); }
static int32_t rd32(const uint8_t *b) { return (int32_t)(b[0] | (b[1] << 8) | (b[2] << 16) | ((uint32_t)b[3] << 24)); }

static void handle_frame(const link_frame_t *f)
{
  link_last_ms = HAL_GetTick();
  link_seen = 1;
  link_timeout = 0;
  test_run = 0;                               /* Pi dieu khien -> bo che do thu */
  drive_counts = 0;

  switch (f->cmd) {
  case LINK_CMD_SPEED:
  case LINK_CMD_PWM:
    Drive_Off();
    if (f->len == 8) {
      int16_t v[4];
      for (int i = 0; i < 4; i++) v[i] = rd16(&f->payload[2 * i]);
      if (f->cmd == LINK_CMD_SPEED) Wheel_SetSpeed(v);
      else Wheel_SetPwm(v);
    }
    break;
  case LINK_CMD_BODY:
    if (f->len == 7) {
      Drive_Set(rd16(f->payload), rd16(&f->payload[2]), rd16(&f->payload[4]), f->payload[6] != 0);
    }
    break;
  case LINK_CMD_STOP:
    stop_all();
    break;
  case LINK_CMD_LIFT_MOVE:
    if (f->len == 6) lift_move_up(rd32(f->payload), (float)(uint16_t)rd16(&f->payload[4]));
    break;
  case LINK_CMD_LIFT_HOME:
    if (f->len == 2) {
      Lift_Enable(1);
      HAL_Delay(2);
      Lift_Home((float)(uint16_t)rd16(f->payload), LIFT_TEST_ACCEL);
    }
    break;
  default:                                    /* PING hoac lenh la: chi lam moi lien ket */
    break;
  }
}

static void send_telem(void)
{
  static uint16_t batt_mv;
  static uint8_t batt_div;
  link_telem_t t;
  memset(&t, 0, sizeof t);
  t.ms = HAL_GetTick();
  for (int i = 0; i < 4; i++) {
    t.enc[i] = Wheel_Count(i);
    t.speed[i] = Wheel_Speed(i);
  }
  if (!imu_who) imu_who = IMU_Init();
  int16_t acc[3] = {0}, gyro[3] = {0};
  uint8_t imu_ok = imu_who && IMU_Read(acc, gyro);
  memcpy(t.acc, acc, sizeof acc);             /* struct packed: khong truyen con tro thang vao */
  memcpy(t.gyro, gyro, sizeof gyro);
  if (batt_div++ % 10U == 0U) batt_mv = (uint16_t)Batt_ReadMv();   /* pin doi cham: 5 Hz du */
  t.batt_mv = batt_mv;
  t.lift_pos = Lift_Pos();
  t.flags = (HAL_GPIO_ReadPin(LIMIT_TOP_GPIO_Port, LIMIT_TOP_Pin) == GPIO_PIN_RESET ? LINK_F_LIMIT_TOP : 0)
          | (HAL_GPIO_ReadPin(LIMIT_BOT_GPIO_Port, LIMIT_BOT_Pin) == GPIO_PIN_RESET ? LINK_F_LIMIT_BOT : 0)
          | (Lift_Busy() ? LINK_F_LIFT_BUSY : 0)
          | (Lift_Homed() ? LINK_F_LIFT_HOMED : 0)
          | (imu_ok ? LINK_F_IMU_OK : 0)
          | (link_timeout ? LINK_F_TIMEOUT : 0)
          | (wdg_reset ? LINK_F_WDG_RESET : 0);
  t.wheel_mode = Wheel_Mode();
  float h = Drive_Heading();
  while (h > 180.0f) h -= 360.0f;
  while (h < -180.0f) h += 360.0f;
  t.heading_cdeg = (int16_t)(h * 100.0f);
  Link_Send(LINK_CMD_TELEM, &t, sizeof t);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  wdg_reset = (RCC->CSR & RCC_CSR_IWDGRSTF) != 0;
  RCC->CSR |= RCC_CSR_RMVF;
  Con_Init();
  Motor_Init();
  Wheel_Init();
  Lift_Init();
  imu_who = IMU_Init();
  if (!imu_who) { HAL_Delay(150); imu_who = IMU_Init(); }   /* MPU khoi dong cham */
  Drive_Init();                               /* do lech goc gyro: xe dung yen ~0.3 s */
  Con_Printf(Params_Load() ? "thong so: ban da luu trong flash\r\n" : "thong so: mac dinh agv_config.h\r\n");
  print_help();
  if (wdg_reset) Con_Printf("!!! khoi dong lai do WATCHDOG (chip bi treo)\r\n");
  Con_Printf("MPU6050 WHO_AM_I = 0x%02X %s\r\n", imu_who,
             imu_who == 0x68 ? "(OK)" : imu_who ? "(chip nhai, van dung duoc)" : "(KHONG THAY - kiem tra day)");
  iwdg_start();

  uint32_t last_ms = HAL_GetTick();
  uint32_t t_telem = last_ms, t_led = last_ms, t_link = last_ms;
  uint8_t ctrl_div = 0;
  char cmd[64];
  link_frame_t frame;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    IWDG->KR = 0xAAAA;                  /* cho cho an */
    uint32_t now = HAL_GetTick();
    while (last_ms != now) {             /* bu lai neu vong lap bi tre do in */
      last_ms++;
      Enc_Update();
      Lift_Tick1ms();
      if (++ctrl_div >= CTRL_PERIOD_MS) {
        ctrl_div = 0;
        Drive_Ctrl();
        Wheel_Ctrl();
      }
    }

    /* nang ha dung -> tat driver cho mat (vitme tu ham) */
    if (LIFT_AUTO_DISABLE && lift_was_busy && !Lift_Busy()) Lift_Enable(0);
    lift_was_busy = Lift_Busy();

    /* an toan: Pi dang dieu khien ma mat khung qua LINK_TIMEOUT_MS -> dung */
    if (link_seen && !link_timeout && now - link_last_ms > LINK_TIMEOUT_MS) {
      stop_all();
      link_timeout = 1;
    }
    uint8_t link_active = link_seen && now - link_last_ms < LINK_ACTIVE_MS;

    if (test_run && drive_counts) {             /* lenh "d": dung khi trung binh 4 banh du xung */
      int32_t sum = 0;
      for (int ch = 0; ch < 4; ch++) {
        int32_t d = Wheel_Count(ch) - enc_start[ch];
        sum += d < 0 ? -d : d;
      }
      uint8_t timeout = now - test_t0 >= DRIVE_TIMEOUT_MS;
      if (sum / 4 >= drive_counts || timeout) {
        Wheel_Stop();
        test_run = 0;
        drive_counts = 0;
        Con_Printf("%s, xung: %ld %ld %ld %ld\r\n", timeout ? "HET GIO" : "da toi",
                   (long)(Wheel_Count(0) - enc_start[0]), (long)(Wheel_Count(1) - enc_start[1]),
                   (long)(Wheel_Count(2) - enc_start[2]), (long)(Wheel_Count(3) - enc_start[3]));
      }
    } else if (test_run && now - test_t0 >= test_len) {
      Drive_Off();
      Wheel_Stop();
      test_run = 0;
      Con_Printf("xong, xung encoder trong %lu ms: %ld %ld %ld %ld\r\n", (unsigned long)MOTOR_TEST_MS,
                 (long)(Wheel_Count(0) - enc_start[0]), (long)(Wheel_Count(1) - enc_start[1]),
                 (long)(Wheel_Count(2) - enc_start[2]), (long)(Wheel_Count(3) - enc_start[3]));
    }

    Con_Poll();
    while (Link_Get(&frame))
      handle_frame(&frame);
    if (Con_GetLine(cmd, sizeof cmd)) {
      link_seen = 0;                     /* go lenh chu -> ve che do console */
      handle_cmd(cmd);
    }

    if (link_active) {
      if (now - t_link >= LINK_TELEM_MS) {
        t_link = now;
        send_telem();
      }
    } else if (telem_on && now - t_telem >= telem_ms) {
      t_telem = now;
      print_status();
    }

    if (now - t_led >= (link_active ? 100U : 500U)) {   /* nhay nhanh = Pi dang dieu khien */
      t_led = now;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10) {
    Lift_StepISR();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  Motor_EmergencyOff();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
