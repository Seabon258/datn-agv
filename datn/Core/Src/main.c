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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Mot motor = 1 timer PWM + 2 kenh (IN1, IN2 cua DRV8871) */
typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t ch_in1;
  uint32_t ch_in2;
} Motor_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_COUNT        4U
#define TEST_DUTY_PERCENT  100U    /* duty khi test */
#define TEST_DRIVE         1       /* 1: lap tren xe, ca xe chay toi / lui (uu tien hon TEST_STREAM) */
#define WHEEL_DIAMETER_MM  80.0f   /* duong kinh banh xe (mm) - DO LAI cho dung banh that */
#define DRIVE_SPEED_MPM    3.0f    /* toc do toi da khi chay (m/phut); 1 m/phut qua cham, banh giat */
#define DRIVE_MOVE_MM      200.0f  /* chay toi quang nay, nghi, roi lui lai dung quang nay (mm) */
#define DRIVE_ACCEL_S      1.0f    /* thoi gian tang toc 0 -> DRIVE_SPEED_MPM (s), giam toc cung vay */
#define DRIVE_PAUSE_MS     1500U   /* nghi giua luot toi va luot lui */
#define SPEED_DT_MS        10U     /* chu ky vong PI toc do */
#define SPEED_KP           1.0f    /* duty / (count/s) */
#define SPEED_KI           10.0f   /* duty / count */
#define POS_KP             4.0f    /* bam vi tri: (count/s) bu them / count sai lech */
#define POS_CORR_MAX       0.5f    /* toc do bu toi da = POS_CORR_MAX * toc do toi da */
#define POS_TOL_MM         2.0f    /* sai so vi tri chap nhan khi dung (mm) */
#define DRIVE_START_MS     3000U   /* cho sau khi reset de dat xe xuong */
#define FRICTION_TEST      0       /* 1: KE XE LEN, do duty toi thieu de moi banh bat dau quay / con quay */
#define HEADING_HOLD       0       /* 1: dung gyro Z cua MPU6050 giu xe di thang khi chay DRIVE_MOVE_MM */
#define IMU_ADDR           (0x68U << 1) /* AD0 = GND; AD0 = 3V3 thi 0x69 */
#define IMU_YAW_SIGN       1       /* +1: module nam ngua (chip quay len) -> xe quay trai (CCW) yaw tang; up nguoc thi -1 */
#define IMU_PERIOD_MS      5U      /* doc gyro 200 Hz */
#define IMU_CALIB_MS       1000U   /* thoi gian xe dung yen de do bias gyro */
#define TRACK_MM           200.0f  /* khoang cach tam banh trai - phai (mm) - DO LAI tren xe */
#define WHEELBASE_MM       200.0f  /* khoang cach tam banh truoc - sau (mm) - DO LAI tren xe */
#define HEAD_GAIN          1.5f    /* 1.0 = bu dung bang hinh hoc, tang neu san tron xe van lech */
#define HEAD_OFF_MAX_MM    20.0f   /* do lech quang duong trai/phai toi da khi bu huong */
#define WHEEL_CHECK        0       /* 1: KE XE LEN, quay tung banh M1..M4 theo chieu "tien" de kiem tra wheel_dir */
#define TEST_STREAM        1       /* 1: chay cac motor trong STREAM_MASK, gui CSV de ve do thi tren PC (tools/enc_plot.py) */
#define STREAM_MASK        0xFU    /* motor chay cung luc o che do stream: bit0=M1, bit1=M2, bit2=M3, bit3=M4 */
#define STREAM_RUN_MS      5000U   /* thoi gian chay moi chieu o che do stream */
#define TEST_CONTINUOUS    1       /* 1: ca 4 motor chay toi lien tuc de do dien ap, 0: test tung motor */
#define TEST_RUN_MS        2000U   /* thoi gian chay moi chieu */
#define TEST_PAUSE_MS      500U    /* nghi giua 2 chieu de motor dung han */
#if TEST_STREAM
#define PRINT_PERIOD_MS    20U     /* stream CSV 50 Hz */
#else
#define PRINT_PERIOD_MS    200U    /* chu ky in ra USART3 */
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Bang anh xa motor -> timer/kenh, dung thu tu trong bang di day */
static const Motor_t motors[MOTOR_COUNT] =
{
  { &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2 },  /* M1: PE9 / PE11  */
  { &htim1, TIM_CHANNEL_3, TIM_CHANNEL_4 },  /* M2: PE13 / PE14 */
  { &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2 },  /* M3: PC6 / PB14 (CH2N, van dung thanh ghi CCR2) */
  { &htim8, TIM_CHANNEL_3, TIM_CHANNEL_4 },  /* M4: PC8 / PC9   */
};

/* Encoder i dung timer enc_tim[i]: ENC1=TIM2, ENC2=TIM3, ENC3=TIM4, ENC4=TIM5 */
static TIM_HandleTypeDef *const enc_tim[MOTOR_COUNT] = { &htim2, &htim3, &htim4, &htim5 };
static uint32_t enc_last[MOTOR_COUNT];   /* gia tri CNT lan doc truoc */
/* Dau dem cua tung encoder: -1 neu chay toi ma count giam (ENC2 dau day A/B nguoc) */
static const int8_t enc_sign[MOTOR_COUNT] = { 1, -1, 1, 1 };
/* Chieu lap banh tren xe (M1 truoc-trai, M2 sau-trai, M3 sau-phai, M4 truoc-phai):
 * +1 neu motor chay toi lam xe tien, -1 neu nguoc. Kiem tra bang WHEEL_CHECK (day motor moi banh noi khac nhau) */
static const int8_t wheel_dir[MOTOR_COUNT] = { 1, -1, 1, -1 };

/* Bien toan cuc de xem bang Live Expressions */
volatile int32_t enc_count[MOTOR_COUNT]; /* tong count tich luy (co dau) */
volatile int32_t enc_delta[MOTOR_COUNT]; /* so count trong 1 ms gan nhat */
volatile uint8_t test_motor;             /* motor dang test: 1..4, 0 = khong (che do stream: STREAM_MASK) */
volatile int8_t  test_dir;               /* +1 toi, -1 lui, 0 dung */
static uint32_t next_print;              /* tick lan in tiep theo */
#if TEST_DRIVE
volatile float drive_target_cps;         /* toc do dat cho ca xe (count/s), duong = tien */
volatile float wheel_cps[MOTOR_COUNT];   /* toc do do duoc tung banh (count/s) */
volatile uint8_t drive_brake;            /* 1: phanh ca 4 banh (IN1 = IN2 = 1) */
volatile uint8_t drive_mask = 0xFU;      /* banh nao duoc dieu khien: bit0=M1 .. bit3=M4, banh ngoai mask tha troi */
/* Bam vi tri: ca 4 banh bam cung mot quang duong dat drive_ref_pos (count, tinh tu
 * drive_pos0). Banh nao bi hut (ket, chiu tai nang) se duoc day nhanh de duoi kip. */
volatile uint8_t drive_pos_mode;         /* 1: bat vong vi tri */
volatile uint8_t drive_manual;           /* 1: tat vong PI, main tu dat duty (do ma sat) */
volatile int16_t fric_result[MOTOR_COUNT][2][2]; /* [banh][0 tien,1 lui][0 khoi dong,1 duy tri], 0.1 %; -1 = khong quay */
volatile uint8_t fric_done;              /* 1: do xong */
volatile float drive_ref_pos;            /* quang duong dat (count) */
volatile float drive_corr_max;           /* toc do bu toi da (count/s) */
static int32_t drive_pos0[MOTOR_COUNT];  /* enc_count luc bat dau chay */
/* Bu huong: banh trai (M1, M2) bam ref + drive_head_off, banh phai (M3, M4) bam ref - drive_head_off */
volatile float drive_head_off;           /* count */
#endif
/* MPU6050 */
volatile uint8_t imu_ok;                 /* 1: doc duoc MPU6050 */
volatile uint8_t imu_whoami;             /* thanh ghi WHO_AM_I (MPU6050 that = 0x68) */
volatile uint32_t imu_err;               /* so lan doc I2C loi */
volatile float imu_bias;                 /* bias gyro Z (do/s, chua nhan IMU_YAW_SIGN) */
volatile float imu_rate;                 /* toc do quay quanh Z (do/s), duong = quay trai */
volatile float imu_yaw;                  /* goc yaw tich phan (do), 0 = huong luc xuat phat */
static uint32_t imu_last;                /* tick lan doc truoc */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Motor_Set(uint8_t m, int32_t duty);
static void Encoder_Update(void);
static void Print_Status(void);
static void Wait_And_Print(uint32_t ms);
#if TEST_DRIVE
static void Drive_Move(float move_mm);
#endif
static uint8_t IMU_Init(void);
static void IMU_Calibrate(uint32_t ms);
static void IMU_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * Dat toc do motor m (0..3).
  * duty > 0: chay toi  -> PWM tren IN1, IN2 = 0
  * duty < 0: chay lui  -> IN1 = 0, PWM tren IN2
  * duty = 0: ca hai = 0 -> DRV8871 o che do tha troi (coast)
  * |duty| toi da = ARR (8399) tuong ung 100 %.
  */
static void Motor_Set(uint8_t m, int32_t duty)
{
  const Motor_t *mt = &motors[m];
  int32_t arr = (int32_t)__HAL_TIM_GET_AUTORELOAD(mt->htim);

  if (duty > arr)  duty = arr;
  if (duty < -arr) duty = -arr;

  if (duty >= 0)
  {
    __HAL_TIM_SET_COMPARE(mt->htim, mt->ch_in1, (uint32_t)duty);
    __HAL_TIM_SET_COMPARE(mt->htim, mt->ch_in2, 0U);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(mt->htim, mt->ch_in1, 0U);
    __HAL_TIM_SET_COMPARE(mt->htim, mt->ch_in2, (uint32_t)(-duty));
  }
}

/**
  * Doc 4 bo dem encoder, tinh so count thay doi tu lan truoc va cong don.
  * Goi trong ngat TIM6 (1 kHz). Phep tru so khong dau roi ep ve so co dau
  * cho ket qua dung ca khi bo dem tran (65535 -> 0 hoac 0 -> 65535).
  */
static void Encoder_Update(void)
{
  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
  {
    uint32_t now = __HAL_TIM_GET_COUNTER(enc_tim[i]);
    int32_t d;

    if (IS_TIM_32B_COUNTER_INSTANCE(enc_tim[i]->Instance))
      d = (int32_t)(now - enc_last[i]);              /* TIM2, TIM5: 32-bit */
    else
      d = (int16_t)(uint16_t)(now - enc_last[i]);    /* TIM3, TIM4: 16-bit */

    d *= enc_sign[i];
#if TEST_DRIVE
    d *= wheel_dir[i];                 /* count duong = xe tien */
#endif

    enc_last[i]  = now;
    enc_delta[i] = d;
    enc_count[i] += d;
  }
}

#if TEST_DRIVE
/**
  * Vong PI toc do cho 4 banh, goi trong ngat TIM6 moi SPEED_DT_MS.
  * Vong ho khong chay cham duoc vi motor can ~10-15 % duty moi thang ma sat,
  * nen dung PI theo encoder. Muc tieu = 0 -> tha troi va xoa tich phan.
  */
static void Speed_Control(void)
{
  static int32_t last[MOTOR_COUNT];
  static float integ[MOTOR_COUNT];
  const float dt = SPEED_DT_MS / 1000.0f;
  const float arr = (float)__HAL_TIM_GET_AUTORELOAD(&htim1);
  const float target = drive_target_cps;
  const uint8_t pos_mode = drive_pos_mode;
  const float ref = drive_ref_pos;

  for (uint8_t m = 0; m < MOTOR_COUNT; m++)
  {
    int32_t now = enc_count[m];
    float meas = (float)(now - last[m]) / dt;
    float tgt = target;
    float err;
    float u;

    last[m] = now;
    wheel_cps[m] = meas;

    if (drive_brake)
    {
      /* DRV8871: IN1 = IN2 = 1 -> ngan mach 2 dau motor, dung nhanh hon tha troi */
      integ[m] = 0.0f;
      __HAL_TIM_SET_COMPARE(motors[m].htim, motors[m].ch_in1, (uint32_t)arr + 1U);
      __HAL_TIM_SET_COMPARE(motors[m].htim, motors[m].ch_in2, (uint32_t)arr + 1U);
      continue;
    }
    if ((!pos_mode && target == 0.0f) || !(drive_mask & (1U << m)))
    {
      integ[m] = 0.0f;
      Motor_Set(m, 0);
      continue;
    }

    if (pos_mode)
    {
      float off = (m < 2U) ? drive_head_off : -drive_head_off;   /* M1, M2 ben trai */
      float c = POS_KP * (ref + off - (float)(now - drive_pos0[m]));
      if (c >  drive_corr_max) c =  drive_corr_max;
      if (c < -drive_corr_max) c = -drive_corr_max;
      tgt += c;
    }
    err = tgt - meas;

    integ[m] += SPEED_KI * err * dt;
    if (integ[m] >  arr) integ[m] =  arr;
    if (integ[m] < -arr) integ[m] = -arr;

    u = SPEED_KP * err + integ[m];
    Motor_Set(m, wheel_dir[m] * (int32_t)u);
  }
}
#endif

/* ---------------- MPU6050 (I2C1: PB8 = SCL, PB9 = SDA) ---------------- */
#define MPU_SMPLRT_DIV   0x19U
#define MPU_CONFIG       0x1AU
#define MPU_GYRO_CONFIG  0x1BU
#define MPU_GYRO_ZOUT_H  0x47U
#define MPU_PWR_MGMT_1   0x6BU
#define MPU_WHO_AM_I     0x75U
#define MPU_GYRO_LSB     65.5f     /* LSB / (do/s) o thang +-500 do/s */

static HAL_StatusTypeDef IMU_Write(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1U, 5U);
}

/* Doc gyro Z tho (do/s, chua tru bias). Tra ve 0 neu loi I2C. */
static uint8_t IMU_ReadGz(float *dps)
{
  uint8_t b[2];

  if (HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, MPU_GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT, b, 2U, 2U) != HAL_OK)
  {
    imu_err++;
    return 0U;
  }
  *dps = (float)(int16_t)((b[0] << 8) | b[1]) / MPU_GYRO_LSB;
  return 1U;
}

/* Kiem tra WHO_AM_I roi cau hinh: clock PLL gyro, loc 44 Hz, 200 Hz, +-500 do/s */
static uint8_t IMU_Init(void)
{
  uint8_t id = 0;

  if (HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, MPU_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &id, 1U, 10U) != HAL_OK)
    return 0U;
  imu_whoami = id;
  if (IMU_Write(MPU_PWR_MGMT_1, 0x80U) != HAL_OK) return 0U;   /* reset chip */
  HAL_Delay(100);
  if (IMU_Write(MPU_PWR_MGMT_1, 0x01U) != HAL_OK) return 0U;   /* thoat sleep, clock = PLL gyro X */
  if (IMU_Write(MPU_CONFIG, 0x03U) != HAL_OK) return 0U;       /* DLPF 44 Hz */
  if (IMU_Write(MPU_SMPLRT_DIV, 0x04U) != HAL_OK) return 0U;   /* 1 kHz / (1 + 4) = 200 Hz */
  if (IMU_Write(MPU_GYRO_CONFIG, 0x08U) != HAL_OK) return 0U;  /* +-500 do/s */
  HAL_Delay(50);
  return 1U;
}

/* Xe PHAI DUNG YEN: lay trung binh gyro Z lam bias, dat yaw = 0 */
static void IMU_Calibrate(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  float sum = 0.0f, g;
  uint32_t n = 0;

  if (!imu_ok) return;
  while ((HAL_GetTick() - t0) < ms)
  {
    if (IMU_ReadGz(&g)) { sum += g; n++; }
    HAL_Delay(IMU_PERIOD_MS);
  }
  if (n > 0U) imu_bias = sum / (float)n;
  imu_yaw = 0.0f;
  imu_last = HAL_GetTick();
}

/* Goi lien tuc trong vong cho: moi IMU_PERIOD_MS doc gyro, tich phan yaw va
 * tinh do lech quang duong trai/phai de keo xe ve huong ban dau. */
static void IMU_Update(void)
{
  uint32_t now = HAL_GetTick();
  float g;

  if (!imu_ok || (now - imu_last) < IMU_PERIOD_MS) return;
  if (!IMU_ReadGz(&g))
  {
    /* Bus I2C treo (nhieu tu motor): khoi dong lai I2C1 */
    if ((imu_err & 0x0FU) == 0U)
    {
      HAL_I2C_DeInit(&hi2c1);
      HAL_I2C_Init(&hi2c1);
    }
    return;
  }
  imu_rate = IMU_YAW_SIGN * (g - imu_bias);
  imu_yaw += imu_rate * (float)(now - imu_last) / 1000.0f;
  imu_last = now;

#if TEST_DRIVE && HEADING_HOLD
  {
    /* Xe lech trai (yaw > 0) -> banh trai di truoc banh phai mot doan
     * yaw * (TRACK + WHEELBASE) / 2 (dong hoc mecanum) de quay ve */
    const float cnt_per_mm = ENC_CPR / (3.14159f * WHEEL_DIAMETER_MM);
    float off = HEAD_GAIN * imu_yaw * (3.14159f / 180.0f) * (TRACK_MM + WHEELBASE_MM) * 0.5f;

    if (off >  HEAD_OFF_MAX_MM) off =  HEAD_OFF_MAX_MM;
    if (off < -HEAD_OFF_MAX_MM) off = -HEAD_OFF_MAX_MM;
    drive_head_off = drive_pos_mode ? off * cnt_per_mm : 0.0f;
  }
#endif
}

/* In mot dong trang thai ra USART3 (PD8) */
static void Print_Status(void)
{
#if TEST_STREAM
  /* Chup 4 count cung 1 thoi diem (khong bi ngat TIM6 chen giua) */
  int32_t c[MOTOR_COUNT];
  uint32_t t;

  __disable_irq();
  t = HAL_GetTick();
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) c[i] = enc_count[i];
  __enable_irq();

  /* Dong du lieu: D,t_ms,motor,dir,E1,E2,E3,E4 */
  printf("D,%lu,%u,%d,%ld,%ld,%ld,%ld\r\n",
         (unsigned long)t, test_motor, test_dir,
         (long)c[0], (long)c[1], (long)c[2], (long)c[3]);
  {
    /* Dong goc: Y,t_ms,yaw(0.1 do),toc_do_quay(0.1 do/s), 10 Hz */
    static uint8_t k;
    if (imu_ok && ++k >= 5U)
    {
      k = 0;
      printf("Y,%lu,%ld,%ld\r\n", (unsigned long)t,
             (long)(imu_yaw * 10.0f), (long)(imu_rate * 10.0f));
    }
  }
#else
  printf("M%u dir=%+d | E1=%ld E2=%ld E3=%ld E4=%ld\r\n",
         test_motor, test_dir,
         (long)enc_count[0], (long)enc_count[1],
         (long)enc_count[2], (long)enc_count[3]);
#endif
}

/* Cho ms mili-giay, trong luc cho thi in trang thai moi PRINT_PERIOD_MS.
 * next_print giu nguyen giua cac lan goi nen nhip in deu ca khi goi Wait ngan. */
static void Wait_And_Print(uint32_t ms)
{
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < ms)
  {
    IMU_Update();
    if ((int32_t)(HAL_GetTick() - next_print) >= 0)
    {
      next_print += PRINT_PERIOD_MS;
      Print_Status();
    }
  }
}
#if TEST_DRIVE
/**
  * Chay move_mm (am = lui) roi phanh va dung han.
  * Tao quang duong dat drive_ref_pos theo bien dang hinh thang (tang toc
  * DRIVE_ACCEL_S, giam toc v = sqrt(2 a quang_con_lai)); ca 4 banh bam theo
  * no trong Speed_Control nen di cung mot quang duong.
  */
static void Drive_Move(float move_mm)
{
  /* mm -> count, m/phut -> count/s */
  const float cnt_per_mm = ENC_CPR / (3.14159f * WHEEL_DIAMETER_MM);
  const float vmax = DRIVE_SPEED_MPM * 1000.0f / 60.0f * cnt_per_mm;
  const float acc  = vmax / DRIVE_ACCEL_S;
  const float vmin = 0.2f * vmax;          /* duoi muc nay banh de ket */
  const float goal = fabsf(move_mm) * cnt_per_mm;
  const float sgn  = (move_mm >= 0.0f) ? 1.0f : -1.0f;
  const float tol  = POS_TOL_MM * cnt_per_mm;
  float pos = 0.0f;
  float v = 0.0f;
  uint32_t t0;

  __disable_irq();
  for (uint8_t m = 0; m < MOTOR_COUNT; m++) drive_pos0[m] = enc_count[m];
  drive_ref_pos = 0.0f;
  drive_corr_max = POS_CORR_MAX * vmax;
  drive_pos_mode = 1U;
  __enable_irq();

  test_dir = (int8_t)sgn;
  while (pos < goal)
  {
    float vstop = sqrtf(2.0f * acc * (goal - pos));

    v += acc * 0.01f;
    if (v > vmax)  v = vmax;
    if (v > vstop) v = vstop;
    if (v < vmin)  v = vmin;

    pos += v * 0.01f;
    if (pos > goal) pos = goal;
    drive_ref_pos = sgn * pos;
    drive_target_cps = sgn * v;
    Wait_And_Print(10U);
  }
  drive_target_cps = 0.0f;

  /* Cho cac banh tre ve dung vi tri (toi da 1 s) roi phanh */
  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 1000U)
  {
    uint8_t ok = 1U;
    for (uint8_t m = 0; m < MOTOR_COUNT; m++)
    {
      float off = (m < 2U) ? drive_head_off : -drive_head_off;
      if (fabsf(sgn * goal + off - (float)(enc_count[m] - drive_pos0[m])) > tol) ok = 0U;
    }
    if (ok) break;
    Wait_And_Print(10U);
  }
  drive_pos_mode = 0U;
  drive_brake = 1U;
  Wait_And_Print(500U);
  drive_brake = 0U;                        /* het phanh -> tha troi, khong ton dien */
  test_dir = 0;
}
#endif
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
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Tat bo dem cua printf: moi lan printf gui ngay ra UART */
  setvbuf(stdout, NULL, _IONBF, 0);

  /* --- PWM: 8 kenh, duty ban dau = 0 (Pulse = 0 trong MX_TIMx_Init) --- */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);  /* M3_IN2 o PB14 la kenh bu CH2N */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  /* --- Encoder: dem ca 2 kenh A/B, x4 --- */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  /* LUU Y TIM5 / ENC4_A = PA0: chan nay noi voi nut USER B1 tren board.
   * Kiem tra schematic MB997: neu PA0 co tu chong doi thi phai thao tu,
   * neu khong encoder 4 se dem sai. Khong nhan B1 khi dang chay. */
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
  {
    enc_last[i] = __HAL_TIM_GET_COUNTER(enc_tim[i]);
  }

  /* --- TIM6: ngat 1 kHz, goi Encoder_Update() --- */
  HAL_TIM_Base_Start_IT(&htim6);

  printf("\r\n=== DATN test buoc 1: PWM + encoder, ENC_CPR=%d ===\r\n", ENC_CPR);
#if HEADING_HOLD
  imu_ok = IMU_Init();
  printf("MPU6050: %s, WHO_AM_I=0x%02X\r\n", imu_ok ? "OK" : "KHONG THAY (kiem tra day PB8/PB9)", imu_whoami);
  IMU_Calibrate(IMU_CALIB_MS);
#else
  (void)IMU_Init;                          /* giu code MPU6050 cho luc bat lai */
  printf("MPU6050: tat (HEADING_HOLD = 0)\r\n");
#endif
  next_print = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const int32_t duty = (int32_t)(__HAL_TIM_GET_AUTORELOAD(&htim1) * TEST_DUTY_PERCENT / 100U);

#if TEST_DRIVE && FRICTION_TEST
    /* Voi tung banh, moi chieu: tang duty 0.5 %/50 ms (vong ho) toi khi banh
     * quay -> duty khoi dong; roi giam dan toi khi banh dung -> duty duy tri.
     * In ket qua dang: F,banh,chieu,khoi_dong_x10,duy_tri_x10 (don vi 0.1 %). */
    {
      static uint8_t done;
      const int32_t arr = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim1);

      if (done)
      {
        HAL_Delay(1000);
        continue;
      }
      drive_manual = 1U;
      HAL_Delay(DRIVE_START_MS);
      printf("=== Do ma sat: F,banh,chieu,khoi_dong(0.1%%),duy_tri(0.1%%) ===\r\n");

      for (uint8_t m = 0; m < MOTOR_COUNT; m++)
      {
        for (int8_t dir = 1; dir >= -1; dir -= 2)
        {
          int32_t pm = 5;              /* duty theo 0.1 % */
          int32_t brk = -1, keep = -1;
          uint8_t moving = 0;
          int32_t prev = enc_count[m];

          for (; pm <= 600 && moving < 2U; pm += 5)
          {
            int32_t now;
            Motor_Set(m, wheel_dir[m] * dir * arr * pm / 1000);
            HAL_Delay(50);
            now = enc_count[m];
            moving = (dir * (now - prev) > 10) ? (uint8_t)(moving + 1U) : 0U;
            prev = now;
          }
          if (moving >= 2U)
          {
            brk = pm - 5;
            for (pm = brk; pm > 0; pm -= 5)
            {
              int32_t now;
              Motor_Set(m, wheel_dir[m] * dir * arr * pm / 1000);
              HAL_Delay(50);
              now = enc_count[m];
              if (dir * (now - prev) < 3) break;
              prev = now;
            }
            keep = pm;
          }
          Motor_Set(m, 0);
          fric_result[m][dir > 0 ? 0 : 1][0] = (int16_t)brk;
          fric_result[m][dir > 0 ? 0 : 1][1] = (int16_t)keep;
          printf("F,M%u,%+d,%ld,%ld\r\n", m + 1U, dir, (long)brk, (long)keep);
          HAL_Delay(800);
        }
      }
      printf("=== Xong ===\r\n");
      fric_done = 1U;
      drive_manual = 0U;
      done = 1U;
    }
    continue;
#endif

#if TEST_DRIVE && WHEEL_CHECK
    /* Quay lan luot M1, M2, M3, M4 theo chieu "xe tien" 2 s, nghi 2 s.
     * Banh nao lan nguoc ve phia duoi xe -> doi dau banh do trong wheel_dir. */
    {
      const float cps = DRIVE_SPEED_MPM * 1000.0f / 60.0f * ENC_CPR / (3.14159f * WHEEL_DIAMETER_MM);

      for (uint8_t m = 0; m < MOTOR_COUNT; m++)
      {
        test_motor = m + 1U;
        test_dir = 1;
        drive_mask = (uint8_t)(1U << m);
        drive_target_cps = cps;
        Wait_And_Print(2000U);
        drive_target_cps = 0.0f;
        test_dir = 0;
        Wait_And_Print(2000U);
      }
      drive_mask = 0xFU;
    }
    continue;
#endif

#if TEST_DRIVE
    /* Chay toi DRIVE_MOVE_MM, nghi, lui ve cho cu, nghi, lap lai mai */
    {
      static uint8_t started;

      test_motor = 0xFU;
      if (!started)
      {
        Wait_And_Print(DRIVE_START_MS);
        IMU_Calibrate(IMU_CALIB_MS);           /* xe da dat xuong, dung yen -> do bias, yaw = 0 */
        started = 1U;
      }
      Drive_Move(DRIVE_MOVE_MM);
      Wait_And_Print(DRIVE_PAUSE_MS);
      Drive_Move(-DRIVE_MOVE_MM);
      Wait_And_Print(DRIVE_PAUSE_MS);
    }
    continue;
#endif

#if TEST_STREAM
    /* Chay cung luc cac motor trong STREAM_MASK (cung duty -> de so sanh):
     * tang toc 1 s, toi STREAM_RUN_MS, dung 1 s, lui STREAM_RUN_MS, dung 1 s.
     * In du lieu ca luc tang toc. */
    test_motor = STREAM_MASK;
    for (int8_t dir = 1; dir >= -1; dir -= 2)
    {
      test_dir = dir;
      for (int32_t step = 1; step <= 100; step++)
      {
        for (uint8_t m = 0; m < MOTOR_COUNT; m++)
          if (STREAM_MASK & (1U << m)) Motor_Set(m, dir * duty * step / 100);
        Wait_And_Print(10U);
      }
      Wait_And_Print(STREAM_RUN_MS);
      for (uint8_t m = 0; m < MOTOR_COUNT; m++)
        if (STREAM_MASK & (1U << m)) Motor_Set(m, 0);
      test_dir = 0;
      Wait_And_Print(1000U);
    }
    continue;
#endif

#if TEST_CONTINUOUS
    /* Chay tung motor mot: tang duty tu tu trong 1 s (tranh dong khoi dong lon
     * lam sut nguon), giu 3 s, roi dung 1 s truoc khi sang motor ke tiep */
    for (uint8_t m = 0; m < MOTOR_COUNT; m++)
    {
      test_motor = m + 1U;
      test_dir = 1;
      for (int32_t step = 1; step <= 100; step++)
      {
        Motor_Set(m, duty * step / 100);
        HAL_Delay(10);
      }
      Wait_And_Print(3000U);
      Motor_Set(m, 0);
      test_dir = 0;
      Wait_And_Print(1000U);
    }
    continue;
#endif

    for (uint8_t m = 0; m < MOTOR_COUNT; m++)
    {
      int32_t c0, c_fwd, c_rev;

      test_motor = m + 1U;

      /* Chay toi 2 s */
      c0 = enc_count[m];
      test_dir = 1;
      Motor_Set(m, duty);
      Wait_And_Print(TEST_RUN_MS);
      Motor_Set(m, 0);
      test_dir = 0;
      Wait_And_Print(TEST_PAUSE_MS);
      c_fwd = enc_count[m] - c0;

      /* Chay lui 2 s */
      c0 = enc_count[m];
      test_dir = -1;
      Motor_Set(m, -duty);
      Wait_And_Print(TEST_RUN_MS);
      Motor_Set(m, 0);
      test_dir = 0;
      Wait_And_Print(TEST_PAUSE_MS);
      c_rev = enc_count[m] - c0;

      /* Tong ket: so vong banh = count / ENC_CPR (in dang phan nghin de khong can printf float) */
      printf(">> M%u: toi %ld count (%ld mvong), lui %ld count (%ld mvong)\r\n",
             m + 1U,
             (long)c_fwd, (long)(c_fwd * 1000 / ENC_CPR),
             (long)c_rev, (long)(c_rev * 1000 / ENC_CPR));
    }

    test_motor = 0;
    printf("=== Xong 1 chu ky, lap lai sau 3 s ===\r\n");
    Wait_And_Print(3000U);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* HAL goi ham nay khi bat ky timer nao tran (update event). Chi xu ly TIM6. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    Encoder_Update();
#if TEST_DRIVE
    static uint8_t div;
    if (++div >= SPEED_DT_MS)
    {
      div = 0;
      if (!drive_manual) Speed_Control();
    }
#endif
  }
}

/* Chuyen printf sang USART3 (PD8 -> RX cua mach USB-UART).
 * Ghi de ham _write weak trong syscalls.c. */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
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
