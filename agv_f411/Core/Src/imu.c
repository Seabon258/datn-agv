#include "imu.h"
#include "i2c.h"
#include "main.h"

#define MPU_ADDR      (0x68U << 1)
#define REG_CONFIG    0x1AU
#define REG_GYRO_CFG  0x1BU
#define REG_ACC_CFG   0x1CU
#define REG_ACC_X     0x3BU
#define REG_PWR1      0x6BU
#define REG_WHOAMI    0x75U

static HAL_StatusTypeDef wr(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 10);
}

/* MCU reset giua luc doc I2C -> MPU giu SDA thap mai (loi 0x20). Phat 9 xung SCL + STOP de MPU nha bus,
 * reset khoi I2C (loi BUSY cua F4) roi khoi tao lai. */
static void bus_recover(void)
{
  GPIO_InitTypeDef g = {0};
  HAL_I2C_DeInit(&hi2c1);
  g.Pin = IMU_SCL_Pin | IMU_SDA_Pin;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);
  HAL_GPIO_WritePin(GPIOB, IMU_SDA_Pin, GPIO_PIN_SET);
  for (int i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(GPIOB, IMU_SCL_Pin, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 300; d++) { }
    HAL_GPIO_WritePin(GPIOB, IMU_SCL_Pin, GPIO_PIN_SET);
    for (volatile int d = 0; d < 300; d++) { }
  }
  HAL_GPIO_WritePin(GPIOB, IMU_SDA_Pin, GPIO_PIN_RESET);   /* STOP: SDA len khi SCL cao */
  for (volatile int d = 0; d < 300; d++) { }
  HAL_GPIO_WritePin(GPIOB, IMU_SDA_Pin, GPIO_PIN_SET);
  I2C1->CR1 |= I2C_CR1_SWRST;
  I2C1->CR1 &= ~I2C_CR1_SWRST;
  MX_I2C1_Init();
}

uint8_t IMU_Init(void)
{
  uint8_t who = 0;
  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, REG_WHOAMI, I2C_MEMADD_SIZE_8BIT, &who, 1, 10) != HAL_OK) {
    bus_recover();
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, REG_WHOAMI, I2C_MEMADD_SIZE_8BIT, &who, 1, 10) != HAL_OK)
      return 0;
  }
  wr(REG_PWR1, 0x01);       /* thuc day, clock theo gyro X */
  HAL_Delay(10);
  wr(REG_CONFIG, 0x03);     /* loc DLPF ~44 Hz */
  wr(REG_GYRO_CFG, 0x08);   /* +-500 do/s -> 65.5 LSB/(do/s) */
  wr(REG_ACC_CFG, 0x00);    /* +-2 g -> 16384 LSB/g */
  return who;
}

uint8_t IMU_Read(int16_t acc[3], int16_t gyro[3])
{
  uint8_t b[14];
  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, REG_ACC_X, I2C_MEMADD_SIZE_8BIT, b, 14, 10) != HAL_OK)
    return 0;
  for (int i = 0; i < 3; i++) {
    acc[i]  = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    gyro[i] = (int16_t)((b[8 + 2 * i] << 8) | b[9 + 2 * i]);   /* bo qua nhiet do o b[6..7] */
  }
  return 1;
}
