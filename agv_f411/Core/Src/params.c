#include "params.h"
#include "agv_config.h"
#include "console.h"
#include "drive.h"
#include "link.h"
#include "main.h"
#include "motor.h"
#include "wheel.h"
#include <stddef.h>
#include <string.h>

#define PARAMS_ADDR     0x08060000UL
#define PARAMS_SECTOR   FLASH_SECTOR_7
#define PARAMS_MAGIC    0x41475631UL    /* "AGV1" */
#define PARAMS_VERSION  1U

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  float    kf, kp, ki, ks;              /* PID banh */
  int32_t  rev_off;                     /* bu vung chet lui */
  float    rev_gain;
  int32_t  gyro_sign;                   /* giu huong */
  float    hkp, hkd, hki;
  uint32_t crc;                         /* CRC16 cua cac truong tren, mo rong 32 bit */
} params_t;

static uint32_t crc_of(const params_t *p)
{
  return Link_Crc16((const uint8_t *)p, offsetof(params_t, crc), 0xFFFF);
}

static void collect(params_t *p)
{
  memset(p, 0, sizeof *p);
  p->magic = PARAMS_MAGIC;
  p->version = PARAMS_VERSION;
  p->size = sizeof *p;
  Wheel_GetGains(&p->kf, &p->kp, &p->ki, &p->ks);
  Motor_GetRevComp(&p->rev_off, &p->rev_gain);
  int8_t s;
  float bias;
  Drive_GetGyro(&s, &p->hkp, &p->hkd, &bias);
  p->gyro_sign = s;
  p->hki = Drive_GetHeadingKi();
  p->crc = crc_of(p);
}

uint8_t Params_Load(void)
{
  params_t p;
  memcpy(&p, (const void *)PARAMS_ADDR, sizeof p);
  if (p.magic != PARAMS_MAGIC || p.version != PARAMS_VERSION || p.size != sizeof p || p.crc != crc_of(&p))
    return 0;
  Wheel_SetGains(p.kf, p.kp, p.ki, p.ks);
  Motor_SetRevComp(p.rev_off, p.rev_gain);
  Drive_SetGyro((int8_t)p.gyro_sign, p.hkp, p.hkd);
  Drive_SetHeadingKi(p.hki);
  return 1;
}

/* Xoa sector 128 KB mat ~1-2 s, lau hon watchdog 500 ms -> tam keo dai watchdog */
static void iwdg_long(uint8_t on)
{
  IWDG->KR = 0xAAAA;
  IWDG->KR = 0x5555;
  IWDG->PR = on ? 6 : 3;                 /* /256: 8 ms/don vi; /32: 1 ms/don vi */
  IWDG->RLR = on ? 0xFFF : IWDG_TIMEOUT_MS;
  while (IWDG->SR) { }
  IWDG->KR = 0xAAAA;
}

static uint8_t erase(void)
{
  FLASH_EraseInitTypeDef e = {0};
  uint32_t bad = 0;
  e.TypeErase = FLASH_TYPEERASE_SECTORS;
  e.Sector = PARAMS_SECTOR;
  e.NbSectors = 1;
  e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  return HAL_FLASHEx_Erase(&e, &bad) == HAL_OK;
}

uint8_t Params_Save(void)
{
  params_t p;
  collect(&p);
  iwdg_long(1);
  HAL_FLASH_Unlock();
  uint8_t ok = erase();
  const uint32_t *w = (const uint32_t *)&p;
  for (uint32_t i = 0; ok && i < sizeof p / 4; i++)
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAMS_ADDR + 4 * i, w[i]) == HAL_OK;
  HAL_FLASH_Lock();
  iwdg_long(0);
  return ok && memcmp(&p, (const void *)PARAMS_ADDR, sizeof p) == 0;
}

uint8_t Params_Clear(void)
{
  iwdg_long(1);
  HAL_FLASH_Unlock();
  uint8_t ok = erase();
  HAL_FLASH_Lock();
  iwdg_long(0);
  return ok;
}

void Params_Print(void)
{
  params_t p;
  collect(&p);
  Con_Printf("PID k %ld %ld %ld %ld | bu lui c %ld %ld | gyro g %ld %ld %ld | j %ld  (phan nghin)\r\n",
             (long)(p.kf * 1000), (long)(p.kp * 1000), (long)(p.ki * 1000), (long)(p.ks * 1000),
             (long)p.rev_off, (long)(p.rev_gain * 1000),
             (long)p.gyro_sign, (long)(p.hkp * 1000), (long)(p.hkd * 1000), (long)(p.hki * 1000));
}
