#ifndef LINK_H
#define LINK_H

#include <stdint.h>

/* Khung nhi phan voi Pi, little-endian:
 *   0xAA 0x55 | len | cmd | payload[len] | crc16 lo | crc16 hi
 * crc16 = CRC-16/CCITT-FALSE (khoi tao 0xFFFF, da thuc 0x1021) tinh tren len, cmd, payload.
 * Mo ta day du: docs/UART_PROTOCOL.md */

/* Pi -> STM32 */
#define LINK_CMD_SPEED      0x01  /* 4 x int16: toc do banh, xung/s */
#define LINK_CMD_PWM        0x02  /* 4 x int16: PWM tho -5000..5000 (de thu) */
#define LINK_CMD_STOP       0x03  /* dung banh + nang ha */
#define LINK_CMD_LIFT_MOVE  0x04  /* int32 so buoc (+ = len), uint16 buoc/s */
#define LINK_CMD_LIFT_HOME  0x05  /* uint16 buoc/s: di xuong toi cong tac BOT, dat vi tri = 0 */
#define LINK_CMD_PING       0x06  /* giu ket noi, khong lam gi */
#define LINK_CMD_BODY       0x07  /* 3 x int16 vx (toi +), vy (trai +), wz (xoay trai +) xung/s; uint8 giu huong 0/1 */

/* STM32 -> Pi */
#define LINK_CMD_TELEM      0x81  /* xem link_telem_t */

#define LINK_MAX_PAYLOAD    64

typedef struct __attribute__((packed)) {
  uint32_t ms;            /* HAL_GetTick */
  int32_t  enc[4];        /* tong xung moi banh (da nhan ENC_SIGN) */
  int16_t  speed[4];      /* xung/s */
  int16_t  acc[3];        /* MPU6050 tho, +-2 g */
  int16_t  gyro[3];       /* MPU6050 tho, +-500 do/s */
  uint16_t batt_mv;
  int32_t  lift_pos;      /* buoc, + = len, 0 = goc duoi */
  uint8_t  flags;         /* LINK_F_* */
  uint8_t  wheel_mode;    /* WHEEL_OFF / PWM / SPEED */
  int16_t  heading_cdeg;  /* huong xe tu gyro, phan tram do, + = xoay trai, quan -18000..18000 */
} link_telem_t;

#define LINK_F_LIMIT_TOP    0x01
#define LINK_F_LIMIT_BOT    0x02
#define LINK_F_LIFT_BUSY    0x04
#define LINK_F_LIFT_HOMED   0x08
#define LINK_F_IMU_OK       0x10
#define LINK_F_TIMEOUT      0x20  /* da tu dung vi mat lien ket (xoa khi nhan lenh moi) */
#define LINK_F_WDG_RESET    0x40  /* lan khoi dong nay do watchdog reset */

typedef struct {
  uint8_t cmd;
  uint8_t len;
  uint8_t payload[LINK_MAX_PAYLOAD];
} link_frame_t;

uint8_t Link_RxByte(uint8_t c);                 /* goi trong ngat RX; 1 = byte thuoc khung nhi phan */
uint8_t Link_Get(link_frame_t *f);              /* 1 = co khung moi hop le */
void    Link_Send(uint8_t cmd, const void *payload, uint8_t len);
uint16_t Link_Crc16(const uint8_t *d, uint32_t n, uint16_t crc);

#endif
