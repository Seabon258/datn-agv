#include "link.h"
#include "usart.h"
#include <string.h>

enum { S_SYNC1, S_SYNC2, S_LEN, S_CMD, S_PAYLOAD, S_CRC_LO, S_CRC_HI };

static uint8_t st = S_SYNC1;
static link_frame_t rx, ready;
static uint8_t idx;
static uint16_t crc_rx;
static volatile uint8_t ready_flag;

uint16_t Link_Crc16(const uint8_t *d, uint32_t n, uint16_t crc)
{
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
  }
  return crc;
}

uint8_t Link_RxByte(uint8_t c)
{
  switch (st) {
  case S_SYNC1:
    if (c != 0xAA) return 0;                  /* khong phai khung -> de console chu xu ly */
    st = S_SYNC2;
    return 1;
  case S_SYNC2:
    st = (c == 0x55) ? S_LEN : S_SYNC1;
    return 1;
  case S_LEN:
    if (c > LINK_MAX_PAYLOAD) { st = S_SYNC1; return 1; }
    rx.len = c;
    st = S_CMD;
    return 1;
  case S_CMD:
    rx.cmd = c;
    idx = 0;
    st = rx.len ? S_PAYLOAD : S_CRC_LO;
    return 1;
  case S_PAYLOAD:
    rx.payload[idx++] = c;
    if (idx >= rx.len) st = S_CRC_LO;
    return 1;
  case S_CRC_LO:
    crc_rx = c;
    st = S_CRC_HI;
    return 1;
  default: {                                  /* S_CRC_HI */
    crc_rx |= (uint16_t)c << 8;
    st = S_SYNC1;
    uint8_t hdr[2] = { rx.len, rx.cmd };
    uint16_t crc = Link_Crc16(hdr, 2, 0xFFFF);
    crc = Link_Crc16(rx.payload, rx.len, crc);
    if (crc == crc_rx && !ready_flag) {       /* khung cu chua xu ly thi bo khung moi */
      memcpy(&ready, &rx, sizeof rx);
      ready_flag = 1;
    }
    return 1;
  }
  }
}

uint8_t Link_Get(link_frame_t *f)
{
  if (!ready_flag) return 0;
  memcpy(f, &ready, sizeof ready);
  ready_flag = 0;
  return 1;
}

void Link_Send(uint8_t cmd, const void *payload, uint8_t len)
{
  uint8_t buf[4 + LINK_MAX_PAYLOAD + 2];
  if (len > LINK_MAX_PAYLOAD) return;
  buf[0] = 0xAA;
  buf[1] = 0x55;
  buf[2] = len;
  buf[3] = cmd;
  memcpy(&buf[4], payload, len);
  uint16_t crc = Link_Crc16(&buf[2], 2U + len, 0xFFFF);
  buf[4 + len] = (uint8_t)crc;
  buf[5 + len] = (uint8_t)(crc >> 8);
  HAL_UART_Transmit(&huart2, buf, (uint16_t)(6U + len), 10);
}
