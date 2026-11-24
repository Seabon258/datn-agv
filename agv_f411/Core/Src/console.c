#include "console.h"
#include "link.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RX_SZ    64U
#define LINE_SZ  64U

static uint8_t rx_dma[RX_SZ];
static uint16_t rx_pos;
static char line[LINE_SZ];
static uint32_t line_len;
static uint8_t line_bad;                    /* dong co ky tu rac / qua dai / loi UART -> bo */
static char ready[LINE_SZ];
static volatile uint8_t ready_flag;
static volatile uint32_t rx_bytes, rx_errors;

void Con_Init(void)
{
  rx_pos = 0;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma, RX_SZ);   /* DMA vong, ngat khi day hoac nghi */
}

void Con_Printf(const char *fmt, ...)
{
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof buf - 1) n = sizeof buf - 1;
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
}

uint8_t Con_GetLine(char *out, uint32_t size)
{
  if (!ready_flag) return 0;
  strncpy(out, ready, size - 1);
  out[size - 1] = '\0';
  ready_flag = 0;
  return 1;
}

void Con_Poll(void)
{
  if (huart2.RxState == HAL_UART_STATE_READY)  /* DMA RX khong chay (vd sau loi): chay lai */
    Con_Init();
}

void Con_Stats(uint32_t *bytes, uint32_t *errors)
{
  *bytes = rx_bytes;
  *errors = rx_errors;
}

static void push(char c)
{
  if (c == '\r' || c == '\n') {
    if (line_len > 0 && !line_bad && !ready_flag) {
      line[line_len] = '\0';
      memcpy(ready, line, line_len + 1);
      ready_flag = 1;
    }
    line_len = 0;
    line_bad = 0;
  } else if (c < 0x20 || c > 0x7E || line_len >= LINE_SZ - 1) {
    line_bad = 1;                             /* nhieu khi cam rut cap -> khong duoc ghep vao lenh */
  } else {
    line[line_len++] = c;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance != USART2) return;
  uint16_t end = size % RX_SZ;
  while (rx_pos != end) {
    uint8_t c = rx_dma[rx_pos];
    if (!Link_RxByte(c)) push((char)c);       /* khung nhi phan cho Pi, con lai la lenh chu */
    rx_bytes++;
    rx_pos = (rx_pos + 1U) % RX_SZ;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2) return;
  rx_errors++;
  line_bad = 1;                               /* dong dang go do co the da hong */
  HAL_UART_AbortReceive(huart);
  Con_Init();                               /* loi nhieu / tran: chay lai DMA */
}
