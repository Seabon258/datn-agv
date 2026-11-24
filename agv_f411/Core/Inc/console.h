#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

/* Console chu qua USART2 (C11), dung de test mach. Giao thuc that voi Pi lam sau. */
void    Con_Init(void);
void    Con_Printf(const char *fmt, ...);
uint8_t Con_GetLine(char *out, uint32_t size);   /* 1 = co 1 dong lenh moi */
void    Con_Poll(void);                              /* goi trong vong lap: chay lai RX neu bi dung */
void    Con_Stats(uint32_t *bytes, uint32_t *errors);   /* de kiem tra day RX */

#endif
