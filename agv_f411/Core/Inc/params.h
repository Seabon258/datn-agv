#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

/* Thong so chinh duoc luu o sector 7 flash (0x08060000, linker da chua ra).
 * Khoi dong: neu co ban luu hop le thi dung, khong thi dung gia tri trong agv_config.h. */
uint8_t Params_Load(void);      /* 1 = da nap ban luu */
uint8_t Params_Save(void);      /* 1 = OK. Dung dong co truoc khi goi: CPU dung ~1-2 s khi xoa flash */
void    Params_Print(void);
uint8_t Params_Clear(void);     /* xoa ban luu -> lan khoi dong sau dung mac dinh */

#endif
