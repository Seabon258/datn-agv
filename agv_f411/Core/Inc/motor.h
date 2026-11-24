#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* ch = 0..3 tuong ung M1..M4 */
void    Motor_Init(void);
void    Motor_Set(int ch, int32_t duty);   /* -MOTOR_PWM_TOP .. +MOTOR_PWM_TOP, 0 = tha troi */
void    Motor_StopAll(void);
void    Motor_SetRevComp(int32_t offset, float gain);   /* bu vung chet chieu lui (lenh "c") */
void    Motor_GetRevComp(int32_t *offset, float *gain);
void    Motor_EmergencyOff(void);          /* chi ghi thanh ghi, goi duoc trong Error_Handler / HardFault */

void    Enc_Update(void);                  /* goi moi 1 ms */
int32_t Enc_Get(int ch);                   /* tong so xung tu luc khoi dong */

#endif
