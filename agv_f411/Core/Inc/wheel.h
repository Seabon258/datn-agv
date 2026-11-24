#ifndef WHEEL_H
#define WHEEL_H

#include <stdint.h>

/* Dieu khien 4 banh: tat / PWM tho / PID toc do. Dau theo MOTOR_SIGN, ENC_SIGN trong agv_config.h */
enum { WHEEL_OFF = 0, WHEEL_PWM = 1, WHEEL_SPEED = 2 };

void    Wheel_Init(void);
void    Wheel_Stop(void);                       /* tha troi ca 4 */
void    Wheel_SetPwm(const int16_t duty[4]);    /* -MOTOR_PWM_TOP .. +MOTOR_PWM_TOP */
void    Wheel_SetSpeed(const int16_t cps[4]);   /* xung encoder / giay */
void    Wheel_Ctrl(void);                       /* goi moi CTRL_PERIOD_MS */
int16_t Wheel_Speed(int ch);                    /* toc do do duoc, xung/s */
int32_t Wheel_Count(int ch);                    /* tong xung (da nhan ENC_SIGN) */
uint8_t Wheel_Mode(void);
void    Wheel_SetGains(float kf, float kp, float ki, float ks);   /* doi he so PID khi dang chay */
void    Wheel_GetGains(float *kf, float *kp, float *ki, float *ks);

#endif
