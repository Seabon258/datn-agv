#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

/* Dieu khien ca xe mecanum (tren Wheel PID): vx toi +, vy sang TRAI +, wz xoay TRAI + (xung/s moi banh).
 * hold = 1 va wz = 0: dung gyro MPU6050 giu huong xe. */
void    Drive_Init(void);                       /* goi sau IMU_Init: do lech goc gyro (xe phai dung yen) */
void    Drive_Set(int16_t vx, int16_t vy, int16_t wz, uint8_t hold);
void    Drive_Off(void);                        /* bo dieu khien ca xe (lenh banh rieng le / dung) */
void    Drive_Ctrl(void);                       /* goi moi CTRL_PERIOD_MS, truoc Wheel_Ctrl */
float   Drive_Heading(void);                    /* do, + = xoay trai, tich phan tu luc khoi dong */
uint8_t Drive_Active(void);
void    Drive_SetGyro(int8_t sign, float kp, float kd);   /* doi chieu gyro + he so giu huong */
void    Drive_GetGyro(int8_t *sign, float *kp, float *kd, float *bias);
void    Drive_SetHeadingKi(float ki);           /* lenh "j" */
float   Drive_GetHeadingKi(void);

#endif
