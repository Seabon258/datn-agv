#ifndef IMU_H
#define IMU_H

#include <stdint.h>

/* MPU6050 tren I2C1, dia chi 0x68. Tra ve WHO_AM_I (0x68 la chuan), 0 = khong tra loi */
uint8_t IMU_Init(void);
uint8_t IMU_Read(int16_t acc[3], int16_t gyro[3]);   /* 1 = OK */

#endif
