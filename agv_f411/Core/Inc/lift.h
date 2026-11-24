#ifndef LIFT_H
#define LIFT_H

#include <stdint.h>

/* TMC2209 o che do STEP/DIR, MS1 = MS2 = GND -> 1/8 buoc */
#define LIFT_STEPS_REV   1600      /* 200 buoc x 8 */
#define LIFT_LEAD_MM     2.0f      /* vitme T8 lead 2 */
#define LIFT_GEAR_RATIO  1.0f      /* puly GT2 1:1 */
#define LIFT_STEPS_MM    (LIFT_STEPS_REV * LIFT_GEAR_RATIO / LIFT_LEAD_MM)   /* 800 */

void    Lift_Init(void);
void    Lift_Enable(uint8_t on);
uint8_t Lift_MoveSteps(int32_t steps, float max_sps, float accel_sps2);   /* 0 = bi chan boi cong tac */
void    Lift_Stop(void);
void    Lift_Home(float sps, float accel_sps2);   /* di xuong toi BOT, vi tri = 0 */
uint8_t Lift_Homed(void);
uint8_t Lift_Homing(void);
uint8_t Lift_Busy(void);
int32_t Lift_Pos(void);                         /* buoc, + = len */

void    Lift_Tick1ms(void);   /* goi moi 1 ms: cap nhat toc do, dung khi cham cong tac */
void    Lift_StepISR(void);   /* goi trong ngat TIM10: phat 1 xung STEP */

#endif
