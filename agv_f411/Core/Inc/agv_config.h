#ifndef AGV_CONFIG_H
#define AGV_CONFIG_H

/* Cac gia tri chinh duoc - sua o day, khong sua trong code */

/* Dong co DC: TIM1 ARR = 4999 -> 5000 muc */
#define MOTOR_PWM_TOP      5000
/* Bu vung chet chieu lui: chi IN2 co PWM nen luc lui khoang tat la tha troi (mat luc).
 * Do tren M4 2026-11-15: toc do lui bang toc do toi khi PWM_lui ~= 25% + 0.65 x lenh. */
#define MOTOR_REV_OFFSET   1800     /* 36% cua MOTOR_PWM_TOP (tren san co tai, 2026-11-19; khong tai do duoc 25%) */
#define MOTOR_REV_GAIN     0.65f
#define MOTOR_TEST_MS      2000U    /* lenh "m" chay bao lau roi tu dung */
#define DRIVE_TIMEOUT_MS   5000U    /* lenh "d" chua du xung sau khoang nay -> dung (ket banh, mat encoder) */

/* Console / telemetry qua USART2 */
#define TELEM_PERIOD_MS    500U

/* Do pin PA6: cau chia 47k / 10k, sau diode SS54 */
#define BATT_VREF_MV       3300U    /* 3V3 cua Black Pill, do that roi sua */
#define BATT_DIV_NUM       427U     /* hieu chinh 2026-11-18: doc 15.62 V khi dong ho do 11.8 V (ly thuyet 47k/10k = 5.7) */
#define BATT_DIV_DEN       100U
#define BATT_DIODE_MV      350U     /* sut ap SS54 */
#define BATT_SAMPLES       32U

/* Nang ha: chieu DIR nao la di LEN - CHUA KIEM TRA, thu tren xe roi sua (+1 hoac -1) */
#define LIFT_DIR_UP        1
#define LIFT_TEST_SPS      1600.0f  /* 1 vong/s */
#define LIFT_TEST_ACCEL    3200.0f
#define LIFT_HOME_SPS      1600.0f  /* toc do ve goc (di xuong toi cong tac BOT) */
#define LIFT_HOME_MAX_STEPS 16000   /* ve goc di toi da ~20 mm roi dung. Khi da gan cong tac + biet hanh trinh: dat = hanh trinh + 10% */
#define LIFT_MAX_MOVE_STEPS 16000   /* lenh "l" toi da moi lan */
#define LIFT_AUTO_DISABLE  1        /* vitme lead 2 tu ham: tat driver khi dung cho mat */

/* PID toc do banh - don vi: xung encoder/giay -> muc PWM. CHUA CHINH, chinh khi co xe */
#define CTRL_PERIOD_MS     10U      /* vong PID 100 Hz */
#define WHEEL_KF           0.70f    /* feed-forward: do 2026-11-15, toi 50% (2500) ~ 3400 xung/s */
#define WHEEL_KP           0.20f    /* chinh tren san 2026-11-19 (0.50 lam xe giat/lac) */
#define WHEEL_KI           1.50f
#define WHEEL_KS           400.0f   /* bu ma sat tinh: cong them khi banh co lenh quay (8%) */
#define WHEEL_SPEED_LPF    0.20f    /* loc toc do do duoc (0..1, lon = nhanh) */
#define WHEEL_ACCEL_CPS2   2500.0f  /* tang/giam toc do dat toi da, xung/s moi giay (0 -> 1500 mat 0.6 s) */
#define WHEEL_PWM_SLEW     10000.0f /* che do PWM: muc PWM thay doi toi da moi giay (0 -> 100% mat 0.5 s) */
/* Huong xe: DAU XE = phia M3, M4. TIEN = ve phia M3, M4.
 * Cach cam day tu 2026-11-20: M4 truoc-trai, M3 truoc-phai, M2 sau-trai, M1 sau-phai.
 * Kiem tra banh ke len: lenh toc do [+,+,-,-] voi dau {-1,1,1,-1} -> ca 4 banh lan ve dau xe -> dau moi. */
#define MOTOR_SIGN         { -1, 1, -1, 1 }
#define ENC_SIGN           { -1, 1, -1, 1 }

/* Giu huong bang gyro MPU6050 (lenh "b", khung BODY). Doi duoc qua lenh "g" */
#define GYRO_Z_SIGN        1        /* +1 neu gyro Z duong khi xe xoay TRAI. CHUA KIEM TRA: thu "b 0 0 800 0" */
#define GYRO_LSB_PER_DPS   65.5f    /* thang +-500 do/s */
#define HEADING_KP         60.0f    /* xung/s moi banh cho moi do lech huong (120 lam xe giang co, truot tai cho) */
#define HEADING_KD         6.0f     /* xung/s cho moi do/s toc do xoay */
#define HEADING_KI         40.0f    /* xung/s cho moi do x giay: bu xu huong queo co dinh (doi bang lenh "j") */
#define HEADING_MAX_CPS    400.0f   /* bu xoay toi da (lon qua -> 2 ben gianh nhau, banh mecanum truot) */
#define DRIVE_TEST_MS      3000U    /* lenh chu "b" chay bao lau */

/* Banh xe: JGB37-520 111 vong/phut theo nha san xuat (hop so 1:90), encoder 11 xung x4 -> ~3960 xung/vong banh (uoc tinh,
 * chua do). Banh mecanum 80 mm -> chu vi 251.3 mm -> ~15.8 xung/mm. */
#define WHEEL_CPR_EST      3960

/* Lien ket voi Pi (khung nhi phan) */
#define LINK_TIMEOUT_MS    300U     /* khong nhan khung hop le trong khoang nay -> dung dong co */
#define LINK_TELEM_MS      20U      /* gui telemetry 50 Hz */
#define LINK_ACTIVE_MS     1000U    /* con nhan khung trong 1 s -> tat in chu, chi gui nhi phan */

/* Watchdog: chip treo qua thoi gian nay -> tu reset, dong co tha troi */
#define IWDG_TIMEOUT_MS    500U

#endif
