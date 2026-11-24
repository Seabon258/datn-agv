# AGV controller board — hardware reference for firmware / software

> Board: `machdieukhien.PcbDoc` (Altium project `mach1.PrjPcb`), 2-layer, 100 × 100 mm.
> A single-layer hand-made variant `mach1lop.PcbDoc` uses the SAME netlist and pin map — firmware is identical.
> Netlist below was extracted directly from the PCB file (2026-11-02). Treat it as the source of truth.
> Anything marked **TBD** is not decided in hardware — ask the user, do not guess.

## 1. System overview

```
12 V lead-acid battery
   │  (JDC1 DC jack  or  HD2 2-pin header — same net "12V")
   ▼
 F1 fuse ──► D1 SS54 Schottky (reverse-polarity protection, ~0.3–0.5 V drop)
   │
   ▼  net "12V sau loc"  (≈ Vbat − 0.4 V)
   ├──► DRV1..DRV4 VM   (4 × DRV8871 module, brushed DC motors JGB37 with Hall encoder)
   ├──► TMC2209 VM      (stepper for the lift)
   ├──► R7/R8 divider ──► PA6 (battery voltage ADC)
   ├──► R1 + LED "1"    (power indicator)
   └──► HD1 ──► external LM2596 buck module (12 V → 5 V)
                   │ 5 V back in through C8 (net "5VIN")
                   ▼
              L2 ─► C5 ─► L1   (2-stage LC filter)
                   ▼  net "5V"
              Black Pill 5V pin ──► on-board 3.3 V LDO ──► net "3V3"
                                    (encoders, MPU6050, TMC2209 VIO, pull-ups)

Raspberry Pi 4 (upper deck, own 5 V / 3 A supply) ◄── UART 3-pin header C11 ──► STM32
```

MCU: **WeAct Black Pill STM32F411CEU6**, HSE 25 MHz, run SYSCLK = 100 MHz.
GND is common for everything (including the Raspberry Pi through C11 pin 1).

## 2. STM32 pin map (complete)

| MCU pin | Function in this board | Peripheral to configure | Notes |
|---|---|---|---|
| PA8  | DRV1 IN2 (PWM) | TIM1_CH1 PWM | 10 k pull-down R2 |
| PA9  | DRV2 IN2 (PWM) | TIM1_CH2 PWM | 10 k pull-down R3 |
| PA10 | DRV3 IN2 (PWM) | TIM1_CH3 PWM | 10 k pull-down R4 |
| PA11 | DRV4 IN2 (PWM) | TIM1_CH4 PWM | 10 k pull-down R5. **Also USB D− on Black Pill → USB cannot be used** |
| PB12 | DRV1 IN1 (DIR) | GPIO output | |
| PB13 | DRV2 IN1 (DIR) | GPIO output | |
| PB14 | DRV3 IN1 (DIR) | GPIO output | |
| PB15 | DRV4 IN1 (DIR) | GPIO output | |
| PA15 | Motor 1 encoder C2 | TIM2_CH1 encoder | 32-bit timer. JTAG pin → set Debug = Serial Wire |
| PB3  | Motor 1 encoder C1 | TIM2_CH2 encoder | JTAG pin → set Debug = Serial Wire |
| PB4  | Motor 2 encoder | TIM3_CH1 encoder | 16-bit timer |
| PB5  | Motor 2 encoder | TIM3_CH2 encoder | |
| PB6  | Motor 3 encoder | TIM4_CH1 encoder | 16-bit timer |
| PB7  | Motor 3 encoder | TIM4_CH2 encoder | |
| PA0  | Motor 4 encoder | TIM5_CH1 encoder | 32-bit timer. Black Pill KEY button is also on PA0 — do not press it |
| PA1  | Motor 4 encoder | TIM5_CH2 encoder | |
| PA2  | UART TX → Raspberry Pi RX | USART2_TX | |
| PA3  | UART RX ← Raspberry Pi TX | USART2_RX | |
| PB8  | MPU6050 SCL | I2C1_SCL | module has its own pull-ups |
| PB9  | MPU6050 SDA | I2C1_SDA | |
| PA5  | TMC2209 EN | GPIO output | **active LOW**. R6 10 k pull-up → driver disabled at reset |
| PB0  | TMC2209 STEP | GPIO output | toggled from a timer ISR (see §5) |
| PB1  | TMC2209 DIR | GPIO output | |
| PA4  | Limit switch TOP (HD3 pin 1) | GPIO input, internal pull-up | switch closes to GND → active LOW |
| PB10 | Limit switch BOTTOM (HD3 pin 3) | GPIO input, internal pull-up | active LOW |
| PA6  | Battery voltage | ADC1_IN6 | divider 47 k / 10 k + 100 nF (C12) |
| PC13 | Black Pill on-board LED | GPIO output | active LOW, free for status |
| PA13 / PA14 | SWDIO / SWCLK | SYS Serial Wire | flash with ST-Link (recommended, see §6) |
| PA7, PA12, PB2, PC14, PC15 | not used | — | PA12 = USB D+ |

Peripherals left free: TIM9, TIM10, TIM11, ADC other channels, I2C2/SPI.

## 3. DC motors — DRV8871 ×4

Connector DCx pin order (6-pin): 1 = M1 (motor +), 2 = GND, 3 = C2, 4 = C1, 5 = VCC (3.3 V), 6 = M2.
Encoders are powered from **3.3 V**, so their outputs are 3.3 V logic.

| Motor | DRV | PWM (IN2) | DIR (IN1) | Encoder timer |
|---|---|---|---|---|
| M1 | DRV1 → DC1 | PA8  TIM1_CH1 | PB12 | TIM2 (PA15, PB3) |
| M2 | DRV2 → DC2 | PA9  TIM1_CH2 | PB13 | TIM3 (PB4, PB5) |
| M3 | DRV3 → DC3 | PA10 TIM1_CH3 | PB14 | TIM4 (PB6, PB7) |
| M4 | DRV4 → DC4 | PA11 TIM1_CH4 | PB15 | TIM5 (PA0, PA1) |

Robot FRONT = the M3/M4 end. Connectors re-plugged 2026-11-20: **M4 front-left, M3 front-right, M2 rear-left, M1 rear-right.**
Firmware `MOTOR_SIGN`/`ENC_SIGN` = { -1, 1, -1, 1 } so "+" = wheel rolls forward (checked with wheels lifted).
Encoder CPR (pulses per wheel revolution): **TBD** — depends on the JGB37 gear ratio
(JGB37 Hall encoder is usually 11 PPR on the motor shaft × gear ratio × 4 in TI12 mode).

### PWM
- TIM1 on APB2 = 100 MHz, PSC = 0, ARR = 4999 → **20 kHz**, 5000 steps.
- TIM1 is an advanced timer: call `HAL_TIM_PWM_Start` **and** make sure MOE is set (HAL does it).

### DRV8871 truth table
| IN1 | IN2 | Result |
|---|---|---|
| 0 | 0 | coast (outputs Hi-Z) |
| 0 | 1 | reverse: OUT1 L, OUT2 H |
| 1 | 0 | forward: OUT1 H, OUT2 L |
| 1 | 1 | brake (both low) |

Because only IN2 has PWM, the duty must be **inverted when IN1 = 1**:

```c
// speed: -4999 .. +4999   (ARR = 4999)
void motor_set(int ch, int speed) {
    if (speed >= 0) {                 // forward: IN1 = 1, IN2 = PWM, IN2 low = drive
        HAL_GPIO_WritePin(dir_port[ch], dir_pin[ch], GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, pwm_ch[ch], 4999 - speed);
    } else {                          // reverse: IN1 = 0, IN2 = PWM, IN2 high = drive
        HAL_GPIO_WritePin(dir_port[ch], dir_pin[ch], GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, pwm_ch[ch], -speed);
    }
}
// stop (coast): IN1 = 0, CCR = 0.   brake: IN1 = 1, CCR = ARR + 1.
```
"Forward" here means OUT1 high; the real wheel direction per motor must be checked on the robot.

### Encoders
- TIM2/TIM5 are 32-bit, TIM3/TIM4 16-bit. Read the counter at a fixed period and use a
  signed difference (`(int16_t)(now - last)` for 16-bit timers) to handle overflow.
- Use input filter (ICxF ≈ 6–10) because motor noise is nearby.

## 4. IMU — MPU6050 (GY-521 module)
- I2C1, PB8 = SCL, PB9 = SDA, 400 kHz.
- AD0 not connected → module pull-down → address **0x68**.
- INT not connected → poll the sensor.

## 5. Lift stepper — TMC2209 standalone (step/dir)
- EN = PA5 (LOW = enabled), STEP = PB0, DIR = PB1, VIO = 3.3 V, VM = 12 V.
- MS1, MS2, PDN_UART, CLK not connected → TMC2209 internal pull-downs → **1/8 microstep**,
  StealthChop. Full-step motor 200 steps/rev → 1600 microsteps/rev.
- No UART to the driver. Motor current is set by the Vref potentiometer on the module.
- PB0/PB1 belong to TIM3 (already the encoder of M2), so generate STEP pulses by toggling
  the GPIO from a basic timer ISR (TIM10 or TIM11). STEP pulse ≥ 100 ns high.
- Homing: drive down until PB10 (BOTTOM) reads LOW; never drive up when PA4 (TOP) is LOW.
- Steps per mm of lift travel: **TBD**.

Motor output P5: 1 = M1B (B1), 2 = M1A (A1), 3 = M2A (A2), 4 = M2B (B2).

## 6. Battery monitoring (PA6)
```
V_adc   = raw / 4095 * 3.3
V_after_diode = V_adc * (47 + 10) / 10 = V_adc * 5.7
V_bat  ≈ V_after_diode + 0.35   // SS54 drop, ~0.3 V at low current, up to ~0.5 V at 5 A
```
- 14.4 V (charging) → 2.47 V on ADC, safe.
- Use long sample time (≥ 144 cycles), average 32–64 samples, low-pass filter.
- Voltage sags while motors pull current; estimate % only when motors are idle,
  or compensate with load.
- 12 V lead-acid open-circuit voltage → state of charge:

| V | 12.73 | 12.62 | 12.50 | 12.37 | 12.24 | 12.10 | 11.96 | 11.81 | 11.66 | 11.51 | ≤11.3 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| % | 100 | 90 | 80 | 70 | 60 | 50 | 40 | 30 | 20 | 10 | 0 |

Interpolate linearly. Recommend a low-battery stop around 11.5 V at rest.

## 7. UART link to Raspberry Pi 4

Header C11 (3 pins, no power pin):

| C11 pin | STM32 | Raspberry Pi 4 header |
|---|---|---|
| 1 | GND | pin 6 (GND) |
| 2 | PA2 = USART2_TX | pin 10 = GPIO15 RXD |
| 3 | PA3 = USART2_RX | pin 8 = GPIO14 TXD (recommend 1 kΩ series resistor in this wire) |

- Baud **460800**, 8N1, 3.3 V logic both sides (no level shifter needed).
- STM32: use DMA RX with idle-line detection (`HAL_UARTEx_ReceiveToIdle_DMA`).
- Raspberry Pi: use the PL011 UART, not the mini-UART. In `/boot/firmware/config.txt`
  (older OS: `/boot/config.txt`):
  ```
  enable_uart=1
  dtoverlay=disable-bt
  ```
  then disable the serial console (`sudo raspi-config` → Interface → Serial: login shell **No**,
  hardware **Yes**) and reboot. Device: `/dev/serial0` (→ `/dev/ttyAMA0`).
- The Pi has its own 5 V / 3 A supply; it is never powered through this board.

### Protocol — implemented v1, see `docs/UART_PROTOCOL.md` (original proposal below)
Binary frames: `0xAA 0x55 | len | cmd | payload | CRC16-CCITT (lo, hi)`, little-endian.
- Pi → STM32: wheel speed setpoints (4 × int16), lift command, enable/estop.
- STM32 → Pi (e.g. 50–100 Hz): encoder counts (4 × int32), IMU raw (6 × int16),
  battery mV (uint16), limit switch bits, status flags.
- STM32 must stop all motors if no valid command is received for ~200–300 ms (watchdog).

## 8. Firmware notes
- CubeMX file: `agv_f411.ioc`. SYS → Debug = **Serial Wire** (frees PA15/PB3/PB4).
- Flash via **ST-Link (SWD)**. Avoid USB DFU: PA11 (USB D−) is wired to DRV4 with a 10 k pull-down.
- Safe start: after reset all IN2 = 0 (pull-downs) → DC motors coast; TMC2209 disabled (EN pulled up).
  Enable motors only after the Pi link is up.
- Suggested timing: control loop 1 kHz or 200 Hz on TIM11 (PID per wheel), telemetry 50–100 Hz,
  IMU read 100–200 Hz.
