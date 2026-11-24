# AGV project — instructions for Claude

This repo contains the software for an AGV (line-following delivery robot):
- `firmware/` — STM32F411 (Black Pill) firmware, STM32CubeIDE + HAL, C.
- `pi/` — Raspberry Pi 4 software, Python 3.

Read these before writing code:
- `docs/AGV_HARDWARE.md` — pin map, timers, motor/encoder/IMU/stepper/battery/UART facts. **Hardware source of truth.**
- `docs/AGV_SOFTWARE_SPEC.md` — mission, architecture, state machine, database, phases.
- `docs/mach1.pdf` — schematic. `docs/mach1_BOM.csv` — parts list.

Rules:
- Never guess anything marked **TBD**; ask the user.
- Do not change pin assignments — the PCB is already manufactured.
- Motors must stop on UART timeout and on any exception; test new motor code with wheels off the ground.
- Keep tunable values (PID gains, thresholds, speeds, map) in `pi/config.yaml` / a firmware config header.
- The user is Vietnamese; reply in Vietnamese.
