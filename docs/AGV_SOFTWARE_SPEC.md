# AGV software specification (Raspberry Pi 4 + STM32 firmware)

> Read together with `AGV_HARDWARE.md` (pin map, electrical facts — source of truth for hardware).
> Items marked **TBD** are not decided — ask the user, do not invent values.
> The user is Vietnamese; reply in Vietnamese (no need for diacritics if the user writes without them).

## 1. Mission

A 4-wheel AGV follows a **black line** on a light floor and delivers parcels:

1. Drive along the black line (line following).
2. **"+" (cross) marker** on the line = rest / stop point (station). Stop, update current position.
3. **Circle marker** (fallback: **filled square** if circles are hard to print) = pickup point.
   Stop, scan the **QR code** on the parcel with the second camera.
4. QR content → item type (**3 to 7 types**) → look up destination warehouse → drive there
   via the line graph → unload (lift stepper) → return.
5. **Lidar**: stop for obstacles; if the line is lost (broken / covered), use IMU + encoder
   dead-reckoning (and lidar) to drive forward and re-acquire the line.
6. Log battery, trips, events into a database and show them on a web page.

**No machine-learning training is required.** Everything uses classic OpenCV + pyzbar.

## 2. Hardware on the Pi side

| Item | Value |
|---|---|
| SBC | Raspberry Pi 4 (RAM: **TBD**), own 5 V / 3 A supply |
| OS | Raspberry Pi OS 64-bit (Bookworm). Ubuntu 24.04 + ROS2 Jazzy only if SLAM phase is done |
| Line camera | **TBD**: RPi Camera v3 or "rapoo C260" USB webcam, pointing down at the floor |
| QR camera | the other one of the two cameras above, facing the parcel |
| Lidar | model **TBD** (USB) |
| Link to STM32 | `/dev/serial0`, 460800 8N1 — see `AGV_HARDWARE.md` §7 |

## 3. Software architecture (Pi, Python 3)

```
agv/
  stm32_link.py     # UART framing, CRC16, TX setpoints, RX telemetry thread, watchdog keep-alive
  line_vision.py    # line camera: threshold → centroid/error, marker detection
  qr_reader.py      # QR camera: pyzbar decode
  lidar.py          # lidar driver wrapper, obstacle zone check
  odom.py           # encoders + IMU → x, y, heading (dead reckoning)
  controller.py     # PID line following → wheel speed setpoints
  mission.py        # state machine (below)
  route_graph.py    # map as graph of nodes/edges, BFS shortest path
  db.py             # SQLite
  web.py            # Flask dashboard
  config.yaml       # all tunable values (thresholds, PID gains, speeds, node map, item→warehouse)
```

### 3.1 Line following
- Grayscale → blur → threshold (Otsu or fixed, inverted so black line = white) → ROI of the
  lower part of the frame → largest contour → centroid `cx`.
- Error = `cx − width/2` → PID → differential speed (left/right wheel pairs).
- Lower resolution (e.g. 320×240) for ≥ 30 FPS.

### 3.2 Marker detection (on the thresholded line image)
| Marker | Rule (tune in config) |
|---|---|
| "+" cross | blob much wider than the normal line width across the frame; solidity ≈ 0.4–0.6 |
| Circle | circularity `4πA/P²` > ~0.8 |
| Square (fallback) | `approxPolyDP` gives 4 corners and solidity > 0.9 |

Require the marker in N consecutive frames before acting (debounce). After acting, ignore the
same marker until it has left the frame.

### 3.3 Route graph
- Nodes = markers (stations, pickup points, warehouses), edges = line segments with the turn
  to take at each junction (left / straight / right). Defined in `config.yaml` — **map TBD**.
- Position = last marker passed. Path = BFS over the graph.
- Item type → warehouse mapping in config (3–7 types, **TBD**).

### 3.4 Mission state machine
`IDLE → FOLLOW_LINE → AT_STATION → AT_PICKUP (scan QR) → LOAD → GO_TO_WAREHOUSE → UNLOAD → RETURN`
plus `OBSTACLE_WAIT`, `LINE_LOST_RECOVER`, `LOW_BATTERY`, `ESTOP` from any state.

### 3.5 Lidar behaviour
- Obstacle in the front sector closer than threshold (e.g. 30 cm) → stop, wait, resume when clear.
- Line lost: keep heading from IMU/odom, drive slowly forward a limited distance while
  searching; if not found, stop and raise an event.

### 3.6 Database (SQLite) + web (Flask)
Tables: `items(type, name, warehouse)`, `orders`, `trips(start, end, item, from, to, result)`,
`events(time, kind, detail)`, `battery_log(time, volt, percent)`.
Battery % from voltage using the lead-acid table in `AGV_HARDWARE.md` §6.
Web page: current state, position, battery chart, trip/event history.

## 4. STM32 firmware (C, STM32CubeIDE + HAL)

Pin map, timers, motor direction logic, battery formula: **all in `AGV_HARDWARE.md`.**
Tasks:
- 4× wheel PID on encoder speed (TIM2..TIM5 encoder mode), PWM TIM1 20 kHz.
- MPU6050 over I2C1, lift stepper (TMC2209 STEP/DIR/EN) with limit switches.
- Battery ADC on PA6.
- UART protocol (proposal in `AGV_HARDWARE.md` §7 — finalize it in both sides together).
- **Safety:** stop motors if no valid command for ~200–300 ms; stop lift at limit switches.

## 5. Phases

| Phase | Content |
|---|---|
| 1 | Firmware: motors + encoders + UART telemetry. Pi: `stm32_link.py`, manual drive test |
| 2 | Line following + PID tuning |
| 3 | Markers + route graph + QR + lift |
| 4 | Lidar obstacle stop + line recovery |
| 5 | Database + web dashboard |
| 6 (only if 2 weeks available) | SLAM: ROS2 Jazzy, UART bridge node publishing `/odom` `/imu` tf, `slam_toolbox`, AMCL/Nav2 |

## 6. Open items (ask the user)
- Which camera is for the line and which for QR; lidar model; Pi RAM.
- Wheel layout (which DRV channel = which wheel, sign of forward), encoder counts per revolution.
- Lift steps per mm, travel length.
- Track map (nodes, junctions), item types and their warehouses, QR content format.
