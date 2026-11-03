# CoVAPSy 1/10 Autonomous Car

Production codebase for the **CoVAPSy 1/10-scale autonomous racing car** (Tamiya TT-02 chassis). Built with a clean, low-latency dual-processor architecture: **bare Python on a Raspberry Pi 5** for LiDAR perception and follow-the-gap navigation, a compact **STM32G431 firmware** for hard real-time actuator control and safety interlocking, and a **Webots R2025a simulation environment** running the identical driving algorithms. **No ROS overhead.**

---

## System Architecture

```
  RPLIDAR A2M12   USB --------+
  RealSense D435i USB --------+   Raspberry Pi 5             USB-CDC        STM32G431KB (Nucleo on Custom Hat)
                              |    race.py (state machine)   /dev/ttyACM0     +- ESC Throttle PWM  (PA8, 50 Hz)
  Trackside Laptop (TCP/IP) --+    gap_follower (driving)    ------------>    +- Steering Servo PWM(PA11, 50 Hz)
     START / STOP / STATUS    |    lidar / camera / link                      +- Rear Sharp IR ADC (PA1)
                              +---------------------------------------------> +- I2C Bus: OLED + BNO055 IMU
                                                                              +- Piezo Buzzer      (PB6)
```

### Dual-Processor Split

1. **Raspberry Pi 5 (Perception & Navigation Loop — 15 Hz)**:
   - Reads 360° LiDAR range scans via USB.
   - Executes the core [`common/gap_follower.py`](common/gap_follower.py) algorithm.
   - Computes Ackerman steering angle and target cruising speed.
   - Manages race state machine, reverse recovery, and trackside telemetry.
2. **STM32G431KB (Hard Real-Time Control & Safety Watchdog — 50 Hz)**:
   - Generates jitter-free 50 Hz PWM signals for the ESC and high-torque steering servo.
   - Samples rear analog infrared distance sensor (Sharp GP2Y0A21YK0F) and wheel encoders.
   - Reads BNO055 9-DOF absolute orientation IMU and drives onboard I2C status OLED.
   - **Hardware Safety Watchdog**: If no valid USB-CDC heartbeat is received from the Pi within 250 ms, the STM32 immediately neutralizes motor throttle and centres steering.

---

## Follow-The-Gap (FTG) Driving Algorithm

The core driving brain resides in [`common/gap_follower.py`](common/gap_follower.py) as pure, stateless Python. It is directly imported by both the physical vehicle runtime ([`pi/race.py`](pi/race.py)) and the Webots simulation controller ([`simulation/controllers/covapsy_sim/covapsy_sim.py`](simulation/controllers/covapsy_sim/covapsy_sim.py)), ensuring 100% sim-to-real code parity.

```
Raw LiDAR Scan (360°)
   │
   ▼
[1. Preprocessing & Clamping]  ── Clip max range (5.0 m) & filter sensor blind spots
   │
   ▼
[2. Disparity Extension]       ── Inflate obstacle edges by half car width + clearance margin
   │
   ▼
[3. Safety Bubble Carving]     ── Zero out circular safety radius around closest obstacle
   │
   ▼
[4. Forward Gap Selection]     ── Find gaps > car width; score by: Width × (1.0 + 0.5 × cos(heading))
   │
   ▼
[5. Aim Point Computation]     ── Blend between gap center and deepest beam (aim_deep_bias = 0.80)
   │
   ▼
[6. Speed & TTC Calculation]   ── Scale pace via Time-To-Collision + dynamic adaptive forward cone
   │
   ▼
[7. Recovery Hysteresis]       ── Reverse Schmitt trigger: enter reverse at <0.30 m, exit at >0.65 m
```

### Key Algorithm Features

- **Disparity Extension & Obstacle Inflation**: Sharp range jumps between consecutive beams indicate occluded wall corners. The algorithm extends the nearer distance across adjacent beams by `(car_width_m / 2) + clearance_margin_m` to prevent clipping wall edges.
- **Forward-Biased Gap Scoring**: Rather than naively picking the widest gap (which can seduce the car into dead-end side pockets), gaps are scored with a cosine weighting toward the vehicle heading:
  $$\text{Score} = \text{Width} \times (1.0 + 0.5 \times \cos(\theta))$$
- **Apex-Biased Aim Point (`aim_deep_bias`)**: Blends between the center of the gap (safe) and the deepest beam within the gap (racing line). High bias ($\ge 0.80$) pulls the car tight into corner exits and straightens out curves.
- **Dynamic Adaptive Front Cone**: Adjusts the forward clearance sampling window dynamically between $\pm 15^\circ$ (on straights) and $\pm 35^\circ$ (under heavy steering), unlocking top speeds on straights while preventing false-braking into corner entries.
- **Time-to-Collision (TTC) Braking & Corner Speed Floor**: Calculates projected forward closing distance and throttles speed to maintain time-to-collision above `ttc_target_s` (1.08 s), while enforcing `min_speed_mps` (0.98 m/s) to ensure the car never stalls in tight hairpins.
- **Reverse Recovery Schmitt Trigger**: If front clearance drops below `emergency_front_m` (0.30 m), the car enters emergency reverse with mirrored steering. To prevent rapid forward/reverse hunting ("chattering"), it enforces hysteresis by maintaining reverse until clearance exceeds `reverse_exit_m` (0.65 m).

---

## Webots Simulation & Digital Twin

The project includes a faithful Webots R2025a simulation of the 1/10 TT-02 car ([`simulation/`](simulation/)):

- **Accurate Kinematics**: Ackermann steering geometry clamped strictly to $\pm 18^\circ$ ($\pm 0.314\text{ rad}$) to match physical servo limits and avoid simulation warnings.
- **Steering Slew-Rate Limiter (`max_steer_rate_dps: 180.0`)**: In default Webots, `setSteeringAngle` applies instantaneous step changes up to $36^\circ$ within a single 32 ms step ($>1100^\circ/\text{s}$), causing artificial tire slip, lateral snap, and scrubbing. The controller incorporates a slew-rate limiter modeled after physical digital RC servos ($180^\circ/\text{s}$ under load), producing realistic weight transfer and stable cornering.
- **GPS Start/Finish Lap Timer**: Embedded `LapTimer` tracks start/finish line crossings via GPS with debounce filtering, logging split times and lap durations in high-visibility console output.

---

## Automated Parameter Tuning (`tune_headless.py`)

An automated parameter optimizer designed for headless Webots execution:

- **Latin Hypercube Sampling (LHS)**: Multi-dimensional exploration over cruise speed, cornering floor speed, steering gain, deep-aim bias, TTC threshold, steering slew rate, and clearance margins.
- **Non-Destructive Recovery**: Allows the car to back up and recover naturally during trial laps, only triggering fail-fast aborts if the car is physically immobilized ($<0.10\text{ m}$ motion over 3.0 s) or exceeds timeout.
- **Live Leaderboard & Data Export**: Displays real-time trial rankings in the console and logs completed runs to [`tuning_results.csv`](tuning_results.csv).

---

## Configuration Reference (`common/config.py`)

All driving and simulation parameters are centrally maintained in [`common/config.py`](common/config.py):

| Parameter | Default | Unit | Description |
|:---|:---:|:---:|:---|
| `max_speed_mps` | `3.143` | m/s | Cruise Pace on open straights (~11.3 km/h) |
| `min_speed_mps` | `0.977` | m/s | Speed floor in tight turns (prevents corner stalling) |
| `reverse_speed_mps`| `0.900` | m/s | Reverse speed (simulation only) |
| `reverse_pwm_us` | `1250` | µs | Reverse pulse width sent to physical ESC |
| `steer_gain` | `1.347` | — | Proportional gain from gap angle to steering angle |
| `aim_deep_bias` | `0.800` | — | 0 = gap center, 1 = deepest point (apex pull) |
| `ttc_target_s` | `1.082` | s | Time-to-collision safety horizon for progressive braking |
| `max_steer_deg` | `18.0` | deg | Steering angle clamp (keeps inside Ackermann limit) |
| `max_steer_rate_dps` | `180.0` | deg/s | Steering rate limit (models physical servo speed) |
| `car_width_m` | `0.30` | m | Vehicle width used for disparity obstacle inflation |
| `clearance_margin_m` | `0.055` | m | Safety buffer added to half-car width during inflation |
| `safety_radius_m` | `0.18` | m | Radius carved around nearest obstacle point |
| `emergency_front_m` | `0.30` | m | Forward clearance threshold triggering emergency reverse |
| `reverse_exit_m` | `0.65` | m | Hysteresis clearance required before exiting reverse |
| `loop_hz` | `15` | Hz | Control loop rate on Raspberry Pi 5 |
| `wheel_moving_pps`| `5` | pulses/s | Encoder pulse rate below which wheels are stalled |
| `stuck_secs` | `3.0` | s | Stalled duration under forward throttle triggering recovery |

> [!TIP]
> Parameters can be overridden at runtime without modifying code by setting environment variables (`TUNE_<param>=<value>`, e.g., `TUNE_max_speed_mps=3.5`) or via `common/.tune_overrides.json`.

---

## Repository Layout

```
covapsy/
├── common/                     # Shared driving logic & configuration
│   ├── config.py               # Central tuning dictionary & dynamic override parser
│   └── gap_follower.py         # Pure Follow-The-Gap implementation (Pi & Sim)
├── pi/                         # On-vehicle Raspberry Pi 5 runtime
│   ├── race.py                 # Autonomous state machine (IDLE, RUNNING, STOPPED)
│   ├── covapsy_ctl.py          # Remote TCP client for START / STOP / STATUS commands
│   ├── lidar.py                # RPLIDAR A2M12 driver (returns canonical 360-float array)
│   ├── stm32_link.py           # USB-CDC protocol wrapper (PA8/PA11 commands & telemetry)
│   ├── camera.py               # Intel RealSense D435i OpenCV wrong-way detector (advisory)
│   └── requirements.txt        # Pi Python dependencies
├── simulation/                 # Webots R2025a digital twin
│   ├── worlds/
│   │   ├── standalone.wbt      # Single-car training & tuning track
│   │   └── multicar.wbt        # Multi-car head-to-head racing grid
│   ├── controllers/
│   │   └── covapsy_sim/        # Webots controller (GPS lap timer & slew rate limiter)
│   └── protos/
│       └── TT02_2025a.proto    # TT-02 4x4 Ackermann vehicle model with RPLIDAR proto
├── stm32/                      # STM32G431KB Bare-Metal C / HAL Firmware
│   ├── src/ & inc/             # PWM generation, ADC sampling, I2C IMU/OLED drivers
│   └── Makefile                # Cross-compilation & OpenOCD flashing rules
├── tests/                      # Automated test suite
│   ├── test_gap_follower.py    # 14 unit tests covering FTG logic, hysteresis, & clamping
│   └── test_*.py               # Pi hardware verification scripts (motor, steering, lidar)
├── run_sim.sh                  # One-click simulation launch script
├── tune_headless.py            # Automated Latin Hypercube headless parameter tuner
├── tuning_results.csv          # Benchmark logs and trial results
└── README.md                   # Project documentation
```

---

## Quickstart

### 1. Physical Car Setup (Raspberry Pi 5 + STM32)

1. **Flash STM32 Firmware** (directly from the Pi over USB):
   ```bash
   cd stm32
   make && make flash
   ```
2. **Install Python Dependencies**:
   ```bash
   cd ../pi
   pip3 install -r requirements.txt
   ```
3. **Launch Autonomous Race Control**:
   ```bash
   python3 race.py                 # Boots into IDLE, arms 250 ms watchdog
   # Optional flags:
   #   --dry-run                   # Steers to LiDAR but forces throttle to 0
   #   --camera                    # Enables RealSense RGB wrong-way monitor
   #   --gamepad                   # Enables Xbox controller manual teleop
   ```
4. **Trigger Start/Stop** (from the trackside laptop over Wi-Fi):
   ```bash
   python3 pi/covapsy_ctl.py start --host <pi-ip>
   python3 pi/covapsy_ctl.py stop  --host <pi-ip>
   ```

### 2. Manual Xbox Controller Mode (`--gamepad`)

Connect an Xbox Series X/One controller via Bluetooth or USB-C:
```bash
python3 pi/race.py --gamepad --manual-speed-cap 0.8
```

| Control | Action |
|:---|:---|
| **Left Stick X** | Proportional Ackermann steering (±18°) |
| **Right Trigger (RT)** | Proportional forward throttle (0 to `manual-speed-cap`) |
| **Left Trigger (LT)** | Proportional reverse throttle |
| **Bumper RB** | Arm / Disarm motor throttle (toggle safety switch) |
| **D-Pad Left / Right** | Live **Steering Trim** in ±0.25° increments (compensates mechanical drift) |
| **Button X** | Switch into `MANUAL` driving mode |
| **Button A** | Switch into autonomous `RUNNING` mode |
| **Button B** | Immediate emergency stop (`STOPPED` / neutral) |

> [!TIP]
> While driving forward down a corridor, if the car drifts slightly left, tap **D-Pad Right** until it tracks dead straight. The console will report the calibrated offset (e.g. `steer trim: -0.75 deg`), which you can save in `common/config.py` under `steer_trim_deg`.

### 2. Running the Simulation

Launch Webots:
```bash
./run_sim.sh
```
- Click the 3D viewport and press **`A`** to enable autonomous driving.
- Press **`N`** to stop and neutralize the car.
- Lap times and sector splits will print in real-time in the Webots console.

### 3. Automated Headless Parameter Tuning

Run Latin Hypercube Sampling trials in headless mode:
```bash
python3 tune_headless.py --trials 20 --seed 42
```
Results and the live top-10 leaderboard will stream to stdout and update `tuning_results.csv`.

### 4. Running Unit Tests

Run the full logic verification test suite (requires no hardware or simulator):
```bash
pytest tests/test_gap_follower.py -v
```

---

## Hardware & Sensor Status

| Hardware Component | Interface | Role | Status |
|:---|:---|:---|:---:|
| **RPLIDAR A2M12** | USB (`/dev/ttyUSB0`) | 360° 2D laser scan for obstacle avoidance and FTG | **Operational** |
| **STM32G431KB** | USB-CDC (`/dev/ttyACM0`) | 50 Hz PWM actuator drive + 250 ms safety watchdog | **Operational** |
| **Sharp GP2Y0A21YK0F** | ADC (`PA1`) | Rear obstacle detection for reverse gating | **Operational** |
| **BNO055 9-DOF IMU** | I2C (`PB7/PA15`) | Heading telemetry on OLED and race logging | **Operational** |
| **0.96" SSD1306 OLED** | I2C (`PB7/PA15`) | Car IP, battery voltage, and state display | **Operational** |
| **RealSense D435i** | UVC (`/dev/video4`) | Track border wrong-way detection (advisory) | **Operational** |
| **Webots GPS Node** | Simulation Driver | Real-time $(X,Y)$ tracking and start/finish lap timing | **Operational** |
