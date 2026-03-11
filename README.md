# Ball & Beam Control

Stepper-driven ball-beam balancer: firmware (C++/PlatformIO) + first-principles gain design (Python) + telemetry analysis.

---

## 1 — Build & Upload (One-Time or After Code Changes)

```bash
cd /Users/piyush/code/ball-beam-control

# Build
pio run -e nano_new

# Upload (close any monitor first!)
pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1
```

> If upload hangs, try `nano_old` (57600 baud bootloader).
> If auto-detect picks Bluetooth, add `--upload-port /dev/cu.usbserial-*`.

---

## 2 — Connect

```bash
cd /Users/piyush/code/ball-beam-control
./.venv/bin/python analysis/serial_logger.py --port /dev/cu.usbserial-A10N20X1
```

Logger local commands: `/help`, `/diag`, `/bringup`, `/std ...`, `/quit`.
Raw fallback: `cd firmware && pio device monitor -b 115200 --echo --filter send_on_enter --eol LF`

---

## 3 — Complete Workflow

Use this mental model:

1. Capture runner endpoints with `l` and `u`.
2. Capture the physical runner center with `p`.
3. Jog sign calibration with `b`.
4. Save with `v`.
5. Choose a target with `q ...`.
6. Start control with `r`.
7. Change target live with more `q ...` commands.

### What `q`, `q c`, `q n`, and `q f` mean

- `q`
  - Query only.
  - It does not move the motor.
  - It prints the current target plus the two stored presets.
- `q c`
  - Target is runner center.
- `q n`
  - Target is the midpoint between runner center and the near-sensor end.
- `q f`
  - Target is the midpoint between runner center and the far end.
- `q <cm>`
  - Target is an explicit offset in cm relative to center.
  - Example: `q -2.0`

Important:

- If the system is idle, `q ...` only changes the target. Motion starts after `r`.
- If the system is already `RUNNING`, `q ...` changes target live. Do not send another `r`.

### Workflow 1 — Fresh Calibration From Scratch

Use this after a fresh flash, a mechanical change, or when `s` shows any calibration flag as `no`.

#### Physical setup before each capture

- Before `l`: hold the beam fully DOWN, with the ball at the far end away from the ultrasonic sensor.
- Before `u`: hold the beam fully UP, with the ball at the near end close to the ultrasonic sensor.
- Before `p`: place the ball at the physical center of the runner and hold it steady there. The firmware records runner center and actuator control origin here.
- Before `b`: remove your hands and keep clear of the mechanism.

#### Exact command sequence

```text
s
d
l
u
p
e 1
b
v
s
```

What you want after the final `s`:

- `CAL,zero_calibrated=yes`
- `CAL,limits_calibrated=yes`
- `CAL,sign_calibrated=yes`
- `ACT_CFG,...,v=1`

### Workflow 2 — First Closed-Loop Run After Calibration

Use this immediately after the fresh calibration above:

```text
q c
e 1
r
```

What happens:

- `q c` sets the desired ball target to the physical runner center.
- `r` now starts `RUNNING` directly.
- There is no center-search phase anymore.

What you do physically:

- Keep hands off the beam and ball.
- Let the controller move the ball toward the selected target.

### Workflow 3 — Normal Daily Use After Calibration Is Saved

Center run:

```text
s
q c
e 1
r
k
e 0
```

Near-side run:

```text
s
q n
e 1
r
k
e 0
```

Far-side run:

```text
s
q f
e 1
r
k
e 0
```

Custom target:

```text
s
q -2.0
e 1
r
k
e 0
```

### Change Target During a Run

If the system is already running, send target changes directly:

```text
q n
q f
q c
q -2.0
```

Do not send `k`, `e 0`, or another `r` unless you actually want to stop.

### If `r` Does Not Start

If `r` prints `ERR,run_blocked`, read the `BLOCK,...` lines.

- `BLOCK,zero`
  - runner center has not been captured
  - run `p`
- `BLOCK,limits`
  - lower/upper endpoints are missing
  - run `l`, then `u`
- `BLOCK,sign`
  - sign jog has not been done
  - run `b`
- `BLOCK,sensors`
  - sonar or AS5600 is not currently valid
  - run `s` and wait for `SENSORS,angle=ok,pos=ok`
- `BLOCK,faults`
  - run `f`, then `s`

### Stop And Quit

Stop control:

```text
k
e 0
```

Quit the logger:

```text
/quit
```

---

## 5 — Telemetry & Analysis

Telemetry line format (10 Hz whenever telemetry is enabled):
```
TEL,<t_ms>,<state>,<x_cm>,<x_filt_cm>,<theta_deg>,<theta_cmd_deg>,<u_step_rate>,<fault_flags>,<x_ref_cm>,<sonar_age_ms>,<sonar_valid>,<sonar_miss_count>,<theta_cmd_unclamped_deg>,<theta_cmd_clamped_deg>,<theta_cmd_saturated>,<act_deg_abs>,<trim_deg>,<search_phase>
```

```bash
cd /Users/piyush/code/ball-beam-control

# Plot latest run
LATEST="$(ls -t data/runs/run_*_telemetry.csv | head -n 1)"
./.venv/bin/python analysis/plot_run.py --input "$LATEST"

# Compare two runs
./.venv/bin/python analysis/compare_runs.py \
    --inputs data/runs/<run_a>_telemetry.csv data/runs/<run_b>_telemetry.csv
```

`analysis/plot_run.py` auto-loads the sibling `*_events.txt` file when present, overlays setpoint/disturbance markers, and writes a sibling `*_metrics.csv`.

### Standard Logged Runs

Run these from the local serial logger:

```text
/std center_reg
/std step3
/std disturb
```

- `center_reg`: ball released off-center, recover to `q c`, total `12 s`
- `step3`: `center -> near_mid -> center -> far_mid -> center`, total `36 s`
- `disturb`: hold `q c`, manual disturbance prompts at `5 s` and `10 s`, total `18 s`

Recommended operator usage:

| Test | What you do before starting | Local logger command | What happens |
|------|-----------------------------|----------------------|--------------|
| Center regulation | Place ball away from center, be ready to release | `/std center_reg` | Logger sets `q c`, starts run, logs `12 s` recovery |
| 3-position step tracking | Start with ball near center | `/std step3` | Logger steps `q c -> q n -> q c -> q f -> q c` |
| Disturbance rejection | Start centered, keep hand ready to nudge | `/std disturb` | Logger prompts you when to disturb |

---

## 6 — Serial Command Reference

### Quick Keys

| Key | Action | Notes |
|-----|--------|-------|
| `h` / `?` | Help | |
| `s` / `i` | Status | Shows state, calibration flags, sensor health |
| `t` | Toggle telemetry | Suppress during calibration |
| `e 0\|1` | Driver disable/enable | |
| `p` | Capture physical runner center | Required center reference for `r` |
| `q [c\|n\|f\|<cm>]` | Set/query ball-position target | `q` alone prints current target + presets |
| `l` / `[` / `1` | Capture lower limit | Beam at max DOWN / ball far |
| `u` / `]` / `2` | Capture upper limit | Beam at max UP / ball near |
| `b` | Sign cal — begin | Motor jogs, derives AS5600/stepper signs |
| `v` | Save cal to EEPROM | Persists across reboots |
| `d` | Reset to defaults | Clears calibration and saves defaults immediately |
| `r` | Run | Starts closed-loop control immediately |
| `k` | Stop | |
| `f` | Clear faults | |

---

## 7 — Architecture Overview

### Control Loop

- **50 Hz** single PID in the main thread
- Input: angle-corrected ball-position error
- Output: absolute actuator target in relative step counts
- Motion generation converts target-position error into bounded step rate and acceleration

### ISR Design

- **Timer1 CTC** (40 kHz): step-pulse generation, integer-only, no `digitalWrite()`
- **PCINT** (HC-SR04): echo timestamp capture, flag + time only

### State Machine

```
SAFE_DISABLED → CALIB_SIGN → READY ⇄ RUNNING → FAULT
```

### Control Frame

- HC-SR04 remains the ball-position sensor.
- Ball position is angle-corrected at runtime: the sonar offset from center is multiplied by `cos(theta_est)`.
- `p` captures the physical runner center and also defines the actuator control origin.
- AS5600 is used for calibration, run-start synchronization, and missed-step / drift verification.
- `theta_deg` and `theta_cmd_deg` in telemetry are step-count-relative actuator coordinates around the `p` origin.

---

## 8 — Position Sensor

### Current: HC-SR04 (Ultrasonic)

Wiring: D8 → TRIG, D9 → ECHO, 5 V → VCC, GND → GND.  
PCINT edge-capture ISR for non-blocking echo measurement. ~25 Hz trigger rate.

Valid range: 2–65 cm. Conversion: `raw_cm = pulse_us × 0.0343 × 0.5`.

### Isolation Tests

```bash
# HC-SR04 only
./.venv/bin/python analysis/switch_firmware_main.py --mode hcsr04_check
# AS5600 only
./.venv/bin/python analysis/switch_firmware_main.py --mode as5600_check
# Restore full controller
./.venv/bin/python analysis/switch_firmware_main.py --mode ballbeam
```

> Legacy Sharp IR and TFMini code is in `firmware/experiments/backup/`.

---

## 9 — Runtime Tuning

The default runtime controller is now the single position PID in `firmware/include/config.h`.

Main tuning constants:

| Constant | Meaning |
|----------|---------|
| `kPosPidKpStepsPerCm` | Position proportional gain |
| `kPosPidKiStepsPerCmSec` | Position integral gain |
| `kPosPidKdStepsSecPerCm` | Position derivative gain |
| `kRunMotionAccelSps2` | Motion-generator acceleration limit |
| `kAs5600VerifyMaxErrDeg` | Allowed step-count vs AS5600 mismatch before fault |

After changing these, rebuild and upload:

```bash
pio run -e nano_new -t upload
```

---

## 10 — Sonar Filtering Tuning (HC-SR04)

Non-blocking PCINT echo-capture → rolling median → min-valid gating → EMA → hold-last-good across intermittent misses.

| Parameter | Default | Effect |
|-----------|---------|--------|
| `SONAR_TRIGGER_PERIOD_US` | 40000 | Ping interval (~25 Hz) |
| `SONAR_ECHO_TIMEOUT_US` | 25000 | Max echo wait |
| `SONAR_MEDIAN_WINDOW` | 11 | Samples in rolling median |
| `SONAR_MIN_VALID_IN_WINDOW` | 3 | Min valid in window |
| `SONAR_EMA_ALPHA` | 0.3f | EMA smoothing (lower = smoother) |
| `SONAR_POS_SAMPLE_FRESH_MS` | 500 | Fresh-sample window |
| `SONAR_MAX_VALID_MM` | 650.0f | Distance clamp |
| `SONAR_MAX_JUMP_CM` | 3.0f | Per-sample jump clamp |
| `SONAR_MAX_CONSECUTIVE_MISSES` | 6 | Miss burst tolerance before invalid |

Override via `build_flags` in `firmware/platformio.ini`:
```ini
build_flags = -D SONAR_MEDIAN_WINDOW=7 -D SONAR_EMA_ALPHA=0.3f -D SONAR_MAX_JUMP_CM=2.0f
```

---

## 11 — AS5600 Filtering Tuning

Median-of-3 raw reads → slew-limit → EMA.

| Parameter | Default | Effect |
|-----------|---------|--------|
| `AS5600_EMA_ALPHA` | 0.3f | EMA smoothing |
| `AS5600_MAX_JUMP_DEG` | 25.0f | Max per-sample delta |

---

## 12 — Key Files

| File | Purpose |
|------|---------|
| `firmware/src/main.cpp` | Main firmware (switchable) |
| `firmware/include/config.h` | Loop rates, limits, unit constants |
| `firmware/include/types.h` | `SensorData`, `Setpoint`, `ActuatorCmd`, `FaultFlags` |
| `firmware/include/app/state_machine.h` | State machine |
| `firmware/src/calibration_runtime.cpp` | Runtime cal + EEPROM persistence |
| `firmware/src/control/cascade_controller.cpp` | Single position PID + motion generator |
| `analysis/serial_logger.py` | Primary host logger |
| `analysis/switch_firmware_main.py` | Firmware variant switcher |
| `analysis/plot_run.py` | 4-panel telemetry plot + metrics export |
| `analysis/compare_runs.py` | Side-by-side run comparison |
| `docs/calibration_signs.md` | Sign convention reference |

---

## 13 — Flash Budget

ATmega328P: 30,720 bytes flash, 2,048 bytes RAM.

When adding code:
- `F()` macro for all `Serial.print` string literals
- Reuse existing buffers
- No `std::` containers or new virtual functions
- Check: `pio run -e nano_new` reports usage

---

## 14 — Common Pitfalls

| Pitfall | Fix |
|---------|-----|
| `ERR,run_blocked` with `BLOCK,zero` | You did not capture physical runner center with `p` |
| `ERR,run_blocked` with `BLOCK,limits` | Redo `l`, then `u` |
| `ERR,run_blocked` with `BLOCK,sign` | Redo `b` |
| Ball goes to the wrong side and stays there | Redo `d`, `l`, `u`, `p`, `b`, then verify `q` presets |
| Float math in ISR | Never — use integer only |
| Upload hangs | Wrong bootloader profile, or monitor still open |
| Motor barely moves | Check telemetry for `theta_cmd_saturated`, `actuator_drift`, and sonar age before retuning |
| `FAULT` immediately on `r` | Usually sonar dropout, actuator drift, or a hard limit violation — inspect `s` and the saved limits |
| Wrong motor direction | Sign calibration is wrong — redo `l`, `u`, then `b` |

Stop when needed:

```text
k
```

Minimal raw serial flow:

```text
t
d
l
u
p
e 1
b
v
s
q c
e 1
r
k
e 0
```

Capture and analyze a run:

```bash
cd /Users/piyush/code/ball-beam-control
LATEST="$(ls -t data/runs/run_*_telemetry.csv | head -n 1)"
./.venv/bin/python analysis/plot_run.py --input "$LATEST"
```

## Modeling and Analysis

- Canonical modeling reference: `docs/modeling.md`
- Plotting/tuning workflow: `docs/tuning.md`

## VS Code One-Click Tasks

Use the Tasks panel (`Terminal -> Run Task...`) with:
- `Firmware: Build (nano_new)`
- `Firmware: Upload (nano_new)`
- `Firmware: Upload (nano_old)`
- `Firmware: Monitor (115200)`
- `Firmware: Serial Logger (CSV+Events)`
- `Firmware: Switch Main -> HC-SR04 Check`
- `Firmware: Switch Main -> AS5600 Check`
- `Firmware: Switch Main -> BallBeam`
- `Model: Design Gains`
- `Model: Export Gains`

Sources
Ding et al., Position control for ball and beam system based on active disturbance rejection control — uses step, square-wave, sinusoidal references, and disturbance tests: https://www.tandfonline.com/doi/full/10.1080/21642583.2019.1575297
Ghude thesis excerpt, The Ball and Beam Experiment — explicitly tests balancing at center and disturbance rejection by pushing the ball left/right: https://research.usq.edu.au/download/f72c533a3c86c6748f769e555ccf90868643913541e043b35dd38c99d94e9216/3488
