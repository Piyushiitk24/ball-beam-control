# Ball & Beam Control

Stepper-driven ball-beam balancer: firmware (C++/PlatformIO) + first-principles gain design (Python) + telemetry analysis.

---

## Quick Start — Exact Commands

This is the simplest full workflow:

1. Build the firmware.
2. Upload the firmware.
3. Start the serial logger.
4. Calibrate the system.
5. Run closed-loop control.
6. Plot the saved run.

If you want a single copy-paste guide, use the steps below exactly.

### 1. Build and Upload

Run these in a normal terminal:

```bash
cd /Users/piyush/code/ball-beam-control

# Restore the archived HC-SR04 runtime as the active firmware path
./.venv/bin/python analysis/switch_firmware_main.py --mode hcsr04_runtime

# Build
pio run -e nano_new

# Upload
pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1
```

### 2. Start the Serial Logger

Run this in the terminal after upload:

```bash
cd /Users/piyush/code/ball-beam-control
./.venv/bin/python analysis/serial_logger.py --port /dev/cu.usbserial-A10N20X1
```

After this, type the device commands below into the logger terminal and press Enter after each one.

### 3. Fresh Calibration and First Retest

Recommended workflow from a fresh power-up or after reflashing:

```text
/bringup
s
q c
e 1
r
q f
q c
s
/quit
```

Timing for the live setpoint sequence:
- hold the first `q c` segment for about `8 s`
- then `q f` for about `8 s`
- then the final `q c` for about `15 s`

Important:
- `/bringup` already performs the full calibration flow and sends `v` to save calibration for you
- if you do calibration manually instead of `/bringup`, you must still send `v` yourself after `b`
- `/quit` sends `k`, then `e 0`, exits the logger, and triggers automatic plot generation for that run folder

Manual calibration equivalent to `/bringup`:

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

### 4. What You Should Physically Do During Calibration and Test

If you use `/bringup`, the logger will prompt you for the required physical actions. The equivalent manual actions are:

- Before `s`
  - Do nothing special.
  - This only prints status.

- Before `d`
  - Do nothing special.
  - This clears old calibration.

- Before `l`
  - Move the beam fully down by hand.
  - Keep the ball at the far end, away from the HC-SR04.
  - Hold it steady.

- Before `u`
  - Move the beam fully up by hand.
  - Keep the ball at the near end, close to the HC-SR04.
  - Hold it steady.

- Before `p`
  - Place the ball at the physical center of the runner.
  - Hold it steady.

- Before `e 1`
  - Keep your hands clear of the mechanism.
  - This enables the driver.

- Before `b`
  - Keep fully clear.
  - The motor will jog by itself.

- Before `v`
  - Do nothing special.
  - This saves calibration.

- Before the second `s`
  - Do nothing special.
  - Check that calibration is complete.

- Before `t`
  - Do nothing special.
  - This turns telemetry on.

- Before `q c`
  - Do nothing special.
  - This sets the target to runner center.

- Before `r`
  - Keep your hands off the beam and ball.
  - The controller will start running.

- Before `q f`
  - Do nothing special.
  - This changes the target to the far endpoint while running.

- Before the final `q c`
  - Do nothing special.
  - This changes the target back to center.

- Before `k`
  - Do nothing special.
  - This stops the run.

- Before `e 0`
  - Do nothing special.
  - This disables the driver.

- Before `/quit`
  - Do nothing special.
  - This stops the run cleanly, disables the driver, exits the logger, and writes the summary plot.

### 5. What the Main Commands Mean

- `s`
  - Show status.

- `d`
  - Reset calibration.

- `l`
  - Capture lower beam limit and far ball endpoint.

- `u`
  - Capture upper beam limit and near ball endpoint.

- `p`
  - Capture ball center.

- `e 1`
  - Enable motor driver.

- `b`
  - Jog motor and check direction/sign.

- `v`
  - Save calibration.

- `t`
  - Turn telemetry on or off.

- `q c`
  - Set target to center.

- `q f`
  - Set target to far endpoint.

- `r`
  - Start closed-loop control.

- `k`
  - Stop closed-loop control.

- `e 0`
  - Disable motor driver.

### 6. What You Want to See After Calibration

After `/bringup` completes, or after the manual final `s`, you want these:

- `CAL,zero_calibrated=yes`
- `CAL,limits_calibrated=yes`
- `CAL,sign_calibrated=yes`
- `ACT_CFG,...,v=1`

If any of these are missing, redo calibration before running.

### 7. Plot the Results After the Run

If you want to plot a specific run, pass its run folder:

```bash
cd /Users/piyush/code/ball-beam-control
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/mpl ./.venv/bin/python analysis/plot_run.py --input data/runs/run_20260312_101232
```

Then open the PNG:

```bash
open data/runs/run_20260312_101232/plots/run_20260312_101232_summary.png
```

If you want to plot the most recent run automatically:

```bash
cd /Users/piyush/code/ball-beam-control
LATEST_RUN="$(find data/runs -maxdepth 1 -type d -name 'run_*' | sort | tail -n 1)"
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/mpl ./.venv/bin/python analysis/plot_run.py --input "$LATEST_RUN"
RUN_STEM="$(basename "$LATEST_RUN")"
open "$LATEST_RUN/plots/${RUN_STEM}_summary.png"
```

If you want the curated recent-runs report used for debugging history and thesis/report work:

```bash
cd /Users/piyush/code/ball-beam-control
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/mpl ./.venv/bin/python analysis/report_recent_runs.py
```

That writes:
- `docs/experiments/2026-03-control-debugging/generated/recent_runs_summary.csv`
- `docs/experiments/2026-03-control-debugging/generated/recent_runs_summary.md`
- `docs/experiments/2026-03-control-debugging/generated/plots/`
- `docs/experiments/2026-03-control-debugging/generated/metrics/`

### 8. Morning Hardware Test Workflow

Use this exact workflow when you connect hardware and want one clean logged test run.

In Terminal A:

```bash
cd /Users/piyush/code/ball-beam-control
ls /dev/cu.usb*
./.venv/bin/python analysis/switch_firmware_main.py --mode hcsr04_runtime
pio run -e nano_new
pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1
```

In Terminal B:

```bash
cd /Users/piyush/code/ball-beam-control
./.venv/bin/python analysis/serial_logger.py --port /dev/cu.usbserial-A10N20X1
```

Then in the logger terminal:

```text
/bringup
s
q c
e 1
r
```

Wait about `8 s`, then:

```text
q f
```

Wait about `8 s`, then:

```text
q c
```

Wait about `15 s`, then finish with:

```text
s
/quit
```

After the logger exits, inspect the latest run:

```bash
cd /Users/piyush/code/ball-beam-control
LATEST_RUN="$(find data/runs -maxdepth 1 -type d -name 'run_*' | sort | tail -n 1)"
RUN_STEM="$(basename "$LATEST_RUN")"
find "$LATEST_RUN" -maxdepth 2 -type f | sort
open "$LATEST_RUN/plots/${RUN_STEM}_summary.png"
```

Keep these files for analysis:
- `run_<timestamp>_telemetry.csv`
- `run_<timestamp>_events.txt`
- `run_<timestamp>_raw.log`

### 9. Daily Use After Calibration Is Already Saved

If you already calibrated and nothing changed mechanically, you can skip `d l u p b v`.

Use:

```text
s
t
q c
e 1
r
q f
q c
k
e 0
```

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

### What `q`, `q c`, and `q f` mean

- `q`
  - Query only.
  - It does not move the motor.
  - It prints the current target plus the two stored presets.
- `q c`
  - Target is runner center.
- `q f`
  - Target is the calibrated far endpoint.
- `q <cm>`
  - Target is an explicit offset in cm relative to center.
  - Example: `q -2.0`

Important:

- If the system is idle, `q ...` only changes the target. Motion starts after `r`.
- If the system is already `RUNNING`, `q ...` changes target live. Do not send another `r`.
- `q n` still exists in the restored HC-SR04 runtime for compatibility, but near-side validation is paused and it is not part of the active workflow.

### Workflow 1 — Fresh Calibration From Scratch

Use this after a fresh flash, a mechanical change, or when `s` shows any calibration flag as `no`.

#### Physical setup before each capture

- Before `l`: hold the beam fully DOWN, with the ball at the far end away from the HC-SR04.
- Before `u`: hold the beam fully UP, with the ball at the near end close to the HC-SR04.
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

Each logger session now writes to its own run folder:

```text
data/runs/run_<timestamp>/
  run_<timestamp>_raw.log
  run_<timestamp>_events.txt
  run_<timestamp>_telemetry.csv
  run_<timestamp>_metrics.csv
  plots/run_<timestamp>_summary.png
```

Telemetry line format (10 Hz whenever telemetry is enabled):
```
TEL,<t_ms>,<state>,<x_cm>,<x_filt_cm>,<theta_deg>,<theta_cmd_deg>,<u_step_rate>,<fault_flags>,<x_ref_cm>,<sonar_age_ms>,<sonar_valid>,<sonar_miss_count>,<theta_cmd_unclamped_deg>,<theta_cmd_clamped_deg>,<theta_cmd_saturated>,<act_deg_abs>,<trim_deg>,<search_phase>,<x_linear_cm>,<x_linear_filt_cm>,<x_ctrl_cm>,<x_ctrl_filt_cm>,<x_feedback_cm>,<feedback_blend>
```

```bash
cd /Users/piyush/code/ball-beam-control

# Plot latest run
LATEST_RUN="$(find data/runs -maxdepth 1 -type d -name 'run_*' | sort | tail -n 1)"
./.venv/bin/python analysis/plot_run.py --input "$LATEST_RUN"

# Compare two runs
./.venv/bin/python analysis/compare_runs.py \
    --inputs data/runs/<run_a> data/runs/<run_b>
```

`analysis/plot_run.py` accepts either a run folder or a telemetry CSV, auto-loads the run `*_events.txt` file when present, and writes the summary PNG into the run `plots/` folder plus metrics into the run folder.

### Standard Logged Runs

Run these from the local serial logger:

```text
/std center_reg
/std center_far
/std disturb
```

- `center_reg`: ball released off-center, recover to `q c`, total `12 s`
- `center_far`: `center -> far endpoint -> center`, total `31 s`
- `disturb`: hold `q c`, manual disturbance prompts at `5 s` and `10 s`, total `18 s`

Recommended operator usage:

| Test | What you do before starting | Local logger command | What happens |
|------|-----------------------------|----------------------|--------------|
| Center regulation | Place ball away from center, be ready to release | `/std center_reg` | Logger sets `q c`, starts run, logs `12 s` recovery |
| Center/far step tracking | Start with ball near center | `/std center_far` | Logger steps `q c -> q f -> q c` with `8 s -> 8 s -> 15 s` holds |
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
- HC-SR04 echo capture runs through the live PCINT path while Timer1 continues to generate step pulses

### State Machine

```
SAFE_DISABLED → CALIB_SIGN → READY ⇄ RUNNING → FAULT
```

### Control Frame

- HC-SR04 on `D8/D9` is the live ball-position sensor.
- Ball position is angle-corrected at runtime: the stored `sonar_*` offset from center is multiplied by `cos(theta_est)`.
- `p` captures the physical runner center and also defines the actuator control origin.
- AS5600 is used for calibration, run-start synchronization, and missed-step / drift verification.
- Telemetry and calibration keep `sonar_*` names for compatibility with the existing logger and reports.
- `theta_deg` and `theta_cmd_deg` in telemetry are step-count-relative actuator coordinates around the `p` origin.

---

## 8 — Position Sensor

### Current: HC-SR04 (Ultrasonic)

Wiring: D8 → TRIG, D9 → ECHO, 5 V → VCC, GND → GND.  
Sampling runs through the live HC-SR04 driver with median filtering, EMA smoothing, and hold-last-good validity handling.

Compatibility note:
- runtime state, telemetry, and calibration use `sonar_*` names because HC-SR04 is the active runtime path
- Sharp check firmware and characterization tooling remain in the repo as archived exploratory work

### Isolation Tests

```bash
# HC-SR04 only
./.venv/bin/python analysis/switch_firmware_main.py --mode hcsr04_check
# Archived HC-SR04 runtime
./.venv/bin/python analysis/switch_firmware_main.py --mode hcsr04_runtime
# Sharp IR only
./.venv/bin/python analysis/switch_firmware_main.py --mode sharp_ir_check
# AS5600 only
./.venv/bin/python analysis/switch_firmware_main.py --mode as5600_check
# Restore full controller
./.venv/bin/python analysis/switch_firmware_main.py --mode ballbeam
```

> Archived HC-SR04 and TFMini code is in `firmware/experiments/backup/`.

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

## 10 — Position Sensor Filtering Tuning (HC-SR04)

Live HC-SR04 processing uses trigger/echo acquisition, median filtering, EMA smoothing, jump rejection, and age-based hold-last-good validity.

| Parameter | Default | Effect |
|-----------|---------|--------|
| `SONAR_TRIGGER_PERIOD_US` | 40000 | Ping interval |
| `SONAR_ECHO_TIMEOUT_US` | 25000 | Echo timeout guard |
| `SONAR_POS_SAMPLE_FRESH_MS` | 500 | Fresh-sample window |
| `SONAR_MEDIAN_WINDOW` | 11 | Median filter window |
| `SONAR_MIN_VALID_IN_WINDOW` | 3 | Minimum accepted samples per window |
| `SONAR_MAX_VALID_MM` | 650.0f | Maximum accepted distance |
| `SONAR_MAX_JUMP_CM` | 3.0f | Per-sample jump clamp |
| `SONAR_EMA_ALPHA` | 0.3f | EMA smoothing |
| `SONAR_MAX_CONSECUTIVE_MISSES` | 6 | Diagnostic miss counter only |

Override via `build_flags` in `firmware/platformio.ini`:
```ini
build_flags = -D SONAR_TRIGGER_PERIOD_US=40000UL -D SONAR_MAX_JUMP_CM=3.0f -D SONAR_EMA_ALPHA=0.3f
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
| `analysis/plot_run.py` | Standard 3-panel response plot + diagnostic mode + metrics export |
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
LATEST_RUN="$(find data/runs -maxdepth 1 -type d -name 'run_*' | sort | tail -n 1)"
./.venv/bin/python analysis/plot_run.py --input "$LATEST_RUN"
```

## Modeling and Analysis

- Canonical modeling reference: `docs/modeling.md`
- Plotting/tuning workflow: `docs/tuning.md`
- Debugging/report record: `docs/experiments/2026-03-control-debugging/README.md`
- Sensor/target selection record: `docs/experiments/2026-03-sensor-selection/README.md`
- Sensor characterization workflow:
  - Sharp point capture: `./.venv/bin/python analysis/capture_sharp_points.py --port /dev/cu.usbserial-A10N20X1 --timer1-stress 1`
  - HC-SR04 point capture: `./.venv/bin/python analysis/capture_hcsr04_points.py --port /dev/cu.usbserial-A10N20X1 --mode pcint --timer1-stress 1`
  - Block report: `./.venv/bin/python analysis/report_sensor_characterization.py --input data/runs/run_<timestamp_a> --input data/runs/run_<timestamp_b> --output-dir docs/experiments/2026-03-sensor-selection/generated/<phase_name>`

## Sensor Selection Workflow

This is archived exploratory workflow. The active controller path has reverted to the HC-SR04 runtime, and current closed-loop validation is center/far only.

1. Mark the candidate runner span at fixed points `p00 ... pNN`, about `2 cm` apart.
2. Mount one sensor only. Do not switch sensors within the same block.
3. Capture one full out-and-back characterization run with explicit labels:
   - outbound auto labels start at `out_p00`
   - type `/reverse` at the turnaround
   - return auto labels become `ret_pNN ... ret_p00`
4. Repeat the same block once more with the same sensor and target.
5. Mount the other sensor and repeat the same two blocks.
6. Generate a report from those run folders and choose the sensor with the longest contiguous reliable window, then the lower mismatch/stddev/invalid rate.
7. Keep the winning sensor mounted and repeat the same process for `table_tennis_40mm` vs `golf_ball`.

Recommended scored sensor settings:
- `Sharp`: `sharp_ir_check` with `--timer1-stress 1`
- `HC-SR04`: `hcsr04_check` in `--mode pcint` with `--timer1-stress 1`

Per-run characterization artifacts:
- `run_<timestamp>_raw.log`
- `run_<timestamp>_*_points.csv`
- `run_<timestamp>_*_point_samples.csv`
- `run_<timestamp>_characterization_meta.json`

## VS Code One-Click Tasks

Use the Tasks panel (`Terminal -> Run Task...`) with:
- `Firmware: Build (nano_new)`
- `Firmware: Upload (nano_new)`
- `Firmware: Upload (nano_old)`
- `Firmware: Upload (sharp_ir_check)`
- `Firmware: Upload (hcsr04_check)`
- `Firmware: Monitor (115200)`
- `Firmware: Serial Logger (CSV+Events)`
- `Firmware: Capture Sharp Points`
- `Firmware: Capture HC-SR04 Points`
- `Analysis: Report Sensor Characterization`
- `Firmware: Switch Main -> HC-SR04 Check`
- `Firmware: Switch Main -> Sharp IR Check`
- `Firmware: Switch Main -> AS5600 Check`
- `Firmware: Switch Main -> BallBeam`
- `Model: Design Gains`
- `Model: Export Gains`

Sources
Ding et al., Position control for ball and beam system based on active disturbance rejection control — uses step, square-wave, sinusoidal references, and disturbance tests: https://www.tandfonline.com/doi/full/10.1080/21642583.2019.1575297
Ghude thesis excerpt, The Ball and Beam Experiment — explicitly tests balancing at center and disturbance rejection by pushing the ball left/right: https://research.usq.edu.au/download/f72c533a3c86c6748f769e555ccf90868643913541e043b35dd38c99d94e9216/3488
