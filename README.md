# Ball & Beam Control

Stepper-driven ball-beam balancer: firmware (C++/PlatformIO) + first-principles gain design (Python) + telemetry analysis.

---

## 1 — Build & Upload (One-Time or After Code Changes)

```bash
cd /Users/piyush/code/ball-beam-control

# Generate gains from measured plant parameters
./.venv/bin/python model/first_principles/design_cascade_pid.py \
    --params model/first_principles/params_measured_v1.yaml
./.venv/bin/python model/first_principles/export_gains.py

# Build
cd firmware && pio run -e nano_new

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

Logger local commands: `/help`, `/diag`, `/bringup`, `/quit`.  
Raw fallback: `cd firmware && pio device monitor -b 115200 --echo --filter send_on_enter --eol LF`

---

## 3 — Calibration (Step-by-Step, No Ambiguity)

### Run Preconditions

`r` (run) is **blocked** unless ALL of these are true:

| Condition | What satisfies it | Command |
|-----------|-------------------|---------|
| **Actuator trim ready** | Seeded from `l/u` midpoint or learned and saved | `l` + `u` |
| **Position zero captured** | Sonar midpoint derived from `l/u`, or manually overridden | `l` + `u` (or optional `p`) |
| **Lower limit captured** | Beam DOWN limit stored | `l` (or `[` or `1`) |
| **Upper limit captured** | Beam UP limit stored | `u` (or `]` or `2`) |
| **Signs calibrated** | AS5600 + stepper signs derived; sonar direction fixed by `l/u` | `l`, `u`, then `b` |
| **Sensors OK** | AS5600 + sonar returning valid data | (automatic) |
| **No faults** | Fault flags clear | `f` to clear |

### Full Calibration Sequence (AS5600 Actuator Mode)

Do this on first power-up or after `d` (reset defaults). Steps are **order-sensitive**.

```
Step  Command   Action                                          You do
────  ────────  ──────────────────────────────────────────────  ──────────────────────────
 0    s         Status — confirm firmware version, sensor OK    Read output
 1    t         Toggle telemetry OFF (reduces noise)            —
 2    l         Capture lower actuator limit                    Tilt beam fully DOWN by hand
               (also captures the far sonar distance)
 3    u         Capture upper actuator limit                    Tilt beam fully UP by hand
               (also captures the near sonar distance and fixes sonar direction: upper = near)
 4    e 1       Enable stepper driver                           —
 5    b         Begin sign calibration                          Keep hands clear — motor jogs
                (auto-derives AS5600 sign + stepper dir sign)
 6    g         Optional sonar validation                       Only as a cross-check if you want it
 7    v         Save limits, sonar midpoint, and trim seed      —
 8    s         Status — confirm all flags = yes                Read output
               Check `SONAR_CFG,s=-1,m=o,u=1`
               Check `ACT_CFG` for trim/source info
```

`a` is no longer required. The firmware seeds actuator trim from the midpoint of the captured lower/upper travel and can learn a better trim during `RUNNING`.

After `v`, calibration **persists across reboots**. You only need to redo this if hardware changes.

### Quick Re-Calibration (Already Saved in EEPROM)

On subsequent power-ups, calibration auto-loads. Just verify:

```
s         # all flags should show "yes"
s         # and SONAR_CFG should show s=-1,m=o,u=1
e 1       # enable driver
r         # run!
```

If any flag shows "no", redo `l`, `u`, and `b`, then `v` to re-save. `p` is optional and only needed if you want to override the auto-derived sonar midpoint.

---

## 4 — Run

```
e 1       # enable stepper driver
r         # start closed-loop control
t         # turn on telemetry (if off)
```

**If `r` prints `ERR,run_blocked`**: the firmware also prints `BLOCK,<reason>` lines telling you exactly what's missing. Common fixes:

| BLOCK message | Fix |
|---------------|-----|
| `BLOCK,sign` | Run `b` (use `g` only if you want sonar validation) |
| `BLOCK,zero` | Run `l` + `u` to derive center/trim, or `p` for a manual sonar-center override |
| `BLOCK,limits` | Run `l` and `u` |
| `BLOCK,sensors` | Check wiring; use `x` for compact sonar/fault diagnostics |
| `BLOCK,faults` | Run `f` to clear; check `x` for details |

### Stop

```
k         # stop control loop
e 0       # disable driver (optional, saves power)
```

### Quit Logger

```
/quit     # sends k → e 0 → exits cleanly
```

---

## 5 — Telemetry & Analysis

Telemetry line format (10 Hz while RUNNING):
```
TEL,<t_ms>,<state>,<x_cm>,<x_filt_cm>,<theta_deg>,<theta_cmd_deg>,<u_step_rate>,<fault_flags>
```

```bash
cd /Users/piyush/code/ball-beam-control

# Plot latest run
LATEST="$(ls -t data/runs/run_*_telemetry.csv | head -n 1)"
./.venv/bin/python analysis/plot_run.py --input "$LATEST"

# Compare two runs
./.venv/bin/python analysis/compare_runs.py \
    --a data/runs/<run_a>_telemetry.csv \
    --b data/runs/<run_b>_telemetry.csv
```

---

## 6 — Serial Command Reference

### Quick Keys

| Key | Action | Notes |
|-----|--------|-------|
| `h` / `?` | Help | |
| `s` / `i` | Status | Shows state, calibration flags, sensor health |
| `t` | Toggle telemetry | Suppress during calibration |
| `e 0\|1` | Driver disable/enable | |
| `j <steps> <rate>` | Jog motor | Positive = upward |
| `p` | Capture sonar-center override | Optional manual ball-center override |
| `l` / `[` / `1` | Capture lower limit | Beam at max DOWN / ball far |
| `u` / `]` / `2` | Capture upper limit | Beam at max UP / ball near |
| `z` | Show zero calibration | |
| `m` | Show limits | |
| `b` | Sign cal — begin | Motor jogs, derives AS5600/stepper signs |
| `g` | Sign cal — validate | Optional near/far sonar cross-check |
| `v` | Save cal to EEPROM | Persists across reboots |
| `o` | Load cal from EEPROM | Normally auto at boot |
| `d` | Reset to defaults | Clears calibration and saves defaults immediately |
| `r` | Run (start control) | Checks all preconditions |
| `k` | Stop | |
| `f` | Clear faults | |
| `x` | Print fault details | Decoded fault flags + compact sonar diag |

---

## 7 — Architecture Overview

### Control Loop

- **50 Hz** cascade PID in main thread
- **Outer loop**: position error (m) → angle command (rad)
- **Inner loop**: angle error (rad) → step rate (microsteps/s)
- Gains auto-generated in `firmware/include/generated/controller_gains.h` — **never edit by hand**

### ISR Design

- **Timer1 CTC** (40 kHz): step-pulse generation, integer-only, no `digitalWrite()`
- **PCINT** (HC-SR04): echo timestamp capture, flag + time only

### State Machine

```
SAFE_DISABLED → CALIB_SIGN → READY ⇄ CALIB_SCALE → RUNNING → FAULT
```

### Control Frame

- AS5600 is used as an actuator-position sensor between the captured lower and upper travel limits.
- Ultrasonic center is defined from the midpoint of the captured near/far sonar distances, unless `p` is used as a manual override.
- `theta_deg` and `theta_cmd_deg` in telemetry are actuator-relative coordinates around the active trim, not physical balanced-beam angles.

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

## 9 — Gain Design Pipeline

Source of truth: `model/first_principles/params_measured_v1.yaml`

```bash
# 1. Edit params if hardware changed
# 2. Design gains
./.venv/bin/python model/first_principles/design_cascade_pid.py \
    --params model/first_principles/params_measured_v1.yaml
# 3. Export to firmware header
./.venv/bin/python model/first_principles/export_gains.py
# 4. Build + upload
cd firmware && pio run -e nano_new -t upload
```

---

## 10 — Sonar Filtering Tuning (HC-SR04)

Non-blocking PCINT echo-capture → rolling median → min-valid gating → EMA → hold-last-good.

| Parameter | Default | Effect |
|-----------|---------|--------|
| `SONAR_TRIGGER_PERIOD_US` | 40000 | Ping interval (~25 Hz) |
| `SONAR_ECHO_TIMEOUT_US` | 25000 | Max echo wait |
| `SONAR_MEDIAN_WINDOW` | 11 | Samples in rolling median |
| `SONAR_MIN_VALID_IN_WINDOW` | 1 | Min valid in window |
| `SONAR_EMA_ALPHA` | 0.3f | EMA smoothing (lower = smoother) |
| `SONAR_POS_SAMPLE_FRESH_MS` | 200 | Hold-last-good window |
| `SONAR_MAX_VALID_MM` | 650.0f | Distance clamp |
| `SONAR_MAX_JUMP_CM` | 3.0f | Per-sample jump clamp |

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
| `firmware/include/generated/controller_gains.h` | **Auto-generated** PID gains |
| `firmware/include/app/state_machine.h` | State machine |
| `firmware/src/calibration_runtime.cpp` | Runtime cal + EEPROM persistence |
| `model/first_principles/params_measured_v1.yaml` | Plant parameters (source of truth) |
| `model/first_principles/design_cascade_pid.py` | Gain design script |
| `model/first_principles/export_gains.py` | Gains → firmware header |
| `analysis/serial_logger.py` | Primary host logger |
| `analysis/switch_firmware_main.py` | Firmware variant switcher |
| `analysis/plot_run.py` | 4-panel telemetry plot |
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
| `ERR,run_blocked` with no obvious reason | You forgot sign calibration (`l`, `u`, then `b`) |
| Editing `controller_gains.h` by hand | Always regenerate via `export_gains.py` |
| Float math in ISR | Never — use integer only |
| Upload hangs | Wrong bootloader profile, or monitor still open |
| Motor barely moves | Inner gains too low — re-run design pipeline |
| `FAULT` immediately on `r` | Sonar dropout or hard limit overshoot — check `x` and the saved limits |
| Wrong motor direction | Sign calibration is wrong — redo `l`, `u`, then `b` |

Stop when needed:

```text
k
```

10. Manual compatibility flow (raw serial / device-only):

```text
t
a
p
[
]
b
g         # optional sonar validation
# or sonar sign 1|-1
v
t
s
r
```

11. Capture and analyze a run:

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
