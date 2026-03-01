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
| **Angle zero captured** | AS5600 reference stored | `a` |
| **Position zero captured** | Sonar center stored | `p` |
| **Lower limit captured** | Beam DOWN limit stored | `l` (or `[` or `1`) |
| **Upper limit captured** | Beam UP limit stored | `u` (or `]` or `2`) |
| **Signs calibrated** | AS5600/stepper/sonar signs derived | `b` then `g` |
| **Sensors OK** | AS5600 + sonar returning valid data | (automatic) |
| **No faults** | Fault flags clear | `f` to clear |

> In **stepper-count mode** (`y`): sign, limits, and angle-zero are bypassed.  
> Only `p` + valid sonar are required.

### Full Calibration Sequence (AS5600 Mode)

Do this on first power-up or after `d` (reset defaults). Steps are **order-sensitive**.

```
Step  Command   Action                                          You do
────  ────────  ──────────────────────────────────────────────  ──────────────────────────
 0    s         Status — confirm firmware version, sensor OK    Read output
 1    t         Toggle telemetry OFF (reduces noise)            —
 2    a         Capture angle zero                              Level the beam horizontally
 3    p         Capture position zero                           Place ball at beam center
 4    l         Capture lower angle limit                       Tilt beam fully DOWN by hand
 5    u         Capture upper angle limit                       Tilt beam fully UP by hand
 6    e 1       Enable stepper driver                           —
 7    b         Begin sign calibration                          Keep hands clear — motor jogs
                (auto-derives AS5600 sign + stepper dir sign)
 8    g         Save sign calibration                           Motor should be at far end
                (derives sonar sign from near/far distance)
                If `g` shows ERR, use: sonar sign 1  (or -1)
 9    v         Save ALL calibration to EEPROM                  —
10    s         Status — confirm all flags = yes                Read output
```

After `v`, calibration **persists across reboots**. You only need to redo this if hardware changes.

### Quick Re-Calibration (Already Saved in EEPROM)

On subsequent power-ups, calibration auto-loads. Just verify:

```
s         # all flags should show "yes"
e 1       # enable driver
r         # run!
```

If any flag shows "no", redo that specific step from the table above, then `v` to re-save.

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
| `BLOCK,sign` | Run `b` then `g` (sign calibration) |
| `BLOCK,zero` | Run `a` and/or `p` |
| `BLOCK,limits` | Run `l` and `u` |
| `BLOCK,sensors` | Check wiring; run `sonar diag` / `as5600 diag` |
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
| `y` | Toggle angle source | AS5600 ↔ stepper-count |
| `e 0\|1` | Driver disable/enable | |
| `j <steps> <rate>` | Jog motor | Positive = upward |
| `a` | Capture angle zero | Beam level |
| `p` | Capture position zero | Ball at center |
| `l` / `[` / `1` | Capture lower limit | Beam at max DOWN |
| `u` / `]` / `2` | Capture upper limit | Beam at max UP |
| `z` | Show zero calibration | |
| `m` | Show limits | |
| `b` | Sign cal — begin | Motor jogs, derives AS5600/stepper signs |
| `g` | Sign cal — save | Derives sonar sign from distance change |
| `sonar sign ±1` | Manual sonar sign | Bypass `g` if needed |
| `v` | Save cal to EEPROM | Persists across reboots |
| `o` | Load cal from EEPROM | Normally auto at boot |
| `d` | Reset to defaults | Clears all calibration (RAM only until `v`) |
| `r` | Run (start control) | Checks all preconditions |
| `k` | Stop | |
| `f` | Clear faults | |
| `x` | Print fault details | Decoded fault flags + sonar diag |
| `sonar diag` | Sonar health | timeout_cnt, valid_streak, freshness |
| `as5600 diag` | AS5600 health | status, AGC, raw angle, error count |

---

## 7 — Architecture Overview

### Control Loop

- **50 Hz** cascade PID in main thread
- **Outer loop**: position error (m) → angle command (rad)
- **Inner loop**: angle error (rad) → step rate (microsteps/s)
- Gains auto-generated in `firmware/include/generated/controller_gains.h` — **never edit by hand**

### ISR Design

- **Timer1 CTC** (40 kHz): step-pulse generation, integer-only, no `digitalWrite()`
- **PCINT** (HC-SR04 only): echo timestamp capture, flag + time only

### State Machine

```
SAFE_DISABLED → CALIB_SIGN → READY ⇄ CALIB_SCALE → RUNNING → FAULT
```

### Angle Source Modes

| Mode | Key | Angle from | Run requires |
|------|-----|------------|--------------|
| AS5600 (default) | — | I2C sensor | `a` + `p` + `l` + `u` + `b`/`g` + sensors OK |
| Stepper-count | `y` | Step counter | `p` + sensors OK |

---

## 8 — Sensor Variants

### Current: TFMini v1.7 (LiDAR)

Wiring: TFMini TX → Nano D10, TFMini RX → Nano D11, common GND.

Switch firmware:
```bash
./.venv/bin/python analysis/switch_firmware_main.py --mode tfmini
cd firmware && pio run -e nano_new -t upload
```

### Default: HC-SR04 (Ultrasonic)

Restore:
```bash
./.venv/bin/python analysis/switch_firmware_main.py --mode ballbeam
cd firmware && pio run -e nano_new -t upload
```

### Isolation Tests

```bash
# AS5600 only
./.venv/bin/python analysis/switch_firmware_main.py --mode as5600_check
# HC-SR04 only
./.venv/bin/python analysis/switch_firmware_main.py --mode hcsr04_check
# Restore
./.venv/bin/python analysis/switch_firmware_main.py --mode ballbeam
```

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
| `SONAR_EMA_ALPHA` | 0.6f | EMA smoothing (lower = smoother) |
| `SONAR_POS_SAMPLE_FRESH_MS` | 200 | Hold-last-good window |
| `SONAR_MAX_VALID_MM` | 650.0f | Distance clamp |

Override via `build_flags` in `firmware/platformio.ini`:
```ini
build_flags = -D SONAR_MEDIAN_WINDOW=7 -D SONAR_EMA_ALPHA=0.3f
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
| `ERR,run_blocked` with no obvious reason | You forgot sign calibration (`b` then `g`) |
| Editing `controller_gains.h` by hand | Always regenerate via `export_gains.py` |
| Float math in ISR | Never — use integer only |
| Upload hangs | Wrong bootloader profile, or monitor still open |
| Motor barely moves | Inner gains too low — re-run design pipeline |
| `FAULT` immediately on `r` | Sonar dropout — check `sonar diag`, increase `SONAR_POS_SAMPLE_FRESH_MS` |
| Wrong motor direction | Sign calibration is wrong — redo `b` + `g` |

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
g
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
- `Firmware: Switch Main -> AS5600 Check`
- `Firmware: Switch Main -> TFMini`
- `Firmware: Switch Main -> BallBeam`
- `Model: Design Gains`
- `Model: Export Gains`
