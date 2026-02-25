# Ball & Beam Control

Ball and beam firmware + first-principles modeling + run analysis.

## Project Root

`/Users/piyush/code/ball-beam-control`

PlatformIO extension note:
- Repo root now contains `platformio.ini` as a wrapper for `firmware/platformio.ini`, so opening the top-level folder in VS Code is supported.

## Quick Start (Offline)

1. Open the repo in VS Code.
2. Create and activate Python virtual environment using your custom command:

```bash
mkvenv
```

3. Install dependencies inside `.venv`:

```bash
pip install -r /Users/piyush/code/ball-beam-control/requirements.txt
```

4. Generate SI-consistent controller gains and exported firmware header:

```bash
python model/first_principles/design_cascade_pid.py
python model/first_principles/export_gains.py
```

5. Build firmware for both Nano upload profiles:

```bash
cd firmware
pio run -e nano_new
pio run -e nano_old
```

## Runtime Architecture (Hardening v2)

- Step pulses are generated in a Timer1 ISR with integer scheduling only.
- ISR stepping path avoids float math and `digitalWrite()`.
- HC-SR04 echo capture uses PCINT timestamp ISR (edge+time+flag only).
- Sonar filtering/mapping runs in the main thread.
- Control math is SI internal (`m`, `rad`) and telemetry stays human-readable (`cm`, `deg`).

## Firmware Notes

- Target MCU: Arduino Nano (ATmega328P)
- Driver: TMC2209 in STEP/DIR mode
- Sensor 1: AS5600 (I2C addr `0x36`)
- Sensor 2: HC-SR04 ultrasonic
- Current hardware note: HC-SR04 calibration steps use a flat target for stable echoes.
- Closed-loop mode is blocked until runtime zero, limits, and sign calibration are complete.

## Serial Commands (Primary UX: Quick Keys)

### Quick Keys

| Key | Action | Full command equivalent |
|---|---|---|
| `?` / `h` | help | `help` |
| `s` | status | `status` |
| `t` | toggle telemetry | `telemetry 0|1` |
| `e 0|1` | driver enable/disable | `en 0|1` |
| `j <steps> <rate>` | jog motor | `jog <signed_steps> <rate>` |
| `a` | capture angle zero | `cal_zero set angle` |
| `p` | capture position zero | `cal_zero set position` |
| `z` | show zero calibration | `cal_zero show` |
| `[` / `l` / `1` | capture physical DOWN limit | `cal_limits set down` |
| `]` / `u` / `2` | capture physical UP limit | `cal_limits set up` |
| `m` | show limits | `cal_limits show` |
| `b` | sign calibration begin | `cal_sign begin` |
| `g` | sign calibration save | `cal_sign save` |
| `v` | save calibration to EEPROM | `cal_save` |
| `o` | load calibration from EEPROM | `cal_load` |
| `d` | reset runtime defaults | `cal_reset defaults` |
| `i` | print bring-up menu | `guide` |
| `x` | print decoded fault help | `faults` |
| `sonar diag` | print sonar health stats | `sonar diag` |
| `as5600 diag` | print AS5600 health stats | `as5600 diag` |
| `sonar sign 1|-1` | manual sonar sign override | `sonar sign 1|-1` |
| `r` | start control | `run` |
| `k` | stop control | `stop` |
| `f` | clear fault | `fault_reset` |

### Guided Wizard Keys

- `w`: start guided calibration (`zero -> limits -> sign -> save`)
- `n`: run current wizard step and advance
- `c`: confirm final EEPROM save at wizard end
- `q`: abort wizard (restores telemetry state)

### Full Commands (Compatibility)

- `help`
- `status`
- `telemetry 0|1`
- `en 0|1`
- `jog <signed_steps> <rate>`
- `cal_zero set angle|position`
- `cal_zero show|angle|position`
- `cal_limits set down|up|lower|upper`
- `cal_limits show`
- `cal_sign begin|save`
- `sonar diag | sonar sign 1|-1`
- `as5600 diag`
- `cal_save`
- `cal_load`
- `cal_reset defaults`
- `run`
- `stop`
- `fault_reset`

## Calibration Workflow

Preferred workflow:

1. Start guided flow with `w`.
2. Follow printed `WIZ,...` instructions and press `n` at each physical step.
3. At final summary, press `c` to persist calibration.
4. Use `r` to run after status is healthy.

Manual flow (without wizard) stays available using quick keys:

```text
t
a
p
[
]
b
g        # optional if sonar near/far capture is valid
# OR use manual sonar sign:
# sonar sign 1
v
t
s
r
```

See `docs/calibration_signs.md`.

## What Changed (Important)

- You no longer need to edit `firmware/include/calibration.h` for normal bring-up.
- Zero, limits, and sign are now captured at runtime via serial commands.
- Calibration is persisted on-device with `cal_save` and restored at boot.
- `run` is blocked until runtime calibration is complete (`zero + limits + sign`).
- Use `telemetry 0` while entering commands if monitor spam is distracting.
- In bring-up states, sensor faults are warnings/blockers (not forced FAULT latch).
- In `RUNNING`, faults remain hard safety faults.

## Daily Workflow (Short)

1. Upload firmware from VS Code task: `Firmware: Upload (nano_new)` (or `nano_old`).
2. Open monitor task: `Firmware: Monitor (115200)`.
3. Preferred bring-up run:
```text
w
```
Then follow `WIZ,...` prompts and press `n` each step, finally `c`, then:
```text
s
r
```
4. Manual equivalent (no wizard):
```text
telemetry 0
cal_zero set angle
cal_zero set position
cal_limits set down
cal_limits set up
cal_sign begin
cal_sign save            # optional if sonar near/far is valid
# or manual sonar sign:
# sonar sign 1
cal_save
telemetry 1
status
run
```
5. On next power-up, run `cal_load` only if needed; normally saved calibration auto-loads.

## First Hardware Bring-Up Checklist

Run this sequence exactly for first power-up after assembly.

1. From repo root, generate gains from measured parameters:

```bash
cd /Users/piyush/code/ball-beam-control
./.venv/bin/python model/first_principles/design_cascade_pid.py --params model/first_principles/params_measured_v1.yaml
./.venv/bin/python model/first_principles/export_gains.py
```

2. Build and upload firmware:

```bash
cd firmware
pio run -e nano_new -t upload
```

If upload fails due to bootloader profile mismatch:

```bash
pio run -e nano_old -t upload
```

If upload auto-detects Bluetooth instead of the Nano, force the USB port:

```bash
pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1
```

Important:
- Close monitor before upload (`Ctrl+C`), or upload can fail.
- After reboot, run `h`. New firmware must show `HELP,keys` and `sonar diag`.
- If you still see old `STEP,1,cmd=t...` menu, old firmware is still running.

3. Find serial port:

```bash
pio device list
```

4. Open serial monitor:

```bash
pio device monitor -b 115200 --echo --filter send_on_enter --eol LF
```

5. In monitor, run:

```text
help
status
```

6. Start guided calibration (recommended):

```text
w
```

Then follow the printed `WIZ,...` instructions and press `n` for each step:

```text
n
n
n
n
n
n
```

7. At wizard step 7, persist runtime calibration:

```text
c
```

8. Sanity check:

```text
s
e 1
j 120 500
j -120 500
e 0
```

9. Start closed-loop control:

```text
r
```

Stop when needed:

```text
k
```

10. Manual compatibility flow (if you do not want wizard):

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
python analysis/capture_serial.py --port <SERIAL_PORT> --seconds 30
python analysis/parse_log.py --input data/runs/<run>_raw.log
python analysis/plot_run.py --input data/runs/<run>_clean.csv
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
- `Model: Design Gains`
- `Model: Export Gains`
