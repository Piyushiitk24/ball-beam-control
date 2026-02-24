# Ball & Beam Control

Ball and beam firmware + first-principles modeling + run analysis.

## Project Root

`/Users/piyush/code/ball-beam-control`

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
- Closed-loop mode is blocked until runtime zero, limits, and sign calibration are complete.

## Serial Commands

- `help`
- `status`
- `telemetry 0|1`
- `en 0|1`
- `jog <signed_steps> <rate>`
- `cal_zero set angle`
- `cal_zero set position`
- `cal_zero show`
- `cal_limits set lower`
- `cal_limits set upper`
- `cal_limits show`
- `cal_sign begin`
- `cal_sign save`
- `cal_save`
- `cal_load`
- `cal_reset defaults`
- `run`
- `stop`
- `fault_reset`

## Calibration Workflow

Use runtime flow `zero -> limits -> sign -> save` before entering `run`.
See `docs/calibration_signs.md`.

## What Changed (Important)

- You no longer need to edit `firmware/include/calibration.h` for normal bring-up.
- Zero, limits, and sign are now captured at runtime via serial commands.
- Calibration is persisted on-device with `cal_save` and restored at boot.
- `run` is blocked until runtime calibration is complete (`zero + limits + sign`).
- Use `telemetry 0` while entering commands if monitor spam is distracting.

## Daily Workflow (Short)

1. Upload firmware from VS Code task: `Firmware: Upload (nano_new)` (or `nano_old`).
2. Open monitor task: `Firmware: Monitor (115200)`.
3. Run:
```text
telemetry 0
cal_zero set angle
cal_zero set position
cal_limits set lower
cal_limits set upper
cal_sign begin
cal_sign save
cal_save
telemetry 1
status
run
```
4. On next power-up, run `cal_load` only if needed; normally saved calibration auto-loads.

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

6. Optionally pause telemetry spam while entering commands:

```text
telemetry 0
```

7. Capture runtime zeroes at neutral beam/ball setup:

```text
cal_zero set angle
cal_zero set position
cal_zero show
```

8. Capture runtime travel limits at mechanical extremes:

```text
cal_limits set lower
cal_limits set upper
cal_limits show
```

9. Run sign calibration:

```text
cal_sign begin
```

Then move ball manually to far `+x` and run:

```text
cal_sign save
```

10. Persist runtime calibration and optionally re-enable telemetry:

```text
cal_save
telemetry 1
```

11. Sanity check:

```text
status
en 1
jog 120 500
jog -120 500
en 0
```

12. Start closed-loop control:

```text
run
```

Stop when needed:

```text
stop
```

13. Capture and analyze a run:

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
