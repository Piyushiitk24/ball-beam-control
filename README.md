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
- Closed-loop mode is blocked until sign calibration is complete.

## Serial Commands

- `help`
- `status`
- `en 0|1`
- `jog <signed_steps> <rate>`
- `cal_sign begin`
- `cal_sign save`
- `cal_zero angle`
- `cal_zero position`
- `run`
- `stop`
- `fault_reset`

## Calibration Workflow

Use `cal_sign begin` then `cal_sign save` before entering `run`. See `docs/calibration_signs.md`.

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
pio device monitor -b 115200
```

5. In monitor, run:

```text
help
status
```

6. Run sign calibration:

```text
cal_sign begin
```

Then move ball manually to far `+x` and run:

```text
cal_sign save
```

7. Apply sign suggestions in `firmware/include/calibration.h`:
- `AS5600_SIGN`
- `STEPPER_DIR_SIGN`
- `SONAR_POS_SIGN`

Then rebuild/upload and re-open monitor.

8. Get zero suggestions:

```text
cal_zero angle
cal_zero position
```

9. Apply zero suggestions in `firmware/include/calibration.h`:
- `AS5600_ZERO_DEG`
- `SONAR_CENTER_CM`

Then rebuild/upload and re-open monitor.

10. Sanity check:

```text
status
en 1
jog 120 500
jog -120 500
en 0
```

11. Start closed-loop control:

```text
run
```

Stop when needed:

```text
stop
```

12. Capture and analyze a run:

```bash
cd /Users/piyush/code/ball-beam-control
python analysis/capture_serial.py --port <SERIAL_PORT> --seconds 30
python analysis/parse_log.py --input data/runs/<run>_raw.log
python analysis/plot_run.py --input data/runs/<run>_clean.csv
```

## Modeling and Analysis

- Modeling pipeline: `docs/modeling.md`
- Plotting/tuning workflow: `docs/tuning.md`
