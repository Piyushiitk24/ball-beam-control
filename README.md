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

4. Generate initial controller gains and exported firmware header:

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

## Modeling and Analysis

- Modeling pipeline: `docs/modeling.md`
- Plotting/tuning workflow: `docs/tuning.md`
