# Ball & Beam Control — Project Guidelines

## Architecture

Two subsystems that must stay in sync:

- **Firmware** (`firmware/`) — C++/Arduino, ATmega328P, PlatformIO. Cascade PID loop at 50 Hz, Timer1 ISR step-pulse generation, PCINT ISR for HC-SR04 echo capture. All code lives in `namespace bb`.
- **Model** (`model/first_principles/`) — Python. First-principles plant + gain design. Output is `firmware/include/generated/controller_gains.h`, which is **auto-generated — never edit by hand**.
- **Analysis** (`analysis/`) — Python scripts to parse, clean, and plot serial telemetry CSVs from `data/runs/`.

State machine (see [firmware/include/app/state_machine.h](../firmware/include/app/state_machine.h)):
`SAFE_DISABLED → CALIB_SIGN → READY → RUNNING → FAULT`

## Build & Test

All Python commands use the project venv:

```bash
# Create + activate venv (custom alias)
mkvenv
pip install -r requirements.txt

# Design gains → writes controller_initial_gains.json
./.venv/bin/python model/first_principles/design_cascade_pid.py --params model/first_principles/params_measured_v1.yaml

# Export gains → overwrites firmware/include/generated/controller_gains.h
./.venv/bin/python model/first_principles/export_gains.py

# Build firmware
cd firmware && pio run -e nano_new          # new bootloader (default)
cd firmware && pio run -e nano_old          # old bootloader (57600 baud upload)

# Upload
cd firmware && pio run -e nano_new -t upload

# Serial monitor
cd firmware && pio device monitor -b 115200 --echo --filter send_on_enter --eol LF
```

VS Code tasks in `.vscode/tasks.json` wrap the common commands above.

## Conventions

**Units — strict SI internally, human-readable in telemetry:**
- Control math uses `m` (position), `rad` (angle), `s` (time).
- Telemetry output and serial display use `cm` and `deg`.
- Float constants carry the `f` suffix: `0.01745f`, not `0.01745`.

**ISR safety (Timer1 step-pulse path and PCINT echo capture):**
- No floating-point math.
- No `digitalWrite()` — use direct port manipulation.
- Absolute minimum work: set flags/timestamps only; all filtering/control in main thread.

**Cascade controller** ([firmware/include/control/cascade_controller.h](../firmware/include/control/cascade_controller.h)):
- Outer loop: position error (`m`) → angle command (`rad`).
- Inner loop: angle error (`rad`) → step rate (`sps`).
- Gains loaded from `namespace bb::generated` in `controller_gains.h`.

**Plant parameters** — single source of truth: [`model/first_principles/params_measured_v1.yaml`](../model/first_principles/params_measured_v1.yaml). Edit params there, then re-run the design + export pipeline.

**PlatformIO layout** — repo root has a wrapper `platformio.ini`; the real config is `firmware/platformio.ini`. Two environments: `nano_new` (new ATmega328P bootloader) and `nano_old` (original).

## Key Files

| File | Purpose |
|---|---|
| `firmware/include/config.h` | All loop rates, limits, unit conversion constants |
| `firmware/include/types.h` | `SensorData`, `Setpoint`, `ActuatorCmd`, `FaultFlags` |
| `firmware/include/generated/controller_gains.h` | **Auto-generated** PID gains — do not edit |
| `model/first_principles/params_measured_v1.yaml` | Plant + design parameters (source of truth) |
| `docs/modeling_source_of_truth.md` | Modeling conventions and derivation notes |
| `docs/calibration_signs.md` | Sign convention for sensors and actuators |

## Common Pitfalls

- Editing `controller_gains.h` directly — always regenerate via `export_gains.py`.
- Adding float math or `digitalWrite()` inside an ISR.
- Using `nano_old` profile to upload to a new-bootloader Nano (upload will hang or fail).
- Forgetting to re-export gains after changing `params_measured_v1.yaml` or design bandwidth targets.
