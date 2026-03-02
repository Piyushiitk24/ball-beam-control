# Ball & Beam Control — Project Guidelines

## Architecture

Three subsystems that must stay in sync:

| Subsystem | Path | Language | Purpose |
|-----------|------|----------|---------|
| **Firmware** | `firmware/` | C++/Arduino (ATmega328P, PlatformIO) | Cascade PID at 50 Hz, Timer1 ISR step-pulse generation |
| **Model** | `model/first_principles/` | Python | First-principles plant + gain design → auto-generates `firmware/include/generated/controller_gains.h` |
| **Analysis** | `analysis/` | Python | Parse, clean, and plot serial telemetry from `data/runs/` |

**Active sensors:** TFMini v1.7 LiDAR (position, SoftwareSerial D10/D11) + AS5600 (angle, I2C). HC-SR04 code is legacy-only — **never swap main.cpp to HC-SR04**.

State machine: `SAFE_DISABLED → CALIB_SIGN → READY ⇄ CALIB_SCALE → RUNNING → FAULT`

## Build & Deploy

```bash
# Gain pipeline (after editing params_measured_v1.yaml)
./.venv/bin/python model/first_principles/design_cascade_pid.py --params model/first_principles/params_measured_v1.yaml
./.venv/bin/python model/first_principles/export_gains.py

# Build + upload
cd firmware && pio run -e nano_new -t upload

# Host logger (recommended)
./.venv/bin/python analysis/serial_logger.py --port /dev/cu.usbserial-A10N20X1
```

Use `nano_old` env only for old-bootloader Nanos. VS Code tasks wrap these commands.

## Critical Rules

1. **Never edit `controller_gains.h`** — always regenerate via the design + export pipeline.
2. **Never remove `#include "sensors/tfmini_sensor.cpp"`** from main.cpp — required for compilation (platformio.ini excludes it from default builds to avoid ISR collisions).
3. **Never add float math or `digitalWrite()` in ISRs** — use direct port manipulation, flags/timestamps only.
4. **Always use `F()` macro** for `Serial.print` string literals (saves RAM on ATmega328P).
5. **Check flash after every change** — `pio run -e nano_new` reports usage. Budget: 30,720 bytes flash, 2,048 bytes RAM.

## Conventions

- **Units:** SI internally (`m`, `rad`, `s`); human-readable in telemetry (`cm`, `deg`).
- **Float literals:** Always use `f` suffix (`0.01745f`).
- **Naming:** `namespace bb`, `g_` globals, `k` constants, `m_` members.
- **Telemetry format:** `TEL,<t_ms>,<state>,<x_cm>,<x_filt_cm>,<theta_deg>,<theta_cmd_deg>,<u_step_rate>,<fault_flags>`
- **Sensor protocol:** TFMini uses the same command/diagnostic/telemetry protocol as the original HC-SR04 (`sonar diag`, `SONAR_DIAG,...` prefix). Host tools work unchanged.
- **Plant parameters:** Single source of truth is `model/first_principles/params_measured_v1.yaml`.

## Key References

| Topic | File |
|-------|------|
| Loop rates, limits, constants | `firmware/include/config.h` |
| Data structures | `firmware/include/types.h` |
| Cascade controller | `firmware/include/control/cascade_controller.h` |
| Calibration signs | `docs/calibration_signs.md` |
| Modeling derivations | `docs/modeling_source_of_truth.md` |
| Wiring | `docs/wiring.md` |
| Serial command reference | `commands.md` |
| Calibration workflow | `README.md` §3 |

## Scoped Guidance

Detailed area-specific instructions are in `.github/instructions/`:
- `firmware.instructions.md` — C++ firmware conventions, ISR safety, flash budget, TFMini notes
- `model.instructions.md` — Gain design pipeline, parameter file conventions
- `analysis.instructions.md` — Python analysis scripts, telemetry parsing, plotting
