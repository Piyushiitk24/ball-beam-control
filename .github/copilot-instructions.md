# Ball & Beam Control — Project Guidelines

## Architecture

Two subsystems that must stay in sync:

- **Firmware** (`firmware/`) — C++/Arduino, ATmega328P, PlatformIO. Cascade PID loop at 50 Hz, Timer1 ISR step-pulse generation, PCINT ISR for HC-SR04 echo capture. All code lives in `namespace bb`.
- **Model** (`model/first_principles/`) — Python. First-principles plant + gain design. Output is `firmware/include/generated/controller_gains.h`, which is **auto-generated — never edit by hand**.
- **Analysis** (`analysis/`) — Python scripts to parse, clean, and plot serial telemetry CSVs from `data/runs/`.

State machine (see [firmware/include/app/state_machine.h](../firmware/include/app/state_machine.h)):
`SAFE_DISABLED → CALIB_SIGN → READY ⇄ CALIB_SCALE → RUNNING → FAULT`

`CALIB_SCALE` is reached from `READY` during scale/runtime calibration; `requestStop()` from either `RUNNING` or `CALIB_SCALE` returns to `READY`.

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

**macOS USB serial port** — PlatformIO uses glob `/dev/cu.usbserial-*` for upload and monitor. If upload hangs, confirm the correct port and bootloader environment (`nano_new` vs `nano_old`).

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
| `docs/SonarSphere.FCMacro` | FreeCAD macro — hollow rolling sphere + optional faceted "sonar belt" (recommended for V-groove runners) |
| `docs/SonarCylinder.FCMacro` | FreeCAD macro — hollow rolling cylinder/tube A/B test target (optionally faceted); also supports `TARGET_GEOMETRY="ball_belt"` (default, V-groove recommended) |
| `docs/SonarTargets.FCMacro` | FreeCAD macro — combined/experimental generator (sphere+belt / cylinder / spool) |

## Telemetry & Analysis

**Telemetry line format** (emitted at `kTelemetryHz = 10 Hz` while running):
```
TEL,<t_ms>,<state>,<x_cm>,<x_filt_cm>,<theta_deg>,<theta_cmd_deg>,<u_step_rate>,<fault_flags>
```
Columns map directly to `parse_log.py`'s `COLUMNS` list: `t_ms`, `state`, `x_cm`, `x_filt_cm`, `theta_deg`, `theta_cmd_deg`, `u_step_rate`, `fault_flags` (uint8 bitmask: bit0=sonar_timeout, bit1=i2c_error, bit2=angle_oob, bit3=pos_oob).

**Analysis workflow:**
```bash
# Capture live serial to a raw log file
cd firmware && pio device monitor -b 115200 --echo --filter send_on_enter --eol LF > ../data/runs/<name>_raw.txt

# Parse raw log → clean CSV
./.venv/bin/python analysis/parse_log.py --input data/runs/<name>_raw.txt --output data/runs/<name>_clean.csv

# Plot clean CSV (4-panel: position, angle, step rate, fault)
./.venv/bin/python analysis/plot_run.py --input data/runs/<name>_clean.csv

# Compare two runs
./.venv/bin/python analysis/compare_runs.py --a data/runs/<a>_clean.csv --b data/runs/<b>_clean.csv
```

## HC-SR04 Sonar — Known Behaviours

- **Jump-filter deadlock**: `kSonarMaxJumpCm` (in `hcsr04_sensor.cpp`) gates large position steps. If many echoes are missed (e.g. ball at far end), `last_distance_cm_` freezes; every subsequent valid echo is then rejected. The fix: on rejection the baseline slides toward the new reading by `kSonarMaxJumpCm`, so recovery takes at most 2 pings (≈70 ms).
- **`e 1` returns `driver_disabled`**: driver enable is blocked whenever `hasAnyFault()` is true (including `sonar_timeout`). Fix the sonar first — `s` and `sonar diag` will tell you which fault is active.
- **Curved targets**: HC-SR04 backscatter from a sphere is ~34× weaker than a same-size flat disc. For bring-up and calibration, use a flat board. For runtime, use a target with planar facets aimed along the sonar axis.
- **Reflector geometry**: `docs/SonarSphere.FCMacro` generates a hollow rolling sphere with an optional faceted belt that provides a strong along-beam echo without changing rolling contact. Use `BELT_ENABLE=True` and `BELT_FACETS >= 24` (default 48). Set `BELT_ENABLE=False` for a pure sphere baseline.
- **Cylinder option**: `docs/SonarCylinder.FCMacro` also supports `TARGET_GEOMETRY="spool"` for a wheel/flanged spool. Set `BODY_PROFILE="faceted"` with `BODY_FACETS >= 24` for stronger echo. Default (`"ball_belt"`) is recommended for V-groove runners.
- **Combined macro**: `docs/SonarTargets.FCMacro` contains the earlier combined generator (sphere+belt / cylinder / spool) if you want all options in one file.

## Flash Budget

ATmega328P: 30,720 bytes flash, 2,048 bytes RAM. The firmware currently sits near the limit (~30,710 bytes). When adding code:
- Prefer reusing existing buffers over new allocations.
- `F()` macro for all `Serial.print` string literals (saves RAM).
- Avoid new virtual functions or `std::` containers.
- Check size after every non-trivial change: `pio run -e nano_new` reports usage.

## Rolling Reflector — Model Update Checklist

When switching rolling targets (`docs/SonarSphere.FCMacro` / `docs/SonarCylinder.FCMacro`), update `params_measured_v1.yaml` and regenerate:

```yaml
ball_mass_kg:  <weigh printed target>

# If using SonarSphere (hollow sphere, optional belt):
ball_inertia_ratio: 0.6667     # thin-shell sphere approx (hollow ball; belt adds small deviation)
ball_radius_m: <R_BALL / 1000> # rolling radius in metres

# If using SonarCylinder (hollow tube):
# Hollow cylinder k = I/(m r^2) = 0.5 * (1 + (r_inner/r_outer)^2)
# Example: R=25mm, shell=1.2mm -> k ~= 0.95
ball_inertia_ratio: 0.95
ball_radius_m: <CYL_RADIUS / 1000>
```

```bash
./.venv/bin/python model/first_principles/design_cascade_pid.py --params model/first_principles/params_measured_v1.yaml
./.venv/bin/python model/first_principles/export_gains.py
cd firmware && pio run -e nano_new -t upload
```

Then redo the full calibration wizard (`w`) since `sonar_center_cm` changes with the new reflector height.

## Common Pitfalls

- Editing `controller_gains.h` directly — always regenerate via `export_gains.py`.
- Adding float math or `digitalWrite()` inside an ISR.
- Using `nano_old` profile to upload to a new-bootloader Nano (upload will hang or fail).
- Forgetting to re-export gains after changing `params_measured_v1.yaml` or design bandwidth targets.
- Calibrating with a proxy object (phone, board) for sonar — always recalibrate with the real reflector in place on the actual beam.
- After any power cycle, load calibration with `o` before `r` — or run the wizard again.
