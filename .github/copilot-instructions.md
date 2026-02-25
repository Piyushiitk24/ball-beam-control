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
| `docs/SonarCylinder.FCMacro` | FreeCAD macro — builds rolling sonar target (ball+belt or spool) |

## HC-SR04 Sonar — Known Behaviours

- **Jump-filter deadlock**: `kSonarMaxJumpCm` (in `hcsr04_sensor.cpp`) gates large position steps. If many echoes are missed (e.g. ball at far end), `last_distance_cm_` freezes; every subsequent valid echo is then rejected. The fix: on rejection the baseline slides toward the new reading by `kSonarMaxJumpCm`, so recovery takes at most 2 pings (≈70 ms).
- **`e 1` returns `driver_disabled`**: driver enable is blocked whenever `hasAnyFault()` is true (including `sonar_timeout`). Fix the sonar first — `s` and `sonar diag` will tell you which fault is active.
- **Curved targets**: HC-SR04 backscatter from a sphere is ~34× weaker than a same-size flat disc. For bring-up and calibration, use a flat board. For runtime, use a target with planar facets aimed along the sonar axis.
- **Reflector geometry**: `docs/SonarCylinder.FCMacro` supports `TARGET_GEOMETRY="ball_belt"` (recommended for V-groove runners): a hollow rolling sphere with a narrow faceted belt that provides a strong along-beam echo without changing rolling contact. Use `BELT_FACETS >= 24` (default 48). `TARGET_GEOMETRY="spool"` remains available if you want wheel-like guidance.

## Flash Budget

ATmega328P: 30,720 bytes flash, 2,048 bytes RAM. The firmware currently sits near the limit (~30,710 bytes). When adding code:
- Prefer reusing existing buffers over new allocations.
- `F()` macro for all `Serial.print` string literals (saves RAM).
- Avoid new virtual functions or `std::` containers.
- Check size after every non-trivial change: `pio run -e nano_new` reports usage.

## Rolling Reflector — Model Update Checklist

When switching rolling targets (`docs/SonarCylinder.FCMacro`), update `params_measured_v1.yaml` and regenerate:

```yaml
ball_mass_kg:  <weigh printed target>

# If using TARGET_GEOMETRY="spool":
ball_inertia_ratio: 0.5        # solid cylinder approx (spool)
ball_radius_m: <R_BODY / 1000> # rolling radius in metres

# If using TARGET_GEOMETRY="ball_belt":
ball_inertia_ratio: 0.6667     # thin-shell sphere approx (hollow ball; belt adds small deviation)
ball_radius_m: <R_BALL / 1000> # rolling radius in metres
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
