# Ball & Beam Control — Project Guidelines

## Architecture

The repo now has two controller tracks that must be documented differently:

| Track | Path | Status | Purpose |
| --- | --- | --- | --- |
| **Runtime controller** | `firmware/` | **Authoritative** | Current single position PID, dual-coordinate ball feedback, AS5600 safety/verification, HC-SR04-based runner control |
| **Archived design model** | `model/first_principles/` | Historical/reference | First-principles nonlinear plant, linearization, and old cascade gain-design pipeline |
| **Analysis/reporting** | `analysis/` | Active | Serial logging, plotting, run comparison, and thesis/report record generation |

Current firmware state machine: `SAFE_DISABLED -> CALIB_SIGN -> READY -> RUNNING -> FAULT`

Active sensors and their roles:
- **HC-SR04**: ball position along the runner; control-relevant signal with hold-last-good timeout policy
- **AS5600**: actuator travel calibration, run-start synchronization, travel-limit safety, and drift verification

## Current Runtime Controller

The runtime controller is **not** the old cascade controller from the first-principles design scripts.

The current firmware uses:
- `l`, `u`, `p` calibration frame
- physical runner position `x_linear`
- angle-corrected position `x_ctrl = x_linear * cos(theta_est)`
- blended feedback position `x_feedback`
- a **single position PID** that outputs an absolute actuator target in step counts
- a motion generator that converts actuator position error to signed step rate
- center-only bias adaptation and center hold logic
- positive-side gain scaling to handle near-end asymmetry

Key implementation references:
- current controller: `firmware/src/control/cascade_controller.cpp`
- runtime wiring/calibration/control loop: `firmware/src/main.cpp`
- constants/tuning: `firmware/include/config.h`
- telemetry/state types: `firmware/include/types.h`

## Build, Upload, Logging, and Reporting

```bash
# Build
pio run -e nano_new

# Upload
pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1

# Host logger
./.venv/bin/python analysis/serial_logger.py --port /dev/cu.usbserial-A10N20X1

# Single-run plot
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/mpl ./.venv/bin/python analysis/plot_run.py --input data/runs/<run>_telemetry.csv

# Curated recent-runs report
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/mpl ./.venv/bin/python analysis/report_recent_runs.py
```

## Critical Rules

1. **Treat `docs/modeling.md` as the canonical math document.** If firmware behavior changes, update the math doc in the same change set.
2. **Do not describe `controller_gains.h` as the active runtime controller path.** It belongs to the archived design pipeline unless a task is explicitly about that path.
3. **Always check flash after firmware changes.** Budget: 30,720 bytes flash, 2,048 bytes RAM.
4. **Use `F()` for `Serial.print` string literals and keep ISR work minimal.**
5. **When documenting debugging progress, use the curated experiment record in `docs/experiments/2026-03-control-debugging/` rather than reconstructing history ad hoc.**

## Current Sources of Truth

| Topic | File |
| --- | --- |
| Canonical math / controller description | `docs/modeling.md` |
| Debugging chronology and representative runs | `docs/experiments/2026-03-control-debugging/README.md` |
| Generated run index / plots | `docs/experiments/2026-03-control-debugging/generated/` |
| Current firmware tuning knobs | `firmware/include/config.h` |
| Calibration workflow / commands | `README.md`, `commands.md` |
| Current telemetry plotting | `analysis/plot_run.py` |
| Recent run report generation | `analysis/report_recent_runs.py` |

## Scoped Guidance

Detailed area-specific instructions live in `.github/instructions/`:
- `firmware.instructions.md` — current runtime controller, calibration, safety, flash budget
- `model.instructions.md` — archived first-principles pipeline and how it relates to runtime
- `analysis.instructions.md` — telemetry schema, plotting, and experiment-record generation
