# Ball & Beam Control — Project Guidelines

## Architecture

This repo has three distinct tracks:

| Track | Path | Role |
| --- | --- | --- |
| Runtime controller | `firmware/` | **Authoritative** closed-loop implementation used on hardware |
| Analysis and reporting | `analysis/` | Logging, plotting, run comparison, and experiment/report generation |
| Archived design model | `model/first_principles/` | Historical first-principles derivation and old cascade gain-design pipeline |

Current runtime state machine: `SAFE_DISABLED -> CALIB_SIGN -> READY -> RUNNING -> FAULT`

The active runtime controller is the **single position PID** in `firmware/src/control/cascade_controller.cpp`, wired through `firmware/src/main.cpp` and tuned mainly from `firmware/include/config.h`.

Treat `docs/modeling.md` as the canonical math/controller document. If runtime behavior changes, update it in the same change set.

## Build and validation

Use these commands for agent-driven verification:

- Build firmware: `pio run -e nano_new`
- Upload firmware: `pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1`
- Fallback upload target for older bootloaders: `pio run -e nano_old -t upload`
- Start host logger: `./.venv/bin/python analysis/serial_logger.py --port /dev/cu.usbserial-A10N20X1`
- Plot one run: `MPLBACKEND=Agg MPLCONFIGDIR=/tmp/mpl ./.venv/bin/python analysis/plot_run.py --input data/runs/<run>`
- Regenerate curated debug report: `MPLBACKEND=Agg MPLCONFIGDIR=/tmp/mpl ./.venv/bin/python analysis/report_recent_runs.py`

There is no single automated test suite; validation is typically firmware build + relevant analysis/report scripts.

## Project-specific conventions

- **HC-SR04 is the active ball-position sensor.** AS5600 is for calibration, synchronization, travel-limit safety, and drift verification.
- **Three runner coordinates matter:** `x_linear` (physical), `x_ctrl = x_linear * cos(theta_est)`, and `x_feedback` (target-dependent blend). Do not collapse them when reasoning about behavior.
- **Center mode is special.** `q c` uses center-only logic and linear-position feedback; non-center targets use blended feedback.
- **Calibration is ball-centric** via `l`, `u`, `p`, then `b`, then `v`. Signs are auto-derived from calibration measurements and the jog.
- **Telemetry keeps `sonar_*` field names for compatibility** even though HC-SR04 is the live runtime sensor.
- **`controller_gains.h` is not the active runtime path.** Treat it as archived model output unless the firmware is explicitly switched back.

## Important gotchas

- Flash budget is tight on ATmega328P: **30,720 bytes flash, 2,048 bytes RAM**. Always check build size after firmware edits.
- Use `F()` for `Serial.print` string literals and keep ISRs minimal: no floating-point work, no heavy logic, no verbose output.
- `analysis/switch_firmware_main.py --mode ballbeam` is a script mode, **not** a PlatformIO environment.
- When documenting debugging history, use `docs/experiments/2026-03-control-debugging/README.md` and its generated artifacts instead of reconstructing events from memory.
- Current active validation scope is centered on `q c` and `q f`; `q n` remains implemented but is not the main validation path right now.

## Sources of truth

- Runtime math and controller description: `docs/modeling.md`
- Operational workflow and calibration steps: `README.md`
- Device command reference: `commands.md`
- Debugging chronology and representative runs: `docs/experiments/2026-03-control-debugging/README.md`
- Current runtime tuning knobs: `firmware/include/config.h`
- Telemetry plotting: `analysis/plot_run.py`
- Recent-runs report generation: `analysis/report_recent_runs.py`

## Scoped guidance

Use the file-specific instructions in `.github/instructions/` when working in those areas:

- `firmware.instructions.md` — flash budget, ISR safety, runtime control path, calibration, safety
- `analysis.instructions.md` — telemetry schema, plotting, reporting, experiment record rules
- `model.instructions.md` — archived first-principles/design pipeline and documentation rules
