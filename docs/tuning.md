# Runtime Tuning Workflow

This document describes the **current empirical tuning workflow** for the runtime controller in firmware.

The active runtime controller is the single position PID in `firmware/src/control/cascade_controller.cpp`, not the archived cascade-design pipeline.

## 1. What We Tune Today

Primary tuning knobs:
- base position PID gains in `firmware/include/config.h`
  - `kPosPidKpStepsPerCm`
  - `kPosPidKiStepsPerCmSec`
  - `kPosPidKdStepsSecPerCm`
- center-only behavior
  - `kCenterFeedbackBlend` in `firmware/src/main.cpp`
  - `kCenterPidKpScale`
  - `kCenterPidKiScale`
  - `kCenterHoldPosTolCm`
  - `kCenterHoldRateTolCmS`
  - `kCenterIntegralBleedPerSec`
  - `kCenterBiasLearnStepsPerCmSec`
  - `kCenterBiasClampSteps`
- positive-side asymmetry
  - `kPositiveSideKpScale`
  - `kPositiveSideKiScale`
- motion profile
  - `kMaxStepRateSps`
  - `kMaxStepRateChangeSpsPerTick`
  - `kRunPositionDeadbandSteps`
- sonar fault tolerance
  - `kSensorInvalidFaultMsBringup`
  - `kSensorInvalidFaultMsRunning`
  - `SONAR_POS_SAMPLE_FRESH_MS`
  - `SONAR_MAX_JUMP_CM`

## 2. Standard Runtime Tests

Use these target classes for runtime tuning:
- `q c`: center regulation / neutral-bias stability
- `q f`: far-end endpoint hold
- `q n`: near-end endpoint hold
- `q c -> q f -> q n -> q c`: transition and asymmetry test

This project's recent debugging work showed that center behavior and endpoint behavior are not tuned by the same parameters.

## 3. What to Look For in the Plots

Use the single-run plot from `analysis/plot_run.py` and the recent-runs report from `analysis/report_recent_runs.py`.

Most important signals:
- `x_ref_cm`
- `x_linear_filt_cm`
- `x_ctrl_filt_cm`
- `x_feedback_cm`
- `theta_deg`
- `theta_cmd_deg`
- `u_step_rate`
- `sonar_age_ms`
- `sonar_valid`
- `fault_flags`
- `feedback_blend`

Key metrics to judge each run:
- center crossings during `q c`
- steady-state error at `q c`, `q f`, and `q n`
- endpoint shortfall, especially on the positive/near side
- invalid sonar rows and peak sonar age
- fault rows and fault labels
- whether `theta_deg` is reaching `theta_cmd_deg` while the ball is still short of target

## 4. Practical Tuning Sequence

Tune in this order:

### Step 1 — Sensor validity and safety
Fix first if any of these occur:
- `sonar_timeout`
- `actuator_drift`
- `angle_oob`
- control stopping while `u_step_rate` collapses to zero even though the target was not reached

Reason:
- false validity/fault transitions corrupt every later tuning decision

### Step 2 — Endpoint asymmetry
Use `q f` and `q n`.

If `q f` works but `q n` is short:
- increase `kPositiveSideKpScale`
- then increase `kPositiveSideKiScale` if needed

If both ends are weak:
- revisit base `Kp`
- then base `Ki`

### Step 3 — Center regulation
Use a long `q c` run.

If the ball crosses the center repeatedly:
- adjust `kCenterFeedbackBlend`
- then reduce center-only aggressiveness via:
  - `kCenterPidKpScale`
  - `kCenterPidKiScale`
- use the center hold zone and integral bleed to stop endless micro-chasing
- if a persistent center bias remains, adjust the adaptive center-bias learning rate/clamp

### Step 4 — Motion feel
If the controller is correct but too abrupt or too soft:
- adjust `kMaxStepRateChangeSpsPerTick`
- then `kRunPositionDeadbandSteps`
- only then revisit base PID gains

## 5. Commands

Single-run plot:

```bash
LATEST="$(ls -t data/runs/run_*_telemetry.csv | head -n 1)"
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/mpl ./.venv/bin/python analysis/plot_run.py --input "$LATEST"
```

Curated recent-runs report:

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/mpl ./.venv/bin/python analysis/report_recent_runs.py
```

Generated report artifacts:
- `docs/experiments/2026-03-control-debugging/generated/recent_runs_summary.csv`
- `docs/experiments/2026-03-control-debugging/generated/recent_runs_summary.md`
- `docs/experiments/2026-03-control-debugging/generated/plots/`

## 6. Current Known Themes From Recent Runs

Recent tuning/debugging work established these recurring themes:
- center regulation and endpoint tracking require different handling
- the near side is harder than the far side and often needs extra positive-side authority
- HC-SR04 intermittent miss bursts can mimic a control problem if validity gating is too strict
- the controller must be interpreted using both `x_linear` and `x_ctrl`, not only one coordinate
- `q c`, `q f`, and `q n` should be evaluated separately before making global tuning changes
