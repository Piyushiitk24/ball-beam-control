# March 2026 Control Debugging Record

This document is the curated record of the recent debugging campaign that moved the project from non-working closed-loop control to the current single-PID runtime controller.

Current active validation scope is narrower than the historical record here:
- active runtime path is again the archived HC-SR04-based controller
- current validation targets only `q c` and `q f`
- `q n` remains part of the historical debugging record below, but near-side work is paused

Use it for:
- preserving context across chats
- reconstructing the engineering progression without relying on memory
- thesis/report writing
- linking representative telemetry runs to the code changes that addressed them

## Current Handoff State — 2026-03-16

- Final hardware beam has been replaced and is now the active hardware baseline.
- The active measured-model record is:
  - `model/first_principles/params_measured.yaml`
  - `docs/modeling_measured_calculations.md`
- The active runtime path is still the restored HC-SR04 controller.
- The active closed-loop validation workflow is now:

```text
/bringup
s
q
q c
e 1
r
q f
q c
s
/quit
```

- `q n` remains part of the historical record below, but near-side work is paused and should not be used for current validation runs.
- Representative pre-fix runs from the new beam:
  - `run_20260316_155405`: `q f` worked, `q c` behavior remained poor, and HC-SR04 stayed valid enough that the main issue was not a simple sensor dropout.
  - `run_20260316_160450`: with the ball checked at center before the run, `q f` still worked, but return `q c` from about `-10.3 cm` remained underpowered even though HC-SR04 stayed valid and fault-free.
- Those runs led to the latest controller change in `firmware/src/control/cascade_controller.cpp`:
  - near center, `q c` still uses softened center-only logic, hold behavior, and adaptive bias
  - far from center, `q c` now uses the normal stronger PID path
  - large positive return-to-center recovery can use the positive-side gain scaling
- Fresh hardware validation after that controller change is still outstanding. A fresh agent should treat the next `q c -> q f -> q c` run as the first post-fix confirmation run.

The generated appendix artifacts live in:
- `generated/recent_runs_summary.csv`
- `generated/recent_runs_summary.md`
- `generated/plots/`
- `generated/metrics/`

## 1. Experimental Setup and Current Controller Version

Hardware used in the recent runs:
- ball-and-beam apparatus with TMC2209-driven stepper actuator
- HC-SR04 ultrasonic sensor for ball position
- AS5600 for actuator travel calibration and verification
- Arduino Nano (`nano_new` PlatformIO environment)

Current runtime control architecture:
- ball-centric calibration using `l`, `u`, `p`
- step-count actuator frame synchronized from AS5600 at run start
- dual-coordinate ball-position interpretation:
  - physical runner coordinate `x_linear`
  - angle-corrected coordinate `x_ctrl`
  - blended non-center control coordinate `x_feedback`
- single position PID with motion-generator output to signed step rate
- center-only hold and adaptive bias logic
- positive-side gain scaling to address near-side asymmetry
- HC-SR04 hold-last-good timeout policy

Canonical references:
- runtime maths: `../../modeling.md`
- tuning workflow: `../../tuning.md`
- project instructions: `../../../.github/copilot-instructions.md`

## 2. Calibration and Test Protocol Used in These Runs

Standard calibration flow for the recent runs:

```text
s
d
l
u
p
e 1
b
v
s
t
q c
e 1
r
```

Target-change protocol used for most tracking runs:

```text
q f
q n
q c
k
e 0
```

Interpretation:
- `q c`: center regulation
- `q f`: far-end endpoint tracking
- `q n`: near-end endpoint tracking

These runs therefore cover three standard control-system experiment classes:
- regulation at center
- setpoint tracking to both ends of the runner
- failure analysis during transitions and sensor dropouts

Historical note:
- the active workflow has since been narrowed to center/far/center only
- keep the `q n` material below as engineering history, not as the current validation procedure

## 3. Milestone Timeline

### Milestone 1 — Early closed-loop failure from the wrong control frame
Representative run:
- [run_20260310_112753_telemetry.csv](../../../data/runs/run_20260310_112753/run_20260310_112753_telemetry.csv)
- plot: [run_20260310_112753.png](generated/plots/run_20260310_112753.png)

Observed behavior:
- `q c` pulled the ball toward the sensor side and then the run faulted
- later `q n` / `q f` appeared ineffective because the system was already faulted or using the wrong state estimate

Diagnosis:
- the earlier controller/calibration path did not have a stable, ball-centric center reference
- runtime control was operating in the wrong coordinate frame

Change introduced afterward:
- moved to sonar-centered calibration (`l/u/p`)
- stopped relying on a guessed beam-neutral reference as the primary center definition

### Milestone 2 — Endpoint tracking became visible, but near-end asymmetry remained
Representative run:
- [run_20260311_121358_telemetry.csv](../../../data/runs/run_20260311_121358/run_20260311_121358_telemetry.csv)
- plot: [run_20260311_121358.png](generated/plots/run_20260311_121358.png)

Observed behavior:
- far endpoint tracking was already acceptable
- near endpoint tracking backed away early and held short of target
- no firmware fault dominated the run

Diagnosis:
- the system was no longer fundamentally broken; it was control-asymmetric
- near-end control needed a different treatment from center and far-end control

Change introduced afterward:
- separated physical runner position from angle-corrected position
- introduced endpoint-aware feedback blending so endpoint targets stayed literal

### Milestone 3 — Center-only oscillation after endpoint improvements
Representative runs:
- [run_20260312_103903_telemetry.csv](../../../data/runs/run_20260312_103903/run_20260312_103903_telemetry.csv)
- plot: [run_20260312_103903.png](generated/plots/run_20260312_103903.png)
- [run_20260312_110245_telemetry.csv](../../../data/runs/run_20260312_110245/run_20260312_110245_telemetry.csv)
- plot: [run_20260312_110245.png](generated/plots/run_20260312_110245.png)

Observed behavior:
- `q f` and later `q n` could work better than before
- `q c` still crossed the center repeatedly and would not settle cleanly

Diagnosis:
- center behavior had become its own problem class
- the controller needed explicit center-only handling instead of being tuned only by the endpoint logic

Change introduced afterward:
- center-only gain scaling
- center hold zone and integral bleed
- runtime center-bias adaptation

### Milestone 4 — Positive-side undercommand after the architecture stabilized
Representative run:
- [run_20260312_112718_telemetry.csv](../../../data/runs/run_20260312_112718/run_20260312_112718_telemetry.csv)
- plot: [run_20260312_112718.png](generated/plots/run_20260312_112718.png)

Observed behavior:
- `q f` tracked acceptably
- `q n` stayed short of the near endpoint even when the actuator had already reached the position the controller asked for

Diagnosis:
- the plant is asymmetric
- the positive/near side needs more control authority than the far side

Change introduced afterward:
- positive-side gain scaling for non-center positive targets

### Milestone 5 — False sonar-validity gating that shut control off early
Representative run:
- [run_20260312_113933_telemetry.csv](../../../data/runs/run_20260312_113933/run_20260312_113933_telemetry.csv)
- plot: [run_20260312_113933.png](generated/plots/run_20260312_113933.png)

Observed behavior:
- initial `q c` improved
- later `q n` / return-to-center degraded and the controller appeared to stop before the final fault
- the run ended in `sonar_timeout`

Diagnosis:
- intermittent HC-SR04 miss bursts were marking `valid_pos` false too early
- that disabled the actuator before the held sample was actually stale
- the later visible `sonar_timeout` fault was the end of that process, not the beginning

Change introduced afterward:
- keep `valid_pos` true based on sample age only
- treat miss count as diagnostic, not as an early shutdown gate

## 4. Novel or Important Techniques Introduced During Debugging

These are the main implementation ideas that emerged from the debugging process and should be retained in the thesis narrative:
- **Sonar-centered calibration** using `l/u/p` instead of a manually guessed beam-neutral reference
- **AS5600 reframing** from “beam level sensor” to actuator calibration/safety/verification sensor
- **Single position PID** replacing the older runtime cascade path
- **Dual-coordinate feedback** using both `x_linear` and `x_ctrl`
- **Target-dependent feedback blending** so endpoint commands are literal runner endpoints, not modeled offsets
- **Center-only hold and adaptive bias** to reduce endless center chasing
- **Positive-side gain scaling** to compensate for observed mechanical asymmetry
- **HC-SR04 hold-last-good policy** with age-based validity instead of immediate drop on short miss bursts
- **Expanded telemetry** (`x_linear`, `x_ctrl`, `x_feedback`, `feedback_blend`, `sonar_age_ms`, `sonar_valid`) so diagnoses can be made from logs rather than guesswork

## 5. Current Known Limitations

At the time of this record, the dominant open issues are still control-quality issues rather than architecture failures:
- center regulation remains more delicate than endpoint tracking
- near-end tracking remains harder than far-end tracking
- HC-SR04 data quality near the runner ends remains a practical constraint even with the improved validity policy
- the archived first-principles cascade design and the current runtime controller are no longer identical; the report must describe both tracks honestly

## 6. Generated Evidence

The generated appendix artifacts provide the full recent run index and the representative figures used above:
- [recent_runs_summary.csv](generated/recent_runs_summary.csv)
- [recent_runs_summary.md](generated/recent_runs_summary.md)

To regenerate them:

```bash
cd /Users/piyush/code/ball-beam-control
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/mpl ./.venv/bin/python analysis/report_recent_runs.py
```
