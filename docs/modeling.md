# Modeling and Control Reference

This is the canonical math and control document for the repo.

If firmware and docs diverge, update them together.

The repo currently has **two controller tracks**:
- **Track A — Archived design history:** first-principles plant, linearization, and the older cascade gain-design pipeline in `model/first_principles/`
- **Track B — Current implemented controller:** the single-PID runtime controller in `firmware/`

Track B is the authoritative description of the firmware that is currently flashed and tested on hardware.

## 1. Current Physical Setup

Measured/control-relevant quantities:
- `d` [cm]: HC-SR04 distance to the ball-side reflector target
- `theta_est` [deg]: actuator angle estimate from step counts, synchronized to AS5600 at run start
- `theta_abs` [deg]: AS5600-derived absolute actuator pose in the calibrated actuator frame

Runtime roles:
- HC-SR04 provides the control-relevant runner position
- AS5600 provides actuator travel calibration, run-start synchronization, travel-limit safety, and drift verification
- the stepper executes signed step-rate commands from the motion generator

State machine:
- `SAFE_DISABLED`
- `CALIB_SIGN`
- `READY`
- `RUNNING`
- `FAULT`

## 2. Calibration Frame Used by the Current Firmware

The current controller is **ball-centric**, not beam-level-centric.

### 2.1 Calibration commands
- `l`: capture lower actuator limit and one runner endpoint
- `u`: capture upper actuator limit and the other runner endpoint
- `p`: place the ball at the physical runner center and capture the sonar center plus the current actuator trim/origin
- `b`: stepper direction/sign jog

### 2.2 Stored runtime quantities
From `l/u/p`, the firmware stores:
- lower/upper raw AS5600 angles
- lower/upper sonar distances
- sonar center distance
- actuator trim/origin in the normalized actuator frame
- sign conventions for AS5600, sonar, and stepper direction

### 2.3 Sign conventions
Current hardware convention is fixed in firmware as:
- upper actuator limit = ball nearer to the sensor
- lower actuator limit = ball farther from the sensor

The sonar sign is therefore derived from hardware orientation, not from a user guess.

## 3. Track B — Current Implemented Runtime Controller

This section is the authoritative implementation model.

### 3.1 Actuator coordinates
The current runtime controller operates in a step-count actuator frame.

Definitions:
- `k_step = kStepperDegPerStep`
- `n` = relative stepper position in microsteps
- `theta_est_deg = n * k_step`
- `theta_est_rad = theta_est_deg * pi / 180`

At run start, step counts are synchronized from AS5600:
- `theta_abs_deg = runtimeMapActuatorDeg(raw_as5600_deg)`
- `theta_est_deg <- theta_abs_deg - trim_deg`

So the step-count frame is re-anchored to the calibrated actuator trim before entering `RUNNING`.

### 3.2 Ball-position coordinates
The current controller uses three related runner coordinates.

#### Physical runner coordinate
`x_linear_cm = sonar_sign * (d_cm - sonar_center_cm)`

This is the direct physical runner coordinate in the `p`-captured center frame.

#### Angle-corrected coordinate
`x_ctrl_cm = x_linear_cm * cos(theta_est_rad)`

This compensates for beam tilt changing the apparent sonar geometry.

#### Blended control coordinate
Let `b = feedback_blend` with `0 <= b <= 1`.

`x_feedback_cm = (1 - b) * x_ctrl_cm + b * x_linear_cm`

The same structure is applied to filtered signals:
`x_feedback_filt_cm = (1 - b) * x_ctrl_filt_cm + b * x_linear_filt_cm`

### 3.3 Setpoint semantics
- `q c`: target the physical runner center, `x_ref = 0`
- `q n`: target the calibrated **near endpoint** from `l/u`
- `q f`: target the calibrated **far endpoint** from `l/u`
- `q <cm>`: target an explicit physical runner offset from center

`q n` and `q f` are literal physical endpoints derived from the stored `l/u` sonar captures.

### 3.4 Feedback blend law
The firmware computes a target-dependent blend:

For calibrated near/far endpoint magnitudes `x_near > 0`, `x_far < 0`:
- if `x_ref = 0`, `b = kCenterFeedbackBlend`
- otherwise let `x_side = x_near` for positive targets and `|x_far|` for negative targets
- `b = kCenterFeedbackBlend + (1 - kCenterFeedbackBlend) * clamp(|x_ref| / x_side, 0, 1)`

Interpretation:
- near the center, use more angle-corrected information
- near the endpoints, rely more on the physical runner coordinate

### 3.5 Center-mode special case
The current controller intentionally overrides the general blended measurement at the center.

In `q c` mode:
- measurement = `x_linear_filt_cm`
- error = `-x_linear_filt_cm`

So center regulation uses the direct linear runner coordinate and separate center-only logic, rather than the generic `x_feedback` path.

### 3.6 Single-PID control law
The active runtime controller is in `firmware/src/control/cascade_controller.cpp`.

Base gains from `firmware/include/config.h`:
- `Kp = kPosPidKpStepsPerCm`
- `Ki = kPosPidKiStepsPerCmSec`
- `Kd = kPosPidKdStepsSecPerCm`

For each control tick with sample time `dt`:
- `e = x_ref - x_meas`
- `x_dot_meas ~= (x_meas - x_meas_prev) / dt`
- proportional term: `P = Kp_eff * e`
- derivative term: `D = -Kd * x_dot_meas`
- integral term is clamped and updated with anti-windup

The controller output is an **absolute actuator target** in step counts:

`n_cmd_unclamped = bias_center + P + I + D`

`n_cmd = clamp(n_cmd_unclamped, n_min, n_max)`

where `n_min`, `n_max` are the calibrated actuator soft limits expressed in step counts.

### 3.7 Center-only modifications
When `q c` is active, the controller adds center-specific behavior.

#### Reduced center gains
- `Kp_eff = Kp * kCenterPidKpScale`
- `Ki_eff = Ki * kCenterPidKiScale`

#### Hold zone
If both are true:
- `|x_linear_filt_cm| <= kCenterHoldPosTolCm`
- `|dx/dt| <= kCenterHoldRateTolCmS`

then:
- the control error is forced to zero
- the integral term is bled toward zero with `kCenterIntegralBleedPerSec`

This creates a neutral hold region around the center instead of continuously chasing tiny crossings.

#### Adaptive center bias
A runtime-only center bias is learned during center operation:

`bias_center <- clamp(bias_center + kCenterBiasLearnStepsPerCmSec * e * dt, -kCenterBiasClampSteps, +kCenterBiasClampSteps)`

This bias is used only in center mode and is cleared by recalibration.

### 3.8 Positive-side asymmetry compensation
When the target is positive (`q n` side), the controller can scale gains upward:
- `Kp_eff *= kPositiveSideKpScale`
- `Ki_eff *= kPositiveSideKiScale`

This is an empirical correction for the observed plant asymmetry where the near side required more actuator command than the far side.

### 3.9 Motion generator
The PID does not directly output a step rate.

Instead, it outputs a target actuator position `n_cmd`, and a motion generator converts actuator position error into signed step rate.

Let:
- `e_n = n_cmd - n_est`
- `a_run = kRunMotionAccelSps2`

If `|e_n| <= kRunPositionDeadbandSteps`, command zero rate.

Otherwise:

`u_desired = sign(e_n) * min(sqrt(2 * a_run * |e_n|), kMaxStepRateSps)`

Then apply a per-tick slew limit based on `kMaxStepRateChangeSpsPerTick`.

This makes the actuator command behave like a bounded position servo using rate-limited stepper motion.

## 4. HC-SR04 Runtime Validity and Fault Policy

The current firmware uses a hold-last-good policy.

### 4.1 Filtering path
HC-SR04 processing uses:
- range validity check
- jump clamp on raw distance
- rolling median window
- EMA smoothing

### 4.2 Validity logic
The last good sample remains usable until it becomes truly stale.

Current effective rule:
- `valid_pos = (sample_age_ms <= stale_threshold_ms)`

Miss counts are still recorded (`sonar_miss_count`) but are diagnostic only; they do not by themselves disable control.

### 4.3 Fault timing
A `sonar_timeout` fault is raised only when the time since the last accepted sonar sample exceeds the active stale threshold.

Thresholds are wider in `RUNNING` than in bring-up.

This preserves control through short HC-SR04 miss bursts, especially near the runner ends.

## 5. AS5600 Safety and Verification

AS5600 is no longer the primary ball-position signal, but it remains essential for actuator supervision.

### 5.1 Travel limits
Actuator travel limits are derived from the calibrated AS5600 span.

Soft limits are used for commanded actuator bounds.
Hard margins are used for `angle_oob` fault detection.

### 5.2 Drift verification
During `RUNNING`, the firmware compares:
- AS5600-derived actuator pose relative to trim
- step-count-derived actuator pose

Verification error:

`e_verify_deg = (theta_abs_deg - trim_deg) - theta_est_deg`

If `|e_verify_deg|` stays above the configured threshold for the configured dwell time, the controller raises `actuator_drift`.

## 6. Runtime Fault Set

Current runtime fault bits:
- `0x01`: `sonar_timeout`
- `0x02`: `i2c_error`
- `0x04`: `angle_oob`
- `0x08`: `pos_oob`
- `0x10`: `actuator_drift`

Important runtime interpretations:
- `pos_oob` is checked on the **linear** runner coordinate
- `angle_oob` is checked from the calibrated actuator frame
- `sonar_timeout` is an age-based stale-sample fault

## 7. Track A — Archived First-Principles Design Path

This section preserves the original design and thesis history.
It is no longer the authoritative runtime controller description.

### 7.1 Archived plant states
The first-principles derivation models:
- ball position `x`, velocity `x_dot`
- beam angle `theta`, rate `theta_dot`
- actuator input mapped through `theta = f(phi)`
- sonar map `x = g(d, theta)`

### 7.2 Archived nonlinear coupled equations
The archived coupled model uses:
- rolling effective mass
- beam inertia and COM terms
- actuator torque model
- rolling and pivot damping/friction

In compact form:

`m_e * x_ddot + m_e * R * theta_ddot = m * x * theta_dot^2 + m * g * sin(theta) + Q_x`

`m_e * R * x_ddot + (J + m_e * R^2 + m * x^2) * theta_ddot = Q_theta + gravity/coriolis terms`

This model remains useful for thesis derivation and offline simulation.

### 7.3 Archived linearized cascade design
The older design pipeline treated the plant as:
- outer loop: reduced double-integrator `x_ddot = k * theta`
- inner loop: stepper/beam integrator

That pipeline lives in:
- `model/first_principles/derive_nonlinear.py`
- `model/first_principles/linearize.py`
- `model/first_principles/design_cascade_pid.py`
- `model/first_principles/export_gains.py`

Its outputs include:
- `model/first_principles/controller_initial_gains.json`
- `firmware/include/generated/controller_gains.h`

These outputs should be treated as **archived design artifacts** unless the firmware is explicitly switched back to consume them.

## 8. Relationship Between Track A and Track B

The archived first-principles model remains valuable for:
- plant interpretation
- controller-design history
- thesis/report derivations
- explaining why the project initially pursued a cascade design

The current runtime diverged from that path because hardware debugging showed that the implemented controller needed to address:
- sonar geometry and dropouts
- actuator-frame calibration drift
- near/far asymmetry
- center-only neutral bias behavior

The experiment/debugging evidence for that evolution is recorded in:
- `docs/experiments/2026-03-control-debugging/README.md`

## 9. Files to Keep Synchronized

If the runtime controller changes, update these together:
- `firmware/src/main.cpp`
- `firmware/src/control/cascade_controller.cpp`
- `firmware/include/config.h`
- `docs/modeling.md`
- `docs/tuning.md`
- `.github/copilot-instructions.md`
- `.github/instructions/firmware.instructions.md`
- `.github/instructions/analysis.instructions.md`

If the archived design/model changes, update:
- `model/first_principles/*`
- `docs/modeling.md` Track A sections
- `.github/instructions/model.instructions.md`
