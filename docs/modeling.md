# Modeling and Control Reference

This is the canonical modeling document for the repository.

The current authoritative controller architecture is the standalone reference PID
sketch in `firmware/src/main.cpp`.

## 1. Active Controller Architecture

The active firmware is intentionally simple:
- HC-SR04 provides the ball distance
- a Kalman-style scalar filter smooths the distance
- the controller computes
  - `h = 13.5 - ball_position`
  - `a = current_steps * radiansPerStep`
  - `input = h * cos(a)`
- a single PID maps position error to an absolute microstep target
- the actuator command is `stepper.runToNewPosition(-output)`

Important consequences:
- AS5600 is not used by the active controller path
- there is no runtime state machine, bringup flow, or cascade controller in the
  active firmware
- the active controller output is an absolute actuator position target in steps,
  not a step rate

## 2. First-Principles Model That Matches The Active Code

The active first-principles design in `model/first_principles/` now targets the
same control architecture as `firmware/src/main.cpp`.

Reduced plant assumptions:
- beam-angle actuator is ideal for design purposes
- ball-beam dynamics are linearized around `theta = 0`
- the controller output is the commanded absolute microstep target `N`

Reduced plant:

`x_ddot + beta * x_dot = alpha * g * theta`

`theta = k_step_rad * N`

So the controller-design transfer function is

`G(s) = X_cm(s) / N(s) = kappa / (s * (s + beta))`

with
- `alpha = m / m_e`
- `m_e = m + I / R^2`
- `beta = b_x / m_e`
- `k_step_rad = 2*pi / stepper_steps_per_rev * linkage_ratio`
- `kappa = 100 * alpha * g * k_step_rad`

The generated reduced linear model lives in
`model/first_principles/linearized_model.json`.

## 3. PID Design Method

The active design script is still
`model/first_principles/design_cascade_pid.py`, but it now designs a
single-loop reference PID instead of the archived cascade controller.

Controller form:

`C(s) = Kp + Ki/s + Kd*s`

Closed-loop characteristic polynomial:

`s^3 + (beta + kappa*Kd) * s^2 + kappa*Kp * s + kappa*Ki = 0`

Pole-placement target:

`(s^2 + 2*zeta*wn*s + wn^2) * (s + p3)`

where
- `wn = 2*pi*outer_bw_hz`
- `p3 = extra_pole_factor * wn`

The script first solves the PID in physical beam-angle units
(`rad/m`, `rad/(m*s)`, `rad*s/m`) and then converts those gains into the
microstep-per-centimeter units used by firmware.

The full step-by-step derivation and numerical substitution are documented in
`docs/reference_pid_first_principles_derivation.md`.

## 4. Current Measured-Parameter Design Result

Using `model/first_principles/params_measured.yaml`, the current seed gains are:
- `Kp = 3.836778 steps/cm`
- `Ki = 1.483519 steps/(cm*s)`
- `Kd = 1.400977 step*s/cm`

Key intermediate quantities are:
- `alpha = 0.600000000`
- `beta = 2.678571429 1/s`
- `rad_per_step = 0.001963495408`
- `kappa = 1.155713397 cm/s^2/step`

These values are exported to:
- `model/first_principles/controller_initial_gains.json`
- `firmware/include/generated/controller_gains.h`

## 5. Important Modeling Caveat

The active reference firmware currently clamps the PID output to `+/-400` steps.

With the current actuator geometry:
- `400 steps = 0.785398163 rad = 45.0 deg`

The measured parameter file still records the small-angle modeling limit as
`+/-4.0 deg`.

So the current first-principles design is now correct for the active controller
architecture and units, but the active software clamp is still much larger than
the strict linear model range. That mismatch is preserved intentionally because
the user asked the math artifacts to match the active code path first.

## 6. Archived Paths

The older nonlinear coupled model files are still kept for reference:
- `model/first_principles/derive_nonlinear.py`
- `model/first_principles/first_principles_core.py`

The older command-driven runtime controller in the restored BallBeam firmware is
also archived, but it is not the current authoritative controller for modeling.

## 7. Files To Keep In Sync

If the active reference PID architecture changes, update these together:
- `firmware/src/main.cpp`
- `model/first_principles/design_cascade_pid.py`
- `model/first_principles/linearize.py`
- `model/first_principles/controller_initial_gains.json`
- `firmware/include/generated/controller_gains.h`
- `docs/modeling.md`
- `docs/modeling_source_of_truth.md`
- `docs/reference_pid_first_principles_derivation.md`
