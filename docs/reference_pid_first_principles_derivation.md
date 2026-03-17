# Reference PID First-Principles Derivation

This document derives the single-loop PID that matches the active reference
firmware in `firmware/src/main.cpp`.

Controller architecture:
- measurement: HC-SR04 ball distance, converted to center-relative position
- control law: single PID on position error
- controller output: absolute microstep target
- actuator command in code: `stepper.runToNewPosition(-output)`

The derivation below uses the measured hardware parameters from
`model/first_principles/params_measured.yaml`.

## 1. Effective Rolling Mass

For a rolling ball, the translational equation is written with the effective mass

`m_e = m + I / R^2`

Measured values:
- `m = 0.0028 kg`
- `R = 0.020 m`
- `I = (2/3) * m * R^2 = 0.000000746667 kg*m^2`

Substitution:

`m_e = 0.0028 + 0.000000746667 / 0.020^2 = 0.004666667 kg`

## 2. Reduced Linear Plant Coefficients

The small-angle reduced plant is

`x_ddot + beta * x_dot = alpha * g * theta`

with

`alpha = m / m_e`

`beta = b_x / m_e`

Measured values:
- `g = 9.81 m/s^2`
- `b_x = 0.0125 N*s/m`

Substitution:

`alpha = 0.0028 / 0.004666667 = 0.600000000`

`beta = 0.0125 / 0.004666667 = 2.678571429 1/s`

## 3. Microstep-To-Beam Conversion

The active reference sketch outputs absolute microstep targets, so the plant
must be written against step input `N`.

`k_step_rad = 2*pi / stepper_steps_per_rev * linkage_ratio`

Measured values:
- `stepper_steps_per_rev = 3200`
- `linkage_ratio = 1.0`

Substitution:

`k_step_rad = 2*pi / 3200 = 0.001963495408 rad/step`

Therefore

`theta = k_step_rad * N`

## 4. Position Plant In Step Units

Substituting `theta = k_step_rad * N` into the reduced plant and converting
position to centimeters gives

`X_cm(s) / N(s) = kappa / (s * (s + beta))`

where

`kappa = 100 * alpha * g * k_step_rad`

Substitution:

`kappa = 100 * 0.600000000 * 9.81 * 0.001963495408`

`kappa = 1.155713397 cm/s^2/step`

So the transfer function used for design is

`G(s) = 1.155713397 / (s * (s + 2.678571429))`

## 5. Continuous-Time PID Characteristic Equation

Use the standard PID

`C(s) = Kp + Ki/s + Kd*s`

with error in centimeters and output in steps.

The closed-loop characteristic polynomial becomes

`s^3 + (beta + kappa*Kd) * s^2 + kappa*Kp * s + kappa*Ki = 0`

## 6. Pole Placement Target

The measured parameter file currently specifies
- `outer_bw_hz = 0.12`
- `outer_zeta = 0.85`
- `extra_pole_factor = 4.0`

Convert bandwidth to natural frequency:

`wn = 2*pi*outer_bw_hz = 2*pi*0.12 = 0.753982237 rad/s`

Choose the third pole:

`p3 = extra_pole_factor * wn = 4.0 * 0.753982237 = 3.015928947 rad/s`

Target polynomial:

`(s^2 + 2*zeta*wn*s + wn^2) * (s + p3)`

Expanded coefficients:

`a2 = 2*zeta*wn + p3 = 4.297698750`

`a1 = wn^2 + 2*zeta*wn*p3 = 4.434215865`

`a0 = wn^2 * p3 = 1.714523075`

## 7. Physical PID Gains

Before converting to steps, the physical beam-angle controller is

`theta_cmd = Kp_theta * e_m + Ki_theta * integral(e_m) + Kd_theta * d(e_m)/dt`

For the physical plant `X_m(s) / Theta(s) = alpha*g / (s*(s + beta))`, matching
coefficients gives

`Kd_theta = (a2 - beta) / (alpha*g)`

`Kp_theta = a1 / (alpha*g)`

`Ki_theta = a0 / (alpha*g)`

Substitution:

`Kp_theta = 4.434215865 / (0.600000000 * 9.81) = 0.753349620 rad/m`

`Ki_theta = 1.714523075 / (0.600000000 * 9.81) = 0.291288324 rad/(m*s)`

`Kd_theta = (4.297698750 - 2.678571429) / (0.600000000 * 9.81) = 0.275081094 rad*s/m`

## 8. Conversion To Firmware Step Units

The firmware PID works in centimeters and absolute microsteps, so divide the
physical gains by `100 * k_step_rad`.

`Kp_steps = Kp_theta / (100 * k_step_rad)`

`Ki_steps = Ki_theta / (100 * k_step_rad)`

`Kd_steps = Kd_theta / (100 * k_step_rad)`

Substitution:

`Kp = 0.753349620 / (100 * 0.001963495408) = 3.836778111 steps/cm`

`Ki = 0.291288324 / (100 * 0.001963495408) = 1.483519252 steps/(cm*s)`

`Kd = 0.275081094 / (100 * 0.001963495408) = 1.400976510 step*s/cm`

These are the seed gains exported by
`model/first_principles/controller_initial_gains.json`.

## 9. Output Clamp Caveat

The active reference firmware currently limits output to `+/-400` steps.

Equivalent beam range:

`theta_max = 400 * 0.001963495408 = 0.785398163 rad = 45.000000 deg`

The small-angle model in `params_measured.yaml` still assumes a design-valid
beam range of `+/-4.0 deg`.

So the first-principles gains are now matched to the active controller
architecture and units, but the active software clamp still exceeds the linear
design region by a large margin. That caveat is intentional and is recorded in
the generated artifacts.
