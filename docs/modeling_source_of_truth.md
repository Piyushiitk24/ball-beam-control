# Ball-Beam Mathematical Model (Source of Truth)

This document is the canonical mathematical specification for this repo.
When docs and code disagree, this model spec is authoritative and must be reconciled in code.

Implementation reference:
- `model/first_principles/first_principles_core.py`
- `model/first_principles/derive_nonlinear.py`
- `model/first_principles/linearize.py`
- `model/first_principles/design_cascade_pid.py`

## 1) Scope and Sign Convention

Coordinate/sign convention used across modeling + firmware:
- `x > 0`: ball toward the chosen positive end of beam.
- `theta > 0`: beam tilt that drives the ball toward `+x` (small-angle behavior).
- `u_step_rate > 0`: command that increases `theta`.

This is intentionally aligned with:
- `docs/calibration_signs.md`
- firmware calibration and control signs.

## 2) Measured vs Physical Variables

Measured:
- Motor angle `phi` (AS5600 on motor shaft).
- Sonar range `d` (HC-SR04).

Physical plant states:
- Ball position `x` and velocity `x_dot`.
- Beam angle `theta` and rate `theta_dot`.

Therefore, all end-to-end models include:
- `theta = f(phi)` (motor-to-beam calibration map, optional hysteresis).
- `x = g(d, theta)` (sonar-to-position calibration map).

## 3) Parameters and Definitions

Primary physical terms:
- `m`: ball mass.
- `R`: ball radius.
- `I`: ball rotational inertia.
- `m_e = m + I/R^2` (effective rolling mass).
- `J`: beam inertia about pivot (excluding ball).
- `M_b, l_b, h_b`: beam/attachments COM terms.
- `b_x, F_cx`: ball viscous and Coulomb rolling resistance.
- `c_theta, tau_c_theta`: pivot viscous and Coulomb damping.
- `tau_act`: actuator torque about beam pivot.

Generalized dissipative terms:
- `Q_x = -b_x * x_dot - F_cx * sgn(x_dot)`
- `Q_theta = tau_act - c_theta * theta_dot - tau_c_theta * sgn(theta_dot)`

## 4) Full Nonlinear Coupled Plant

The model solves `[x_ddot, theta_ddot]` from:

`m_e * x_ddot + m_e * R * theta_ddot = m * x * theta_dot^2 + m * g * sin(theta) + Q_x`

`m_e * R * x_ddot + (J + m_e * R^2 + m * x^2) * theta_ddot =`
`Q_theta - 2*m*x*x_dot*theta_dot + m*g*(x*cos(theta) + R*sin(theta)) + M_b*g*(l_b*cos(theta) + h_b*sin(theta))`

Matrix form:

`[ m_e,                 m_e*R                 ] [x_ddot    ] = [rhs_x    ]`

`[ m_e*R, J + m_e*R^2 + m*x^2                ] [theta_ddot]   [rhs_theta]`

with:
- `rhs_x = m*x*theta_dot^2 + m*g*sin(theta) + Q_x`
- `rhs_theta = Q_theta - 2*m*x*x_dot*theta_dot + m*g*(x*cos(theta) + R*sin(theta)) + M_b*g*(l_b*cos(theta) + h_b*sin(theta))`

Small-angle sign check:
- around `x=0`, low rates, low coupling: `x_ddot ≈ (m/m_e) * g * theta`
- so `theta > 0` drives `x_ddot > 0` (toward `+x`).

## 5) Reduced Outer-Loop Model

For cascade design with fast inner angle loop:

`x_ddot ≈ alpha * g * theta - beta * x_dot`

where:
- `alpha = m/m_e`
- `beta = b_x/m_e`

Solid sphere case:
- `I = (2/5) m R^2`
- `alpha = 5/7`
- `x_ddot ≈ (5/7) * g * theta - beta * x_dot`

## 6) Actuator and End-to-End Command Path

Model command/state path:

1. Motor-angle command and motor lag:
- `phi_dot = (phi_cmd - phi) / phi_tau`

2. Calibration map:
- `theta_ref = f(phi)` (direction-aware when hysteresis is enabled).

3. Inner torque law:
- `tau_act = sat( Kp_theta*(theta_ref - theta) - Kd_theta*theta_dot, +/- tau_limit )`

4. Plant:
- Use full coupled equations in Section 4 to get `x_ddot, theta_ddot`.

## 7) Calibration Maps (Pluggable)

### 7.1 `theta = f(phi)` map

Supported modes:
- `linear`
- `poly`
- `lut`

Optional hysteresis:
- `hysteresis.enabled: true`
- two LUT branches: `up` and `down`
- branch selected by motion direction sign.

Inverse map `phi = f^{-1}(theta)` is available for:
- `linear`
- `lut`
- `poly` (iterative solve).

### 7.2 `x = g(d, theta)` map

Supported modes:
- `linear`
- `affine_theta`
- `lut1d` (with optional linear theta correction)
- `lut2d` (bilinear interpolation)

Inverse `d = g^{-1}(x,theta)` is defined for:
- `linear`
- `affine_theta`
- `lut1d`

## 8) Sensor-Side Model

Measured/estimated signals:
- `theta_hat = f(phi)`
- `x_hat = g(d_meas, theta_hat)`

In simulation exports:
- true states and mapped estimates are both logged for consistency checks.

## 9) Linearization Contract

Linearization script exports Jacobians around equilibrium:
- state order: `[x, x_dot, theta, theta_dot, phi]`
- input order: `[phi_cmd]`

File:
- `model/first_principles/linearized_model.json`

## 10) YAML Parameter Contract

Authoritative parameter file:
- `model/first_principles/params_template.yaml`

Key groups:
- `plant`: masses, inertia, gravity, damping/friction, beam COM.
- `limits`: `theta_cmd_deg`, `step_rate_sps`.
- `actuator`: `phi_tau_s`, torque law and limit.
- `calibration.theta_from_phi`: `f(phi)` mode and optional hysteresis LUTs.
- `calibration.x_from_d_theta`: `g(d,theta)` mode.
- `design`: cascade tuning targets.

Backward compatibility supported in code:
- `plant.rolling_factor` and `plant.viscous_damping_1ps` are accepted if present.

## 11) Practical Validation Checklist

Before controller deployment:
1. Confirm sign convention with `docs/calibration_signs.md`.
2. Calibrate `theta = f(phi)` with up/down sweeps if backlash exists.
3. Calibrate `x = g(d,theta)` at `theta≈0`, then optionally add theta correction.
4. Re-run:
   - `python model/first_principles/derive_nonlinear.py`
   - `python model/first_principles/linearize.py`
   - `python model/first_principles/design_cascade_pid.py`
   - `python model/first_principles/export_gains.py`

