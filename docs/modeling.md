# First-Principles Modeling (Canonical + Educational)

This is the canonical modeling reference for the repo.
If code and docs diverge, reconcile both in the same change set (do not let them drift).

Implementation references:
- `model/first_principles/first_principles_core.py`
- `model/first_principles/derive_nonlinear.py`
- `model/first_principles/linearize.py`
- `model/first_principles/design_cascade_pid.py`
- `model/first_principles/export_gains.py`

Primary parameter file for current hardware:
- `model/first_principles/params_measured_v1.yaml`

Companion measured-data worksheet:
- `docs/modeling_measured_calculations_v1.md`

## 1) System Definition

Measured quantities:
- Beam angle `theta` from AS5600 (mounted on beam pivot).
- Position range `d` from TFMini LiDAR (or HC-SR04 as backup).

Physical plant states:
- Ball position `x` (along beam) and velocity `x_dot`.
- Beam angle `theta` and rate `theta_dot`.

Sign convention (must stay aligned with firmware sign calibration):
- `x > 0`: chosen positive beam direction.
- `theta > 0`: tilt that drives ball toward `+x` at small angles.
- Positive control command should increase `theta`.

Therefore the end-to-end model always includes:
- `theta = f(phi)` (motor-to-beam mapping).
- `x = g(d, theta)` (sonar-to-position mapping).

## 2) Parameter Dictionary and YAML Mapping

### 2.1 Core physical symbols

- `m` [kg]: ball mass -> `plant.ball_mass_kg`
- `R` [m]: ball radius -> `plant.ball_radius_m`
- `I` [kg*m^2]: ball inertia
  - from `plant.ball_inertia_kgm2`, or
  - from `plant.ball_inertia_ratio * m * R^2`, or
  - from legacy `plant.rolling_factor`
- `m_e = m + I/R^2` [kg]: effective rolling mass
- `J` [kg*m^2]: beam+attachments inertia about pivot -> `plant.beam_inertia_kgm2`
- `M_b` [kg]: beam+attachments mass (excluding ball) -> `plant.beam_mass_kg`
- `l_b` [m]: beam+attachments COM along beam from pivot -> `plant.beam_com_l_m`
- `h_b` [m]: beam+attachments COM normal offset from pivot -> `plant.beam_com_h_m`
- `g` [m/s^2]: gravity -> `plant.gravity_mps2`

### 2.2 Dissipation/friction symbols

- `b_x` [N*s/m]: rolling viscous resistance -> `plant.rolling_damping_ns_per_m`
  - legacy fallback: `plant.viscous_damping_1ps * m_e`
- `F_cx` [N]: rolling Coulomb resistance -> `plant.rolling_coulomb_n`
- `c_theta` [N*m*s]: pivot viscous damping -> `plant.pivot_damping_nms`
- `tau_c_theta` [N*m]: pivot Coulomb friction -> `plant.pivot_coulomb_nm`

### 2.3 Actuator and map parameters

- `phi_tau` [s]: first-order motor-angle lag -> `actuator.phi_tau_s` (used in nonlinear simulation only)
- `steps_per_rev`: microsteps per motor revolution -> `actuator.stepper_steps_per_rev` (default 3200 = 200 full-steps × 16 µsteps)
- `linkage_ratio` [–]: crank-rocker ratio `r_crank / L_arm` -> `actuator.linkage_ratio` (beam angle per motor angle)
- `K_step_motor` [rad/step]: `2π / steps_per_rev`
- `K_beam` [rad_beam/step]: `K_step_motor × linkage_ratio` — the inner-loop plant gain
- `tau_limit` [N*m]: actuator saturation -> `actuator.torque_limit_nm`
- `f(phi)` configuration -> `calibration.theta_from_phi`
- `g(d,theta)` configuration -> `calibration.x_from_d_theta`

### 2.4 Which values are measured vs estimated vs identified

Measured now (from hardware):
- `m`, `R`, `M_b`, `l_b`, geometry notes in `params_measured_v1.yaml`.

Estimated currently:
- `J` (initial estimate).
- `h_b` set to `0` placeholder.

To be identified experimentally:
- `b_x`, `F_cx`, `c_theta`, `tau_c_theta`
- full calibrated `f(phi)` and `g(d,theta)` maps.

## 3) First-Principles Plant Derivation

Define generalized dissipative terms:
- `Q_x = -b_x*x_dot - F_cx*sgn(x_dot)`
- `Q_theta = tau_act - c_theta*theta_dot - tau_c_theta*sgn(theta_dot)`

Define effective mass:
- `m_e = m + I/R^2`

Coupled equations used in code (`full_coupled_accels`):

`m_e*x_ddot + m_e*R*theta_ddot = m*x*theta_dot^2 + m*g*sin(theta) + Q_x`

`m_e*R*x_ddot + (J + m_e*R^2 + m*x^2)*theta_ddot =`
`Q_theta - 2*m*x*x_dot*theta_dot + m*g*(x*cos(theta) + R*sin(theta)) + M_b*g*(l_b*cos(theta) + h_b*sin(theta))`

Matrix form:

`[m_e, m_e*R; m_e*R, J + m_e*R^2 + m*x^2] * [x_ddot, theta_ddot]^T = [rhs_x, rhs_theta]^T`

where:
- `rhs_x = m*x*theta_dot^2 + m*g*sin(theta) + Q_x`
- `rhs_theta = Q_theta - 2*m*x*x_dot*theta_dot + m*g*(x*cos(theta)+R*sin(theta)) + M_b*g*(l_b*cos(theta)+h_b*sin(theta))`

Small-angle/slow-motion reduction (outer-loop model basis):
- `x_ddot ~= alpha*g*theta - beta*x_dot`
- `alpha = m/m_e`
- `beta = b_x/m_e`

Sign check:
- `theta > 0` implies `x_ddot > 0` at equilibrium neighborhood.

## 4) Measurement Maps and Inverse Availability

Implemented in `first_principles_core.py`:

### 4.1 Beam map `theta = f(phi)`

Modes:
- `linear`
- `poly`
- `lut`

Optional hysteresis:
- `hysteresis.enabled = true`
- branch LUTs: `up`, `down`
- selected by motion direction.

Inverse `phi = f^{-1}(theta)`:
- supported for `linear`, `lut`, `poly` (iterative Newton solve).

### 4.2 Sonar map `x = g(d,theta)`

Modes:
- `linear`
- `affine_theta`
- `lut1d`
- `lut2d` (forward map only for inverse).

Inverse `d = g^{-1}(x,theta)`:
- supported for `linear`, `affine_theta`, `lut1d`
- not implemented for `lut2d`.

## 5) Actuator Model

### 5.1 Physical actuator chain

Motor: NEMA 17 (17HS4401-D), 0.40 N·m holding torque, 1.7 A rated.  
Driver: TMC2209 in STEP/DIR mode, 1/16 microstepping (hardware DIP), 12 V supply.  
Linkage: Crank-rocker — motor shaft → lower arm (`r_crank = 47.65 mm`) → bolt joint → upper arm → beam pivot (`L_arm = 323.08 mm`).  
Linkage ratio: `linkage_ratio = r_crank / L_arm = 0.1475`.

### 5.2 Inner-loop plant model (for PID gain design)

The firmware inner loop measures **beam angle** `theta` (AS5600) and outputs
**motor step rate** `u` (microsteps/s).  The stepper is treated as a pure
integrator — each microstep rotates the motor by a fixed angle, and the
motor angle maps to beam angle through the linkage ratio:

```
θ_beam(s) / u(s) = K_beam / s
```

where:
- `K_step_motor = 2π / steps_per_rev = 2π / 3200 ≈ 0.001963 rad_motor/step`
- `K_beam = K_step_motor × linkage_ratio = 0.001963 × 0.1475 ≈ 0.000289 rad_beam/step`

This is a single integrator, so a PI controller yields a second-order closed loop.

### 5.3 Nonlinear simulation torque model (separate)

The nonlinear time-domain simulation in `first_principles_core.py` still uses
a first-order motor-angle lag and torque law for numerical integration:
1. `phi_dot = (phi_cmd - phi) / phi_tau`
2. `theta_ref = f(phi)`
3. `tau_act = sat(Kp_theta*(theta_ref - theta) - Kd_theta*theta_dot, ± tau_limit)`
4. Coupled equations for `x_ddot`, `theta_ddot`.

This captures transient dynamics beyond the integrator approximation and is useful
for full-system simulation but is **not** the model used for PID gain synthesis.

## 6) Controller Design Derivation (matches `design_cascade_pid.py`)

### 6.1 Outer reduced plant for gain synthesis

Use reduced form:
- `x_ddot = k*theta` with `k = alpha*g`

### 6.2 Inner gain synthesis (stepper integrator plant)

Inner design treats the stepper + linkage as a pure integrator:
- Plant: `theta_beam(s) / u(s) = K_beam / s`
- `K_beam = (2π / steps_per_rev) × linkage_ratio`

PI controller `C(s) = Kp_i + Ki_i/s` on integrator plant gives closed-loop:
- `s^2 + Kp_i·K_beam·s + Ki_i·K_beam = 0`

Matched to standard second-order `s^2 + 2·ζ·ωn·s + ωn² = 0`:

Code formulas (in `design_cascade_pid.py`):
- `K_beam = (2π / stepper_steps_per_rev) × linkage_ratio`
- `Kp_i = max(0.05, 2·ζ_i·ωn_i / K_beam)`
- `Ki_i = max(0.05, ωn_i² / K_beam)`
- `Kd_i = 0`

### 6.3 Outer PID pole matching

Target 3rd-order polynomial via damping/zeta and extra pole:
- `wn_o = 2*pi*outer_bw_hz`
- `p3 = extra_pole_factor*wn_o`
- `a2 = 2*zeta_o*wn_o + p3`
- `a1 = wn_o^2 + 2*zeta_o*wn_o*p3`
- `a0 = wn_o^2*p3`

For plant `k/s^2` with PID:
- `s^3 + k*Kd_o*s^2 + k*Kp_o*s + k*Ki_o = 0`

Hence:
- `Kp_o = a1/k`
- `Kd_o = a2/k`
- `Ki_o = a0/k`

### 6.4 Anti-windup/integral limit formulas

Inner:
- `inner_i_lim = step_rate_limit / |Ki_i|`

Outer:
- `outer_i_lim = theta_cmd_limit_rad / |Ki_o|`

## 7) Worked Numeric Example (from `params_measured_v1.yaml`)

Parameter file:
- `model/first_principles/params_measured_v1.yaml`

### 7.1 Given plant parameters

- `m = 0.0028 kg`
- `R = 0.020035 m`
- `ball_inertia_ratio = 0.6667` (thin-shell hollow sphere)
- `J = 0.00439 kg·m²`
- `M_b = 0.1392 kg`
- `l_b = 0.145042 m`
- `b_x = 0.0125 N·s/m`
- `g = 9.81 m/s²`

### 7.2 Actuator parameters

- `stepper_steps_per_rev = 3200` (200 full-steps × 16 µsteps)
- `linkage_ratio = 0.1475` (r_crank / L_arm = 47.65 mm / 323.08 mm)
- `step_rate_limit = 5000 sps`
- `theta_cmd_limit = 8° = 0.1396 rad`

### 7.3 Derived quantities

Ball dynamics:
- `I = (2/3)·m·R² = 7.49e-7 kg·m²`
- `m_e = m + I/R² = 0.00467 kg`
- `α = m/m_e = 0.6`
- `β = b_x/m_e = 2.679 s⁻¹`
- `k = α·g = 5.886 m/s²/rad`

Inner plant gain:
- `K_step_motor = 2π / 3200 = 0.001963 rad_motor/step`
- `K_beam = K_step_motor × 0.1475 = 0.000289 rad_beam/step`

### 7.4 Design bandwidth targets

- Inner: `ωn_i = 2π × 4.0 = 25.13 rad/s`, `ζ_i = 0.9`
- Outer: `ωn_o = 2π × 1.1 = 6.912 rad/s`, `ζ_o = 0.85`, `extra_pole_factor = 4.0`

### 7.5 Gain computation

Inner PI (on integrator plant `K_beam/s`):
- `Kp_i = 2·ζ_i·ωn_i / K_beam = 2 × 0.9 × 25.13 / 0.000289 = 156,203`
- `Ki_i = ωn_i² / K_beam = 25.13² / 0.000289 = 2,181,011`
- `i_lim = step_rate_limit / Ki_i = 5000 / 2,181,011 = ±0.00229 rad·s`
- `out = ±5000 sps`

Outer PID (on double-integrator plant `k/s²`):
- `p3 = 4.0 × 6.912 = 27.65`
- `a2 = 2 × 0.85 × 6.912 + 27.65 = 39.40`
- `a1 = 6.912² + 2 × 0.85 × 6.912 × 27.65 = 372.38`
- `a0 = 6.912² × 27.65 = 1321.0`
- `Kp_o = 372.38 / 5.886 = 63.30`
- `Kd_o = 39.40 / 5.886 = 6.693`
- `Ki_o = 1321.0 / 5.886 = 224.4`
- `out = ±0.1396 rad (±8°)`
- `i_lim = 0.1396 / 224.4 = ±0.000622 m·s`

### 7.6 Current gain snapshot (`controller_gains.h`)

- Inner:
  - `kp = 156,203.39`
  - `ki = 2,181,010.76`
  - `kd = 0.0`
  - `out = ±5000 step/s`
  - `i_lim = ±0.00229`
- Outer:
  - `kp = 63.30`
  - `ki = 224.37`
  - `kd = 6.693`
  - `out = ±0.1396 rad`
  - `i_lim = ±0.000622`

> **Warning:** These are snapshot values and must be regenerated whenever params change.
> Run `design_cascade_pid.py` then `export_gains.py` to update.

## 8) Reproducible Commands

From repo root:

```bash
python model/first_principles/derive_nonlinear.py --params model/first_principles/params_measured_v1.yaml
python model/first_principles/linearize.py --params model/first_principles/params_measured_v1.yaml
python model/first_principles/design_cascade_pid.py --params model/first_principles/params_measured_v1.yaml
python model/first_principles/export_gains.py
```

Generated artifacts:
- `model/first_principles/open_loop_nonlinear.csv`
- `model/first_principles/open_loop_nonlinear.png`
- `model/first_principles/linearized_model.json`
- `model/first_principles/controller_initial_gains.json`
- `firmware/include/generated/controller_gains.h` (consumed by firmware controller)

## 9) Assumptions, Limits, and Pending Identification

Current assumptions/defaults in measured file:
- `beam_com_h_m = 0.0` placeholder
- damping/friction values are initial placeholders pending identification
- map defaults are currently linear (`theta_from_phi`, `x_from_d_theta`)

Post-assembly tasks to finalize model quality:
1. Identify `b_x`, `F_cx`, `c_theta`, `tau_c_theta` from hardware data.
2. Calibrate `theta = f(phi)` with up/down sweeps (hysteresis/backlash aware).
3. Calibrate `x = g(d,theta)` using marked positions (and optional theta dependence).
4. Replace estimated `J` with CAD inertia export or experimental identification.

## 10) Practical Validation Checklist

1. Keep sign convention aligned with `docs/calibration_signs.md`.
2. Re-run the four commands in Section 8 after each significant calibration update.
3. Confirm consistency across:
- `params_measured_v1.yaml`
- `controller_initial_gains.json`
- `firmware/include/generated/controller_gains.h`
