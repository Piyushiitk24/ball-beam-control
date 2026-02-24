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
- Motor angle `phi` from AS5600.
- Sonar range `d` from HC-SR04.

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

- `phi_tau` [s]: first-order motor-angle lag -> `actuator.phi_tau_s`
- `Kp_theta`, `Kd_theta`: inner torque law gains
  - explicit: `actuator.theta_kp_nm_per_rad`, `actuator.theta_kd_nms_per_rad`
  - if absent, auto-derived from `design.inner_bw_hz`, `design.inner_zeta`
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

## 5) Actuator and Inner Torque Model

Command/actuation path:
1. Motor-angle lag:
   - `phi_dot = (phi_cmd - phi)/phi_tau`
2. Reference angle map:
   - `theta_ref = f(phi)`
3. Torque law:
   - `tau_act = sat(Kp_theta*(theta_ref - theta) - Kd_theta*theta_dot, +/- tau_limit)`
4. Plant dynamics:
   - solve coupled equations for `x_ddot`, `theta_ddot`.

Auto-derivation of inner torque gains (when not explicitly provided):
- `J_eq0 = J + m_e*R^2`
- `wn_i = 2*pi*inner_bw_hz`
- `Kp_theta = J_eq0*wn_i^2`
- `Kd_theta = 2*inner_zeta*wn_i*J_eq0`

## 6) Controller Design Derivation (matches `design_cascade_pid.py`)

### 6.1 Outer reduced plant for gain synthesis

Use reduced form:
- `x_ddot = k*theta` with `k = alpha*g`

### 6.2 Inner gain synthesis approximation

Inner design approximates actuator as first-order:
- `phi_dot = (phi_cmd - phi)/tau` where `tau = phi_tau`

PI structure gives characteristic form:
- `tau*s^2 + (1 + Kp_i)*s + Ki_i = 0`

Code formulas:
- `Kp_i = max(0.05, 2*zeta_i*wn_i*tau - 1)`
- `Ki_i = max(0.05, wn_i^2 * tau)`
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

Given:
- `m = 0.0028 kg`
- `R = 0.020035 m`
- `ball_inertia_ratio = 0.6666666666666666` (thin-shell hollow sphere)
- `J = 0.00439 kg*m^2`
- `M_b = 0.1392 kg`
- `l_b = 0.145041716954 m`
- `b_x = 0.0125 N*s/m`
- `phi_tau = 0.08 s`
- `g = 9.81 m/s^2`

Computed:
- `I = (2/3)*m*R^2 = 7.492822866666667e-07 kg*m^2`
- `m_e = m + I/R^2 = 0.004666666666666666 kg`
- `alpha = m/m_e = 0.6`
- `beta = b_x/m_e = 2.678571428571429 1/s`
- `k_theta_to_xddot = alpha*g = 5.886`

Current gain snapshot (`controller_initial_gains.json` / `controller_gains.h`):
- Inner:
  - `kp = 2.6191147369`
  - `ki = 50.5323745336`
  - `kd = 0.0`
  - `out = [-1800, 1800] step/s`
  - `i_lim = +/-35.6207286243`
- Outer:
  - `kp = 63.3022944869`
  - `ki = 224.3661801508`
  - `kd = 6.6930974985`
  - `out = +/-0.1396263402 rad`
  - `i_lim = +/-0.0006223145577`

Warning:
- These are snapshot values and must be regenerated whenever params change.

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
