# Measured Parameter Calculations (v1)

This sheet converts your measured values to model parameters used by:
- `model/first_principles/params_measured_v1.yaml`
- `model/first_principles/first_principles_core.py`

## 1) Raw Inputs (from build measurements)

- Ball mass: `2.8 g`
- Ball diameter: `40.07 mm`
- Ball type: hollow thin-shell sphere
- Beam mass (without sensor mount): `121.9 g`
- Beam COM (without sensor mount): `195.33 mm` from fulcrum end
- Beam COM (without sensor mount): `172.02 mm` from motor end
- Main pivot center from fulcrum end: `36.80 mm`
- Motor-side beam hole center from motor end: `7.47 mm`
- Ultrasonic mount+sensor mass: `17.3 g`
- Ultrasonic mount midpoint from fulcrum end: `86.8 mm`
- Motor shaft to beam motor-hole hypotenuse at `theta=0`: `33.5 mm`
- Additional linkage dimension provided: `47.65 mm`

## 2) SI Conversions

- `m_ball = 0.0028 kg`
- `R = 0.020035 m`
- `m_beam = 0.1219 kg`
- `m_sensor = 0.0173 kg`
- `x_beam_com = 0.19533 m` (from fulcrum end)
- `x_sensor = 0.0868 m` (from fulcrum end)
- `x_pivot = 0.0368 m` (from fulcrum end)

## 3) Ball Dynamics Terms

Thin-shell hollow sphere inertia:
- `I = (2/3) m R^2 = 7.492822866666667e-07 kg·m^2`

Effective mass:
- `m_e = m + I/R^2 = (5/3) m = 0.004666666666666666 kg`

Outer-loop gravity factor:
- `alpha = m/m_e = 0.6`

Useful constants:
- `m g = 0.027468 N`
- `m_e R = 9.349666666666666e-05 kg·m`
- `m_e R^2 = 1.8732057166666665e-06 kg·m^2`

## 4) Beam + Sensor Aggregate COM

Aggregate moving mass excluding ball:
- `M_b = m_beam + m_sensor = 0.1392 kg`

Combined COM from fulcrum end:
- `x_com_total = (m_beam*x_beam_com + m_sensor*x_sensor)/M_b`
- `x_com_total = 0.18184171695 m`

Model COM parameter from pivot:
- `l_b = x_com_total - x_pivot = 0.14504171695 m`

Current vertical offset:
- `h_b = 0` used as temporary placeholder
- Must be measured/finalized later.

## 5) Geometry Cross-Checks

From COM pair:
- `beam_length = 195.33 mm + 172.02 mm = 367.35 mm`

Then implied pivot-to-motor-hole center distance:
- `c2c_measured = 367.35 - 36.80 - 7.47 = 323.08 mm`

CAD source-of-truth given:
- `pivot_to_motor_c2c = 300.00 mm`

Difference:
- `+23.08 mm` (measured-implied vs CAD)

Current locked model value:
- `P -> B = 323.08 mm = 0.32308 m`

Optional later cross-check:
- direct caliper check of pivot-hole center to motor-side beam-hole center after dry fit.

## 6) Beam Inertia About Pivot (`J`) Estimate

`J` is not directly measured yet. For now, a pre-assembly estimate was used.

Using two plausible beam lengths and slender-body approximation for beam body:
- Case A (`L=0.36735 m`): `J_est = 0.00447764 kg·m^2`
- Case B (`L=0.34427 m` from CAD c2c + edge offsets): `J_est = 0.00431080 kg·m^2`

Working estimate in params:
- `beam_inertia_kgm2 = 0.00439`

This is acceptable for initial simulation/tuning seed only.
For final controller design, replace with:
- CAD mass-properties inertia about pivot axis, or
- pendulum identification.

## 7) Resulting Pre-Assembly Parameter Set

Populated in:
- `model/first_principles/params_measured_v1.yaml`

Core finalized now:
- `ball_mass_kg`
- `ball_radius_m`
- `ball_inertia_ratio`
- `beam_mass_kg` (beam + sensor mount)
- `beam_com_l_m`
- initial `beam_inertia_kgm2` estimate

Still pending for true final model:
- `beam_com_h_m`
- damping/friction identification
- calibrated `theta=f(phi)` and `x=g(d,theta)` maps

## 8) Linkage Geometry (Crank-Rocker)

Motor shaft to upper linkage joint (crank arm length, measured with caliper):
- `r_crank = 47.65 mm`

Pivot-to-motor-hole center-to-center (from Section 5 above):
- `L_arm = 323.08 mm`

Linkage ratio (small-angle linear approximation):
- `linkage_ratio = r_crank / L_arm = 47.65 / 323.08 = 0.1475`

This ratio is the beam-angle per motor-angle scaling.
It appears in three places in the model:
- `actuator.linkage_ratio` in `params_measured_v1.yaml`
- `calibration.theta_from_phi.linear.gain_theta_per_phi` in `params_measured_v1.yaml`
- Inner-loop plant gain: `K_beam = (2π / steps_per_rev) × linkage_ratio`

Note: `33.5 mm` (motor_shaft_to_beam_motor_hole_hypotenuse_at_theta0_m) is the
straight-line distance from motor shaft to beam hole at θ=0.  The `47.65 mm`
is the actual crank arm length (motor shaft center to upper joint bolt center).

## 9) Motor, Driver, and Torque Budget

### 9.1 Motor and driver specs

- Motor: NEMA 17HS4401-D (dual shaft), 1.7 A/phase, 0.40 N·m holding torque
- Driver: Two Trees TMC2209 V2.0, STEP/DIR mode (no UART config)
- Microstepping: 1/16 (hardware DIP switches), 3200 µsteps/rev
- Supply: 12 V DC
- Vref: 1.2 V → I_rms ≈ 1.2 / 0.88 ≈ 1.36 A (80% of rated 1.7 A)

### 9.2 Torque budget at maximum tilt

At `θ_max = 8°`:

Gravity torque on beam assembly (referred to motor shaft):
- `τ_gravity_beam = M_b · g · l_b · sin(θ) / linkage_ratio`
- `= 0.1392 × 9.81 × 0.145 × sin(8°) / 0.1475`
- `≈ 0.19 N·m` at the motor shaft

Inertial torque for inner-loop bandwidth (worst case, J only):
- `τ_inertial = J · ωn_i² · θ_max / linkage_ratio`
- `= 0.00439 × 25.13² × 0.14 / 0.1475`
- `≈ 0.034 N·m` at the motor shaft (within steady-state, not accounting for peak transient)

Total worst-case: `≈ 0.22 N·m` (below 0.32 N·m available at 80% rated current)

### 9.3 Margin

Available torque at Vref 1.2 V: ~80% × 0.40 = 0.32 N·m.
Worst-case demand: 0.22 N·m.
Margin: ~45%. Adequate for 8° operation with safety factor.

At higher speeds (>1000 sps), stepper torque drops due to back-EMF and
inductance effects.  5000 sps ≈ 1.56 rev/s — still well within the useful
torque curve for a 17HS4401 at 12 V.
