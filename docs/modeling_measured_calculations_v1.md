# Measured Parameter Calculations (v1)

This sheet converts your measured values to model parameters used by:
- `model/first_principles/params_measured_v1.yaml`
- `model/first_principles/first_principles_core.py`

## 1) Raw Inputs (from build measurements)

- Ball mass: `9.6 g`
- Ball diameter: `27.6 mm`
- Ball type: solid sphere
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

- `m_ball = 0.0096 kg`
- `R = 0.0138 m`
- `m_beam = 0.1219 kg`
- `m_sensor = 0.0173 kg`
- `x_beam_com = 0.19533 m` (from fulcrum end)
- `x_sensor = 0.0868 m` (from fulcrum end)
- `x_pivot = 0.0368 m` (from fulcrum end)

## 3) Ball Dynamics Terms

Solid sphere inertia:
- `I = (2/5) m R^2 = 7.312896e-07 kg·m^2`

Effective mass:
- `m_e = m + I/R^2 = 1.4 m = 0.01344 kg`

Outer-loop gravity factor:
- `alpha = m/m_e = 0.7142857143` (`5/7`)

Useful constants:
- `m g = 0.094176 N`
- `m_e R = 1.85472e-04 kg·m`
- `m_e R^2 = 2.5595136e-06 kg·m^2`

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
- linkage geometry consistency (`33.5 mm` vs `47.65 mm` interpretation)
- damping/friction identification
- calibrated `theta=f(phi)` and `x=g(d,theta)` maps
