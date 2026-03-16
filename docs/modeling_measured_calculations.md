# Measured Parameter Calculations

This sheet converts your measured values to model parameters used by:
- `model/first_principles/params_measured.yaml`
- `model/first_principles/first_principles_core.py`

## 1) Raw Inputs (final hardware)

- Ball mass: `2.8 g`
- Ball diameter: `40.0 mm`
- Ball type: hollow thin-shell sphere
- Bare beam mass: `33.2 g`
- Beam length from pivot-side clevis hole to motor-side clevis hole: `263 mm`
- Mounted HC-SR04 + wires mass: `8.9 g`
- Total moving beam assembly mass: `42.1 g`
- Runner start from pivot-side clevis hole: `48 mm`
- Runner end clearance to motor-side clevis hole: `12-13 mm`
- Runner length: `198-200 mm`
- Nearest ball-to-sensor clearance: `30 mm`
- Runner inner width: `26.5 mm`
- Runner depth: `15 mm`
- Runner flat bottom width: `5 mm`
- Runner wall slant length: `15 mm`
- Neutral-pose motor-side pivot to fulcrum-side pivot spacing: `270 mm`

## 2) SI Conversions

- `m_ball = 0.0028 kg`
- `R = 0.020 m`
- `m_beam = 0.0332 kg`
- `m_sensor = 0.0089 kg`
- `M_b = m_beam + m_sensor = 0.0421 kg`
- `L = 0.263 m`
- `x_beam = L / 2 = 0.1315 m`
- `x_sensor = 0.018 m`
- `runner_start = 0.048 m`
- `runner_length_nominal = 0.199 m`

## 3) Ball Dynamics Terms

Thin-shell hollow sphere inertia:
- `I = (2/3) m R^2 = 7.466666666666668e-07 kg·m^2`

Effective mass:
- `m_e = m + I/R^2 = (5/3) m = 0.004666666666666666 kg`

Outer-loop gravity factor:
- `alpha = m/m_e = 0.6`

Useful constants:
- `m g = 0.027468 N`
- `m_e R = 9.349666666666666e-05 kg·m`
- `m_e R^2 = 1.8732057166666665e-06 kg·m^2`

## 4) Beam + Sensor Aggregate COM

Assumption used in the active measured model:
- treat the bare beam as a uniform rod between the two clevis-hole centers
- treat the mounted HC-SR04 + wires as a point mass
- use the sensor face longitudinal reference as `18 mm` from the pivot-side clevis hole

Combined COM from the pivot-side clevis hole:
- `x_com_total = (m_beam*x_beam + m_sensor*x_sensor) / M_b`
- `x_com_total = (0.0332*0.1315 + 0.0089*0.018) / 0.0421`
- `x_com_total = 0.10750593824 m`

Model COM parameter:
- `beam_com_l_m = 0.10750593824 m`

Current vertical offset:
- `beam_com_h_m = 0` is still a temporary placeholder
- Replace later with a direct measurement if needed.

## 5) Geometry Cross-Checks

Nominal runner span from direct statement:
- `runner_length_nominal = 199 mm`

Hole-offset cross-check:
- `263 mm - 48 mm - 12.5 mm = 202.5 mm`

Interpretation used in the active measured model:
- the directly stated runner span `198-200 mm` is treated as the source of truth
- `199 mm` is used as the nominal modeling value
- the `202.5 mm` hole-offset arithmetic is kept only as an approximate cross-check, not as the active span

Nominal center distance from the HC-SR04 face:
- `center_m = 30 mm + (199 mm / 2)`
- `center_m = 129.5 mm = 0.1295 m`

Important note:
- the `270 mm` neutral-pose motor-side-to-fulcrum-side pivot spacing is **not**
  used as the active model arm length
- the active first-principles beam length remains the measured `263 mm` clevis-hole span

## 6) Beam Inertia About Pivot (`J`) Estimate

`J` is still an estimate. The active measured model uses:
- bare beam as a slender rod about one end
- mounted HC-SR04 + wires as a point mass at `0.018 m`

Beam-body term:
- `J_beam = (1/3) * m_beam * L^2`
- `J_beam = (1/3) * 0.0332 * 0.263^2`
- `J_beam = 0.0007654702 kg·m^2`

Sensor/wires term:
- `J_sensor = m_sensor * x_sensor^2`
- `J_sensor = 0.0089 * 0.018^2`
- `J_sensor = 0.0000028836 kg·m^2`

Combined estimate used in params:
- `J = J_beam + J_sensor`
- `J = 0.0007683538666666666 kg·m^2`

This is acceptable for current Track A modeling only.
If needed later, replace with CAD mass properties or pendulum identification.

## 7) Resulting Measured Parameter Set

Populated in:
- `model/first_principles/params_measured.yaml`

Finalized for the current beam replacement:
- `ball_mass_kg`
- `ball_radius_m`
- `ball_inertia_ratio`
- `beam_mass_kg` (beam + mounted HC-SR04 + wires)
- `beam_com_l_m`
- estimated `beam_inertia_kgm2`
- `calibration.x_from_d_theta.linear.center_m`

Still approximate / pending direct measurement:
- sensor longitudinal reference from the pivot-side clevis hole
- combined COM
- inertia estimate
- `beam_com_h_m`
- damping/friction identification
- calibrated `theta=f(phi)` and `x=g(d,theta)` maps

## 8) Linkage Geometry (Crank-Rocker)

No new beam-per-motor ratio was provided with the beam swap.

Therefore the active measured parameter file intentionally keeps:
- `actuator.linkage_ratio`
- `calibration.theta_from_phi.linear.gain_theta_per_phi`

unchanged from the previously working runtime seed values.

The new `270 mm` neutral-pose assembly spacing is recorded in the geometry notes
only and is not converted into a new linkage ratio without a direct beam-angle
per motor-angle measurement.
