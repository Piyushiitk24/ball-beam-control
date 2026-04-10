# Ball-and-Beam Mathematical Modeling and Control Design

This is the single canonical modeling document for the ball-and-beam control system.
All first-principles derivations, hardware parameters, and controller designs live here.

---

## 1. Definitions

### 1.1 Physical Setup

The system consists of a beam pivoted at one end and actuated at the other by a stepper
motor through a crank-rocker linkage. A hollow ping-pong ball rolls freely along a
V-groove runner mounted on top of the beam. A Sharp GP2Y0A41SK0F IR distance sensor
mounted at the pivot end of the beam measures the ball's distance from the sensor face.
Because the Sharp has a minimum usable detection range of about 4 cm, a tape stop
blocks the near end of the runner and shortens the usable ball-travel window.

```text
  Sharp IR
  sensor
  [====]
  [====]  pivot                              motor crank
  [====]==O====[ tape stop ][ runner ]========O--( )
            ^           ~~~ball~~~            ^
            |                                 |
          fulcrum                         motor-side
          (fixed)                         clevis
```

The pivot-side clevis hole is the coordinate origin for all length measurements along
the beam.

### 1.2 Generalized Coordinates

The system has two generalized coordinates:

| Symbol | Description | Positive direction |
| -------- | ------------- | ------------------- |
| r | Ball position along beam from pivot (m) | Away from pivot (toward motor) |
| θ | Beam angle from horizontal (rad) | Motor side up |

The state vector of the full system is **q** = [r, θ]ᵀ.

### 1.3 Firmware Sign Convention

In the canonical control convention used by this project:

- Positive microstep count is intended to mean motor-side-up actuator motion (`θ > 0`).
- Positive `θ` makes gravity pull the ball toward the pivot, so `r` decreases.
- For the pivot-mounted Sharp sensor, the measured distance `d_sharp` increases toward
  the motor side. The controller coordinate is therefore
  `x_ctrl = d_sharp − center_d`, where `center_d = 0.0853 m` is the center reading.
  Therefore `x_ctrl > 0` means the ball is toward the motor side.
- In `model/first_principles/params_measured.yaml`, this center-distance value is
  stored in the legacy-named calibration field `center_m`.

---

## 2. Parameter Evaluation

### 2.1 Raw Measurements

All values were measured on the final assembled hardware.

| Quantity | Measured value | Notes |
| ---------- | --------------- | ------- |
| Ball mass | 2.8 g | Standard 40 mm ping-pong ball |
| Ball diameter | 40.0 mm | |
| Bare beam mass | 33.2 g | Aluminium extrusion, 263 mm span |
| Sharp IR sensor + wires mass | 5.3 g | Pivot-mounted |
| Total beam assembly mass | 38.5 g | Bare beam + sensor + wires |
| Beam length (clevis-to-clevis) | 263 mm | |
| Sharp body start from pivot-side clevis | 27.3 mm | Mount footprint start |
| Sharp body end from pivot-side clevis | 40.3 mm | Mount footprint end |
| Sharp sensing face from pivot-side clevis | 40.3 mm | Front face used for distance mapping |
| Sharp mass lump from pivot-side clevis | 33.8 mm | Body midpoint used for COM/inertia |
| Nearest ball–sensor clearance | 44.0 mm | Tape-limited near stop |
| Farthest ball–sensor clearance | 126.6 mm | Far usable stop |
| Usable sensing window | 82.6 mm | `126.6 − 44.0` mm |
| Nearest valid ball position from pivot-side clevis | 84.3 mm | `40.3 + 44.0` mm |
| Farthest valid ball position from pivot-side clevis | 166.9 mm | `40.3 + 126.6` mm |
| Effective runner end clearance to motor clevis | 96.1 mm | `263.0 − 166.9` mm |
| Stepper full-steps per revolution | 200 | NEMA 17 |
| Microstep divisor | 16 | TMC2209 driver |
| Rolling viscous damping | 0.0125 N·s/m | Pre-identification estimate |
| Pivot viscous damping | 0.006 N·m·s | Pre-identification estimate |

### 2.2 SI Symbol Table

The symbols used throughout the derivation are defined and evaluated here.

**Ball:**

| Symbol | Formula | Value | Unit |
| -------- | --------- | ------- | ------ |
| m | — | 0.0028 | kg |
| R | — | 0.020 | m |
| I | (2/3) m R² | 7.467 × 10⁻⁷ | kg·m² |

The factor 2/3 comes from the thin-shell hollow-sphere model. A thin spherical shell of
mass m and radius R has moment of inertia I = (2/3) m R² about any diameter.

Numerical substitution:

```text
I = (2/3) × 0.0028 × (0.020)²
  = (2/3) × 0.0028 × 0.0004
  = 7.4667 × 10⁻⁷  kg·m²
```

**Beam assembly:**

| Symbol | Formula | Value | Unit |
| -------- | --------- | ------- | ------ |
| M_b | m_beam + m_sensor | 0.0385 | kg |
| L | — | 0.263 | m |
| L_com | see §2.3 | 0.11805 | m |
| J | see §2.4 | 7.715 × 10⁻⁴ | kg·m² |

**Actuation and sensing:**

| Symbol | Formula | Value | Unit |
| -------- | --------- | ------- | ------ |
| N_steps | 200 × 16 | 3200 | steps/rev |
| k_step_rad | (2π / N_steps) × λ | 1.5068 × 10⁻⁴ | rad/step |
| λ | linkage_ratio | 0.0767406 | — |
| center_d | (44.0 mm + 126.6 mm) / 2 | 0.0853 | m |
| r₀ | x_sensor_face + center_d | 0.1256 | m |

**Physical constants:**

| Symbol | Value | Unit |
| -------- | ------- | ------ |
| g | 9.81 | m/s² |
| b_x | 0.0125 | N·s/m |
| b_θ | 0.006 | N·m·s |

### 2.3 Beam + Sensor Combined Centre of Mass

The beam assembly is modelled as a uniform slender rod (the aluminium extrusion) plus a
lumped point mass (the Sharp sensor and wires) at the sensor body midpoint. The sensing
face location is kept separate for ball-position mapping.

```text
x_com = (m_beam × x_beam_com + m_sensor × x_sensor_com) / M_b
```

where `x_beam_com = L / 2 = 0.263 / 2 = 0.1315 m` (uniform rod) and
`x_sensor_com = (0.0273 + 0.0403)/2 = 0.0338 m` (sensor body midpoint). The sensing
face used for distance mapping remains `x_sensor_face = 0.0403 m`.

Substitution:

```text
x_com = (0.0332 × 0.1315 + 0.0053 × 0.0338) / 0.0385
      = (0.0043658 + 0.00017914) / 0.0385
      = 0.00454494 / 0.0385
      = 0.11805 m
```

### 2.4 Beam Moment of Inertia About the Pivot

The pivot is at the pivot-side clevis hole. The beam inertia is estimated as a uniform
rod about one end plus a point mass for the sensor.

Uniform rod about one end:

```text
J_beam = (1/3) m_beam L²
       = (1/3) × 0.0332 × (0.263)²
       = (1/3) × 0.0332 × 0.069169
       = 7.6547 × 10⁻⁴  kg·m²
```

Sensor lumped at x_sensor_com = 0.0338 m:

```text
J_sensor = m_sensor × x_sensor_com²
         = 0.0053 × (0.0338)²
         = 0.0053 × 0.00114244
         = 6.055 × 10⁻⁶  kg·m²
```

Total beam inertia about pivot:

```text
J = J_beam + J_sensor
  = 7.6547 × 10⁻⁴ + 6.055 × 10⁻⁶
  = 7.715 × 10⁻⁴  kg·m²
```

### 2.5 Nominal Ball-Position Centre

The Sharp sensing face is at 40.3 mm from the pivot clevis. The tape-limited usable
ball window is measured directly from the sensor face as 44.0 mm to 126.6 mm, so the
usable window and center reading are:

```text
usable_window = 126.6 mm − 44.0 mm = 82.6 mm = 0.0826 m

center_d = (44.0 mm + 126.6 mm) / 2
         = 85.3 mm
         = 0.0853 m
```

The corresponding usable near, far, and center ball positions from the pivot are:

```text
r_near = 40.3 mm + 44.0 mm
       = 84.3 mm
       = 0.0843 m

r_far  = 40.3 mm + 126.6 mm
       = 166.9 mm
       = 0.1669 m

r₀     = 40.3 mm + 85.3 mm
       = 125.6 mm
       = 0.1256 m
```

This is the desired ball setpoint (center of the usable Sharp window).

---

## 3. Kinematics

The ball slides and rolls along the beam. The beam can rotate about the fixed pivot.
We express the ball's position in the inertial (lab) frame, then differentiate to get
velocity.

### 3.1 Ball Position

With the pivot at the origin, the ball's Cartesian coordinates in the inertial frame
are:

```text
x_I = r cos θ
y_I = r sin θ
```

where r is the distance along the beam and θ is the beam angle from horizontal.

### 3.2 Velocity Components

Differentiate with respect to time using the product and chain rules:

```text
ẋ_I = d/dt (r cos θ)
     = ṙ cos θ − r θ̇ sin θ

ẏ_I = d/dt (r sin θ)
     = ṙ sin θ + r θ̇ cos θ
```

The first term in each line is the rate of change of position along the beam; the
second term is the tangential velocity due to beam rotation.

### 3.3 Speed Squared

The square of the ball's speed in the inertial frame:

```text
v² = ẋ_I² + ẏ_I²
   = (ṙ cos θ − r θ̇ sin θ)² + (ṙ sin θ + r θ̇ cos θ)²
```

Expand:

```text
= ṙ² cos²θ − 2rṙθ̇ cosθ sinθ + r²θ̇² sin²θ
+ ṙ² sin²θ + 2rṙθ̇ sinθ cosθ + r²θ̇² cos²θ
```

The cross terms cancel. Group by cos²+sin²=1:

```text
v² = ṙ²(cos²θ + sin²θ) + r²θ̇²(sin²θ + cos²θ)
   = ṙ² + r²θ̇²
```

This is the standard result for polar coordinates: radial speed squared plus tangential
speed squared.

### 3.4 Rolling Constraint

The ball rolls without slipping along the beam. The contact point velocity relative to
the beam is the rate of change of r, i.e. ṙ. Therefore the ball's angular velocity
about its own centre is:

```text
ω_ball = ṙ / R
```

This constraint eliminates spin as an independent degree of freedom — the ball's
rotation is fully determined by its translational motion along the beam.

---

## 4. Energies

### 4.1 Ball Translational Kinetic Energy

The ball (mass m) moves through the inertial frame with speed v:

```text
T_trans = ½ m v²
        = ½ m (ṙ² + r²θ̇²)
```

### 4.2 Ball Rotational Kinetic Energy

Using the rolling constraint ω_ball = ṙ/R and I = (2/3) m R²:

```text
T_rot = ½ I ω_ball²
      = ½ × (2/3 m R²) × (ṙ/R)²
      = ½ × (2/3 m R²) × ṙ²/R²
      = (1/3) m ṙ²
```

Note that only radial motion (ṙ) contributes to ball spin — beam rotation θ̇ does not
spin the ball because θ̇ corresponds to the whole beam rotating, not the ball sliding
along it.

### 4.3 Beam Kinetic Energy

The beam assembly (mass M_b, inertia J about pivot) rotates at rate θ̇:

```text
T_beam = ½ J θ̇²
```

### 4.4 Total Kinetic Energy

Sum all three contributions:

```text
T = T_trans + T_rot + T_beam
  = ½ m (ṙ² + r²θ̇²) + (1/3) m ṙ² + ½ J θ̇²
  = ½ m ṙ² + (1/3) m ṙ² + ½ m r²θ̇² + ½ J θ̇²
  = (3/6 + 2/6) m ṙ² + ½(mr² + J)θ̇²
  = (5/6) m ṙ² + ½(mr² + J)θ̇²
```

The factor 5/6 in front of ṙ² is the hallmark of ball-and-beam dynamics: five-sixths
of the ball mass participates in radial kinetic energy because the remaining one-sixth
is stored in spin.

### 4.5 Potential Energy

Taking the pivot as the reference height, the potential energy has two contributions.

Ball at height r sin θ:

```text
V_ball = m g r sin θ
```

Beam centre of mass at height L_com sin θ:

```text
V_beam = M_b g L_com sin θ
```

Total:

```text
V = (m r + M_b L_com) g sin θ
```

### 4.6 Lagrangian

```text
ℒ = T − V

ℒ = (5/6) m ṙ² + ½(mr² + J)θ̇² − (m r + M_b L_com) g sin θ
```

---

## 5. Euler–Lagrange Equations

The Euler–Lagrange equations for each generalised coordinate qᵢ are:

```text
d/dt (∂ℒ/∂q̇ᵢ) − ∂ℒ/∂qᵢ = Qᵢ
```

where Qᵢ is the generalised force for coordinate i. Viscous damping is included as
generalised forces: Q_r = −b_x ṙ (rolling friction opposing sliding) and
Q_θ = τ − b_θ θ̇ (actuator torque minus pivot friction).

### 5.1 Equation in r (Ball Along Beam)

Compute the partial derivatives of ℒ:

```text
∂ℒ/∂ṙ = (5/3) m ṙ

d/dt(∂ℒ/∂ṙ) = (5/3) m r̈

∂ℒ/∂r = m r θ̇² − m g sin θ
```

Applying the Euler–Lagrange equation with Q_r = −b_x ṙ:

```text
(5/3) m r̈ − m r θ̇² + m g sin θ = −b_x ṙ
```

Rearranged into standard form:

```text
(5/3) m r̈ + b_x ṙ − m r θ̇² + m g sin θ = 0          ... (EOM-r)
```

Physical interpretation:

- (5/3) m r̈ — effective inertia (translational + rolling)
- b_x ṙ — viscous rolling damping
- −m r θ̇² — centrifugal force due to beam rotation
- m g sin θ — gravitational component along the beam

### 5.2 Equation in θ (Beam Rotation)

Compute the partial derivatives of ℒ:

```text
∂ℒ/∂θ̇ = (mr² + J) θ̇

d/dt(∂ℒ/∂θ̇) = (mr² + J) θ̈ + 2mr ṙ θ̇

∂ℒ/∂θ = −(m r + M_b L_com) g cos θ
```

Applying the Euler–Lagrange equation with Q_θ = τ − b_θ θ̇:

```text
(mr² + J) θ̈ + 2mr ṙ θ̇ + (mr + M_b L_com) g cos θ + b_θ θ̇ = τ
```

Rearranged:

```text
(J + mr²) θ̈ + 2mr ṙ θ̇ + b_θ θ̇ + (mr + M_b L_com) g cos θ = τ   ... (EOM-θ)
```

Physical interpretation:

- (J + mr²) θ̈ — total rotational inertia (beam + ball)
- 2mr ṙ θ̇ — Coriolis coupling between ball sliding and beam rotating
- b_θ θ̇ — pivot viscous damping
- (mr + M_b L_com) g cos θ — gravitational restoring torque
- τ — motor torque applied at beam (through crank linkage)

---

## 6. Nonlinear Model

Equations (EOM-r) and (EOM-θ) form the complete nonlinear coupled ODE system. Written
explicitly in state-space form with state **z** = [r, ṙ, θ, θ̇]ᵀ:

**Ball equation** (solving EOM-r for r̈):

```text
r̈ = (m r θ̇²) / m_e  −  (m g / m_e) sin θ  −  (b_x / m_e) ṙ
```

where `m_e = (5/3) m` is the effective rolling mass.

**Beam equation** (solving EOM-θ for θ̈):

```text
θ̈ = [ τ − 2mr ṙ θ̇ − b_θ θ̇ − (mr + M_b L_com) g cos θ ]
     / (J + mr²)
```

These two second-order ODEs must be integrated simultaneously because r appears in the
beam equation and θ appears in the ball equation.

**Effective rolling mass:**

```text
m_e = (5/3) m = (5/3) × 0.0028 = 0.004667 kg
```

The factor 5/3 arises directly from the 2/3 inertia factor of a thin-shell sphere:

```text
m_e = m + I/R² = m + (2/3)mR²/R² = m(1 + 2/3) = (5/3) m
```

---

## 7. Linearization about Horizontal Equilibrium

The nonlinear model is linearized for control design. The equilibrium of interest is
the beam horizontal with the ball at rest at position r₀:

```text
Equilibrium:  r = r₀,  θ = 0,  ṙ = 0,  θ̇ = 0
```

For this to be an equilibrium the actuator must supply a torque that balances gravity:

```text
τ₀ = (m r₀ + M_b L_com) g
```

In the control architecture below, θ is commanded directly, so τ₀ is handled
implicitly by the actuator.

### 7.1 Perturbation Variables

Define small deviations from equilibrium:

```text
r  = r₀ + x      (x is the small displacement along beam)
θ  = 0 + δ       (δ is the small beam angle)
ṙ  = ẋ
θ̇  = δ̇
```

We substitute into (EOM-r) and (EOM-θ) and drop all second-order terms (products of
small quantities: xδ, δ², ẋδ, xδ̇, etc.).

### 7.2 Linearized Ball Equation

Substitute into EOM-r:

```text
(5/3) m ẍ + b_x ẋ − m(r₀ + x)(δ̇)² + m g sinδ = 0
```

Second-order terms: the centrifugal term m(r₀+x)δ̇² is second-order (product of δ̇²)
and drops. Apply sinδ ≈ δ for small δ:

```text
(5/3) m ẍ + b_x ẋ + m g δ = 0
```

Divide through by m_e = (5/3) m:

```text
ẍ + β ẋ = − α g δ      ... (LBALL)
```

where:

```text
α = m / m_e  =  m / (5/3 m)  =  3/5  = 0.6            (dimensionless)
β = b_x / m_e  =  0.0125 / 0.004667  =  2.6786  s⁻¹
```

**Sign note:** With the positive-θ convention (motor side up), a positive `δ` tilts the
beam so that gravity pulls the ball toward the pivot (decreasing `r`). Any control
coordinate that defines positive position toward the motor side therefore needs the
actuation mapping to preserve the intended logical meaning of "positive command =
motor side up" rather than silently flipping PID or sensor signs. The derivation here
tracks the physics; sign bookkeeping is done in the unit-conversion step (§9).

### 7.3 Linearized Beam Equation

Substitute into EOM-θ with cosδ ≈ 1 and ṙ δ̇ ≈ 0:

```text
(J + m r₀²) δ̈ + b_θ δ̇ + (m r₀ + M_b L_com) g δ = τ − τ₀
```

The archived ideal-angle PID design omitted this beam equation. The active staged
controller reintroduces the beam dynamics through the second-order inner-loop actuator
model used in §9.4.

---

## 8. Numerical Specialization

All hardware values are now substituted. Every step is shown explicitly.

### 8.1 Effective Rolling Mass and Gravity Coefficients

```text
m_e = (5/3) × 0.0028
    = 0.0046667 kg

α   = 0.0028 / 0.0046667
    = 0.600000                          (exact, = 3/5)

β   = 0.0125 / 0.0046667
    = 2.678571  s⁻¹

α g = 0.600000 × 9.81
    = 5.886000  m/s²
```

### 8.2 Linearized Ball Plant (SI units)

From (LBALL):

```text
ẍ + 2.6786 ẋ = −5.886 δ          [x in m, δ in rad]
```

Laplace transform (zero initial conditions):

```text
s² X(s) + 2.6786 s X(s) = −5.886 Θ(s)

X(s) / Θ(s) = −5.886 / (s² + 2.6786 s)
             = −5.886 / (s (s + 2.6786))   [m/rad]
```

This is an integrator cascaded with a first-order lag (pole at s = −β).

### 8.3 Beam Inertia at Nominal Operating Point

At the design equilibrium r₀ = 0.1256 m (centre of the usable Sharp window):

```text
J_total(r₀) = J + m r₀²
             = 7.715 × 10⁻⁴ + 0.0028 × (0.1256)²
             = 7.715 × 10⁻⁴ + 0.0028 × 0.015775
             = 7.715 × 10⁻⁴ + 4.417 × 10⁻⁵
             = 8.157 × 10⁻⁴  kg·m²
```

### 8.4 Gravitational Restoring Torque Coefficient (Linearized)

```text
(m r₀ + M_b L_com) g = (0.0028 × 0.1256 + 0.0385 × 0.11805) × 9.81
                      = (0.00035168 + 0.00454494) × 9.81
                      = 0.00489662 × 9.81
                      = 0.04804  N·m/rad
```

The linearized beam equation at r₀ (ideal actuator, no damping) has natural frequency:

```text
ω_beam = √(0.04804 / 8.157 × 10⁻⁴)
        = √(58.89)
        = 7.67  rad/s  (1.22 Hz)
```

This is well above the ball control bandwidth (0.175 Hz), which justifies treating the
beam as an ideal angle actuator for ball position control.

---

## 9. Active Controller Design: Staged Sharp + AS5600 Cascade

The active controller architecture is the staged design implemented by the Sharp +
AS5600 sketch:

- `M0`: telemetry/state readout only
- `M1`: inner beam-angle hold using AS5600 only
- `M2`: outer ball-position cascade using `x`, `ẋ`, `θ`, `θ̇`, and an integral state

The first-principles source for the controller constants is
`model/first_principles/design_staged_controller.py`, which reads
`model/first_principles/params_measured.yaml` and generates:

- `model/first_principles/staged_controller_constants.json`
- `firmware/include/generated/staged_controller_constants.h`

ADC averaging counts, EMA smoothing factors, telemetry cadence, and step pulse timing
remain implementation settings rather than first-principles control outputs.

### 9.1 Runtime States, Units, and Sign Convention

The staged controller uses the exact runtime state definitions from the sketch:

```text
x_cm            = d_sharp_cm − D_SETPOINT_DEFAULT_CM
x_dot_cm_s      = d/dt (x_cm)
theta_rel_deg   = theta_cal_deg − theta_balance_deg
theta_dot_deg_s = d/dt (theta_rel_deg)
xi_cm_s         = ∫ x_cm dt
```

with command:

```text
theta_cmd_rel_deg
```

The physical sign convention remains the canonical one from §1.3:

- `x_cm > 0`: ball is toward the motor side
- `theta_rel_deg > 0`: motor side of the beam is up

The exact outer-loop command law used by the staged controller is

```text
theta_cmd_rel_deg
  = clamp(
      OUTER_SIGN_DEFAULT
      × (OUTER_KX_DEFAULT x_cm
       + OUTER_KV_DEFAULT x_dot_cm_s
       + OUTER_KT_DEFAULT theta_rel_deg
       + OUTER_KW_DEFAULT theta_dot_deg_s
       + OUTER_KI_DEFAULT xi_cm_s),
      ±THETA_CMD_LIMIT_DEFAULT_DEG)
```

with the sign audit constants:

```text
DIR_SIGN         = -1
INNER_STEP_SIGN  = -1
OUTER_SIGN_DEFAULT = -1
```

These are kept explicit so the wiring/sign bookkeeping is never hidden inside the gain
magnitudes.

### 9.2 Calibration-Backed Sensor and Angle Mapping

The staged controller uses the measured Sharp fit

```text
d_sharp_cm = 12.25 / V_sharp − 0.62
```

with the validated usable window

```text
D_MIN_CM = 4.40
D_MAX_CM = 12.66
```

so the centered default setpoint is

```text
D_SETPOINT_MIDPOINT_CM = (4.40 + 12.66) / 2 = 8.53 cm
D_SETPOINT_DEFAULT_CM  = 8.53 + 0.00 = 8.53 cm
```

The AS5600 calibration used by the controller is

```text
theta_cal_deg = 0.07666806 × theta_as5600_deg − 23.28443907
```

with the safe calibrated-angle clamp inputs

```text
THETA_CAL_MIN_DEG         = -0.70
THETA_CAL_MAX_DEG         =  3.10
THETA_CAL_MARGIN_DEG      =  0.10
THETA_CAL_EXTRAPOLATE_DEG =  0.30
```

The soft runtime clamp range implied by these values is

```text
theta_soft_min = -0.70 + 0.10 − 0.30 = -0.90 deg
theta_soft_max =  3.10 − 0.10 + 0.30 =  3.30 deg
usable_span    = 3.30 − (-0.90)      =  4.20 deg
```

The design envelope is the tighter of the AS5600 safe span and the measured-model
small-angle envelope `theta_cmd_deg = 4.0 deg`, so:

```text
theta_envelope = min(4.20, 4.00) = 4.00 deg
THETA_CMD_LIMIT_DEFAULT_DEG = 0.5 × 4.00 = 2.00 deg
```

The angle-to-step conversion used by the sketch is

```text
STEPS_PER_BEAM_DEG
  = (3200 / 360) / 0.07666806
  = 115.9399219 steps/deg
```

### 9.3 M1 Inner Loop: Discrete Beam-Angle Servo

Ignoring deadband and saturation, the inner loop uses the exact sketch structure

```text
theta[k+1] = theta[k] + delta_steps[k] / STEPS_PER_BEAM_DEG

delta_steps[k]
  = INNER_STEP_SIGN × STEPS_PER_BEAM_DEG
    × (INNER_KP_THETA e_theta[k] − INNER_KD_THETA theta_dot[k])
```

where `e_theta[k] = theta_cmd[k] − theta[k]`.

For the first pass we deliberately choose

```text
INNER_KD_THETA = 0
```

and place the nominal inner closed-loop pole at

```text
z_i = 0.30
```

With the discrete-time error dynamics

```text
theta[k+1] = (1 − INNER_KP_THETA) theta[k] + INNER_KP_THETA theta_cmd[k]
```

this gives

```text
INNER_KP_THETA = 1 − z_i = 0.70
```

The equivalent continuous bandwidth is

```text
omega_i = −ln(0.30) / 0.04 = 30.0993 rad/s
f_i     = omega_i / (2π)   = 4.79046 Hz
```

The deadband is derived from the larger of:

```text
beam_deg_per_step      = 1 / 115.9399219 = 0.00862516 deg
as5600_lsb_beam_deg    = 0.07666806 × (360 / 4096)
                       = 0.00673840 deg
dominant_quantization  = 0.00862516 deg
```

Using a `5×` quantization guard:

```text
INNER_THETA_DEADBAND_DEG
  = 5 × 0.00862516
  = 0.0431258 deg
```

and the corresponding finite-difference rate deadband is

```text
INNER_THETA_RATE_DEADBAND_DEG_S
  = 0.0431258 / (2 × 0.04)
  = 0.539072 deg/s
```

The per-sample step limit is the smaller of the 40 ms step-rate budget and the command
envelope, with an added `0.5×` guard:

```text
step_budget_per_sample = 2000 × 0.04 = 80 steps
theta_limit_steps      = 2.00 × 115.9399219 = 231.88 steps

INNER_MAX_STEP_DELTA
  = floor(0.5 × min(80, 231.88))
  = 40 steps
```

### 9.4 M2 Outer Loop: Discrete LQI Design

Unlike the archived single-loop PID, the active staged controller keeps the inner beam
dynamics in the plant seen by the outer loop. In controller units (`cm`, `deg`, `s`),
the small-angle continuous-time model is

```text
x_dot     = v
v_dot     = -beta v - k_theta theta
theta_dot = omega
omega_dot = -omega_i^2 theta - 2 zeta_i omega_i omega + omega_i^2 theta_cmd
```

where

```text
beta    = 2.678571 s^-1
k_theta = alpha g (pi/180) 100
        = 0.6 × 9.81 × (pi/180) × 100
        = 10.273008 cm/s^2/deg
omega_i = 30.0993 rad/s
zeta_i  = 1.0
```

so

```text
A_c = [ 0      1        0         0 ]
      [ 0   -2.6786  -10.2730     0 ]
      [ 0      0        0         1 ]
      [ 0      0     -905.9691  -60.1986 ]

B_c = [ 0 ]
      [ 0 ]
      [ 0 ]
      [ 905.9691 ]
```

The generator discretizes this model at `DT = 0.04 s` with
`scipy.signal.cont2discrete`, then augments the integral state

```text
xi[k+1] = xi[k] + DT x[k]
```

so the LQI state is

```text
z[k] = [x_cm, x_dot_cm_s, theta_rel_deg, theta_dot_deg_s, xi_cm_s]^T
```

### 9.5 Normalized LQI Weights and Solved Gains

The controller uses the following normalization scales:

```text
x_scale        = 0.5 × (12.66 − 4.40) = 4.13 cm
theta_scale    = THETA_CMD_LIMIT_DEFAULT_DEG = 2.00 deg
x_dot_scale    = outer_bw × x_scale = 1.099557 × 4.13 = 4.54117 cm/s
theta_dot_scale= omega_i × theta_scale = 30.0993 × 2.00 = 60.1986 deg/s
xi_scale       = x_scale / outer_bw = 4.13 / 1.099557 = 3.75606 cm·s
```

with

```text
outer_bw = 2π × 0.175 = 1.099557 rad/s
```

The LQI weight matrices are

```text
Q = diag( 6/x_scale^2,
         12/x_dot_scale^2,
       0.03/theta_scale^2,
       0.03/theta_dot_scale^2,
       0.04/xi_scale^2 )

R = 24/theta_scale^2 = 6.0
```

The generator solves the discrete algebraic Riccati equation with
`scipy.linalg.solve_discrete_are` and produces

```text
K = [OUTER_KX_DEFAULT,
     OUTER_KV_DEFAULT,
     OUTER_KT_DEFAULT,
     OUTER_KW_DEFAULT,
     OUTER_KI_DEFAULT]

  = [-0.272649312,
     -0.180551998,
      0.125117750,
      0.002069559,
     -0.020917328]
```

Because the controller states stay in the physical sign basis (`theta_rel_deg > 0`
means motor side up), the solved LQI gains are negative on `x`, `x_dot`, and `xi`,
positive on `theta` and `theta_dot`, and are then multiplied by
`OUTER_SIGN_DEFAULT = -1` in the runtime command law. This is expected and preserves
the explicit sign audit instead of hiding it in ad-hoc coordinate flips.

The resulting augmented closed-loop discrete poles are

```text
z_cl = {0.28178, 0.32276, 0.84608, 0.97647, 0.99637}
```

so the nominal linearized staged controller is stable.

### 9.6 Model-Derived Guard, Capture, Recovery, and Trim Constants

The remaining sketch constants are derived from the same scales rather than being left
as unexplained literals.

The command and integral guards are

```text
OUTER_INTEGRAL_CLAMP_DEG = 0.09 × 2.00 = 0.18 deg
OUTER_X_DOT_LIMIT_CM_S   = 1.0 × x_dot_scale = 4.54117 cm/s
OUTER_THETA_DOT_LIMIT_DEG_S
  = 2 × outer_bw × theta_scale
  = 2 × 1.099557 × 2.00
  = 4.39823 deg/s
```

The center and capture bands use fixed fractions of the half-window and velocity scale:

```text
OUTER_CENTER_BAND_CM                  = 0.045 × 4.13 = 0.18585 cm
OUTER_CENTER_BAND_X_DOT_CM_S          = 0.08  × 4.54117 = 0.36329 cm/s
OUTER_INTEGRAL_CAPTURE_CM             = 0.11  × 4.13 = 0.45430 cm
OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S     = 0.067 × 4.54117 = 0.30426 cm/s
```

The recovery thresholds are

```text
RECOVERY_ENTER_X_CM                   = 0.29 × 4.13 = 1.19770 cm
RECOVERY_ENTER_X_DOT_CM_S             = 0.08 × 4.54117 = 0.36329 cm/s
RECOVERY_ENTER_COUNT                  = ceil(0.16 / 0.04) = 4
RECOVERY_EXIT_X_CM                    = 0.06 × 4.13 = 0.24780 cm
RECOVERY_EXIT_HANDOFF_X_CM            = 0.36 × 4.13 = 1.48680 cm
RECOVERY_EXIT_INWARD_X_DOT_CM_S       = 0.10 × 4.54117 = 0.45412 cm/s
RECOVERY_FLOOR_DEFAULT_DEG            = 0.35 × 2.00 = 0.70 deg
RECOVERY_FLOOR_MAX_DEG                = 0.55 × 2.00 = 1.10 deg
RECOVERY_FLOOR_GAIN_DEG_PER_CM        = 0.375 × 2.00 / 4.13 = 0.181598 deg/cm
```

The zero-trim estimator thresholds are

```text
ZERO_TRIM_EST_X_CM                    = 0.06 × 4.13 = 0.24780 cm
ZERO_TRIM_EST_X_DOT_CM_S              = 0.055 × 4.54117 = 0.24976 cm/s
ZERO_TRIM_EST_THETA_TRACK_ERR_DEG     = 0.05 × 2.00 = 0.10 deg
ZERO_TRIM_EST_THETA_DOT_DEG_S         = 0.10 × 4.39823 = 0.439823 deg/s
ZERO_TRIM_EST_ALPHA                   = 1 − exp(-0.04 / 0.8)
                                      = 0.0487706
```

The three integral bleed factors remain

```text
OUTER_INTEGRAL_BLEED_OUTSIDE  = 1.0
OUTER_INTEGRAL_BLEED_RECOVERY = 1.0
OUTER_INTEGRAL_BLEED_CENTER   = 1.0
```

by design: the staged controller uses gating and clamping, not active bleed-down.

### 9.7 Exported Sketch Constants

The staged-controller generator exports the exact sketch-facing constants listed below.
`OUTER_SIGN_DEFAULT` is the modeled initial value for the sketch variable
`g_outer_sign`.

**Timing, calibration, and sign constants**

| Constant | Formula / source | Value | Unit |
| -------- | ---------------- | ----- | ---- |
| `LOOP_MS` | `1000 × DT` | `40` | ms |
| `DT` | staged inner-loop design input | `0.040000000` | s |
| `SHARP_FIT_K_V_CM` | measured Sharp fit | `12.250000000` | cm·V |
| `SHARP_FIT_OFFSET_CM` | measured Sharp fit | `-0.620000000` | cm |
| `SHARP_MIN_VALID_V` | measured Sharp validity floor | `0.080000000` | V |
| `D_MIN_CM` | measured near-stop reading | `4.400000000` | cm |
| `D_MAX_CM` | measured far-stop reading | `12.660000000` | cm |
| `D_SETPOINT_MIDPOINT_CM` | `(D_MIN_CM + D_MAX_CM)/2` | `8.530000000` | cm |
| `D_SETPOINT_TRIM_DEFAULT_CM` | design trim input | `0.000000000` | cm |
| `D_SETPOINT_DEFAULT_CM` | midpoint + trim | `8.530000000` | cm |
| `AS5600_RAW_TO_DEG` | `360 / 4096` | `0.087890625` | AS-deg/count |
| `THETA_SLOPE_DEG_PER_AS_DEG` | measured AS5600 beam-angle fit | `0.076668060` | beam-deg/AS-deg |
| `THETA_OFFSET_DEG` | measured AS5600 beam-angle fit | `-23.284439070` | deg |
| `THETA_CAL_MIN_DEG` | measured safe-range input | `-0.700000000` | deg |
| `THETA_CAL_MAX_DEG` | measured safe-range input | `3.100000000` | deg |
| `THETA_CAL_MARGIN_DEG` | measured safe-range input | `0.100000000` | deg |
| `THETA_CAL_EXTRAPOLATE_DEG` | measured safe-range input | `0.300000000` | deg |
| `STEPS_PER_BEAM_DEG` | `(3200/360) / THETA_SLOPE_DEG_PER_AS_DEG` | `115.939921903` | steps/deg |
| `DIR_SIGN` | wiring sign audit | `-1` | sign |
| `INNER_STEP_SIGN` | inner-loop sign audit | `-1` | sign |
| `OUTER_SIGN_DEFAULT` | outer-loop sign audit | `-1` | sign |

**Inner-loop and outer-state-feedback constants**

| Constant | Formula / source | Value | Unit |
| -------- | ---------------- | ----- | ---- |
| `INNER_KP_THETA` | `1 − z_i`, `z_i = 0.30` | `0.700000000` | — |
| `INNER_KD_THETA` | first-pass design choice | `0.000000000` | s |
| `INNER_THETA_DEADBAND_DEG` | `5 × max(beam_deg_per_step, as5600_lsb_beam_deg)` | `0.043125784` | deg |
| `INNER_THETA_RATE_DEADBAND_DEG_S` | `INNER_THETA_DEADBAND_DEG / (2 DT)` | `0.539072297` | deg/s |
| `INNER_MAX_STEP_DELTA` | `floor(0.5 × min(step_budget, theta_limit_steps))` | `40` | steps/sample |
| `THETA_CMD_LIMIT_DEFAULT_DEG` | `0.5 × min(theta_safe_span, theta_model_envelope)` | `2.000000000` | deg |
| `OUTER_KX_DEFAULT` | discrete LQI gain on `x_cm` | `-0.272649312` | deg/cm |
| `OUTER_KV_DEFAULT` | discrete LQI gain on `x_dot_cm_s` | `-0.180551998` | deg·s/cm |
| `OUTER_KT_DEFAULT` | discrete LQI gain on `theta_rel_deg` | `0.125117750` | — |
| `OUTER_KW_DEFAULT` | discrete LQI gain on `theta_dot_deg_s` | `0.002069559` | s |
| `OUTER_KI_DEFAULT` | discrete LQI gain on `xi_cm_s` | `-0.020917328` | deg/(cm·s) |
| `OUTER_INTEGRAL_CLAMP_DEG` | `0.09 × THETA_CMD_LIMIT_DEFAULT_DEG` | `0.180000000` | deg |
| `OUTER_X_DOT_LIMIT_CM_S` | `x_dot_scale` | `4.541172181` | cm/s |
| `OUTER_THETA_DOT_LIMIT_DEG_S` | `2 × outer_bw × theta_scale` | `4.398229715` | deg/s |
| `OUTER_GAIN_SCALE_MIN` | staged design input | `0.500000000` | — |
| `OUTER_GAIN_SCALE_MAX` | staged design input | `1.500000000` | — |

**Capture, recovery, and trim constants**

| Constant | Formula / source | Value | Unit |
| -------- | ---------------- | ----- | ---- |
| `OUTER_INTEGRAL_CAPTURE_CM` | `0.11 × x_scale` | `0.454300000` | cm |
| `OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S` | `0.067 × x_dot_scale` | `0.304258536` | cm/s |
| `OUTER_INTEGRAL_BLEED_OUTSIDE` | gating-only design choice | `1.000000000` | ratio/sample |
| `OUTER_INTEGRAL_BLEED_RECOVERY` | gating-only design choice | `1.000000000` | ratio/sample |
| `OUTER_INTEGRAL_BLEED_CENTER` | gating-only design choice | `1.000000000` | ratio/sample |
| `OUTER_CENTER_BAND_CM` | `0.045 × x_scale` | `0.185850000` | cm |
| `OUTER_CENTER_BAND_X_DOT_CM_S` | `0.08 × x_dot_scale` | `0.363293774` | cm/s |
| `RECOVERY_ENTER_X_CM` | `0.29 × x_scale` | `1.197700000` | cm |
| `RECOVERY_ENTER_X_DOT_CM_S` | `0.08 × x_dot_scale` | `0.363293774` | cm/s |
| `RECOVERY_ENTER_COUNT` | `ceil(0.16 / DT)` | `4` | samples |
| `RECOVERY_EXIT_X_CM` | `0.06 × x_scale` | `0.247800000` | cm |
| `RECOVERY_EXIT_HANDOFF_X_CM` | `0.36 × x_scale` | `1.486800000` | cm |
| `RECOVERY_EXIT_INWARD_X_DOT_CM_S` | `0.10 × x_dot_scale` | `0.454117218` | cm/s |
| `RECOVERY_FLOOR_DEFAULT_DEG` | `0.35 × THETA_CMD_LIMIT_DEFAULT_DEG` | `0.700000000` | deg |
| `RECOVERY_FLOOR_MIN_DEG` | lower floor bound | `0.000000000` | deg |
| `RECOVERY_FLOOR_MAX_DEG` | `0.55 × THETA_CMD_LIMIT_DEFAULT_DEG` | `1.100000000` | deg |
| `RECOVERY_FLOOR_GAIN_DEG_PER_CM` | `0.375 × theta_scale / x_scale` | `0.181598063` | deg/cm |
| `ZERO_TRIM_EST_X_CM` | `0.06 × x_scale` | `0.247800000` | cm |
| `ZERO_TRIM_EST_X_DOT_CM_S` | `0.055 × x_dot_scale` | `0.249764470` | cm/s |
| `ZERO_TRIM_EST_THETA_TRACK_ERR_DEG` | `0.05 × theta_scale` | `0.100000000` | deg |
| `ZERO_TRIM_EST_THETA_DOT_DEG_S` | `0.10 × OUTER_THETA_DOT_LIMIT_DEG_S` | `0.439822972` | deg/s |
| `ZERO_TRIM_EST_ALPHA` | `1 − exp(-DT / 0.8)` | `0.048770575` | ratio/sample |

---

## 10. Archived Reference-Controller Notes

The older single-loop reference PID derivation and the exploratory LQR discussion are
retained only as historical artifacts of the previous controller architecture.

- `model/first_principles/design_cascade_pid.py`
- `model/first_principles/controller_initial_gains.json`
- `firmware/include/generated/controller_gains.h`

They are no longer authoritative for the staged Sharp/AS5600 controller and should not
be used as the source for the constants in the new sketch.

---

## 11. Summary of Key Numerical Results

| Quantity | Symbol | Value | Unit |
| ---------- | -------- | ------- | ------ |
| Beam assembly mass | M_b | 0.038500 | kg |
| Beam COM from pivot | L_com | 0.118050 | m |
| Beam inertia about pivot | J | 7.715 × 10⁻⁴ | kg·m² |
| Center Sharp reading | center_d | 0.085300 | m |
| Nominal ball position | r₀ | 0.125600 | m |
| Effective rolling-mass ratio | α | 0.600000 | — |
| Rolling damping coefficient | β | 2.678571 | s⁻¹ |
| Ball acceleration gain | k_theta | 10.273008 | cm/s²/deg |
| Inner closed-loop bandwidth | f_i | 4.790456 | Hz |
| Outer design bandwidth | f_o | 0.175000 | Hz |
| Beam angle per degree command | STEPS_PER_BEAM_DEG | 115.939922 | steps/deg |
| Default beam-angle limit | THETA_CMD_LIMIT_DEFAULT_DEG | 2.000000 | deg |
| LQI gain on `x_cm` | OUTER_KX_DEFAULT | -0.272649 | deg/cm |
| LQI gain on `x_dot_cm_s` | OUTER_KV_DEFAULT | -0.180552 | deg·s/cm |
| LQI gain on `theta_rel_deg` | OUTER_KT_DEFAULT | 0.125118 | — |
| LQI gain on `theta_dot_deg_s` | OUTER_KW_DEFAULT | 0.002070 | s |
| LQI gain on `xi_cm_s` | OUTER_KI_DEFAULT | -0.020917 | deg/(cm·s) |

---

## 12. Files in Sync with This Document

When the staged controller architecture or Sharp hardware parameters change, update
these together:

| File | Role |
| ------ | ------ |
| `docs/modeling.md` | Canonical plant and staged-controller derivation |
| `docs/modeling_source_of_truth.md` | Canonical sync contract for the modeling artifacts |
| `model/first_principles/params_measured.yaml` | Measured Sharp hardware and staged-design inputs |
| `model/first_principles/linearize.py` | Generates the linearized plant artifact |
| `model/first_principles/design_staged_controller.py` | Generates staged-controller constants |
| `model/first_principles/staged_controller_constants.json` | Generated staged-controller constants and verification data |
| `firmware/include/generated/staged_controller_constants.h` | Generated copy-into-sketch constants header |

Historical reference-PID artifacts remain in the repo for comparison, but they are not
part of the active staged-controller source-of-truth set.
