# Ball-and-Beam Mathematical Modeling and Control Design

This is the single canonical modeling document for the ball-and-beam control system.
All first-principles derivations, hardware parameters, and controller designs live here.

---

## 1. Definitions

### 1.1 Physical Setup

The system consists of a beam pivoted at one end and actuated at the other by a stepper
motor through a crank-rocker linkage. A hollow ping-pong ball rolls freely along a
V-groove runner mounted on top of the beam. An HC-SR04 ultrasonic rangefinder mounted
at the pivot end of the beam measures the ball's distance from the sensor face.

```text
                           HC-SR04
                           sensor
                            ||||
                            ||||
  pivot                     ||||          motor crank
    O===========[ runner ]==============O--( )
    ^            ~~~ball~~~             ^
    |                                  |
  fulcrum                          motor-side
  (fixed)                          clevis
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

In the active firmware (`firmware/src/main.cpp`):

- Positive microstep count → beam tilts motor-side up (θ > 0)
- Positive θ → ball accelerates away from pivot (+r direction)
- The HC-SR04 reports distance from the sensor face; ball position in the firmware
  coordinate is `x_ctrl = center_m − d`, where `d` is the measured distance and
  `center_m = 0.1295 m` is the nominal center

---

## 2. Parameter Evaluation

### 2.1 Raw Measurements

All values were measured on the final assembled hardware.

| Quantity | Measured value | Notes |
| ---------- | --------------- | ------- |
| Ball mass | 2.8 g | Standard 40 mm ping-pong ball |
| Ball diameter | 40.0 mm | |
| Bare beam mass | 33.2 g | Aluminium extrusion, 263 mm span |
| HC-SR04 + wires mass | 8.9 g | Mounted at pivot end of beam |
| Total beam assembly mass | 42.1 g | Bare beam + sensor + wires |
| Beam length (clevis-to-clevis) | 263 mm | |
| Runner start from pivot-side clevis | 48 mm | |
| Runner length (nominal) | 199 mm | Range: 198–200 mm |
| Runner end clearance to motor clevis | 12.5 mm | |
| Sensor face from pivot-side clevis | 18 mm | |
| Nearest ball–sensor clearance | 30 mm | When ball is at runner start |
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
| M_b | m_beam + m_sensor | 0.0421 | kg |
| L | — | 0.263 | m |
| L_com | see §2.3 | 0.10751 | m |
| J | see §2.4 | 7.684 × 10⁻⁴ | kg·m² |

**Actuation and sensing:**

| Symbol | Formula | Value | Unit |
| -------- | --------- | ------- | ------ |
| N_steps | 200 × 16 | 3200 | steps/rev |
| k_step_rad | 2π / N_steps | 1.9635 × 10⁻³ | rad/step |
| λ | linkage_ratio | 1.0 | — |
| center_m | 30 mm + 199/2 mm | 0.1295 | m |

**Physical constants:**

| Symbol | Value | Unit |
| -------- | ------- | ------ |
| g | 9.81 | m/s² |
| b_x | 0.0125 | N·s/m |
| b_θ | 0.006 | N·m·s |

### 2.3 Beam + Sensor Combined Centre of Mass

The beam assembly is modelled as a uniform slender rod (the aluminium extrusion) plus a
lumped point mass (the HC-SR04 and wires) at the sensor face location.

```text
x_com = (m_beam × x_beam_com + m_sensor × x_sensor) / M_b
```

where `x_beam_com = L / 2 = 0.263 / 2 = 0.1315 m` (uniform rod) and
`x_sensor = 0.018 m` (sensor face from pivot-side clevis).

Substitution:

```text
x_com = (0.0332 × 0.1315 + 0.0089 × 0.018) / 0.0421
      = (0.0043658 + 0.0001602) / 0.0421
      = 0.004526 / 0.0421
      = 0.10751 m
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

Sensor lumped at x_sensor = 0.018 m:

```text
J_sensor = m_sensor × x_sensor²
         = 0.0089 × (0.018)²
         = 0.0089 × 0.000324
         = 2.884 × 10⁻⁶  kg·m²
```

Total beam inertia about pivot:

```text
J = J_beam + J_sensor
  = 7.6547 × 10⁻⁴ + 2.884 × 10⁻⁶
  = 7.684 × 10⁻⁴  kg·m²
```

### 2.5 Nominal Ball-Position Centre

The HC-SR04 face is at 18 mm from the pivot clevis. The nearest valid ball position
(ball at runner start, just beyond sensor) is 30 mm beyond the sensor face, so
48 mm from the pivot. The runner is 199 mm long, so the runner midpoint from the
sensor face is:

```text
center_m = 30 mm + (199 mm / 2) = 30 + 99.5 = 129.5 mm = 0.1295 m
```

This is the desired ball setpoint (center of runner).

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

**Sign note:** With the positive-θ convention (motor side up), a positive δ tilts the
beam so that gravity pulls the ball away from the pivot (increasing r). The firmware
uses the mirror of this: positive step count drives positive x_ctrl in the control
frame, which requires a negative sign in the actuation mapping. The derivation here
tracks the physics; sign bookkeeping is done in the unit-conversion step (§9).

### 7.3 Linearized Beam Equation

Substitute into EOM-θ with cosδ ≈ 1 and ṙ δ̇ ≈ 0:

```text
(J + m r₀²) δ̈ + b_θ δ̇ + (m r₀ + M_b L_com) g δ = τ − τ₀
```

For the active firmware architecture the beam angle is commanded directly (the actuator
is treated as ideal for controller design), so the beam equation is not used in the PID
design. It becomes relevant for full-state LQR design (§10).

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
ẍ + 2.6786 ẋ = 5.886 δ          [x in m, δ in rad]
```

Laplace transform (zero initial conditions):

```text
s² X(s) + 2.6786 s X(s) = 5.886 Θ(s)

X(s) / Θ(s) = 5.886 / (s² + 2.6786 s)
             = 5.886 / (s (s + 2.6786))   [m/rad]
```

This is an integrator cascaded with a first-order lag (pole at s = −β).

### 8.3 Beam Inertia at Nominal Operating Point

At the design equilibrium r₀ = 0.1295 m (centre of runner):

```text
J_total(r₀) = J + m r₀²
             = 7.684 × 10⁻⁴ + 0.0028 × (0.1295)²
             = 7.684 × 10⁻⁴ + 0.0028 × 0.016770
             = 7.684 × 10⁻⁴ + 4.696 × 10⁻⁵
             = 8.154 × 10⁻⁴  kg·m²
```

### 8.4 Gravitational Restoring Torque Coefficient (Linearized)

```text
(m r₀ + M_b L_com) g = (0.0028 × 0.1295 + 0.0421 × 0.10751) × 9.81
                      = (0.0003626 + 0.0045261) × 9.81
                      = 0.0048887 × 9.81
                      = 0.04796  N·m/rad
```

The linearized beam equation at r₀ (ideal actuator, no damping) has natural frequency:

```text
ω_beam = √(0.04796 / 8.154 × 10⁻⁴)
        = √(58.82)
        = 7.67  rad/s  (1.22 Hz)
```

This is well above the ball control bandwidth (0.12 Hz), which justifies treating the
beam as an ideal angle actuator for ball position control.

---

## 9. Controller-Oriented Reduced Model and PID Design

### 9.1 Ideal Actuator Assumption

For ball position control the stepper motor is commanded to a microstep count N. The
relationship between step count and beam angle (from calibration):

```text
δ = k_step_rad × N
```

where:

```text
k_step_rad = (2π / N_steps) × λ
           = (2π / 3200) × 1.0
           = 0.0019635  rad/step
```

### 9.2 Plant Transfer Function in Step and Centimetre Units

The firmware measures ball position in centimetres and commands microsteps, so the
plant transfer function must be expressed in those units.

Convert x from metres to centimetres (multiply by 100) and substitute
δ = k_step_rad × N:

```text
ẍ_cm + β ẋ_cm = 100 × α g × k_step_rad × N
               = κ N
```

where:

```text
κ = 100 × α × g × k_step_rad
  = 100 × 0.600000 × 9.81 × 0.0019635
  = 100 × 0.600000 × 0.019262
  = 100 × 0.011557
  = 1.155713  cm / s² / step
```

Laplace transform:

```text
G(s) = X_cm(s) / N(s) = κ / (s (s + β))
                       = 1.155713 / (s (s + 2.678571))
```

This is the plant model used for all controller design.

### 9.3 PID Controller Structure

Use an ideal PID in the continuous-time Laplace domain:

```text
C(s) = Kp + Ki / s + Kd s
```

with error e = x_setpoint − x_cm (centimetres) and output N (microsteps).

The open-loop transfer function is L(s) = C(s) G(s). Write C(s) with a common
denominator:

```text
C(s) = (Kd s² + Kp s + Ki) / s

L(s) = [κ (Kd s² + Kp s + Ki)] / [s² (s + β)]
```

The closed-loop characteristic polynomial is the denominator of 1 + L(s), i.e. the
numerator of 1 + L(s) set to zero:

```text
s²(s + β) + κ(Kd s² + Kp s + Ki) = 0

s³ + β s² + κ Kd s² + κ Kp s + κ Ki = 0

s³ + (β + κ Kd) s² + (κ Kp) s + (κ Ki) = 0       ... (CHARPOLY)
```

### 9.4 Pole-Placement Target

Choose desired closed-loop poles to give a smooth, well-damped response. The target is
a complex-conjugate pair for the dominant dynamics plus one real pole placed further
left (faster) so it has minimal effect on the transient.

Design parameters (from `params_measured.yaml`):

```text
outer_bw_hz  = 0.12   Hz
outer_zeta   = 0.85
extra_pole_factor = 4.0
```

Natural frequency:

```text
ωn = 2π × 0.12 = 0.753982  rad/s
```

Desired dominant pole pair:

```text
s₁,₂ = −ζ ωn ± jωn √(1 − ζ²)
      = −0.85 × 0.753982 ± j × 0.753982 × √(1 − 0.85²)
      = −0.640884 ± j × 0.753982 × 0.526783
      = −0.640884 ± j 0.397114
```

Third (real) pole:

```text
p₃ = extra_pole_factor × ωn = 4.0 × 0.753982 = 3.015929  rad/s
```

Desired characteristic polynomial:

```text
(s² + 2ζωn s + ωn²)(s + p₃)
```

Expand:

```text
s³ + p₃ s²  +  2ζωn s²  +  2ζωn p₃ s  +  ωn² s  +  ωn² p₃

= s³ + (2ζωn + p₃) s² + (ωn² + 2ζωn p₃) s + ωn² p₃
```

Compute each coefficient:

```text
a₂ = 2ζωn + p₃
   = 2 × 0.85 × 0.753982 + 3.015929
   = 1.281768 + 3.015929
   = 4.297698

a₁ = ωn² + 2ζωn p₃
   = (0.753982)² + 2 × 0.85 × 0.753982 × 3.015929
   = 0.568490 + 3.865726
   = 4.434216

a₀ = ωn² × p₃
   = 0.568490 × 3.015929
   = 1.714523
```

Desired polynomial:

```text
s³ + 4.297698 s² + 4.434216 s + 1.714523 = 0
```

### 9.5 PID Gain Calculation — Physical (Beam-Angle) Domain

Before converting to firmware units, solve for the PID gains in the physical domain
where the plant output is x in metres and the actuator input is θ in radians.

The physical plant is:

```text
X(s) / Θ(s) = αg / (s(s + β))
```

For this plant, a PID controller with output θ_cmd gives the characteristic polynomial:

```text
s³ + (β + αg Kd_θ) s² + αg Kp_θ s + αg Ki_θ = 0
```

Match coefficients with the desired polynomial:

```text
β + αg Kd_θ = a₂    →   Kd_θ = (a₂ − β) / (αg)
αg Kp_θ     = a₁    →   Kp_θ = a₁ / (αg)
αg Ki_θ     = a₀    →   Ki_θ = a₀ / (αg)
```

Substitute numerical values (αg = 5.886000):

```text
Kp_θ = 4.434216 / 5.886000 = 0.753350  rad/m
Ki_θ = 1.714523 / 5.886000 = 0.291288  rad/(m·s)
Kd_θ = (4.297698 − 2.678571) / 5.886000
      = 1.619127 / 5.886000
      = 0.275081  rad·s/m
```

### 9.6 Conversion to Firmware Units (steps / cm)

The firmware error is in centimetres and the output is in microsteps, so:

```text
Kp = Kp_θ / (100 × k_step_rad)
Ki = Ki_θ / (100 × k_step_rad)
Kd = Kd_θ / (100 × k_step_rad)
```

The factor 100 converts m⁻¹ to cm⁻¹; k_step_rad converts rad to steps.

```text
100 × k_step_rad = 100 × 0.0019635 = 0.19635  cm⁻¹·step⁻¹·rad

Kp = 0.753350 / 0.19635 = 3.8368  steps/cm
Ki = 0.291288 / 0.19635 = 1.4835  steps/(cm·s)
Kd = 0.275081 / 0.19635 = 1.4010  step·s/cm
```

These are the seed gains exported to `model/first_principles/controller_initial_gains.json`
and `firmware/include/generated/controller_gains.h`.

### 9.7 Output Clamp and Linear-Model Validity

The firmware limits the output to ±400 steps:

```text
θ_max = 400 × k_step_rad = 400 × 0.0019635 = 0.78540 rad = 45.0°
```

The linearization in §7 is only valid for small θ. The accepted engineering rule of
thumb is |θ| ≲ 5–10°. The design constraint stored in `params_measured.yaml` is
`theta_cmd_deg: 4.0`, meaning the linear model is considered valid up to ±4°.

At ±4° the controller should rarely saturate during normal ball-position regulation
near the centre. The ±45° software clamp exists as a safety backstop only.

### 9.8 Current Runtime Gains (Empirically Tuned)

The first-principles gains above are seed values. After empirical tuning on the
physical hardware the active runtime gains in `firmware/include/config.h` are:

| Gain | Seed (1st principles) | Runtime (tuned) | Unit |
| ------ | ---------------------- | ----------------- | ------ |
| Kp | 3.8368 | 10.0 | steps/cm |
| Ki | 1.4835 | 0.6 | steps/(cm·s) |
| Kd | 1.4010 | 4.5 | step·s/cm |

The empirical Kp is ~2.6× the theoretical value; Ki is lower and Kd is ~3× higher.
This reflects physical effects not captured by the model: sensor noise, quantization,
asymmetric friction, and structural flex.

---

## 10. State-Space Representation and LQR Design

### 10.1 State-Space Model

For a full-state controller, augment the ball state with the beam state. With the ideal
actuator approximation (θ commanded directly), the linearized system has four states:

```text
z = [x, ẋ, δ, δ̇]ᵀ
```

where x is ball displacement from setpoint (m), δ is beam angle (rad), and the input
is the microstep command N (steps). The state equations are:

Ball subsystem (from §7.2):

```text
ẋ = ẋ
ẍ = −β ẋ + αg δ
```

Beam as a first-order actuator (with bandwidth τ_φ = 0.08 s):

```text
δ̇ = δ̇
δ̈ = −(1/τ_φ) δ̇ + (k_step_rad / τ_φ) N
```

Written in matrix form:

```text
ż = A z + B N

A = [ 0    1     0       0     ]
    [ 0   -β    αg       0     ]
    [ 0    0     0       1     ]
    [ 0    0     0   -1/τ_φ   ]

B = [       0         ]
    [       0         ]
    [       0         ]
    [ k_step_rad/τ_φ  ]

C = [1  0  0  0]    (measure ball position only)
D = 0
```

Substituting numerical values:

```text
β       = 2.678571
αg      = 5.886000  (m/s²)/rad
τ_φ     = 0.08  s
k_step_rad = 0.0019635  rad/step
1/τ_φ   = 12.5  s⁻¹
k_step_rad/τ_φ = 0.0019635 / 0.08 = 0.024544  rad/s/step

A = [  0        1       0       0    ]
    [  0   -2.6786   5.886      0    ]
    [  0        0       0       1    ]
    [  0        0       0    -12.5   ]

B = [  0      ]
    [  0      ]
    [  0      ]
    [ 0.024544]
```

### 10.2 Controllability Check

A system is controllable if the controllability matrix:

```text
𝒞 = [B  AB  A²B  A³B]
```

has full rank (rank 4 for this system). For the given A and B, all four states can be
reached from the input N, so the system is controllable. (Verification is best done
numerically.)

### 10.3 Observability Check

Only ball position x (first state) is measured. The observability matrix is:

```text
𝒪 = [C; CA; CA²; CA³]
```

With only ball position measured (C = [1,0,0,0]), the system is observable if all
states are distinguishable from the position output. For this system the ball velocity
state and beam states can be reconstructed from position measurements in principle,
but in practice the beam state is better obtained from the AS5600 encoder.

### 10.4 LQR Design

Linear Quadratic Regulator (LQR) finds the optimal full-state feedback gain vector
K* that minimises the cost functional:

```text
J_LQR = ∫₀^∞ (zᵀ Q z + N² R) dt
```

where Q penalises state deviations and R penalises control effort.

**Algebraic Riccati Equation (ARE):**

```text
Aᵀ P + P A − P B R⁻¹ Bᵀ P + Q = 0
```

Solve for the positive-definite matrix P, then compute the optimal gain:

```text
K* = R⁻¹ Bᵀ P
```

Control law:

```text
N = −K* z = −[K_x  K_xdot  K_δ  K_δdot] [x, ẋ, δ, δ̇]ᵀ
```

**Tuning guidelines:**

Q and R are design knobs. A reasonable starting point:

```text
Q = diag(q_x, q_xdot, q_δ, q_δdot)
  = diag(1.0,  0.1,   10.0, 1.0)    (position errors penalised most)

R = 0.001                             (allow generous control effort)
```

Increasing q_x relative to R speeds up position regulation (at the cost of larger
step commands). Increasing q_δ penalises beam excursions.

**Practical note — state observer required:**

LQR requires the full state z = [x, ẋ, δ, δ̇]ᵀ. The hardware provides:

- x: directly from HC-SR04 (noisy, 25 Hz)
- ẋ: must be estimated (finite difference or Kalman filter)
- δ: available from AS5600 encoder (not currently used in active firmware)
- δ̇: must be estimated from AS5600 readings

A Luenberger observer or Kalman filter using both HC-SR04 and AS5600 readings would
provide the full state for LQR implementation. This represents the natural next step
beyond the current single-loop PID architecture.

---

## 11. Summary of Key Numerical Results

| Quantity | Symbol | Value | Unit |
| ---------- | -------- | ------- | ------ |
| Effective rolling mass | m_e | 0.004667 | kg |
| Ball inertia ratio | α | 0.600000 | — |
| Rolling damping coefficient | β | 2.678571 | s⁻¹ |
| Rad per microstep | k_step_rad | 0.0019635 | rad/step |
| Plant gain | κ | 1.155713 | cm/s²/step |
| Plant pole | −β | −2.678571 | s⁻¹ |
| Design bandwidth | ωn | 0.753982 | rad/s |
| Design damping ratio | ζ | 0.850 | — |
| Third pole | p₃ | 3.015929 | rad/s |
| Kp (seed) | Kp | 3.8368 | steps/cm |
| Ki (seed) | Ki | 1.4835 | steps/(cm·s) |
| Kd (seed) | Kd | 1.4010 | step·s/cm |

---

## 12. Files in Sync with This Document

When the controller architecture or hardware parameters change, update these together:

| File | Role |
| ------ | ------ |
| `docs/modeling.md` | This document — canonical model and control derivation |
| `model/first_principles/params_measured.yaml` | Hardware parameter values |
| `model/first_principles/linearized_model.json` | Generated linear state-space matrices |
| `model/first_principles/controller_initial_gains.json` | Generated seed gains |
| `firmware/include/generated/controller_gains.h` | Generated gains header |
| `firmware/src/main.cpp` | Active controller implementation |
| `firmware/include/config.h` | Runtime-tuned gains and configuration |
| `model/first_principles/linearize.py` | Generates linearized_model.json |
| `model/first_principles/design_cascade_pid.py` | Generates controller gains |
