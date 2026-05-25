# Ball-and-Beam Modeling from First Principles

This document derives the standard ball-and-beam equations of motion from first
principles. It is intentionally independent from this repository's hardware,
firmware, sensors, calibration constants, and controller tuning values. Every
quantity is symbolic.

The goal is to derive a clean educational model that is consistent with the
common ball-and-beam model used in control literature:

```text
(m + J_b / rho^2) x_ddot - m x alpha_dot^2 + m g sin(alpha) = Q_x
```

where `x` is the ball position along the beam, `alpha` is the beam angle, `m` is
the ball mass, `J_b` is the ball moment of inertia about its center, and `rho` is
the effective rolling radius. With a viscous rolling-resistance model,
`Q_x = -b_x x_dot`.

This is the same structural model used by standard ball-and-beam tutorials and
papers, including CTMS/University of Michigan's Ball & Beam modeling tutorial,
MathWorks' ball-on-beam example, Lund University's real-time systems note, and
the comparative study by Gembalczyk, Domogala, and Lesniowski.

## 1. What Is Being Modeled

The idealized system is:

- a rigid beam rotating in a vertical plane;
- a rigid ball rolling without slipping along the beam;
- gravity acting vertically downward;
- one translational coordinate for the ball along the beam;
- one angular coordinate for the beam.

The model can be used in two common ways:

- Beam-angle-input model: the beam angle `alpha(t)` is treated as the control
  input. This is the model used in many introductory control treatments because
  the actuator dynamics are hidden inside a faster inner loop.
- Torque-input model: the beam torque `tau(t)` is treated as the control input,
  and the beam angle is a dynamic state. This is useful when beam dynamics and
  actuator torque limits matter.

Both versions are derived below.

## 2. Coordinates and Sign Convention

Define a fixed inertial frame with:

- horizontal axis `X`;
- vertical axis `Y`, positive upward;
- gravity acceleration vector `-g e_Y`.

Define beam-attached unit vectors:

```text
e_r = [cos(alpha), sin(alpha)]^T
e_n = [-sin(alpha), cos(alpha)]^T
```

where:

- `e_r` points along the beam in the positive ball-position direction;
- `e_n` is normal to the beam;
- `alpha = 0` means the beam is horizontal;
- `alpha > 0` means the positive-`x` end of the beam is raised.

The generalized coordinates are:

```text
q = [x, alpha]^T
```

where:

- `x(t)` is the ball-center coordinate along the beam;
- `alpha(t)` is the beam angle from horizontal.

With this convention, if `alpha > 0`, the positive-`x` end is uphill, so gravity
accelerates the ball in the negative `x` direction. This is why the small-angle
linearized beam-angle-to-ball-position gain is negative.

## 3. Modeling Assumptions

The standard first-principles model assumes:

1. The beam is rigid.
2. The ball is rigid and axisymmetric.
3. The ball rolls without slipping.
4. The contact is ideal, so static friction enforces rolling but does no work.
5. The ball remains in contact with the beam.
6. The ball center moves along the beam line in the simplest model.
7. The beam rotation axis lies on the modeled beam line in the simplest model.
8. Air drag and high-order contact effects are ignored.
9. Optional rolling resistance can be added as a nonconservative generalized
   force.

Assumptions 6 and 7 are the usual simplified ball-and-beam model. More detailed
models may include ball-center offset, V-groove rolling radius, eccentric beam
axis placement, Coulomb/viscous rolling friction, rail contact geometry, actuator
compliance, backlash, or motor dynamics. The simplified model is still the
standard starting point because it gives the correct dominant structure for
control design.

## 4. Symbols

| Symbol | Meaning | Unit |
| --- | --- | --- |
| `x` | ball position along the beam | m |
| `alpha` | beam angle from horizontal | rad |
| `m` | ball mass | kg |
| `R` | geometric ball radius | m |
| `rho` | effective rolling radius at contact | m |
| `J_b` | ball moment of inertia about its center | kg m^2 |
| `J_beam` | beam inertia about its pivot | kg m^2 |
| `M_beam` | beam mass | kg |
| `l_c` | beam center-of-mass distance from pivot | m |
| `g` | gravitational acceleration magnitude | m/s^2 |
| `b_x` | viscous rolling-resistance coefficient | N s/m |
| `b_alpha` | beam rotational damping coefficient | N m s/rad |
| `tau` | external torque applied to the beam | N m |

The ball inertia is often written:

```text
J_b = k_b m R^2
```

where:

- `k_b = 2/5` for a solid sphere;
- `k_b = 2/3` for a thin spherical shell;
- other values can be used for a real ball if measured.

For a ball rolling on a flat surface, `rho = R`. For rails or a V-groove, `rho`
can differ from `R`. Keeping `rho` explicit makes the model more general.

## 5. Kinematics

In the simplest model, the ball center position is:

```text
r_b = x e_r
```

Since:

```text
e_r_dot = alpha_dot e_n
```

differentiate:

```text
r_b_dot = x_dot e_r + x alpha_dot e_n
```

Because `e_r` and `e_n` are orthonormal:

```text
v_b^2 = r_b_dot . r_b_dot
      = x_dot^2 + x^2 alpha_dot^2
```

This is the same structure as polar-coordinate kinetic energy: motion along the
beam plus tangential motion due to beam rotation.

## 6. Rolling Constraint

Let `psi` be the ball spin angle about the rolling axis. Rolling without slipping
requires:

```text
x_dot = rho psi_dot
```

or:

```text
psi_dot = x_dot / rho
```

This is a nonholonomic velocity constraint, but in this one-dimensional rolling
case it can be substituted directly into the kinetic energy.

## 7. Energies

### 7.1 Ball Translational Kinetic Energy

```text
T_trans = (1/2) m v_b^2
        = (1/2) m (x_dot^2 + x^2 alpha_dot^2)
```

### 7.2 Ball Rotational Kinetic Energy

Using `psi_dot = x_dot / rho`:

```text
T_rot = (1/2) J_b psi_dot^2
      = (1/2) J_b (x_dot / rho)^2
      = (1/2) (J_b / rho^2) x_dot^2
```

### 7.3 Beam Kinetic Energy

If beam dynamics are modeled explicitly:

```text
T_beam = (1/2) J_beam alpha_dot^2
```

If the beam angle is treated as a directly commanded input, this term is not
needed for the ball-only plant.

### 7.4 Total Kinetic Energy

For the torque-input model:

```text
T = (1/2) (m + J_b / rho^2) x_dot^2
  + (1/2) (J_beam + m x^2) alpha_dot^2
```

Define the effective rolling mass:

```text
M_e = m + J_b / rho^2
```

Then:

```text
T = (1/2) M_e x_dot^2
  + (1/2) (J_beam + m x^2) alpha_dot^2
```

For a solid sphere rolling on a flat beam:

```text
M_e = m + (2/5 m R^2) / R^2
    = (7/5) m
```

For a thin spherical shell rolling on a flat beam:

```text
M_e = m + (2/3 m R^2) / R^2
    = (5/3) m
```

### 7.5 Potential Energy

The ball height is:

```text
y_b = x sin(alpha)
```

so:

```text
V_ball = m g x sin(alpha)
```

If beam mass is included and the beam center of mass is at distance `l_c` from
the pivot:

```text
V_beam = M_beam g l_c sin(alpha)
```

Total potential energy:

```text
V = m g x sin(alpha) + M_beam g l_c sin(alpha)
```

If the beam center of mass is at the pivot, or if the beam-angle dynamics are not
being modeled, the second term can be omitted.

## 8. Lagrange Equations

The Lagrangian is:

```text
L = T - V
```

The Euler-Lagrange equation for generalized coordinate `q_i` is:

```text
d/dt( partial L / partial q_i_dot ) - partial L / partial q_i = Q_i
```

where `Q_i` is the nonconservative generalized force.

Use:

```text
Q_x     = -b_x x_dot
Q_alpha = tau - b_alpha alpha_dot
```

for viscous rolling resistance and viscous beam damping. Set the damping
coefficients to zero for the ideal lossless model.

## 9. Nonlinear Equation for Ball Motion

Use:

```text
L = (1/2) M_e x_dot^2
  + (1/2) (J_beam + m x^2) alpha_dot^2
  - m g x sin(alpha)
  - M_beam g l_c sin(alpha)
```

Only terms depending on `x` are needed for the ball equation:

```text
partial L / partial x_dot = M_e x_dot
```

```text
d/dt(partial L / partial x_dot) = M_e x_ddot
```

```text
partial L / partial x = m x alpha_dot^2 - m g sin(alpha)
```

Apply Euler-Lagrange:

```text
M_e x_ddot - (m x alpha_dot^2 - m g sin(alpha)) = Q_x
```

Therefore:

```text
M_e x_ddot - m x alpha_dot^2 + m g sin(alpha) = Q_x
```

With viscous rolling resistance:

```text
M_e x_ddot + b_x x_dot - m x alpha_dot^2 + m g sin(alpha) = 0
```

Substituting `M_e = m + J_b / rho^2`:

```text
(m + J_b / rho^2) x_ddot
  + b_x x_dot
  - m x alpha_dot^2
  + m g sin(alpha)
  = 0
```

For the ideal no-damping model:

```text
(m + J_b / rho^2) x_ddot
  - m x alpha_dot^2
  + m g sin(alpha)
  = 0
```

Solving for `x_ddot`:

```text
x_ddot =
  (m / M_e) x alpha_dot^2
  - (m g / M_e) sin(alpha)
  - (b_x / M_e) x_dot
```

This is the standard nonlinear ball-and-beam equation. The centrifugal term
`x alpha_dot^2` is nonlinear and is usually dropped during small-angle
linearization.

## 10. Nonlinear Equation for Beam Rotation

Now use coordinate `alpha`.

```text
partial L / partial alpha_dot = (J_beam + m x^2) alpha_dot
```

Differentiate:

```text
d/dt(partial L / partial alpha_dot)
  = (J_beam + m x^2) alpha_ddot
    + 2 m x x_dot alpha_dot
```

The angle derivative is:

```text
partial L / partial alpha
  = -m g x cos(alpha) - M_beam g l_c cos(alpha)
```

Apply Euler-Lagrange:

```text
(J_beam + m x^2) alpha_ddot
  + 2 m x x_dot alpha_dot
  + m g x cos(alpha)
  + M_beam g l_c cos(alpha)
  = tau - b_alpha alpha_dot
```

Rearranged:

```text
(J_beam + m x^2) alpha_ddot
  + 2 m x x_dot alpha_dot
  + b_alpha alpha_dot
  + (m x + M_beam l_c) g cos(alpha)
  = tau
```

This torque-input equation is useful when the actuator applies torque directly
or when beam rotational dynamics must be modeled.

Many ball-and-beam control tutorials omit this equation because the actuator is
assumed to command `alpha` through a fast position loop. In that case,
`alpha(t)` is an input to the ball equation rather than a state governed by
beam torque.

## 11. State-Space Nonlinear Model

Define:

```text
z_1 = x
z_2 = x_dot
z_3 = alpha
z_4 = alpha_dot
```

### 11.1 Beam-Angle-Input Model

If `alpha(t)` is the input, the minimal nonlinear plant is second order:

```text
x_dot = v
v_dot = (m / M_e) x alpha_dot^2
      - (m g / M_e) sin(alpha)
      - (b_x / M_e) v
```

If the beam-angle input is slowly varying or if the actuator input is treated as
the angle itself rather than both angle and angular rate, the common simplified
plant uses:

```text
x_dot = v
v_dot = -(m g / M_e) sin(alpha) - (b_x / M_e) v
```

The omitted `x alpha_dot^2` term is second order in angular rate and is usually
small near the horizontal operating point.

### 11.2 Torque-Input Model

For torque input `tau`, use all four states:

```text
z_1_dot = z_2
```

```text
z_2_dot =
  (m / M_e) z_1 z_4^2
  - (m g / M_e) sin(z_3)
  - (b_x / M_e) z_2
```

```text
z_3_dot = z_4
```

```text
z_4_dot =
  [tau
   - 2 m z_1 z_2 z_4
   - b_alpha z_4
   - (m z_1 + M_beam l_c) g cos(z_3)]
  / (J_beam + m z_1^2)
```

This is a coupled nonlinear model. The ball position affects the beam inertia
and gravitational torque, while the beam angle affects ball acceleration.

## 12. Equilibrium About a Horizontal Beam

Choose an operating point:

```text
x = x_0
x_dot = 0
alpha = 0
alpha_dot = 0
```

The ball equation is satisfied because `sin(0) = 0`.

The torque-input beam equation requires a static holding torque:

```text
tau_0 = (m x_0 + M_beam l_c) g
```

This is a gravity-balance torque, not an angular stiffness.

## 13. Linearization: Beam-Angle-Input Model

Let:

```text
x = x_0 + xi
alpha = 0 + a
```

where `xi` and `a` are small perturbations.

Use:

```text
sin(a) ~= a
cos(a) ~= 1
a_dot^2 ~= 0
```

Start with:

```text
M_e x_ddot + b_x x_dot - m x alpha_dot^2 + m g sin(alpha) = 0
```

Drop the second-order `x alpha_dot^2` term and substitute the perturbations:

```text
M_e xi_ddot + b_x xi_dot + m g a = 0
```

Therefore:

```text
xi_ddot + beta xi_dot = -K_alpha a
```

where:

```text
beta = b_x / M_e
K_alpha = m g / M_e
```

Without rolling resistance:

```text
xi_ddot = -K_alpha a
```

Taking the Laplace transform with zero initial conditions:

```text
Xi(s) / A(s) = -K_alpha / (s^2 + beta s)
```

If `b_x = 0`:

```text
Xi(s) / A(s) = -K_alpha / s^2
```

This is the well-known double-integrator structure. The negative sign is a direct
consequence of the sign convention: raising the positive-`x` end of the beam
causes the ball to accelerate toward negative `x`.

## 14. Linearization: Servo-Gear-Input Model

Many laboratory ball-and-beam rigs use a servo or crank linkage. Near an
operating point, the geometry is often approximated by a linear map:

```text
alpha = k_g theta
```

where:

- `theta` is the actuator or gear angle;
- `k_g` is the local beam-angle-per-actuator-angle gain.

Substitute into the linearized beam-angle-input model:

```text
xi_ddot + beta xi_dot = -K_alpha k_g theta
```

So:

```text
Xi(s) / Theta(s) = -(K_alpha k_g) / (s^2 + beta s)
```

With no rolling resistance:

```text
Xi(s) / Theta(s) = -(K_alpha k_g) / s^2
```

If a lever geometry gives `k_g = d / L`, this becomes the common CTMS-style
transfer function:

```text
Xi(s) / Theta(s)
  = -m g (d / L)
    / [(m + J_b / rho^2) s^2]
```

for the no-damping case.

## 15. Linearization: Torque-Input Model

Use perturbation variables:

```text
x = x_0 + xi
alpha = a
tau = tau_0 + u
```

The ball equation linearizes to:

```text
M_e xi_ddot + b_x xi_dot + m g a = 0
```

The beam equation is:

```text
(J_beam + m x^2) alpha_ddot
  + 2 m x x_dot alpha_dot
  + b_alpha alpha_dot
  + (m x + M_beam l_c) g cos(alpha)
  = tau
```

At first order:

- `J_beam + m x^2` becomes `J_0 = J_beam + m x_0^2`;
- `2 m x x_dot alpha_dot` is second order and drops;
- `cos(a) ~= 1`;
- `(m x + M_beam l_c) g` becomes `(m x_0 + M_beam l_c) g + m g xi`;
- the constant term is canceled by `tau_0`.

Therefore:

```text
J_0 a_ddot + b_alpha a_dot + m g xi = u
```

where:

```text
J_0 = J_beam + m x_0^2
```

The linear torque-input state-space model is:

```text
d/dt [ xi      ]   [ 0          1          0           0        ] [ xi      ]   [ 0     ]
     [ xi_dot  ] = [ 0   -b_x/M_e   -m g/M_e       0        ] [ xi_dot  ] + [ 0     ] u
     [ a       ]   [ 0          0          0           1        ] [ a       ]   [ 0     ]
     [ a_dot   ]   [ -m g/J_0   0          0    -b_alpha/J_0 ] [ a_dot   ]   [ 1/J_0 ]
```

Important: there is no first-order `a` stiffness term in the beam equation about
`alpha = 0`. The gravitational torque term contains `cos(alpha)`, and
`d cos(alpha) / d alpha = 0` at `alpha = 0`. The first-order beam torque coupling
comes from ball-position perturbation `xi`, not from beam-angle perturbation `a`.

## 16. Literature Consistency Checks

This derivation matches the standard model in the following ways:

- CTMS/University of Michigan gives the Lagrangian ball equation in the form
  `0 = (J/R^2 + m) r_ddot + m g sin(alpha) - m r alpha_dot^2`, then linearizes it
  to `(J/R^2 + m) r_ddot = -m g alpha` about `alpha = 0`.
- MathWorks' ball-on-beam example defines the ball-position plant using the same
  effective gain structure `m / (J/R^2 + m)`.
- Lund University's real-time systems note derives the rolling-ball force balance
  and, for a solid sphere, obtains the familiar `5/7` acceleration factor and
  negative beam-angle-to-position gain.
- Gembalczyk, Domogala, and Lesniowski describe this as the commonly used ball-and-beam
  model, then compare it with extended models including eccentric fixation and
  friction. Their conclusion is a useful warning: the simple model is a good
  theoretical baseline, but friction and real contact effects often dominate
  experimental mismatch.

The equation derived here is therefore appropriate as a first-principles
educational model and as a starting point for controller design. It is not a
substitute for identifying friction, actuator dynamics, backlash, sensor delay,
beam compliance, and rail-contact effects on a real rig.

## 17. Common Mistakes

### 17.1 Treating Static Holding Torque as Beam Stiffness

The horizontal equilibrium torque:

```text
tau_0 = (m x_0 + M_beam l_c) g
```

is a constant torque needed to hold the beam horizontal. It is not a linear
restoring stiffness in `alpha`.

For angle measured from horizontal:

```text
(m x + M_beam l_c) g cos(alpha)
```

linearizes to:

```text
(m x_0 + M_beam l_c) g + m g xi
```

not:

```text
(m x_0 + M_beam l_c) g alpha
```

### 17.2 Forgetting the Ball's Rotational Inertia

Using `m x_ddot` alone overestimates acceleration. The correct effective mass is:

```text
M_e = m + J_b / rho^2
```

For a solid sphere on a flat beam, this gives:

```text
K_alpha = m g / M_e = (5/7) g
```

For a thin shell on a flat beam:

```text
K_alpha = (3/5) g
```

### 17.3 Hiding Sign Conventions

If `alpha > 0` raises the positive-`x` end, then the linearized plant gain from
`alpha` to `x` is negative. If a controller or sensor convention uses the
opposite sign, change the coordinate mapping explicitly. Do not silently bury a
sign flip inside a gain table.

### 17.4 Using the Geometric Radius When the Rolling Radius Differs

On a flat beam, `rho = R`. On rails or a V-groove, the effective rolling radius
may differ from the geometric radius. In that case, use:

```text
M_e = m + J_b / rho^2
```

not automatically `m + J_b / R^2`.

## 18. Final Equation Summary

Effective rolling mass:

```text
M_e = m + J_b / rho^2
```

Nonlinear ball equation:

```text
M_e x_ddot + b_x x_dot - m x alpha_dot^2 + m g sin(alpha) = 0
```

Nonlinear beam equation:

```text
(J_beam + m x^2) alpha_ddot
  + 2 m x x_dot alpha_dot
  + b_alpha alpha_dot
  + (m x + M_beam l_c) g cos(alpha)
  = tau
```

Beam-angle-input linearized plant:

```text
Xi(s) / A(s) = -(m g / M_e) / (s^2 + (b_x / M_e) s)
```

No-damping beam-angle-input plant:

```text
Xi(s) / A(s) = -(m g / M_e) / s^2
```

Torque-input linearized beam equation:

```text
J_0 a_ddot + b_alpha a_dot + m g xi = u
```

where:

```text
J_0 = J_beam + m x_0^2
u = tau - tau_0
tau_0 = (m x_0 + M_beam l_c) g
```

## References

- CTMS, University of Michigan, "Ball & Beam: System Modeling":
  https://ctms.engin.umich.edu/CTMS/?example=BallBeam&section=SystemModeling
- MathWorks, "Intelligent PID Using Ultra Local Model for Ball on Beam Balance":
  https://www.mathworks.com/help/slcontrol/ug/ipid-model-free-control-using-ulm.html
- Lund University, "Model for the Ball and Beam Process":
  https://www.control.lth.se/fileadmin/control/Education/EngineeringProgram/FRTN01/computer_exercises/c3/Links/ballandbeammodel.pdf
- Gembalczyk, G.; Domogala, P.; Lesniowski, K., "Modeling of Underactuated Ball
  and Beam System--A Comparative Study," Actuators, 2023:
  https://www.mdpi.com/2076-0825/12/2/59
