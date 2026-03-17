# Sharp Sign Reference

Use this document as the single sign-convention reference for Sharp-based sign
audits and any later controller sign fix.

## Canonical Variables

- `sensor_mount_end`: physical end where the Sharp is mounted, `pivot` or `motor`
- `d_sharp_cm`: raw Sharp distance from the sensor face
- `x_phys_cm`: physical ball position relative to runner center, positive toward the motor side
- `theta_phys_deg`: physical beam angle, positive when the motor side is up
- `u_cmd`: positive logical actuator jog/command
- `positive_jog_raises_motor`: derived hardware fact from the sign audit
- `sensor_distance_increases_toward_motor`: derived sensor fact from `sensor_mount_end`
- `positive_jog_moves_ball_toward_sensor`: derived plant fact from the centered-ball jog
- `recommended_stepper_dir_sign`: final firmware recommendation for the actuator-direction mapping

## Canonical Rules

- `x_phys_cm > 0` means the ball is farther toward the motor side than the runner center.
- `theta_phys_deg > 0` means the motor side is physically higher than the sensor side.
- `u_cmd > 0` is the logical positive actuator direction that we want to mean `theta_phys_deg > 0`.
- Sharp distance always increases away from the sensor face.

For Sharp-only work:

- if `sensor_mount_end = pivot`, then increasing `d_sharp_cm` means the ball moved toward the motor side
- if `sensor_mount_end = motor`, then increasing `d_sharp_cm` means the ball moved toward the pivot side

Equivalently:

- `sensor_distance_increases_toward_motor = true` when `sensor_mount_end = pivot`
- `sensor_distance_increases_toward_motor = false` when `sensor_mount_end = motor`

## Truth Table

| Physical fact | Expected result |
| --- | --- |
| motor side up | ball moves toward the sensor side |
| motor side down | ball moves toward the motor side |
| ball near the Sharp | `d_sharp_cm` is smaller |
| ball far from the Sharp | `d_sharp_cm` is larger |

Audit interpretation:

- if positive jog raises the motor side, then positive jog should also move the ball toward the sensor side
- if positive jog raises the sensor side, then positive jog should move the ball toward the motor side
- if those two observations disagree, treat the result as a sign contradiction and do not patch PID math yet

## Firmware Mapping Policy

- Fix actuator-direction mistakes at the stepper boundary first.
- Do not compensate for a reversed actuator by flipping unrelated PID, sensor, or setpoint signs.
- `recommended_stepper_dir_sign = +1` means the current logical positive jog already raises the motor side.
- `recommended_stepper_dir_sign = -1` means the current logical positive jog raises the sensor side and the actuator-direction mapping must be inverted.

For the reference PID path in `firmware/src/main.cpp`:

- positive command is intended to produce motor-side-up motion
- therefore the reference PID actuator mapping is correct only when `positive_jog_raises_motor = true`

## Runtime Calibration Notes

Closed-loop mode is blocked until limits, center/trim, and sign calibration are complete.

Required order:

1. Travel limit capture
2. Sign calibration
3. Optional center override
4. Save runtime calibration

Quick key map:

- `p`: manual center override
- `[`/`l`: capture lower actuator limit
- `]`/`u`: capture upper actuator limit
- `b`: stepper sign jog
- `v`: save calibration / learned trim

Operational notes:

- `GUIDE,...` lines tell you the next required action.
- `BLOCK,...` lines explain exactly why `run` was denied.
- `t` toggles telemetry quickly.
- `x` prints decoded fault reasons and compact diagnostics.
- `l/u` derive the midpoint automatically when that runtime path is active.
- AS5600 is used as a bounded actuator-position sensor in the archived runtime path.
