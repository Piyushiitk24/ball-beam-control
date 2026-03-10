# Runtime Calibration Procedure

Closed-loop mode is blocked until limits, center/trim, and sign calibration are complete.

## Coordinate Convention

- `theta_deg > 0`: beam tilt that should drive ball toward `+x`.
- `x_cm > 0`: ball position toward chosen positive end from center.
- `u_step_rate > 0`: command that should increase `theta_deg`.
- Hardware orientation used by firmware: physical UP limit = ball near sensor.

## Required Order

1. Travel limit capture
2. Sign calibration
3. Optional sonar-center override
4. Save runtime calibration

## Manual Workflow (Quick Keys)

Use this workflow:

```text
t
[
]
b
# optional:
# p   (manual sonar-center override)
# g   (validation only)
v
t
s
r
```

Quick key map:
- `p`: manual sonar-center override
- `[`/`l`: capture lower actuator limit
- `]`/`u`: capture upper actuator limit (also fixes sonar direction: upper = near)
- `b`: stepper sign jog
- `g`: optional sonar validation only
- `v`: save calibration / learned trim

## Notes

- `GUIDE,...` lines tell you the next required action.
- `BLOCK,...` lines explain exactly why `run` was denied.
- `t` toggles telemetry quickly.
- `x` prints decoded fault reasons and compact sonar diagnostics.
- `l/u` derive the sonar midpoint automatically; `p` is only needed as a manual override.
- AS5600 is used as a bounded actuator-position sensor; it is no longer calibrated to a manual balanced-beam zero.
- Bring-up keeps sensor faults as warnings/blockers; hard FAULT latching is enforced in `RUNNING`.
