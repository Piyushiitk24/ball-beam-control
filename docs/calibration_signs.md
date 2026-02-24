# Runtime Calibration Procedure

Closed-loop mode is blocked until zero, limits, and sign calibration are complete.

## Coordinate Convention

- `theta_deg > 0`: beam tilt that should drive ball toward `+x`.
- `x_cm > 0`: ball position toward chosen positive end from center.
- `u_step_rate > 0`: command that should increase `theta_deg`.

## Required Order

1. Zero capture
2. Travel limit capture
3. Sign calibration
4. Save runtime calibration

## Preferred Workflow: Guided Wizard

1. Start wizard:
- `w`

2. Follow each printed `WIZ,...` prompt and press:
- `n`

Wizard step sequence:
- Step 1: level beam (parallel) -> capture angle zero
- Step 2: place ball at center -> capture position zero
- Step 3: move beam to lower mechanical limit -> capture lower
- Step 4: move beam to upper mechanical limit -> capture upper
- Step 5: keep ball near reference end -> run sign begin (auto jog)
- Step 6: move ball to far `+x` end -> run sign save
- Step 7: review summary and persist

3. Save and exit wizard:
- `c`

4. Abort wizard without saving:
- `q`

Notes:
- Wizard auto-disables telemetry on start and restores prior telemetry state on exit.
- Wizard writes EEPROM only when you confirm with `c`.

## Manual Workflow (Quick Keys)

Use this if you do not want wizard mode:

```text
t
a
p
l
u
b
g
v
t
s
r
```

Quick key map:
- `a`: `cal_zero set angle`
- `p`: `cal_zero set position`
- `l`: `cal_limits set lower`
- `u`: `cal_limits set upper`
- `b`: `cal_sign begin`
- `g`: `cal_sign save`
- `v`: `cal_save`

## Notes

- `GUIDE,...` lines tell you the next required action.
- `BLOCK,...` lines explain exactly why `run` was denied.
- `t` toggles telemetry quickly (or use `telemetry 0|1`).
- `i` (or `guide`) prints full bring-up step menu.
- `x` (or `faults`) prints decoded fault reasons and recovery hint.
- `cal_load` restores saved calibration at runtime.
- `cal_reset defaults` restores compile-time fallback defaults at runtime.
