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

## Commands

### 1) Zero capture (neutral beam/ball position)

- `cal_zero set angle`
- `cal_zero set position`
- `cal_zero show`

### 2) Travel limits (manual linkage extremes)

- Move to lower extreme, then: `cal_limits set lower`
- Move to upper extreme, then: `cal_limits set upper`
- Verify: `cal_limits show`

### 3) Sign calibration

1. `cal_sign begin`
- Captures initial sonar reading as near reference.
- Applies a short positive jog.
- Reads AS5600 before/after jog.
- Computes recommended sign updates.

2. Move ball to far end manually.

3. `cal_sign save`
- Captures far sonar reading.
- Computes sign suggestions.
- Applies sign values at runtime immediately.
- Emits `SIGN_CAL_RESULT` line for logs.

### 4) Persist to EEPROM

- `cal_save`

## Notes

- `telemetry 0` can be used before calibration commands to reduce console spam.
- `telemetry 1` re-enables telemetry.
- `cal_load` restores saved calibration at runtime.
- `cal_reset defaults` restores compile-time fallback defaults at runtime.
