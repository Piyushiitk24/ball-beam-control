# Sign Calibration Procedure

Closed-loop mode is blocked until sign calibration is complete.

## Coordinate Convention

- `theta_deg > 0`: beam tilt that should drive ball toward `+x`.
- `x_cm > 0`: ball position toward chosen positive end from center.
- `u_step_rate > 0`: command that should increase `theta_deg`.

## Commands

1. `cal_sign begin`
- Captures initial sonar reading as near reference.
- Applies a short positive jog.
- Reads AS5600 before and after jog.
- Computes recommended sign updates.

2. Move ball to far end manually.

3. `cal_sign save`
- Captures far sonar reading.
- Computes recommended `SONAR_POS_SIGN`.
- Emits `SIGN_CAL_RESULT` line for log capture.

## Apply Recommendations

Update compile-time placeholders in `firmware/include/calibration.h`:
- `AS5600_SIGN`
- `STEPPER_DIR_SIGN`
- `SONAR_POS_SIGN`

Then rebuild and repeat until positive command and positive measured `theta_deg` are consistent.
