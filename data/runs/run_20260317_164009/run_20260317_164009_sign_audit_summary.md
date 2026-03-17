# Sharp Sign Audit Summary

- run: `run_20260317_164009`
- status: `pass`
- sensor mount end: `motor`
- `d_near_cm`: `17.3210`
- `d_far_cm`: `20.2684`
- `sensor_distance_increases_toward_motor`: `false`
- `positive_jog_raises_motor`: `false`
- `d_before_jog_cm`: `9.7225`
- `d_after_jog_cm`: `11.6965`
- centered positive-jog delta: `1.9740 cm`
- `positive_jog_moves_ball_toward_sensor`: `False`
- `recommended_stepper_dir_sign`: `-1`
- `reference_pid_actuator_mapping_ok`: `False`

## Recommendation

- Generic actuator-direction recommendation: `-1`
- Current reference PID path: `invert the actuator direction at the stepper boundary`

## Decision Rules Applied

- `d_near_cm < d_far_cm` is required for Sharp geometry to be considered valid.
- Positive jog should move the ball toward the sensor exactly when positive jog raises the motor side.
- Contradictions block any firmware sign change recommendation.

