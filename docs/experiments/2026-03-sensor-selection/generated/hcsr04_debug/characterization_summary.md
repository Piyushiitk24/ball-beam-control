# HC-SR04 Debug

## Winner

- Sensor: `hcsr04_debug`
- Mode: `pulsein`
- Target: `table_tennis_40mm`
- Timer1 stress: `0`
- Block label: `pulsein_nostress`
- Minimum reliable window across repeats: `0` points
- Median round-trip mismatch: `8.1875` cm
- Median stddev: `2.4384` cm
- Median invalid rate: `0.1978`

## Group Summary

| sensor_name | sensor_mode | target_name | timer1_stress | block_label | runs | min_longest_reliable_window_points | median_longest_reliable_window_points | median_roundtrip_mismatch_cm | median_stddev_cm | median_invalid_rate |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| hcsr04_debug | pcint | table_tennis_40mm | 0 | pcint_nostress | 1 | 0 | 0.0 | 12.7250 | 1.8337 | 0.2178 |
| hcsr04_debug | pulsein | table_tennis_40mm | 0 | pulsein_nostress | 1 | 0 | 0.0 | 8.1875 | 2.4384 | 0.1978 |
| hcsr04_debug | pcint | table_tennis_40mm | 1 | pcint_stress | 1 | 0 | 0.0 | 11.8600 | 3.0034 | 0.1600 |

## Run Summary

| run_stem | sensor_name | sensor_mode | target_name | block_label | repeat_index | timer1_stress | total_paired_points | reliable_points | longest_reliable_window_points | reliable_window_start | reliable_window_end | median_roundtrip_mismatch_cm | median_stddev_cm | mean_valid_fraction | invalid_rate | mount_note | photo_ref | notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| run_20260313_154115 | hcsr04_debug | pcint | table_tennis_40mm | pcint_nostress | 1 | 0 | 9 | 0 | 0 |  |  | 12.7250 | 1.8337 | 0.7822 | 0.2178 | current HC-SR04 mount | hcsr04_mount_01 | HC-SR04 debug pcint no stress |
| run_20260313_154352 | hcsr04_debug | pulsein | table_tennis_40mm | pulsein_nostress | 1 | 0 | 9 | 0 | 0 |  |  | 8.1875 | 2.4384 | 0.8022 | 0.1978 | current HC-SR04 mount | hcsr04_mount_01 | HC-SR04 debug pulseIn no stress |
| run_20260313_154531 | hcsr04_debug | pcint | table_tennis_40mm | pcint_stress | 1 | 1 | 9 | 0 | 0 |  |  | 11.8600 | 3.0034 | 0.8400 | 0.1600 | current HC-SR04 mount | hcsr04_mount_01 | HC-SR04 debug pcint with stress |

## Outputs

- Run CSV: `docs/experiments/2026-03-sensor-selection/generated/hcsr04_debug/characterization_run_summary.csv`
- Pair CSV: `docs/experiments/2026-03-sensor-selection/generated/hcsr04_debug/characterization_pair_metrics.csv`
- Group CSV: `docs/experiments/2026-03-sensor-selection/generated/hcsr04_debug/characterization_group_summary.csv`
- Plots: `docs/experiments/2026-03-sensor-selection/generated/hcsr04_debug/plots`
