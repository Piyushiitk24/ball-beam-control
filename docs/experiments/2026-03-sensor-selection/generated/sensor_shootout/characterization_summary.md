# Sensor Shootout

## Winner

- Sensor: `sharp`
- Target: `table_tennis_40mm`
- Minimum reliable window across repeats: `0` points
- Median round-trip mismatch: `10.712` cm
- Median stddev: `1.4292` cm
- Median invalid rate: `0.2141`

## Group Summary

| sensor_name | target_name | runs | min_longest_reliable_window_points | median_longest_reliable_window_points | median_roundtrip_mismatch_cm | median_stddev_cm | median_invalid_rate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| sharp | table_tennis_40mm | 2 | 0 | 0.0 | 10.7120 | 1.4292 | 0.2141 |
| hcsr04 | table_tennis_40mm | 2 | 0 | 0.0 | 14.2441 | 1.9086 | 0.1633 |

## Run Summary

| run_stem | sensor_name | sensor_mode | target_name | block_label | repeat_index | timer1_stress | total_paired_points | reliable_points | longest_reliable_window_points | reliable_window_start | reliable_window_end | median_roundtrip_mismatch_cm | median_stddev_cm | mean_valid_fraction | invalid_rate | mount_note | photo_ref | notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| run_20260313_150501 | sharp | analog | table_tennis_40mm | sharp_block | 1 | 1 | 10 | 0 | 0 |  |  | 10.9140 | 1.5190 | 0.7740 | 0.2260 | current Sharp mount | sharp_mount_01 | sensor shootout repeat 1 |
| run_20260313_150931 | sharp | analog | table_tennis_40mm | sharp_block | 2 | 1 | 9 | 0 | 0 |  |  | 10.5101 | 1.3394 | 0.7978 | 0.2022 | current Sharp mount | sharp_mount_01 | sensor shootout repeat 2 |
| run_20260313_151736 | hcsr04 | pcint | table_tennis_40mm | hcsr04_block | 1 | 1 | 9 | 0 | 0 |  |  | 14.8577 | 2.2193 | 0.8467 | 0.1533 | current HC-SR04 mount | hcsr04_mount_01 | sensor shootout repeat 1 |
| run_20260313_152108 | hcsr04 | pcint | table_tennis_40mm | hcsr04_block | 2 | 1 | 9 | 0 | 0 |  |  | 13.6306 | 1.5979 | 0.8267 | 0.1733 | current HC-SR04 mount | hcsr04_mount_01 | sensor shootout repeat 2 |

## Outputs

- Run CSV: `docs/experiments/2026-03-sensor-selection/generated/sensor_shootout/characterization_run_summary.csv`
- Pair CSV: `docs/experiments/2026-03-sensor-selection/generated/sensor_shootout/characterization_pair_metrics.csv`
- Group CSV: `docs/experiments/2026-03-sensor-selection/generated/sensor_shootout/characterization_group_summary.csv`
- Plots: `docs/experiments/2026-03-sensor-selection/generated/sensor_shootout/plots`
