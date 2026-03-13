# Sharp Retry

## Winner

- Sensor: `sharp`
- Mode: `analog`
- Target: `table_tennis_40mm`
- Timer1 stress: `1`
- Block label: `sharp_retry`
- Minimum reliable window across repeats: `0` points
- Median round-trip mismatch: `6.275` cm
- Median stddev: `1.5643` cm
- Median invalid rate: `0.3012`

## Group Summary

| sensor_name | sensor_mode | target_name | timer1_stress | block_label | runs | min_longest_reliable_window_points | median_longest_reliable_window_points | median_roundtrip_mismatch_cm | median_stddev_cm | median_invalid_rate |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| sharp | analog | table_tennis_40mm | 1 | sharp_retry | 2 | 0 | 0.0 | 6.2750 | 1.5643 | 0.3012 |

## Run Summary

| run_stem | sensor_name | sensor_mode | target_name | block_label | repeat_index | timer1_stress | total_paired_points | reliable_points | longest_reliable_window_points | reliable_window_start | reliable_window_end | median_roundtrip_mismatch_cm | median_stddev_cm | mean_valid_fraction | invalid_rate | mount_note | photo_ref | notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| run_20260313_155332 | sharp | analog | table_tennis_40mm | sharp_retry | 1 | 1 | 9 | 0 | 0 |  |  | 7.6000 | 1.6904 | 0.6733 | 0.3267 | Sharp retry, capacitor installed | sharp_mount_retry_01 | Sharp retry repeat 1 |
| run_20260313_155519 | sharp | analog | table_tennis_40mm | sharp_retry | 2 | 1 | 9 | 0 | 0 |  |  | 4.9500 | 1.4383 | 0.7244 | 0.2756 | Sharp retry, capacitor installed | sharp_mount_retry_01 | Sharp retry repeat 2 |

## Outputs

- Run CSV: `docs/experiments/2026-03-sensor-selection/generated/sharp_retry/characterization_run_summary.csv`
- Pair CSV: `docs/experiments/2026-03-sensor-selection/generated/sharp_retry/characterization_pair_metrics.csv`
- Group CSV: `docs/experiments/2026-03-sensor-selection/generated/sharp_retry/characterization_group_summary.csv`
- Plots: `docs/experiments/2026-03-sensor-selection/generated/sharp_retry/plots`
