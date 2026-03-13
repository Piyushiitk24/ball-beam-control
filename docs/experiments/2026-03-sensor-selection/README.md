# Sensor and Target Selection

This experiment track compares the available ball-position sensors and candidate rolling targets before any further controller tuning.

Status:
- this remains archived exploratory evidence
- the active controller path has reverted to the archived HC-SR04 runtime
- current closed-loop validation is center/far only, not sensor-shootout driven

Decision order:
- compare `Sharp GP2Y0A21YK0F` vs `HC-SR04` using the `40 mm` table tennis ball
- keep the winning sensor mounted
- compare `table_tennis_40mm` vs `golf_ball`
- define the final closed-loop operating window from the winning sensor-target pair

## Capture Workflow

1. Mark the runner at fixed points `p00 ... pNN`, about `2 cm` apart.
2. Mount one sensor only. Do not switch sensors within the same block.
3. Capture one full outbound and return pass with explicit labels:
   - `out_p00 ... out_pNN`
   - `/reverse`
   - `ret_pNN ... ret_p00`
4. Repeat the same block once more with the same sensor/target pair.
5. Generate a characterization report from those run folders.

## Commands

Sharp:

```bash
cd /Users/piyush/code/ball-beam-control/firmware
pio run -e sharp_ir_check -t upload --upload-port /dev/cu.usbserial-A10N20X1

cd /Users/piyush/code/ball-beam-control
./.venv/bin/python analysis/capture_sharp_points.py \
  --port /dev/cu.usbserial-A10N20X1 \
  --target-name table_tennis_40mm \
  --block-label sharp_block \
  --repeat-index 1 \
  --timer1-stress 1
```

HC-SR04:

```bash
cd /Users/piyush/code/ball-beam-control/firmware
pio run -e hcsr04_check -t upload --upload-port /dev/cu.usbserial-A10N20X1

cd /Users/piyush/code/ball-beam-control
./.venv/bin/python analysis/capture_hcsr04_points.py \
  --port /dev/cu.usbserial-A10N20X1 \
  --mode pcint \
  --target-name table_tennis_40mm \
  --block-label hcsr04_block \
  --repeat-index 1 \
  --timer1-stress 1
```

Report:

```bash
cd /Users/piyush/code/ball-beam-control
./.venv/bin/python analysis/report_sensor_characterization.py \
  --input data/runs/run_<timestamp_a> \
  --input data/runs/run_<timestamp_b> \
  --output-dir docs/experiments/2026-03-sensor-selection/generated/<phase_name> \
  --title "<phase_name>"
```

## Per-Run Artifacts

Each characterization run saves:
- `run_<timestamp>_raw.log`
- `run_<timestamp>_*_points.csv`
- `run_<timestamp>_*_point_samples.csv`
- `run_<timestamp>_characterization_meta.json`

The metadata JSON stores the sensor name, sensor mode, target, block label, repeat index, mount note, photo reference, and whether Timer1 stress was enabled.
