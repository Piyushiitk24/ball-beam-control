# Tuning Workflow

Use data-driven tuning with telemetry logs and plots.

## Telemetry Format

`t_ms,state,x_cm,x_filt_cm,theta_deg,theta_cmd_deg,u_step_rate,fault_flags`

## Workflow

1. Capture serial run:
```bash
python analysis/capture_serial.py --port <SERIAL_PORT> --seconds 30
```

2. Parse raw log:
```bash
python analysis/parse_log.py --input data/runs/<run>_raw.log
```

3. Plot run:
```bash
python analysis/plot_run.py --input data/runs/<run>_clean.csv
```

4. Compare multiple runs:
```bash
python analysis/compare_runs.py --inputs data/runs/run1_clean.csv data/runs/run2_clean.csv
```

## Metrics to Monitor

- rise time
- overshoot
- settling time
- steady-state error
- command saturation duration
