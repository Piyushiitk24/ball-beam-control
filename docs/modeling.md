# First-Principles Modeling

All Python scripts are run from activated `.venv` created by `mkvenv`.

## Inputs

Edit:
- `model/first_principles/params_template.yaml`

Key placeholders:
- beam length
- ball radius/mass
- actuator time constant
- damping
- controller design targets

## Pipeline

1. Nonlinear dynamics and open-loop simulation:
```bash
python model/first_principles/derive_nonlinear.py
```

2. Linearization near equilibrium:
```bash
python model/first_principles/linearize.py
```

3. Initial cascaded PID design:
```bash
python model/first_principles/design_cascade_pid.py
```

4. Export gains to firmware header:
```bash
python model/first_principles/export_gains.py
```

## Outputs

- `model/first_principles/controller_initial_gains.json`
- `firmware/include/generated/controller_gains.h`
- simulation plots in `model/first_principles/`
