# Modeling Source of Truth

The single canonical modeling document is:

- `docs/modeling.md`

It contains the complete first-principles derivation (Lagrangian mechanics → Euler–Lagrange
equations → nonlinear model → linearization → PID design → LQR design), all measured
hardware parameters with numerical substitutions, and the controller design workflow.

## Authoritative Implementation

The authoritative controller implementation is the reference PID sketch in
`firmware/src/main.cpp`.

## Companion Artifacts

These files are generated from or consistent with `docs/modeling.md`:

| File | Role |
| ------- | ----- |
| `model/first_principles/params_measured.yaml` | Hardware parameter values |
| `model/first_principles/linearized_model.json` | Generated linear state-space matrices |
| `model/first_principles/controller_initial_gains.json` | Generated seed PID gains |
| `firmware/include/generated/controller_gains.h` | Generated gains header for firmware |
| `firmware/include/config.h` | Runtime-tuned gains and configuration |

## Sync Rule

If the active controller architecture or hardware parameters change, reconcile all of
the above in the same change set. Code and math must never drift.
