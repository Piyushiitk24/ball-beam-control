# Modeling Source of Truth

The single canonical modeling document is:

- `docs/modeling.md`

It contains the complete first-principles derivation (Lagrangian mechanics → Euler–Lagrange
equations → nonlinear model → linearization → staged-controller design), all measured
hardware parameters with numerical substitutions, and the controller design workflow.
The current measured model reflects the pivot-mounted Sharp sensor geometry, its
tape-limited usable ball-travel window, and the staged Sharp + AS5600 controller.

## Authoritative Controller Artifacts

The authoritative staged-controller constants are the generated artifacts:

- `model/first_principles/staged_controller_constants.json`
- `firmware/include/generated/staged_controller_constants.h`

These are produced by `model/first_principles/design_staged_controller.py` from
`model/first_principles/params_measured.yaml`.

The current `firmware/src/main.cpp` has not yet been migrated to this staged Sharp +
AS5600 controller architecture, so it is not the source of truth for the new gains.

## Companion Artifacts

These files are generated from or consistent with `docs/modeling.md` and the active
measured Sharp hardware geometry:

| File | Role |
| ------- | ----- |
| `model/first_principles/params_measured.yaml` | Hardware parameter values |
| `model/first_principles/linearized_model.json` | Generated linear state-space matrices |
| `model/first_principles/design_staged_controller.py` | Generates staged-controller constants |
| `model/first_principles/staged_controller_constants.json` | Generated staged-controller constants + verification |
| `firmware/include/generated/staged_controller_constants.h` | Generated staged-controller header |

Historical-only artifacts from the archived single-loop reference PID path remain in
the repo for comparison:

- `model/first_principles/design_cascade_pid.py`
- `model/first_principles/controller_initial_gains.json`
- `firmware/include/generated/controller_gains.h`

## Sync Rule

If the active controller architecture or hardware parameters change, reconcile all of
the above in the same change set. Code and math must never drift.
