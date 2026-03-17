# Modeling Source of Truth

Canonical modeling and control documents:
- `docs/modeling.md`
- `docs/reference_pid_first_principles_derivation.md`

Interpretation rule:
- the authoritative controller architecture is the reference PID sketch in
  `firmware/src/main.cpp`
- the authoritative first-principles design artifacts are in
  `model/first_principles/`
- the generated gains in `model/first_principles/controller_initial_gains.json`
  and `firmware/include/generated/controller_gains.h` are the seed values for
  that architecture

Companion references:
- active measured parameters: `model/first_principles/params_measured.yaml`
- reduced linear model: `model/first_principles/linearized_model.json`
- measured-parameter worksheet: `docs/modeling_measured_calculations.md`
- archived empirical runtime notes: `docs/tuning.md`

If code and math drift, reconcile both in the same change set.
