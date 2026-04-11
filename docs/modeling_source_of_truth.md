# Modeling Source of Truth

The single canonical modeling document is:

- `docs/modeling.md`

It contains the complete first-principles derivation (Lagrangian mechanics → Euler–Lagrange
equations → nonlinear model → linearization → staged-controller design), all measured
hardware parameters with numerical substitutions, and the controller design workflow.
The current measured model reflects the pivot-mounted Sharp sensor geometry, its
tape-limited usable ball-travel window, and the staged Sharp + AS5600 controller.

## Companion Artifacts

Parameter files, linearized-model exports, controller-design scripts, generated
verification payloads, and firmware-facing constant headers are companion artifacts.
They exist to support implementation, regeneration, and cross-checking, but they do
not supersede the derivation, numerical substitutions, or design decisions recorded in
`docs/modeling.md`.

## Sync Rule

If the active controller architecture or hardware parameters change, update
`docs/modeling.md` first and reconcile any dependent artifacts in the same change set.
Code and math must never drift.
