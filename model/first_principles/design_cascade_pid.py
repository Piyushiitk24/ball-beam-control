#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from math import pi
from pathlib import Path

from first_principles_core import (
    cm_per_s2_per_step,
    effective_mass_kg,
    load_params,
    outer_model_coeffs,
    reference_pid_output_limit_steps,
    reference_pid_physical_to_step_gains,
    stepper_rad_per_step,
)


def design_gains(params: dict) -> dict:
    g = float(params["plant"]["gravity_mps2"])
    m_e = effective_mass_kg(params)
    alpha, beta = outer_model_coeffs(params)
    step_rad = stepper_rad_per_step(params)
    kappa = cm_per_s2_per_step(params)

    design = params["design"]
    outer_bw_hz = float(design["outer_bw_hz"])
    outer_zeta = float(design["outer_zeta"])
    extra_pole_factor = float(design["extra_pole_factor"])

    wn = 2.0 * pi * outer_bw_hz
    p3 = extra_pole_factor * wn
    a2 = (2.0 * outer_zeta * wn) + p3
    a1 = (wn**2) + (2.0 * outer_zeta * wn * p3)
    a0 = (wn**2) * p3

    k_theta_to_xddot_m = alpha * g
    kp_phys = a1 / k_theta_to_xddot_m
    ki_phys = a0 / k_theta_to_xddot_m
    kd_phys = (a2 - beta) / k_theta_to_xddot_m

    kp_step, ki_step, kd_step = reference_pid_physical_to_step_gains(
        kp_phys,
        ki_phys,
        kd_phys,
        params,
    )

    out_limit_steps = reference_pid_output_limit_steps(params)
    out_limit_rad = out_limit_steps * step_rad
    modeled_theta_limit_deg = float(params["limits"]["theta_cmd_deg"])

    return {
        "meta": {
            "method": "linearized-first-principles-reference-pid",
            "controller_architecture": "single-loop-absolute-position-pid",
            "controller_output": "absolute_microstep_target",
            "notes": [
                "This design matches firmware/src/main.cpp reference PID architecture.",
                "The plant assumes an ideal beam-angle actuator and small-angle ball-beam linearization.",
                "The current software output clamp matches the reference sketch and exceeds the small-angle modeling limit.",
            ],
            "units": {
                "physical_error": "m",
                "physical_output": "rad",
                "step_error": "cm",
                "step_output": "steps",
                "physical_kp": "rad/m",
                "physical_ki": "rad/(m*s)",
                "physical_kd": "rad*s/m",
                "kp": "steps/cm",
                "ki": "steps/(cm*s)",
                "kd": "step*s/cm",
            },
            "effective_mass_kg": m_e,
            "alpha": alpha,
            "beta_1ps": beta,
            "rad_per_step": step_rad,
            "kappa_cmps2_per_step": kappa,
            "design_targets": {
                "outer_bw_hz": outer_bw_hz,
                "outer_zeta": outer_zeta,
                "extra_pole_factor": extra_pole_factor,
                "wn_radps": wn,
                "p3_radps": p3,
            },
            "desired_polynomial": {
                "a2": a2,
                "a1": a1,
                "a0": a0,
            },
            "model_theta_limit_deg": modeled_theta_limit_deg,
            "software_output_limit_steps": out_limit_steps,
            "software_output_limit_rad": out_limit_rad,
            "software_output_limit_deg": out_limit_rad * (180.0 / pi),
        },
        "physical_pid": {
            "kp": kp_phys,
            "ki": ki_phys,
            "kd": kd_phys,
        },
        "pid": {
            "kp": kp_step,
            "ki": ki_step,
            "kd": kd_step,
        },
        "out_min_steps": -out_limit_steps,
        "out_max_steps": out_limit_steps,
    }


def print_derivation(gains: dict) -> None:
    meta = gains["meta"]
    phys = gains["physical_pid"]
    step = gains["pid"]
    poly = meta["desired_polynomial"]
    targets = meta["design_targets"]

    print("Reference PID first-principles design")
    print("------------------------------------")
    print(f"m_e = {meta['effective_mass_kg']:.9f} kg")
    print(f"alpha = {meta['alpha']:.9f}")
    print(f"beta = {meta['beta_1ps']:.9f} 1/s")
    print(f"k_step_rad = {meta['rad_per_step']:.12f} rad/step")
    print(f"kappa = {meta['kappa_cmps2_per_step']:.9f} cm/s^2/step")
    print()
    print(f"wn = 2*pi*{targets['outer_bw_hz']:.6f} = {targets['wn_radps']:.9f} rad/s")
    print(
        f"p3 = {targets['extra_pole_factor']:.6f}*wn = {targets['p3_radps']:.9f} rad/s"
    )
    print(f"a2 = 2*zeta*wn + p3 = {poly['a2']:.9f}")
    print(f"a1 = wn^2 + 2*zeta*wn*p3 = {poly['a1']:.9f}")
    print(f"a0 = wn^2*p3 = {poly['a0']:.9f}")
    print()
    print("Physical PID gains:")
    print(f"  Kp = {phys['kp']:.9f} rad/m")
    print(f"  Ki = {phys['ki']:.9f} rad/(m*s)")
    print(f"  Kd = {phys['kd']:.9f} rad*s/m")
    print()
    print("Microstep-target PID gains:")
    print(f"  Kp = {step['kp']:.9f} steps/cm")
    print(f"  Ki = {step['ki']:.9f} steps/(cm*s)")
    print(f"  Kd = {step['kd']:.9f} step*s/cm")
    print()
    print(
        f"Output clamp = [{gains['out_min_steps']:.3f}, {gains['out_max_steps']:.3f}] steps"
    )
    print(
        "Equivalent beam range = "
        f"+/-{meta['software_output_limit_deg']:.6f} deg "
        f"(model small-angle limit is +/-{meta['model_theta_limit_deg']:.6f} deg)"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Design single-loop reference PID gains from the first-principles model"
    )
    parser.add_argument(
        "--params",
        type=Path,
        default=Path(__file__).with_name("params_template.yaml"),
        help="Path to parameter yaml",
    )
    args = parser.parse_args()

    params = load_params(args.params)
    gains = design_gains(params)

    out_dir = Path(__file__).resolve().parent
    gains_path = out_dir / "controller_initial_gains.json"
    with gains_path.open("w", encoding="utf-8") as f:
        json.dump(gains, f, indent=2)

    print_derivation(gains)
    print()
    print("Saved:", gains_path)


if __name__ == "__main__":
    main()
