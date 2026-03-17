#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from first_principles_core import (
    cm_per_s2_per_step,
    load_params,
    outer_model_coeffs,
    stepper_rad_per_step,
)


def linearized_reference_model(params: dict) -> dict:
    alpha, beta = outer_model_coeffs(params)
    kappa_cm = cm_per_s2_per_step(params)
    kappa_m = kappa_cm / 100.0
    step_rad = stepper_rad_per_step(params)

    a = np.array(
        [
            [0.0, 1.0],
            [0.0, -beta],
        ],
        dtype=float,
    )
    b = np.array(
        [
            [0.0],
            [kappa_m],
        ],
        dtype=float,
    )
    c = np.eye(2, dtype=float)
    d = np.zeros((2, 1), dtype=float)

    return {
        "state_order": ["x_m", "x_dot_mps"],
        "input_order": ["n_cmd_steps"],
        "A": a.tolist(),
        "B": b.tolist(),
        "C": c.tolist(),
        "D": d.tolist(),
        "meta": {
            "method": "linearized-reference-pid-plant",
            "model": "x_ddot + beta*x_dot = alpha*g*k_step_rad*n_cmd_steps",
            "alpha": alpha,
            "beta_1ps": beta,
            "rad_per_step": step_rad,
            "kappa_cmps2_per_step": kappa_cm,
            "transfer_function": {
                "output": "X_cm(s)",
                "input": "N_cmd_steps(s)",
                "numerator": [kappa_cm],
                "denominator": [1.0, beta, 0.0],
            },
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Linearize the reduced single-loop reference PID plant"
    )
    parser.add_argument(
        "--params",
        type=Path,
        default=Path(__file__).with_name("params_template.yaml"),
        help="Path to parameter yaml",
    )
    args = parser.parse_args()

    payload = linearized_reference_model(load_params(args.params))
    out_path = Path(__file__).with_name("linearized_model.json")
    with out_path.open("w", encoding="utf-8") as f_out:
        json.dump(payload, f_out, indent=2)

    print("Linearized reduced reference PID model around x=0, x_dot=0, theta=0")
    print("Saved:", out_path)
    print("A=\n", np.array(payload["A"], dtype=float))
    print("B=\n", np.array(payload["B"], dtype=float))


if __name__ == "__main__":
    main()
