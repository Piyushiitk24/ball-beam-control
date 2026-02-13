#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml


def load_params(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def f(state: np.ndarray, u: float, p: dict) -> np.ndarray:
    x, x_dot, theta = state

    g = float(p["plant"]["gravity_mps2"])
    c = float(p["plant"]["viscous_damping_1ps"])
    tau = float(p["plant"]["actuator_tau_s"])
    rolling = float(p["plant"]["rolling_factor"])

    x_ddot = rolling * g * np.sin(theta) - c * x_dot
    theta_dot = (u - theta) / tau

    return np.array([x_dot, x_ddot, theta_dot], dtype=float)


def jacobian_state(x0: np.ndarray, u0: float, p: dict, eps: float = 1e-6) -> np.ndarray:
    n = x0.size
    A = np.zeros((n, n), dtype=float)
    for i in range(n):
        dx = np.zeros(n, dtype=float)
        dx[i] = eps
        fp = f(x0 + dx, u0, p)
        fm = f(x0 - dx, u0, p)
        A[:, i] = (fp - fm) / (2.0 * eps)
    return A


def jacobian_input(x0: np.ndarray, u0: float, p: dict, eps: float = 1e-6) -> np.ndarray:
    fp = f(x0, u0 + eps, p)
    fm = f(x0, u0 - eps, p)
    return ((fp - fm) / (2.0 * eps)).reshape((-1, 1))


def main() -> None:
    parser = argparse.ArgumentParser(description="Linearize nonlinear ball-beam model")
    parser.add_argument(
        "--params",
        type=Path,
        default=Path(__file__).with_name("params_template.yaml"),
        help="Path to parameter yaml",
    )
    args = parser.parse_args()

    params = load_params(args.params)

    x_eq = np.array([0.0, 0.0, 0.0], dtype=float)
    u_eq = 0.0

    A = jacobian_state(x_eq, u_eq, params)
    B = jacobian_input(x_eq, u_eq, params)
    C = np.eye(3)
    D = np.zeros((3, 1), dtype=float)

    payload = {
        "state_order": ["x_m", "x_dot_mps", "theta_rad"],
        "input_order": ["theta_cmd_rad"],
        "A": A.tolist(),
        "B": B.tolist(),
        "C": C.tolist(),
        "D": D.tolist(),
    }

    out_path = Path(__file__).with_name("linearized_model.json")
    with out_path.open("w", encoding="utf-8") as f_out:
        json.dump(payload, f_out, indent=2)

    print("Linearized model around x=0, x_dot=0, theta=0")
    print("Saved:", out_path)
    print("A=\n", A)
    print("B=\n", B)


if __name__ == "__main__":
    main()
