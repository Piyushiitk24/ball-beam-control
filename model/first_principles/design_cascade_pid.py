#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.integrate import solve_ivp


def load_params(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def design_gains(params: dict) -> dict:
    g = float(params["plant"]["gravity_mps2"])
    rolling = float(params["plant"]["rolling_factor"])
    tau = float(params["plant"]["actuator_tau_s"])

    k_theta_to_xddot = rolling * g

    inner_bw_hz = float(params["design"]["inner_bw_hz"])
    inner_zeta = float(params["design"]["inner_zeta"])
    outer_bw_hz = float(params["design"]["outer_bw_hz"])
    outer_zeta = float(params["design"]["outer_zeta"])
    extra_pole_factor = float(params["design"]["extra_pole_factor"])

    wn_i = 2.0 * np.pi * inner_bw_hz
    wn_o = 2.0 * np.pi * outer_bw_hz

    # PI on first-order actuator approximation: tau*s^2 + (1+Kp)s + Ki = 0
    kpi = max(0.05, (2.0 * inner_zeta * wn_i * tau) - 1.0)
    kii = max(0.05, (wn_i**2) * tau)
    kdi = 0.0

    p3 = extra_pole_factor * wn_o
    a2 = (2.0 * outer_zeta * wn_o) + p3
    a1 = (wn_o**2) + (2.0 * outer_zeta * wn_o * p3)
    a0 = (wn_o**2) * p3

    # For plant k/s^2 with PID: s^3 + k*Kd*s^2 + k*Kp*s + k*Ki = 0
    kpo = a1 / k_theta_to_xddot
    kdo = a2 / k_theta_to_xddot
    kio = a0 / k_theta_to_xddot

    theta_lim_deg = float(params["limits"]["theta_cmd_deg"])
    theta_lim_rad = np.deg2rad(theta_lim_deg)
    step_lim = float(params["limits"]["step_rate_sps"])

    eps = 1e-6
    inner_i_lim = step_lim / max(abs(kii), eps)
    outer_i_lim = theta_lim_rad / max(abs(kio), eps)

    return {
        "meta": {
            "method": "linearized-first-principles-cascade",
            "k_theta_to_xddot": k_theta_to_xddot,
            "inner_bw_hz": inner_bw_hz,
            "outer_bw_hz": outer_bw_hz,
            "units": {
                "outer_error": "m",
                "outer_output": "rad",
                "inner_error": "rad",
                "inner_output": "step_per_s",
                "outer_integral_state": "m*s",
                "inner_integral_state": "rad*s",
            },
        },
        "inner": {
            "kp": float(kpi),
            "ki": float(kii),
            "kd": float(kdi),
            "i_min": -float(inner_i_lim),
            "i_max": float(inner_i_lim),
            "out_min": -step_lim,
            "out_max": step_lim,
        },
        "outer": {
            "kp": float(kpo),
            "ki": float(kio),
            "kd": float(kdo),
            "i_min": -float(outer_i_lim),
            "i_max": float(outer_i_lim),
            "out_min": -float(theta_lim_rad),
            "out_max": float(theta_lim_rad),
        },
    }


def simulate_seed_response(
    params: dict, gains: dict, t_end: float = 10.0
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    g = float(params["plant"]["gravity_mps2"])
    rolling = float(params["plant"]["rolling_factor"])
    c = float(params["plant"]["viscous_damping_1ps"])
    inner_bw = 2.0 * np.pi * float(params["design"]["inner_bw_hz"])

    theta_limit_deg = float(params["limits"]["theta_cmd_deg"])
    theta_limit_rad = np.deg2rad(theta_limit_deg)

    k = rolling * g
    ko = gains["outer"]

    def rhs(t: float, y: np.ndarray) -> np.ndarray:
        x, x_dot, theta, i_outer = y
        r = 0.0 if t < 1.0 else 0.03  # 3 cm target step (meters)

        e = r - x
        e_dot = -x_dot

        theta_cmd_rad = ko["kp"] * e + ko["ki"] * i_outer + ko["kd"] * e_dot
        theta_cmd_rad = clamp(theta_cmd_rad, ko["out_min"], ko["out_max"])
        theta_cmd_rad = clamp(theta_cmd_rad, -theta_limit_rad, theta_limit_rad)

        x_ddot = k * np.sin(theta) - c * x_dot
        theta_dot = inner_bw * (theta_cmd_rad - theta)
        i_dot = e

        return np.array([x_dot, x_ddot, theta_dot, i_dot], dtype=float)

    t_eval = np.linspace(0.0, t_end, 1500)
    sol = solve_ivp(rhs, (0.0, t_end), np.zeros(4), t_eval=t_eval, rtol=1e-7, atol=1e-9)

    x = sol.y[0]
    theta_deg = np.rad2deg(sol.y[2])
    ref = np.where(sol.t >= 1.0, 0.03, 0.0)
    return sol.t, ref, x, theta_deg


def main() -> None:
    parser = argparse.ArgumentParser(description="Design initial cascade PID gains")
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

    t, ref, x, theta_deg = simulate_seed_response(params, gains)

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(t, ref * 100.0, label="Reference (cm)", color="#6a994e")
    axes[0].plot(t, x * 100.0, label="x (cm)", color="#1d3557")
    axes[0].set_ylabel("Position (cm)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(t, theta_deg, label="theta (deg)", color="#e63946")
    axes[1].set_ylabel("Beam Angle (deg)")
    axes[1].set_xlabel("Time (s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.suptitle("Seed Closed-Loop Simulation (SI-Consistent Initial Gains)")
    fig.tight_layout()

    plot_path = out_dir / "closed_loop_seed.png"
    fig.savefig(plot_path, dpi=160)

    print("Saved:", gains_path)
    print("Saved:", plot_path)


if __name__ == "__main__":
    main()
