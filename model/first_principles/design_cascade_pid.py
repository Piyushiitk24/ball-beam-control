#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

from first_principles_core import (
    actuator_defaults,
    clamp,
    full_coupled_accels,
    load_params,
    outer_model_coeffs,
    phi_from_theta,
    theta_from_phi,
    theta_tracking_torque,
)

def design_gains(params: dict) -> dict:
    g = float(params["plant"]["gravity_mps2"])
    alpha, beta = outer_model_coeffs(params)
    act = actuator_defaults(params)
    tau = float(act["phi_tau_s"])

    k_theta_to_xddot = alpha * g

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
            "alpha": alpha,
            "beta_1ps": beta,
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
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    act = actuator_defaults(params)
    theta_map_cfg = params.get("calibration", {}).get("theta_from_phi", {})

    theta_limit_deg = float(params["limits"]["theta_cmd_deg"])
    theta_limit_rad = np.deg2rad(theta_limit_deg)

    ko = gains["outer"]
    phi_eq = phi_from_theta(0.0, theta_map_cfg, direction=1.0)

    def rhs(t: float, y: np.ndarray) -> np.ndarray:
        x, x_dot, theta, theta_dot, phi, i_outer = y
        r = 0.0 if t < 1.0 else 0.03  # 3 cm target step (meters)

        e = r - x
        e_dot = -x_dot

        theta_cmd_rad = ko["kp"] * e + ko["ki"] * i_outer + ko["kd"] * e_dot
        theta_cmd_rad = clamp(theta_cmd_rad, ko["out_min"], ko["out_max"])
        theta_cmd_rad = clamp(theta_cmd_rad, -theta_limit_rad, theta_limit_rad)

        phi_cmd = phi_from_theta(theta_cmd_rad, theta_map_cfg, direction=np.sign(theta_cmd_rad - theta))
        phi_dot = (phi_cmd - phi) / max(act["phi_tau_s"], 1e-6)
        theta_ref, _, _ = theta_from_phi(phi, theta_map_cfg, direction=np.sign(phi_dot))

        tau_act = theta_tracking_torque(theta_ref, theta, theta_dot, params)
        x_ddot, theta_ddot = full_coupled_accels(x, x_dot, theta, theta_dot, tau_act, params)
        i_dot = e

        return np.array([x_dot, x_ddot, theta_dot, theta_ddot, phi_dot, i_dot], dtype=float)

    t_eval = np.linspace(0.0, t_end, 1500)
    y0 = np.array([0.0, 0.0, 0.0, 0.0, phi_eq, 0.0], dtype=float)
    sol = solve_ivp(rhs, (0.0, t_end), y0, t_eval=t_eval, rtol=1e-7, atol=1e-9)

    x = sol.y[0]
    theta_deg = np.rad2deg(sol.y[2])
    phi_deg = np.rad2deg(sol.y[4])
    ref = np.where(sol.t >= 1.0, 0.03, 0.0)

    theta_cmd_deg = np.empty_like(sol.t)
    for i in range(sol.t.size):
        e = ref[i] - x[i]
        e_dot = -sol.y[1, i]
        theta_cmd_rad = ko["kp"] * e + ko["ki"] * sol.y[5, i] + ko["kd"] * e_dot
        theta_cmd_rad = clamp(theta_cmd_rad, ko["out_min"], ko["out_max"])
        theta_cmd_rad = clamp(theta_cmd_rad, -theta_limit_rad, theta_limit_rad)
        theta_cmd_deg[i] = np.rad2deg(theta_cmd_rad)

    return sol.t, ref, x, theta_deg, theta_cmd_deg, phi_deg


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

    t, ref, x, theta_deg, theta_cmd_deg, phi_deg = simulate_seed_response(params, gains)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t, ref * 100.0, label="Reference (cm)", color="#6a994e")
    axes[0].plot(t, x * 100.0, label="x (cm)", color="#1d3557")
    axes[0].set_ylabel("Position (cm)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(t, theta_deg, label="theta (deg)", color="#e63946")
    axes[1].plot(t, theta_cmd_deg, label="theta_cmd (deg)", color="#f4a261", alpha=0.9)
    axes[1].set_ylabel("Beam Angle (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    axes[2].plot(t, phi_deg, label="phi (deg)", color="#3d405b")
    axes[2].set_ylabel("Motor Angle (deg)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    fig.suptitle("Seed Closed-Loop Simulation (Full Nonlinear Coupled Plant)")
    fig.tight_layout()

    plot_path = out_dir / "closed_loop_seed.png"
    fig.savefig(plot_path, dpi=160)

    print("Saved:", gains_path)
    print("Saved:", plot_path)


if __name__ == "__main__":
    main()
