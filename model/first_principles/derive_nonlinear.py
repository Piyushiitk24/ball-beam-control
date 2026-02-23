#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

from first_principles_core import (
    actuator_defaults,
    d_from_x_theta,
    full_coupled_accels,
    load_params,
    phi_from_theta,
    theta_from_phi,
    theta_tracking_torque,
    x_from_d_theta,
)

def make_phi_command_profile(
    theta_step_rad: float,
    theta_map_cfg: dict,
) -> tuple[float, float]:
    phi_eq = phi_from_theta(0.0, theta_map_cfg, direction=1.0)
    phi_step = phi_from_theta(theta_step_rad, theta_map_cfg, direction=np.sign(theta_step_rad))
    return phi_eq, phi_step


def main() -> None:
    parser = argparse.ArgumentParser(description="Nonlinear ball-beam simulation")
    parser.add_argument(
        "--params",
        type=Path,
        default=Path(__file__).with_name("params_template.yaml"),
        help="Path to parameter yaml",
    )
    parser.add_argument("--duration", type=float, default=6.0, help="Simulation duration [s]")
    parser.add_argument(
        "--theta-step-deg",
        type=float,
        default=4.0,
        help="Beam angle command step magnitude [deg]",
    )
    parser.add_argument(
        "--step-start-s",
        type=float,
        default=0.4,
        help="Time for command step start [s]",
    )
    args = parser.parse_args()

    params = load_params(args.params)
    out_dir = Path(__file__).resolve().parent

    theta_step_rad = np.deg2rad(args.theta_step_deg)
    theta_map_cfg = params.get("calibration", {}).get("theta_from_phi", {})
    sonar_map_cfg = params.get("calibration", {}).get("x_from_d_theta", {})
    act = actuator_defaults(params)

    phi_eq, phi_step = make_phi_command_profile(theta_step_rad, theta_map_cfg)

    def rhs(t: float, y: np.ndarray) -> np.ndarray:
        x, x_dot, theta, theta_dot, phi = y

        phi_cmd = phi_step if t >= args.step_start_s else phi_eq
        phi_dot = (phi_cmd - phi) / max(act["phi_tau_s"], 1e-6)
        theta_ref, _, _ = theta_from_phi(phi, theta_map_cfg, direction=np.sign(phi_dot))

        tau_act = theta_tracking_torque(theta_ref, theta, theta_dot, params)
        x_ddot, theta_ddot = full_coupled_accels(x, x_dot, theta, theta_dot, tau_act, params)

        return np.array([x_dot, x_ddot, theta_dot, theta_ddot, phi_dot], dtype=float)

    y0 = np.array([0.0, 0.0, 0.0, 0.0, phi_eq], dtype=float)
    t_eval = np.linspace(0.0, args.duration, 1200)
    sol = solve_ivp(rhs, (0.0, args.duration), y0, t_eval=t_eval, rtol=1e-7, atol=1e-9)

    x = sol.y[0]
    theta_rad = sol.y[2]
    theta_deg = np.rad2deg(theta_rad)
    phi_rad = sol.y[4]
    phi_deg = np.rad2deg(phi_rad)

    phi_cmd_rad = np.where(sol.t >= args.step_start_s, phi_step, phi_eq)
    phi_cmd_deg = np.rad2deg(phi_cmd_rad)

    phi_dot = (phi_cmd_rad - phi_rad) / max(act["phi_tau_s"], 1e-6)
    theta_hat_rad = np.array(
        [theta_from_phi(p, theta_map_cfg, direction=np.sign(pd))[0] for p, pd in zip(phi_rad, phi_dot)],
        dtype=float,
    )
    theta_hat_deg = np.rad2deg(theta_hat_rad)

    d_meas_m = np.empty_like(x)
    x_hat_m = np.empty_like(x)
    for i in range(x.size):
        try:
            d = d_from_x_theta(x[i], theta_rad[i], sonar_map_cfg)
            x_hat = x_from_d_theta(d, theta_hat_rad[i], sonar_map_cfg)
        except ValueError:
            d = np.nan
            x_hat = np.nan
        d_meas_m[i] = d
        x_hat_m[i] = x_hat

    csv_path = out_dir / "open_loop_nonlinear.csv"
    np.savetxt(
        csv_path,
        np.column_stack([sol.t, x, x_hat_m, d_meas_m, theta_deg, theta_hat_deg, phi_deg, phi_cmd_deg]),
        delimiter=",",
        header="t_s,x_m,x_hat_m,d_meas_m,theta_deg,theta_hat_deg,phi_deg,phi_cmd_deg",
        comments="",
    )

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(sol.t, x, label="x true (m)", color="#006d77")
    axes[0].plot(sol.t, x_hat_m, label="x from g(d,theta) (m)", color="#5fa8d3", alpha=0.9)
    axes[0].set_ylabel("Ball Position (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(sol.t, theta_deg, label="theta true (deg)", color="#bc4749")
    axes[1].plot(sol.t, theta_hat_deg, label="theta from f(phi) (deg)", color="#f4a261", alpha=0.9)
    axes[1].set_ylabel("Beam Angle (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    axes[2].plot(sol.t, phi_deg, label="phi (deg)", color="#3d405b")
    axes[2].plot(sol.t, phi_cmd_deg, label="phi_cmd (deg)", color="#81b29a", alpha=0.9)
    axes[2].set_ylabel("Motor Angle (deg)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    fig.suptitle("Open-Loop Nonlinear Response (Full Coupled Model + Calibration Maps)")
    fig.tight_layout()
    plot_path = out_dir / "open_loop_nonlinear.png"
    fig.savefig(plot_path, dpi=160)

    print("Nonlinear model assumptions:")
    print("Full coupled first-principles plant with repo sign convention:")
    print("  theta > 0 drives +x at small angles")
    print("End-to-end path:")
    print("  phi_cmd -> phi (1st-order motor model) -> theta_ref=f(phi) -> tau_act -> [x, theta]")
    print("Sensor map:")
    print("  x_hat = g(d_meas, theta_hat), theta_hat = f(phi)")
    print(f"Saved: {csv_path}")
    print(f"Saved: {plot_path}")


if __name__ == "__main__":
    main()
