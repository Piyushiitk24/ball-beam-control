#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.integrate import solve_ivp


def load_params(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def dynamics(t: float, state: np.ndarray, theta_cmd_rad: float, p: dict) -> np.ndarray:
    x, x_dot, theta = state

    g = float(p["plant"]["gravity_mps2"])
    c = float(p["plant"]["viscous_damping_1ps"])
    tau = float(p["plant"]["actuator_tau_s"])
    rolling = float(p["plant"]["rolling_factor"])

    x_ddot = rolling * g * np.sin(theta) - c * x_dot
    theta_dot = (theta_cmd_rad - theta) / tau

    return np.array([x_dot, x_ddot, theta_dot], dtype=float)


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
    args = parser.parse_args()

    params = load_params(args.params)
    out_dir = Path(__file__).resolve().parent

    theta_step_rad = np.deg2rad(args.theta_step_deg)

    def rhs(t: float, y: np.ndarray) -> np.ndarray:
        theta_cmd = theta_step_rad if t >= 0.4 else 0.0
        return dynamics(t, y, theta_cmd, params)

    y0 = np.array([0.0, 0.0, 0.0], dtype=float)
    t_eval = np.linspace(0.0, args.duration, 1200)
    sol = solve_ivp(rhs, (0.0, args.duration), y0, t_eval=t_eval, rtol=1e-7, atol=1e-9)

    x = sol.y[0]
    theta = np.rad2deg(sol.y[2])

    csv_path = out_dir / "open_loop_nonlinear.csv"
    np.savetxt(
        csv_path,
        np.column_stack([sol.t, x, theta]),
        delimiter=",",
        header="t_s,x_m,theta_deg",
        comments="",
    )

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(sol.t, x, label="x (m)", color="#006d77")
    axes[0].set_ylabel("Ball Position x (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(sol.t, theta, label="theta (deg)", color="#bc4749")
    axes[1].set_ylabel("Beam Angle (deg)")
    axes[1].set_xlabel("Time (s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.suptitle("Open-Loop Nonlinear Response")
    fig.tight_layout()
    plot_path = out_dir / "open_loop_nonlinear.png"
    fig.savefig(plot_path, dpi=160)

    print("Nonlinear model assumptions:")
    print("x_ddot = (5/7) * g * sin(theta) - c * x_dot")
    print("theta_dot = (theta_cmd - theta) / tau")
    print(f"Saved: {csv_path}")
    print(f"Saved: {plot_path}")


if __name__ == "__main__":
    main()
