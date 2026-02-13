#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot cleaned run telemetry")
    parser.add_argument("--input", type=Path, required=True, help="Path to *_clean.csv")
    parser.add_argument("--output", type=Path, default=None, help="Output image path")
    args = parser.parse_args()

    if args.output is None:
        args.output = args.input.with_suffix(".png")

    df = pd.read_csv(args.input)
    t_s = df["t_ms"] / 1000.0

    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)

    axes[0].plot(t_s, df["x_cm"], label="x_cm", color="#1d3557")
    axes[0].plot(t_s, df["x_filt_cm"], label="x_filt_cm", color="#457b9d", alpha=0.85)
    axes[0].set_ylabel("Position (cm)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_s, df["theta_deg"], label="theta_deg", color="#e63946")
    axes[1].plot(t_s, df["theta_cmd_deg"], label="theta_cmd_deg", color="#f4a261", alpha=0.9)
    axes[1].set_ylabel("Angle (deg)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t_s, df["u_step_rate"], label="u_step_rate", color="#2a9d8f")
    axes[2].set_ylabel("Step Rate (sps)")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t_s, df["fault_flags"], label="fault_flags", color="#6a040f")
    axes[3].set_ylabel("Fault Bits")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(f"Run Plot: {args.input.name}")
    fig.tight_layout()
    fig.savefig(args.output, dpi=160)

    print("Saved:", args.output)


if __name__ == "__main__":
    main()
