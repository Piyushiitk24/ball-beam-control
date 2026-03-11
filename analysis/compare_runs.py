#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare one metric across multiple run CSV files")
    parser.add_argument("--inputs", nargs="+", type=Path, required=True, help="Paths to *_telemetry.csv or *_clean.csv files")
    parser.add_argument(
        "--metric",
        type=str,
        default="x_cm",
        choices=[
            "x_cm",
            "x_filt_cm",
            "x_ref_cm",
            "x_linear_cm",
            "x_linear_filt_cm",
            "x_ctrl_cm",
            "x_ctrl_filt_cm",
            "x_feedback_cm",
            "feedback_blend",
            "theta_deg",
            "theta_cmd_deg",
            "theta_cmd_unclamped_deg",
            "theta_cmd_clamped_deg",
            "act_deg_abs",
            "trim_deg",
            "u_step_rate",
            "sonar_age_ms",
            "sonar_miss_count",
        ],
        help="Metric to compare",
    )
    parser.add_argument("--output", type=Path, default=Path("compare_runs.png"), help="Output plot path")
    args = parser.parse_args()

    plt.figure(figsize=(10, 5.5))
    for csv_path in args.inputs:
        df = pd.read_csv(csv_path)
        if args.metric not in df.columns:
            print(f"Skipping {csv_path}: missing column {args.metric}")
            continue
        t_s = df["t_ms"] / 1000.0
        plt.plot(t_s, df[args.metric], label=csv_path.stem)

    plt.title(f"Run Comparison: {args.metric}")
    plt.xlabel("Time (s)")
    plt.ylabel(args.metric)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.output, dpi=160)

    print("Saved:", args.output)


if __name__ == "__main__":
    main()
