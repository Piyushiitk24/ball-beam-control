#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare one metric across multiple run CSV files")
    parser.add_argument("--inputs", nargs="+", type=Path, required=True, help="Paths to *_clean.csv files")
    parser.add_argument(
        "--metric",
        type=str,
        default="x_cm",
        choices=["x_cm", "x_filt_cm", "theta_deg", "theta_cmd_deg", "u_step_rate"],
        help="Metric to compare",
    )
    parser.add_argument("--output", type=Path, default=Path("compare_runs.png"), help="Output plot path")
    args = parser.parse_args()

    plt.figure(figsize=(10, 5.5))
    for csv_path in args.inputs:
      df = pd.read_csv(csv_path)
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
