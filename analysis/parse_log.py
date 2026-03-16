#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

COLUMNS = [
    "t_ms",
    "state",
    "x_cm",
    "x_filt_cm",
    "theta_deg",
    "theta_cmd_deg",
    "u_step_rate",
    "fault_flags",
    "x_ref_cm",
    "sonar_age_ms",
    "sonar_valid",
    "sonar_miss_count",
    "theta_cmd_unclamped_deg",
    "theta_cmd_clamped_deg",
    "theta_cmd_saturated",
    "act_deg_abs",
    "trim_deg",
    "search_phase",
    "x_linear_cm",
    "x_linear_filt_cm",
    "x_ctrl_cm",
    "x_ctrl_filt_cm",
    "x_feedback_cm",
    "feedback_blend",
    "pos_fresh",
    "pos_held",
    "pos_control_usable",
    "control_hold_active",
]


def parse_telemetry_lines(lines: list[str]) -> pd.DataFrame:
    rows: list[dict] = []

    for raw in lines:
        line = raw.strip()
        if not line:
            continue

        if line.startswith("TEL,"):
            line = line[4:]

        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 8:
            continue

        try:
            rows.append(
                {
                    "t_ms": int(float(parts[0])),
                    "state": parts[1],
                    "x_cm": float(parts[2]),
                    "x_filt_cm": float(parts[3]),
                    "theta_deg": float(parts[4]),
                    "theta_cmd_deg": float(parts[5]),
                    "u_step_rate": float(parts[6]),
                    "fault_flags": int(float(parts[7])),
                    "x_ref_cm": float(parts[8]) if len(parts) >= 9 else 0.0,
                    "sonar_age_ms": int(float(parts[9])) if len(parts) >= 10 else 0,
                    "sonar_valid": int(float(parts[10])) if len(parts) >= 11 else 0,
                    "sonar_miss_count": int(float(parts[11])) if len(parts) >= 12 else 0,
                    "theta_cmd_unclamped_deg": float(parts[12]) if len(parts) >= 13 else float(parts[5]),
                    "theta_cmd_clamped_deg": float(parts[13]) if len(parts) >= 14 else float(parts[5]),
                    "theta_cmd_saturated": int(float(parts[14])) if len(parts) >= 15 else 0,
                    "act_deg_abs": float(parts[15]) if len(parts) >= 16 else float("nan"),
                    "trim_deg": float(parts[16]) if len(parts) >= 17 else float("nan"),
                    "search_phase": parts[17] if len(parts) >= 18 else "",
                    "x_linear_cm": float(parts[18]) if len(parts) >= 19 else float(parts[2]),
                    "x_linear_filt_cm": float(parts[19]) if len(parts) >= 20 else float(parts[3]),
                    "x_ctrl_cm": float(parts[20]) if len(parts) >= 21 else float(parts[2]),
                    "x_ctrl_filt_cm": float(parts[21]) if len(parts) >= 22 else float(parts[3]),
                    "x_feedback_cm": float(parts[22]) if len(parts) >= 23 else float(parts[3]),
                    "feedback_blend": float(parts[23]) if len(parts) >= 24 else 0.0,
                    "pos_fresh": int(float(parts[24])) if len(parts) >= 25 else 0,
                    "pos_held": int(float(parts[25])) if len(parts) >= 26 else 0,
                    "pos_control_usable": int(float(parts[26])) if len(parts) >= 27 else 0,
                    "control_hold_active": int(float(parts[27])) if len(parts) >= 28 else 0,
                }
            )
        except ValueError:
            continue

    return pd.DataFrame(rows, columns=COLUMNS)


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse raw serial log into clean CSV")
    parser.add_argument("--input", type=Path, required=True, help="Path to raw .log file")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output CSV path (default: sibling *_clean.csv)",
    )
    args = parser.parse_args()

    if args.output is None:
        args.output = args.input.with_name(args.input.stem.replace("_raw", "") + "_clean.csv")

    lines = args.input.read_text(encoding="utf-8", errors="replace").splitlines()
    df = parse_telemetry_lines(lines)

    if df.empty:
        raise SystemExit("No valid telemetry rows found. Confirm firmware emits TEL lines.")

    df.to_csv(args.output, index=False)

    print("Rows:", len(df))
    print("Saved:", args.output)


if __name__ == "__main__":
    main()
