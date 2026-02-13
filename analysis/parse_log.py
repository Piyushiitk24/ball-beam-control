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
        if len(parts) != len(COLUMNS):
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
