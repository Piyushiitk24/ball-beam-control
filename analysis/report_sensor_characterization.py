#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

if __package__ in (None, ""):
    workspace_root = Path(__file__).resolve().parents[1]
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))

from analysis.characterization_common import load_metadata, parse_point_label, stamp

RELIABLE_VALID_FRACTION = 0.90
RELIABLE_STDDEV_CM = 0.75
RELIABLE_ROUNDTRIP_CM = 1.0
RELIABLE_ADJACENT_DIFF_CM = 0.75
RELIABLE_REVERSAL_CM = 0.5


@dataclass
class RunArtifacts:
    run_stem: str
    run_dir: Path
    meta_path: Path
    points_path: Path
    samples_path: Path
    metadata: dict[str, object]


def _resolve_run_artifacts(path: Path) -> RunArtifacts:
    if path.is_file():
        run_dir = path.parent
    else:
        run_dir = path

    meta_candidates = sorted(run_dir.glob("run_*_characterization_meta.json"))
    if len(meta_candidates) != 1:
        raise SystemExit(f"Expected exactly one characterization metadata JSON in {run_dir}, found {len(meta_candidates)}")
    meta_path = meta_candidates[0]
    metadata = load_metadata(meta_path)
    run_stem = str(metadata["run_stem"])
    points_path = run_dir / str(metadata["points_csv_name"])
    samples_path = run_dir / str(metadata["samples_csv_name"])
    if not points_path.exists():
        raise SystemExit(f"Missing points CSV referenced by metadata: {points_path}")
    if not samples_path.exists():
        raise SystemExit(f"Missing samples CSV referenced by metadata: {samples_path}")
    return RunArtifacts(
        run_stem=run_stem,
        run_dir=run_dir,
        meta_path=meta_path,
        points_path=points_path,
        samples_path=samples_path,
        metadata=metadata,
    )


def _coerce_numeric(series: pd.Series) -> pd.Series:
    return pd.to_numeric(series, errors="coerce")


def _prepare_points_df(points_path: Path) -> pd.DataFrame:
    df = pd.read_csv(points_path)
    if "point_direction" not in df.columns or "point_number" not in df.columns:
        directions: list[str] = []
        point_numbers: list[int | None] = []
        for label in df["label"].fillna("").astype(str).tolist():
            parsed = parse_point_label(label)
            if parsed is None:
                directions.append("")
                point_numbers.append(None)
            else:
                direction, point_number = parsed
                directions.append(direction)
                point_numbers.append(point_number)
        df["point_direction"] = directions
        df["point_number"] = point_numbers

    numeric_columns = [
        "capture_index",
        "point_number",
        "samples_requested",
        "samples_captured",
        "valid_samples",
        "valid_fraction",
        "mean_distance_cm",
        "std_distance_cm",
        "min_distance_cm",
        "max_distance_cm",
    ]
    for column in numeric_columns:
        if column in df.columns:
            df[column] = _coerce_numeric(df[column])

    return df.dropna(subset=["point_number"]).copy()


def _build_pair_df(points_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for point_number, group in points_df.groupby("point_number", sort=True):
        out_rows = group[group["point_direction"] == "out"]
        ret_rows = group[group["point_direction"] == "ret"]
        if out_rows.empty or ret_rows.empty:
            continue

        out_row = out_rows.iloc[0]
        ret_row = ret_rows.iloc[0]
        out_mean = float(out_row["mean_distance_cm"]) if pd.notna(out_row["mean_distance_cm"]) else math.nan
        ret_mean = float(ret_row["mean_distance_cm"]) if pd.notna(ret_row["mean_distance_cm"]) else math.nan
        out_std = float(out_row["std_distance_cm"]) if pd.notna(out_row["std_distance_cm"]) else math.nan
        ret_std = float(ret_row["std_distance_cm"]) if pd.notna(ret_row["std_distance_cm"]) else math.nan
        out_valid_fraction = float(out_row["valid_fraction"]) if pd.notna(out_row["valid_fraction"]) else 0.0
        ret_valid_fraction = float(ret_row["valid_fraction"]) if pd.notna(ret_row["valid_fraction"]) else 0.0
        mismatch = math.nan
        paired_mean = math.nan
        if math.isfinite(out_mean) and math.isfinite(ret_mean):
            mismatch = abs(out_mean - ret_mean)
            paired_mean = 0.5 * (out_mean + ret_mean)

        rows.append(
            {
                "point_number": int(point_number),
                "out_label": str(out_row["label"]),
                "ret_label": str(ret_row["label"]),
                "out_mean_cm": out_mean,
                "ret_mean_cm": ret_mean,
                "out_std_cm": out_std,
                "ret_std_cm": ret_std,
                "out_valid_fraction": out_valid_fraction,
                "ret_valid_fraction": ret_valid_fraction,
                "paired_mean_cm": paired_mean,
                "roundtrip_mismatch_cm": mismatch,
            }
        )

    pair_df = pd.DataFrame(rows)
    if pair_df.empty:
        return pair_df

    pair_df = pair_df.sort_values("point_number").reset_index(drop=True)
    pair_df["basic_reliable"] = (
        pair_df["out_valid_fraction"].ge(RELIABLE_VALID_FRACTION)
        & pair_df["ret_valid_fraction"].ge(RELIABLE_VALID_FRACTION)
        & pair_df["out_std_cm"].le(RELIABLE_STDDEV_CM)
        & pair_df["ret_std_cm"].le(RELIABLE_STDDEV_CM)
        & pair_df["roundtrip_mismatch_cm"].le(RELIABLE_ROUNDTRIP_CM)
        & pair_df["paired_mean_cm"].notna()
    )
    pair_df["edge_ok_prev"] = True
    pair_df["edge_ok_next"] = True
    pair_df["final_reliable"] = pair_df["basic_reliable"].copy()

    if len(pair_df) <= 1:
        return pair_df

    finite_means = pair_df["paired_mean_cm"].dropna()
    trend_sign = 1.0
    if len(finite_means) >= 2:
        trend_sign = 1.0 if (finite_means.iloc[-1] - finite_means.iloc[0]) >= 0.0 else -1.0

    paired = pair_df["paired_mean_cm"].tolist()
    points = pair_df["point_number"].tolist()
    for index in range(1, len(pair_df)):
        if points[index] != points[index - 1] + 1:
            pair_df.loc[index - 1, "edge_ok_next"] = False
            pair_df.loc[index, "edge_ok_prev"] = False
            pair_df.loc[index - 1, "final_reliable"] = False
            pair_df.loc[index, "final_reliable"] = False
            continue
        prev_mean = paired[index - 1]
        curr_mean = paired[index]
        if not (math.isfinite(prev_mean) and math.isfinite(curr_mean)):
            pair_df.loc[index - 1, "edge_ok_next"] = False
            pair_df.loc[index, "edge_ok_prev"] = False
            pair_df.loc[index - 1, "final_reliable"] = False
            pair_df.loc[index, "final_reliable"] = False
            continue
        delta = curr_mean - prev_mean
        resolution_ok = abs(delta) >= RELIABLE_ADJACENT_DIFF_CM
        order_ok = (trend_sign * delta) >= (-RELIABLE_REVERSAL_CM)
        if not (resolution_ok and order_ok):
            pair_df.loc[index - 1, "edge_ok_next"] = False
            pair_df.loc[index, "edge_ok_prev"] = False
            pair_df.loc[index - 1, "final_reliable"] = False
            pair_df.loc[index, "final_reliable"] = False

    return pair_df


def _longest_reliable_window(pair_df: pd.DataFrame) -> tuple[int, str, str]:
    if pair_df.empty:
        return 0, "", ""

    best_len = 0
    best_start = ""
    best_end = ""
    run_start = ""
    run_end = ""
    run_len = 0
    prev_point = None

    for row in pair_df.itertuples(index=False):
        point_label = f"p{int(row.point_number):02d}"
        if bool(row.final_reliable):
            contiguous = prev_point is not None and int(row.point_number) == prev_point + 1 and run_len > 0
            if not contiguous:
                run_start = point_label
                run_len = 0
            run_len += 1
            run_end = point_label
            if run_len > best_len:
                best_len = run_len
                best_start = run_start
                best_end = run_end
        else:
            run_len = 0
            run_start = ""
            run_end = ""
        prev_point = int(row.point_number)

    return best_len, best_start, best_end


def _median_or_nan(series: pd.Series) -> float:
    series = series.dropna()
    if series.empty:
        return math.nan
    return float(series.median())


def _float_or_blank(value: float) -> str:
    if not math.isfinite(value):
        return ""
    return f"{value:.4f}"


def _df_to_markdown(df: pd.DataFrame) -> str:
    if df.empty:
        return ""
    headers = [str(column) for column in df.columns]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in df.fillna("").itertuples(index=False):
        lines.append("| " + " | ".join(str(value) for value in row) + " |")
    return "\n".join(lines)


def _plot_run(run_artifacts: RunArtifacts, pair_df: pd.DataFrame, output_path: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10.5, 8.5), sharex=True)
    point_numbers = pair_df["point_number"].astype(int)

    axes[0].plot(point_numbers, pair_df["out_mean_cm"], marker="o", linewidth=1.8, label="Outbound")
    axes[0].plot(point_numbers, pair_df["ret_mean_cm"], marker="s", linewidth=1.8, label="Return")
    axes[0].set_ylabel("Mean distance (cm)")
    axes[0].legend(loc="best")

    axes[1].plot(point_numbers, pair_df["out_std_cm"], marker="o", linewidth=1.6, label="Outbound")
    axes[1].plot(point_numbers, pair_df["ret_std_cm"], marker="s", linewidth=1.6, label="Return")
    axes[1].axhline(RELIABLE_STDDEV_CM, color="tab:red", linestyle="--", linewidth=1.0)
    axes[1].set_ylabel("Std dev (cm)")

    axes[2].plot(point_numbers, pair_df["out_valid_fraction"], marker="o", linewidth=1.6, label="Outbound")
    axes[2].plot(point_numbers, pair_df["ret_valid_fraction"], marker="s", linewidth=1.6, label="Return")
    axes[2].axhline(RELIABLE_VALID_FRACTION, color="tab:red", linestyle="--", linewidth=1.0)
    axes[2].set_ylabel("Valid fraction")
    axes[2].set_xlabel("Runner point index")

    reliable_rows = pair_df[pair_df["final_reliable"]]
    if not reliable_rows.empty:
        start = int(reliable_rows["point_number"].min()) - 0.3
        end = int(reliable_rows["point_number"].max()) + 0.3
        for ax in axes:
            ax.axvspan(start, end, color="#dfeee2", alpha=0.45)

    title = (
        f"{run_artifacts.run_stem} | {run_artifacts.metadata.get('sensor_name', '')}"
        f" | {run_artifacts.metadata.get('target_name', '')}"
    )
    fig.suptitle(title)
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Score and plot settled-point sensor characterization runs."
    )
    parser.add_argument(
        "--input",
        dest="inputs",
        action="append",
        required=True,
        help="Run directory or characterization metadata file. Repeat per run.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "data" / "characterization_reports" / f"report_{stamp()}",
        help="Directory for CSV/Markdown summaries and plots",
    )
    parser.add_argument("--title", default="Sensor Characterization Report", help="Report title")
    args = parser.parse_args()

    output_dir = args.output_dir
    plots_dir = output_dir / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)
    plots_dir.mkdir(parents=True, exist_ok=True)

    run_artifacts = [_resolve_run_artifacts(Path(value).resolve()) for value in args.inputs]

    pair_rows: list[dict[str, object]] = []
    run_rows: list[dict[str, object]] = []

    for artifacts in run_artifacts:
        points_df = _prepare_points_df(artifacts.points_path)
        pair_df = _build_pair_df(points_df)
        if pair_df.empty:
            raise SystemExit(f"No outbound/return point pairs found in {artifacts.points_path}")

        longest_window_points, window_start, window_end = _longest_reliable_window(pair_df)
        median_roundtrip = _median_or_nan(pair_df["roundtrip_mismatch_cm"])
        avg_std = 0.5 * (pair_df["out_std_cm"] + pair_df["ret_std_cm"])
        median_std = _median_or_nan(avg_std)
        avg_valid_fraction = 0.5 * (pair_df["out_valid_fraction"] + pair_df["ret_valid_fraction"])
        mean_valid_fraction = float(avg_valid_fraction.mean()) if not avg_valid_fraction.empty else math.nan
        invalid_rate = math.nan if not math.isfinite(mean_valid_fraction) else 1.0 - mean_valid_fraction

        run_rows.append(
            {
                "run_stem": artifacts.run_stem,
                "sensor_name": artifacts.metadata.get("sensor_name", ""),
                "sensor_mode": artifacts.metadata.get("sensor_mode", ""),
                "target_name": artifacts.metadata.get("target_name", ""),
                "block_label": artifacts.metadata.get("block_label", ""),
                "repeat_index": int(artifacts.metadata.get("repeat_index", 0)),
                "timer1_stress": int(bool(artifacts.metadata.get("timer1_stress", False))),
                "total_paired_points": len(pair_df),
                "reliable_points": int(pair_df["final_reliable"].sum()),
                "longest_reliable_window_points": longest_window_points,
                "reliable_window_start": window_start,
                "reliable_window_end": window_end,
                "median_roundtrip_mismatch_cm": _float_or_blank(median_roundtrip),
                "median_stddev_cm": _float_or_blank(median_std),
                "mean_valid_fraction": _float_or_blank(mean_valid_fraction),
                "invalid_rate": _float_or_blank(invalid_rate),
                "mount_note": artifacts.metadata.get("mount_note", ""),
                "photo_ref": artifacts.metadata.get("photo_ref", ""),
                "notes": artifacts.metadata.get("notes", ""),
            }
        )

        for row in pair_df.itertuples(index=False):
            pair_rows.append(
                {
                    "run_stem": artifacts.run_stem,
                    "sensor_name": artifacts.metadata.get("sensor_name", ""),
                    "target_name": artifacts.metadata.get("target_name", ""),
                    "block_label": artifacts.metadata.get("block_label", ""),
                    "repeat_index": int(artifacts.metadata.get("repeat_index", 0)),
                    "point_number": int(row.point_number),
                    "out_label": row.out_label,
                    "ret_label": row.ret_label,
                    "out_mean_cm": _float_or_blank(float(row.out_mean_cm)),
                    "ret_mean_cm": _float_or_blank(float(row.ret_mean_cm)),
                    "out_std_cm": _float_or_blank(float(row.out_std_cm)),
                    "ret_std_cm": _float_or_blank(float(row.ret_std_cm)),
                    "out_valid_fraction": _float_or_blank(float(row.out_valid_fraction)),
                    "ret_valid_fraction": _float_or_blank(float(row.ret_valid_fraction)),
                    "paired_mean_cm": _float_or_blank(float(row.paired_mean_cm)),
                    "roundtrip_mismatch_cm": _float_or_blank(float(row.roundtrip_mismatch_cm)),
                    "basic_reliable": int(bool(row.basic_reliable)),
                    "edge_ok_prev": int(bool(row.edge_ok_prev)),
                    "edge_ok_next": int(bool(row.edge_ok_next)),
                    "final_reliable": int(bool(row.final_reliable)),
                }
            )

        _plot_run(artifacts, pair_df, plots_dir / f"{artifacts.run_stem}_characterization.png")

    run_df = pd.DataFrame(run_rows)
    pair_report_df = pd.DataFrame(pair_rows)

    group_rows: list[dict[str, object]] = []
    for (sensor_name, target_name), group in run_df.groupby(["sensor_name", "target_name"], sort=False):
        longest_window = pd.to_numeric(group["longest_reliable_window_points"], errors="coerce")
        median_roundtrip = pd.to_numeric(group["median_roundtrip_mismatch_cm"], errors="coerce")
        median_std = pd.to_numeric(group["median_stddev_cm"], errors="coerce")
        invalid_rate = pd.to_numeric(group["invalid_rate"], errors="coerce")
        group_rows.append(
            {
                "sensor_name": sensor_name,
                "target_name": target_name,
                "runs": len(group),
                "min_longest_reliable_window_points": int(longest_window.min()),
                "median_longest_reliable_window_points": float(longest_window.median()),
                "median_roundtrip_mismatch_cm": _float_or_blank(float(median_roundtrip.median())),
                "median_stddev_cm": _float_or_blank(float(median_std.median())),
                "median_invalid_rate": _float_or_blank(float(invalid_rate.median())),
            }
        )

    group_df = pd.DataFrame(group_rows)
    if not group_df.empty:
        sort_df = group_df.copy()
        sort_df["median_roundtrip_mismatch_cm"] = pd.to_numeric(sort_df["median_roundtrip_mismatch_cm"], errors="coerce")
        sort_df["median_stddev_cm"] = pd.to_numeric(sort_df["median_stddev_cm"], errors="coerce")
        sort_df["median_invalid_rate"] = pd.to_numeric(sort_df["median_invalid_rate"], errors="coerce")
        sort_df = sort_df.sort_values(
            [
                "min_longest_reliable_window_points",
                "median_roundtrip_mismatch_cm",
                "median_stddev_cm",
                "median_invalid_rate",
            ],
            ascending=[False, True, True, True],
        ).reset_index(drop=True)
        winner = sort_df.iloc[0]
    else:
        winner = None

    run_csv_path = output_dir / "characterization_run_summary.csv"
    pair_csv_path = output_dir / "characterization_pair_metrics.csv"
    group_csv_path = output_dir / "characterization_group_summary.csv"
    summary_md_path = output_dir / "characterization_summary.md"

    run_df.to_csv(run_csv_path, index=False)
    pair_report_df.to_csv(pair_csv_path, index=False)
    group_df.to_csv(group_csv_path, index=False)

    with summary_md_path.open("w", encoding="utf-8") as handle:
        handle.write(f"# {args.title}\n\n")
        handle.write("## Winner\n\n")
        if winner is None:
            handle.write("No valid characterization groups were found.\n")
        else:
            handle.write(
                f"- Sensor: `{winner['sensor_name']}`\n"
                f"- Target: `{winner['target_name']}`\n"
                f"- Minimum reliable window across repeats: `{int(winner['min_longest_reliable_window_points'])}` points\n"
                f"- Median round-trip mismatch: `{winner['median_roundtrip_mismatch_cm']}` cm\n"
                f"- Median stddev: `{winner['median_stddev_cm']}` cm\n"
                f"- Median invalid rate: `{winner['median_invalid_rate']}`\n\n"
            )

        handle.write("## Group Summary\n\n")
        if group_df.empty:
            handle.write("No groups found.\n\n")
        else:
            handle.write(_df_to_markdown(group_df))
            handle.write("\n\n")

        handle.write("## Run Summary\n\n")
        if run_df.empty:
            handle.write("No runs found.\n\n")
        else:
            handle.write(_df_to_markdown(run_df))
            handle.write("\n\n")

        handle.write("## Outputs\n\n")
        handle.write(f"- Run CSV: `{run_csv_path}`\n")
        handle.write(f"- Pair CSV: `{pair_csv_path}`\n")
        handle.write(f"- Group CSV: `{group_csv_path}`\n")
        handle.write(f"- Plots: `{plots_dir}`\n")

    print("Saved:")
    print(f"  {run_csv_path}")
    print(f"  {pair_csv_path}")
    print(f"  {group_csv_path}")
    print(f"  {summary_md_path}")
    print(f"  {plots_dir}")


if __name__ == "__main__":
    main()
