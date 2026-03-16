#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

import pandas as pd

if __package__ in (None, ""):
    workspace_root = Path(__file__).resolve().parents[1]
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))

from analysis.run_layout import find_run_csv_by_stem, iter_run_csvs, relative_path

FAULT_BITS = {
    0x01: "sonar_timeout",
    0x02: "i2c_error",
    0x04: "angle_oob",
    0x08: "pos_oob",
    0x10: "actuator_drift",
}

DEFAULT_MILESTONES = [
    "run_20260310_112753",
    "run_20260311_121358",
    "run_20260312_103903",
    "run_20260312_110245",
    "run_20260312_112718",
    "run_20260312_113933",
]
CONTROL_USABLE_SONAR_AGE_MS = 150.0


@dataclass
class RunSummary:
    run_stem: str
    telemetry_path: Path
    timestamp: datetime
    target_sequence: str
    duration_s: float
    fault_rows: int
    fault_labels: str
    diagnosis: str
    final_state: str
    final_x_filt_cm: float
    final_theta_deg: float
    center_crossings: int
    invalid_rows: int
    max_sonar_age_ms: int
    stale_control_rows: int
    stale_control_stretches: int
    stale_control_windows: str
    near_end_error_cm: float | None
    far_end_error_cm: float | None
    center_end_error_cm: float | None


@dataclass
class PhaseSummary:
    x_ref_cm: float
    x_end_cm: float
    x_min_cm: float
    x_max_cm: float
    theta_end_deg: float
    theta_cmd_end_deg: float
    u_end_sps: float
    invalid_rows: int
    fault_rows: int
    crossings: int


def _parse_run_timestamp(run_stem: str) -> datetime:
    _, date_s, time_s = run_stem.split("_")
    return datetime.strptime(f"{date_s}_{time_s}", "%Y%m%d_%H%M%S")


def _iter_runs(run_dir: Path, since_yyyymmdd: str) -> list[Path]:
    candidates = iter_run_csvs(run_dir)
    out: list[Path] = []
    for path in candidates:
        stem = path.stem.replace("_telemetry", "")
        if stem.split("_")[1] >= since_yyyymmdd:
            out.append(path)
    return out


def _load_events(events_path: Path) -> tuple[dict[str, float], list[str]]:
    ctx: dict[str, float] = {}
    lines: list[str] = []
    if not events_path.exists():
        return ctx, lines
    for raw in events_path.read_text(encoding="utf-8", errors="replace").splitlines():
        _, _, payload = raw.partition("] ")
        line = payload.strip() if payload else raw.strip()
        if not line:
            continue
        lines.append(line)
        if line.startswith("SET,"):
            parts = [part.strip() for part in line.split(",")]
            if len(parts) == 5:
                try:
                    ctx["near_target_cm"] = float(parts[3])
                    ctx["far_target_cm"] = float(parts[4])
                except ValueError:
                    pass
    return ctx, lines


def _parse_keyvals(prefix: str, line: str) -> dict[str, str]:
    if not line.startswith(prefix):
        return {}
    out: dict[str, str] = {}
    for part in line[len(prefix) :].split(","):
        part = part.strip()
        if not part or "=" not in part:
            continue
        key, value = part.split("=", 1)
        out[key.strip()] = value.strip()
    return out


def _experiment_window_df(df: pd.DataFrame, event_lines: list[str]) -> pd.DataFrame:
    start_ms: float | None = None
    stop_ms: float | None = None
    for line in event_lines:
        if line.startswith("HOST_STD,start,") and start_ms is None:
            kv = _parse_keyvals("HOST_STD,start,", line)
            try:
                start_ms = float(kv["t_ms"])
            except (KeyError, ValueError):
                pass
        elif line.startswith("HOST_STD,stop,") and start_ms is not None and stop_ms is None:
            kv = _parse_keyvals("HOST_STD,stop,", line)
            try:
                stop_ms = float(kv["t_ms"])
            except (KeyError, ValueError):
                pass

    if start_ms is not None:
        t_ms = df["t_ms"].astype(float)
        work = df.loc[t_ms >= start_ms].copy()
        if stop_ms is not None:
            work = work.loc[work["t_ms"].astype(float) <= stop_ms].copy()
        if not work.empty:
            return work

    states = df["state"].fillna("").astype(str)
    active_mask = states.isin(["RUNNING", "FAULT"])
    if active_mask.any():
        start_idx = active_mask[active_mask].index[0]
        work = df.loc[start_idx:].copy()
        if not work.empty:
            return work

    return df.copy()


def _phase_rows(df: pd.DataFrame) -> list[pd.DataFrame]:
    rows: list[pd.DataFrame] = []
    last = None
    start = 0
    for idx, x_ref in enumerate(df["x_ref_cm"].astype(float).tolist()):
        if last is None:
            last = x_ref
            start = idx
            continue
        if abs(x_ref - last) > 1.0e-6:
            rows.append(df.iloc[start:idx].copy())
            start = idx
            last = x_ref
    rows.append(df.iloc[start:].copy())
    return [seg for seg in rows if not seg.empty]


def _count_crossings(values: Iterable[float], ref: float) -> int:
    crossings = 0
    prev = None
    for value in values:
        err = value - ref
        if prev is None:
            prev = err
            continue
        if prev == 0.0:
            prev = err
            continue
        if err != 0.0 and (prev * err) < 0.0:
            crossings += 1
        prev = err
    return crossings


def _phase_summary(seg: pd.DataFrame) -> PhaseSummary:
    x_ref = float(seg["x_ref_cm"].iloc[0])
    x_filt = seg["x_filt_cm"].astype(float)
    theta = seg["theta_deg"].astype(float)
    theta_cmd = seg["theta_cmd_deg"].astype(float)
    u_step = seg["u_step_rate"].astype(float)
    sonar_valid = seg.get("sonar_valid", pd.Series([1] * len(seg))).astype(float)
    faults = seg["fault_flags"].astype(float)
    return PhaseSummary(
        x_ref_cm=x_ref,
        x_end_cm=float(x_filt.iloc[-1]),
        x_min_cm=float(x_filt.min()),
        x_max_cm=float(x_filt.max()),
        theta_end_deg=float(theta.iloc[-1]),
        theta_cmd_end_deg=float(theta_cmd.iloc[-1]),
        u_end_sps=float(u_step.iloc[-1]),
        invalid_rows=int((sonar_valid <= 0.0).sum()),
        fault_rows=int((faults != 0.0).sum()),
        crossings=_count_crossings(x_filt.tolist(), x_ref),
    )


def _stale_control_mask(df: pd.DataFrame) -> pd.Series:
    sonar_valid = df.get("sonar_valid", pd.Series([1] * len(df), index=df.index)).fillna(1).astype(float) > 0.0
    if "pos_control_usable" in df.columns:
        usable = df["pos_control_usable"].fillna(0).astype(float) > 0.0
        return sonar_valid & (~usable)
    sonar_age_ms = df.get("sonar_age_ms", pd.Series([0] * len(df), index=df.index)).fillna(0).astype(float)
    return sonar_valid & (sonar_age_ms > CONTROL_USABLE_SONAR_AGE_MS)


def _stale_control_windows(df: pd.DataFrame) -> list[str]:
    if df.empty:
        return []

    stale = _stale_control_mask(df)
    if not stale.any():
        return []

    groups = (stale != stale.shift(fill_value=False)).cumsum()
    windows: list[str] = []
    for _, seg in df.loc[stale].groupby(groups[stale], sort=False):
        start_ms = float(seg["t_ms"].iloc[0])
        end_ms = float(seg["t_ms"].iloc[-1])
        max_age_ms = int(seg.get("sonar_age_ms", pd.Series([0] * len(seg), index=seg.index)).fillna(0).astype(float).max())
        windows.append(f"{int(start_ms)}-{int(end_ms)}ms(max_age={max_age_ms})")
    return windows


def _format_target_sequence(refs: list[float], ctx: dict[str, float]) -> str:
    near = ctx.get("near_target_cm")
    far = ctx.get("far_target_cm")
    labels: list[str] = []
    for ref in refs:
        if abs(ref) <= 1.0e-4:
            labels.append("q c")
        elif near is not None and abs(ref - near) <= 0.05:
            labels.append("q n")
        elif far is not None and abs(ref - far) <= 0.05:
            labels.append("q f")
        else:
            labels.append(f"q {ref:.2f}")
    return " -> ".join(labels)


def _decode_faults(df: pd.DataFrame) -> list[str]:
    labels: set[str] = set()
    for bits in df["fault_flags"].astype(float).astype(int).tolist():
        for mask, name in FAULT_BITS.items():
            if bits & mask:
                labels.add(name)
    return sorted(labels)


def _diagnosis_label(refs: list[float],
                     phases: list[PhaseSummary],
                     fault_labels: list[str],
                     final_state: str) -> str:
    if "sonar_timeout" in fault_labels:
        return "sonar validity dropout"
    if "angle_oob" in fault_labels:
        return "travel-limit angle_oob"
    if "actuator_drift" in fault_labels:
        return "actuator verification fault"
    if final_state == "FAULT":
        return "fault without logged flag"

    if len(refs) == 1 and abs(refs[0]) <= 1.0e-4 and phases[0].crossings >= 10:
        return "center oscillation"

    center_phases = [phase for phase in phases if abs(phase.x_ref_cm) <= 1.0e-4]
    far_phases = [phase for phase in phases if phase.x_ref_cm < -0.5]
    near_phases = [phase for phase in phases if phase.x_ref_cm > 0.5]

    if center_phases and abs(center_phases[-1].x_end_cm) > 1.5:
        return "center settling error"

    for phase in far_phases:
        if abs(phase.x_end_cm - phase.x_ref_cm) > 2.0:
            return "far-side shortfall"

    for phase in near_phases:
        if abs(phase.x_end_cm - phase.x_ref_cm) > 2.0:
            return "near-side undercommand"

    if phases and all(phase.fault_rows == 0 for phase in phases):
        return "tracking improved"
    return "mixed / tuning"


def _summarize_run(csv_path: Path) -> RunSummary:
    run_stem = csv_path.stem.replace("_telemetry", "")
    df = pd.read_csv(csv_path)
    events_path = csv_path.with_name(run_stem + "_events.txt")
    ctx, event_lines = _load_events(events_path)
    work_df = _experiment_window_df(df, event_lines)
    if work_df.empty:
        work_df = df.copy()

    refs: list[float] = []
    for ref in work_df["x_ref_cm"].astype(float).tolist():
        if not refs or abs(ref - refs[-1]) > 1.0e-6:
            refs.append(ref)

    phases = [_phase_summary(seg) for seg in _phase_rows(work_df)]
    fault_labels = _decode_faults(work_df)
    final_state = str(work_df["state"].iloc[-1])
    stale_control_windows = _stale_control_windows(work_df)

    def _find_end_error(target_sign: int) -> float | None:
        for phase in reversed(phases):
            if target_sign > 0 and phase.x_ref_cm > 0.5:
                return phase.x_end_cm - phase.x_ref_cm
            if target_sign < 0 and phase.x_ref_cm < -0.5:
                return phase.x_end_cm - phase.x_ref_cm
            if target_sign == 0 and abs(phase.x_ref_cm) <= 1.0e-4:
                return phase.x_end_cm
        return None

    return RunSummary(
        run_stem=run_stem,
        telemetry_path=csv_path,
        timestamp=_parse_run_timestamp(run_stem),
        target_sequence=_format_target_sequence(refs, ctx),
        duration_s=((float(work_df["t_ms"].iloc[-1]) - float(work_df["t_ms"].iloc[0])) / 1000.0)
        if not work_df.empty else 0.0,
        fault_rows=int(((work_df["fault_flags"].astype(float) != 0.0) |
                        (work_df["state"] == "FAULT")).sum()),
        fault_labels=", ".join(fault_labels) if fault_labels else "none",
        diagnosis=_diagnosis_label(refs, phases, fault_labels, final_state),
        final_state=final_state,
        final_x_filt_cm=float(work_df["x_filt_cm"].astype(float).iloc[-1]),
        final_theta_deg=float(work_df["theta_deg"].astype(float).iloc[-1]),
        center_crossings=sum(phase.crossings for phase in phases if abs(phase.x_ref_cm) <= 1.0e-4),
        invalid_rows=int((work_df.get("sonar_valid", pd.Series([1] * len(work_df))).astype(float) <= 0.0).sum()),
        max_sonar_age_ms=int(work_df.get("sonar_age_ms", pd.Series([0] * len(work_df))).astype(float).max()),
        stale_control_rows=int(_stale_control_mask(work_df).sum()),
        stale_control_stretches=len(stale_control_windows),
        stale_control_windows="; ".join(stale_control_windows),
        near_end_error_cm=_find_end_error(1),
        far_end_error_cm=_find_end_error(-1),
        center_end_error_cm=_find_end_error(0),
    )


def _write_summary_csv(rows: list[RunSummary], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "run_stem",
                "timestamp",
                "target_sequence",
                "duration_s",
                "fault_rows",
                "fault_labels",
                "diagnosis",
                "final_state",
                "final_x_filt_cm",
                "final_theta_deg",
                "center_crossings",
                "invalid_rows",
                "max_sonar_age_ms",
                "stale_control_rows",
                "stale_control_stretches",
                "stale_control_windows",
                "near_end_error_cm",
                "far_end_error_cm",
                "center_end_error_cm",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    row.run_stem,
                    row.timestamp.isoformat(timespec="seconds"),
                    row.target_sequence,
                    f"{row.duration_s:.2f}",
                    row.fault_rows,
                    row.fault_labels,
                    row.diagnosis,
                    row.final_state,
                    f"{row.final_x_filt_cm:.3f}",
                    f"{row.final_theta_deg:.3f}",
                    row.center_crossings,
                    row.invalid_rows,
                    row.max_sonar_age_ms,
                    row.stale_control_rows,
                    row.stale_control_stretches,
                    row.stale_control_windows,
                    "" if row.near_end_error_cm is None else f"{row.near_end_error_cm:.3f}",
                    "" if row.far_end_error_cm is None else f"{row.far_end_error_cm:.3f}",
                    "" if row.center_end_error_cm is None else f"{row.center_end_error_cm:.3f}",
                ]
            )


def _generate_plots(script_path: Path, output_root: Path, milestone_paths: list[Path]) -> None:
    plot_dir = output_root / "generated" / "plots"
    metrics_dir = output_root / "generated" / "metrics"
    plot_dir.mkdir(parents=True, exist_ok=True)
    metrics_dir.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    env.setdefault("MPLBACKEND", "Agg")
    env.setdefault("MPLCONFIGDIR", "/tmp/mpl")

    for csv_path in milestone_paths:
        stem = csv_path.stem.replace("_telemetry", "")
        out_png = plot_dir / f"{stem}.png"
        out_metrics = metrics_dir / f"{stem}_metrics.csv"
        cmd = [
            sys.executable,
            str(script_path),
            "--input",
            str(csv_path),
            "--output",
            str(out_png),
            "--metrics-output",
            str(out_metrics),
            "--style",
            "standard",
        ]
        subprocess.run(cmd, check=True, env=env)
        if out_metrics.exists() and out_metrics.stat().st_size == 0:
            out_metrics.unlink()


def _write_summary_markdown(rows: list[RunSummary], out_path: Path, output_root: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Recent Run Summary",
        "",
        f"Generated from runs dated 2026-03-10 onward on {datetime.now().isoformat(timespec='seconds')}",
        "",
        "| Run | Targets | Diagnosis | Faults | End state | End x_filt (cm) | Center crossings | Stale-control rows | Stale-control stretches | Max sonar age (ms) |",
        "| --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        run_rel = relative_path(out_path.parent, row.telemetry_path)
        lines.append(
            "| [{run}]({link}) | {targets} | {diag} | {faults} | {state} | {x:.3f} | {cross} | {stale_rows} | {stale_stretches} | {age} |".format(
                run=row.run_stem,
                link=run_rel.as_posix(),
                targets=row.target_sequence,
                diag=row.diagnosis,
                faults=row.fault_labels,
                state=row.final_state,
                x=row.final_x_filt_cm,
                cross=row.center_crossings,
                stale_rows=row.stale_control_rows,
                stale_stretches=row.stale_control_stretches,
                age=row.max_sonar_age_ms,
            )
        )

    lines.extend(
        [
            "",
            "## Milestone Plots",
            "",
        ]
    )
    for stem in DEFAULT_MILESTONES:
        if any(row.run_stem == stem for row in rows):
            lines.append(f"- [{stem} plot](plots/{stem}.png)")
    lines.append("")
    out_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a recent-runs report for thesis/debug documentation")
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=Path("data/runs"),
        help="Directory containing run folders / telemetry files",
    )
    parser.add_argument("--since", default="20260310", help="Include runs on/after this YYYYMMDD date")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("docs/experiments/2026-03-control-debugging"),
        help="Experiment record root directory",
    )
    parser.add_argument(
        "--milestones",
        nargs="*",
        default=DEFAULT_MILESTONES,
        help="Run stems to render as representative plots",
    )
    args = parser.parse_args()

    runs = _iter_runs(args.run_dir, args.since)
    summaries = [_summarize_run(path) for path in runs]
    summaries.sort(key=lambda row: row.timestamp)

    csv_out = args.output_root / "generated" / "recent_runs_summary.csv"
    md_out = args.output_root / "generated" / "recent_runs_summary.md"
    _write_summary_csv(summaries, csv_out)
    _write_summary_markdown(summaries, md_out, args.output_root)

    milestone_paths = [find_run_csv_by_stem(args.run_dir, stem) for stem in args.milestones]
    milestone_paths = [path for path in milestone_paths if path is not None]
    script_path = Path(__file__).with_name("plot_run.py")
    _generate_plots(script_path, args.output_root, milestone_paths)

    print(f"Saved summary CSV: {csv_out}")
    print(f"Saved summary Markdown: {md_out}")
    if milestone_paths:
        print("Saved milestone plots:")
        for path in milestone_paths:
            print(f"  - {args.output_root / 'generated' / 'plots' / (path.stem.replace('_telemetry', '') + '.png')}")


if __name__ == "__main__":
    main()
