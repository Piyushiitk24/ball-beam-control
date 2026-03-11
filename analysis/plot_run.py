#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

RUN_SOFT_LIMIT_BAND_DEG = 1.0
RUN_HARD_LIMIT_MARGIN_DEG = 3.0


def _parse_keyvals(prefix: str, line: str) -> dict[str, str]:
    out: dict[str, str] = {}
    if not line.startswith(prefix):
        return out
    rest = line[len(prefix) :]
    for part in rest.split(","):
        part = part.strip()
        if not part or "=" not in part:
            continue
        key, value = part.split("=", 1)
        out[key.strip()] = value.strip()
    return out


def _default_events_path(csv_path: Path) -> Path:
    name = csv_path.name
    if name.endswith("_telemetry.csv"):
        return csv_path.with_name(name.replace("_telemetry.csv", "_events.txt"))
    if name.endswith("_clean.csv"):
        return csv_path.with_name(name.replace("_clean.csv", "_events.txt"))
    return csv_path.with_suffix(".events.txt")


def _default_metrics_path(csv_path: Path) -> Path:
    name = csv_path.name
    if name.endswith("_telemetry.csv"):
        return csv_path.with_name(name.replace("_telemetry.csv", "_metrics.csv"))
    if name.endswith("_clean.csv"):
        return csv_path.with_name(name.replace("_clean.csv", "_metrics.csv"))
    return csv_path.with_suffix(".metrics.csv")


def _load_events(events_path: Path | None) -> list[dict[str, object]]:
    if events_path is None or not events_path.exists():
        return []

    events: list[dict[str, object]] = []
    for raw in events_path.read_text(encoding="utf-8", errors="replace").splitlines():
        _, _, payload = raw.partition("] ")
        line = payload.strip() if payload else raw.strip()
        if not line:
            continue

        label = ""
        t_ms: int | None = None
        if line.startswith("HOST_STD,phase,"):
            kv = _parse_keyvals("HOST_STD,phase,", line)
            label = kv.get("name", "phase")
            if "t_ms" in kv:
                try:
                    t_ms = int(float(kv["t_ms"]))
                except ValueError:
                    t_ms = None
        elif line.startswith("HOST_STD,mark,"):
            kv = _parse_keyvals("HOST_STD,mark,", line)
            label = kv.get("label", "mark")
            if "t_ms" in kv:
                try:
                    t_ms = int(float(kv["t_ms"]))
                except ValueError:
                    t_ms = None
        elif line.startswith("SET,"):
            parts = [p.strip() for p in line.split(",")]
            if len(parts) == 5:
                label = f"q:{parts[2]}"
                try:
                    t_ms = int(float(parts[1]))
                except ValueError:
                    t_ms = None
        if t_ms is None or not label:
            continue
        events.append({"t_s": t_ms / 1000.0, "label": label})
    return events


def _load_cal_context(events_path: Path | None) -> dict[str, float]:
    if events_path is None or not events_path.exists():
        return {}

    ctx: dict[str, float] = {}
    for raw in events_path.read_text(encoding="utf-8", errors="replace").splitlines():
        _, _, payload = raw.partition("] ")
        line = payload.strip() if payload else raw.strip()
        if not line:
            continue

        if line.startswith("SET,"):
            parts = [p.strip() for p in line.split(",")]
            if len(parts) == 5:
                try:
                    ctx["near_target_cm"] = float(parts[3])
                    ctx["far_target_cm"] = float(parts[4])
                except ValueError:
                    pass
        elif line.startswith("CAL_LIMITS,"):
            kv = _parse_keyvals("CAL_LIMITS,", line)
            try:
                ctx["theta_lower_limit_deg"] = float(kv["lower_deg"])
                ctx["theta_upper_limit_deg"] = float(kv["upper_deg"])
            except (KeyError, ValueError):
                pass
    return ctx


def _find_hold_time(errors_cm: pd.Series, t_s: pd.Series, band_cm: float, hold_s: float) -> float:
    if errors_cm.empty:
        return math.nan
    values = errors_cm.abs().to_list()
    times = t_s.to_list()
    n = len(values)
    for i in range(n):
        if values[i] > band_cm:
            continue
        t_start = times[i]
        ok = True
        j = i
        while j < n and (times[j] - t_start) < hold_s:
            if values[j] > band_cm:
                ok = False
                break
            j += 1
        if ok and j > i and (times[min(j - 1, n - 1)] - t_start) >= hold_s:
            return t_start - times[0]
    return math.nan


def _fault_event_count(flags: pd.Series) -> int:
    nonzero = (flags.fillna(0).astype(int) != 0).astype(int)
    return int(((nonzero == 1) & (nonzero.shift(fill_value=0) == 0)).sum())


def _segment_metrics(phase_df: pd.DataFrame, prev_ref_cm: float | None) -> dict[str, object]:
    start_t_s = float(phase_df["t_ms"].iloc[0]) / 1000.0
    end_t_s = float(phase_df["t_ms"].iloc[-1]) / 1000.0
    duration_s = max(0.0, end_t_s - start_t_s)
    x_ref_cm = float(phase_df["x_ref_cm"].median())
    x_filt = phase_df["x_filt_cm"].astype(float)
    err = x_filt - x_ref_cm
    start_err_cm = float(err.iloc[0])

    if prev_ref_cm is not None and abs(x_ref_cm - prev_ref_cm) > 0.05:
        effective_step_cm = abs(x_ref_cm - prev_ref_cm)
    else:
        effective_step_cm = abs(start_err_cm)

    phase_name = str(phase_df["test_phase"].iloc[0]) if "test_phase" in phase_df.columns else ""
    is_recovery = phase_name.startswith("recover_after_disturb")
    band_cm = 0.5 if is_recovery else max(0.3, 0.05 * effective_step_cm)
    hold_s = 2.0 if is_recovery else 1.0
    hold_metric = _find_hold_time(err, phase_df["t_ms"].astype(float) / 1000.0, band_cm, hold_s)

    overshoot_cm = 0.0
    if start_err_cm > 0.0:
        overshoot_cm = max(0.0, -float(err.min()))
    elif start_err_cm < 0.0:
        overshoot_cm = max(0.0, float(err.max()))

    tail = phase_df[(phase_df["t_ms"] >= (phase_df["t_ms"].iloc[-1] - 2000))]
    if tail.empty:
        tail = phase_df
    steady_state_error_cm = float((tail["x_filt_cm"].astype(float) - x_ref_cm).mean())

    row: dict[str, object] = {
        "phase": phase_name,
        "x_ref_cm": x_ref_cm,
        "duration_s": duration_s,
        "band_cm": band_cm,
        "steady_state_error_cm": steady_state_error_cm,
        "overshoot_cm": overshoot_cm,
        "peak_abs_theta_deg": float(phase_df["theta_deg"].abs().max()),
        "peak_abs_u_step_rate": float(phase_df["u_step_rate"].abs().max()),
        "fault_count": _fault_event_count(phase_df["fault_flags"]),
    }
    if is_recovery:
        row["recovery_time_s"] = hold_metric
        row["settling_time_s"] = math.nan
    else:
        row["settling_time_s"] = hold_metric
        row["recovery_time_s"] = math.nan
    return row


def _compute_metrics(df: pd.DataFrame) -> pd.DataFrame:
    if "test_phase" not in df.columns:
        return pd.DataFrame()

    work = df.copy()
    work["test_phase"] = work["test_phase"].fillna("")
    phase_rows: list[dict[str, object]] = []
    prev_ref_cm: float | None = None

    nonempty = work[work["test_phase"] != ""].copy()
    if nonempty.empty:
        return pd.DataFrame()

    phase_id = (nonempty["test_phase"] != nonempty["test_phase"].shift()).cumsum()
    for _, phase_df in nonempty.groupby(phase_id, sort=False):
        metrics = _segment_metrics(phase_df, prev_ref_cm)
        phase_rows.append(metrics)
        prev_ref_cm = float(metrics["x_ref_cm"])

    return pd.DataFrame(phase_rows)


def _draw_events(axes: list[plt.Axes], events: list[dict[str, object]]) -> None:
    if not events:
        return
    for event in events:
        t_s = float(event["t_s"])
        label = str(event["label"])
        for ax in axes:
            ax.axvline(t_s, color="#6c757d", linestyle="--", linewidth=0.8, alpha=0.35)
        axes[0].text(
            t_s,
            axes[0].get_ylim()[1],
            label,
            rotation=90,
            va="top",
            ha="right",
            fontsize=7,
            color="#495057",
            alpha=0.9,
        )


def _shade_center_search(axes: list[plt.Axes], df: pd.DataFrame) -> None:
    if "state" not in df.columns:
        return
    work = df.copy()
    work["state"] = work["state"].fillna("")
    mask = work["state"] == "CENTER_SEARCH"
    if not mask.any():
        return
    groups = (mask != mask.shift(fill_value=False)).cumsum()
    for _, seg in work[mask].groupby(groups[mask], sort=False):
        t0 = float(seg["t_ms"].iloc[0]) / 1000.0
        t1 = float(seg["t_ms"].iloc[-1]) / 1000.0
        for ax in axes:
            ax.axvspan(t0, t1, color="#ffbe0b", alpha=0.10)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot run telemetry with reference overlays and metrics")
    parser.add_argument("--input", type=Path, required=True, help="Path to *_telemetry.csv or *_clean.csv")
    parser.add_argument("--output", type=Path, default=None, help="Output image path")
    parser.add_argument("--events", type=Path, default=None, help="Optional events log path")
    parser.add_argument("--metrics-output", type=Path, default=None, help="Optional metrics CSV path")
    args = parser.parse_args()

    if args.output is None:
        args.output = args.input.with_suffix(".png")

    events_path = args.events if args.events is not None else _default_events_path(args.input)
    metrics_output = args.metrics_output if args.metrics_output is not None else _default_metrics_path(args.input)

    df = pd.read_csv(args.input)
    if "x_ref_cm" not in df.columns:
        df["x_ref_cm"] = 0.0
    if "test_phase" not in df.columns:
        df["test_phase"] = ""

    t_s = df["t_ms"] / 1000.0
    events = _load_events(events_path)
    cal_ctx = _load_cal_context(events_path)

    fig, axes = plt.subplots(4, 1, figsize=(10.5, 9.5), sharex=True)

    if "x_linear_filt_cm" in df.columns:
        axes[0].plot(t_s, df["x_linear_filt_cm"], label="x_linear_filt_cm", color="#1d3557", alpha=0.9)
    else:
        axes[0].plot(t_s, df["x_cm"], label="x_cm", color="#1d3557")
    if "x_ctrl_filt_cm" in df.columns:
        axes[0].plot(t_s, df["x_ctrl_filt_cm"], label="x_ctrl_filt_cm", color="#457b9d", alpha=0.9)
    else:
        axes[0].plot(t_s, df["x_filt_cm"], label="x_filt_cm", color="#457b9d", alpha=0.9)
    if "x_feedback_cm" in df.columns:
        axes[0].plot(t_s, df["x_feedback_cm"], label="x_feedback_cm", color="#2a9d8f", linewidth=1.2)
    else:
        axes[0].plot(t_s, df["x_filt_cm"], label="x_filt_cm", color="#2a9d8f", linewidth=1.2)
    axes[0].plot(t_s, df["x_ref_cm"], label="x_ref_cm", color="#e76f51", linestyle="--", linewidth=1.4)
    if "near_target_cm" in cal_ctx:
        axes[0].axhline(cal_ctx["near_target_cm"], color="#adb5bd", linestyle=":", linewidth=0.9, alpha=0.7)
    if "far_target_cm" in cal_ctx:
        axes[0].axhline(cal_ctx["far_target_cm"], color="#adb5bd", linestyle=":", linewidth=0.9, alpha=0.7)
    axes[0].set_ylabel("Position (cm)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_s, df["theta_deg"], label="theta_deg", color="#e63946")
    axes[1].plot(t_s, df["theta_cmd_deg"], label="theta_cmd_deg", color="#f4a261", alpha=0.9)
    if "theta_cmd_unclamped_deg" in df.columns:
        axes[1].plot(
            t_s,
            df["theta_cmd_unclamped_deg"],
            label="theta_cmd_unclamped_deg",
            color="#6d597a",
            linestyle="--",
            alpha=0.8,
        )
    if "act_deg_abs" in df.columns and df["act_deg_abs"].notna().any():
        axes[1].plot(t_s, df["act_deg_abs"], label="act_deg_abs", color="#264653", alpha=0.55)
    if "trim_deg" in df.columns and df["trim_deg"].notna().any():
        axes[1].plot(t_s, df["trim_deg"], label="trim_deg", color="#8d99ae", linestyle=":", alpha=0.9)
    if "theta_lower_limit_deg" in cal_ctx and "theta_upper_limit_deg" in cal_ctx:
        lo = cal_ctx["theta_lower_limit_deg"]
        hi = cal_ctx["theta_upper_limit_deg"]
        axes[1].axhline(lo + RUN_SOFT_LIMIT_BAND_DEG, color="#6c757d", linestyle=":", linewidth=0.9, alpha=0.7)
        axes[1].axhline(hi - RUN_SOFT_LIMIT_BAND_DEG, color="#6c757d", linestyle=":", linewidth=0.9, alpha=0.7)
        axes[1].axhline(lo - RUN_HARD_LIMIT_MARGIN_DEG, color="#495057", linestyle="--", linewidth=0.9, alpha=0.7)
        axes[1].axhline(hi + RUN_HARD_LIMIT_MARGIN_DEG, color="#495057", linestyle="--", linewidth=0.9, alpha=0.7)
    axes[1].set_ylabel("Actuator (deg)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t_s, df["u_step_rate"], label="u_step_rate", color="#2a9d8f")
    if "feedback_blend" in df.columns:
        ax2b = axes[2].twinx()
        ax2b.plot(t_s, df["feedback_blend"], label="feedback_blend", color="#8338ec", alpha=0.5)
        ax2b.set_ylabel("Blend")
        ax2b.set_ylim(-0.05, 1.05)
    axes[2].set_ylabel("Step Rate (sps)")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    axes[3].step(t_s, df["fault_flags"], where="post", label="fault_flags", color="#6a040f")
    axes[3].set_ylabel("Fault Bits")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)

    _draw_events(list(axes), events)
    _shade_center_search(list(axes), df)

    fig.suptitle(f"Run Plot: {args.input.name}")
    fig.tight_layout()
    fig.savefig(args.output, dpi=160)

    metrics = _compute_metrics(df)
    if not metrics.empty:
        metrics.to_csv(metrics_output, index=False)
        print("\nMetrics")
        print(metrics.to_string(index=False, float_format=lambda v: f"{v:.3f}"))
        print(f"\nSaved metrics: {metrics_output}")

    print(f"Saved plot: {args.output}")


if __name__ == "__main__":
    main()
