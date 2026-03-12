#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

if __package__ in (None, ""):
    workspace_root = Path(__file__).resolve().parents[1]
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))

from analysis.run_layout import (
    events_path as run_events_path,
    metrics_path as run_metrics_path,
    resolve_run_input,
    summary_plot_path,
)

RUN_SOFT_LIMIT_BAND_DEG = 1.0
RUN_HARD_LIMIT_MARGIN_DEG = 3.0
FAULT_BITS = {
    0x01: "sonar_timeout",
    0x02: "i2c_error",
    0x04: "angle_oob",
    0x08: "pos_oob",
    0x10: "actuator_drift",
}
METRIC_COLUMNS = [
    "phase",
    "x_ref_cm",
    "duration_s",
    "band_cm",
    "steady_state_error_cm",
    "overshoot_cm",
    "peak_abs_theta_deg",
    "peak_abs_u_step_rate",
    "fault_count",
    "settling_time_s",
    "recovery_time_s",
]


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


def _fallback_events_path(csv_path: Path) -> Path:
    name = csv_path.name
    if name.endswith("_telemetry.csv"):
        return csv_path.with_name(name.replace("_telemetry.csv", "_events.txt"))
    if name.endswith("_clean.csv"):
        return csv_path.with_name(name.replace("_clean.csv", "_events.txt"))
    return csv_path.with_suffix(".events.txt")


def _fallback_metrics_path(csv_path: Path) -> Path:
    name = csv_path.name
    if name.endswith("_telemetry.csv"):
        return csv_path.with_name(name.replace("_telemetry.csv", "_metrics.csv"))
    if name.endswith("_clean.csv"):
        return csv_path.with_name(name.replace("_clean.csv", "_metrics.csv"))
    return csv_path.with_suffix(".metrics.csv")


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


def _setpoint_label(ref_cm: float, cal_ctx: dict[str, float]) -> str:
    near = cal_ctx.get("near_target_cm")
    far = cal_ctx.get("far_target_cm")
    if abs(ref_cm) <= 1.0e-4:
        return "Center"
    if near is not None and abs(ref_cm - near) <= 0.05:
        return "Near"
    if far is not None and abs(ref_cm - far) <= 0.05:
        return "Far"
    return f"{ref_cm:.1f} cm"


def _load_events(events_path: Path | None, cal_ctx: dict[str, float]) -> list[dict[str, object]]:
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
            continue
        if line.startswith("HOST_STD,mark,"):
            continue
        if line.startswith("SET,"):
            parts = [p.strip() for p in line.split(",")]
            if len(parts) == 5:
                try:
                    ref_cm = float(parts[2])
                    label = _setpoint_label(ref_cm, cal_ctx)
                    t_ms = int(float(parts[1]))
                except ValueError:
                    t_ms = None
        if t_ms is None or not label:
            continue
        events.append({"t_s": t_ms / 1000.0, "label": label})
    return events


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
        return pd.DataFrame(columns=METRIC_COLUMNS)

    work = df.copy()
    work["test_phase"] = work["test_phase"].fillna("")
    phase_rows: list[dict[str, object]] = []
    prev_ref_cm: float | None = None

    nonempty = work[work["test_phase"] != ""].copy()
    if nonempty.empty:
        return pd.DataFrame(columns=METRIC_COLUMNS)

    phase_id = (nonempty["test_phase"] != nonempty["test_phase"].shift()).cumsum()
    for _, phase_df in nonempty.groupby(phase_id, sort=False):
        metrics = _segment_metrics(phase_df, prev_ref_cm)
        phase_rows.append(metrics)
        prev_ref_cm = float(metrics["x_ref_cm"])

    return pd.DataFrame(phase_rows, columns=METRIC_COLUMNS)


def _decode_fault_label(df: pd.DataFrame) -> str:
    labels: list[str] = []
    seen: set[str] = set()
    for bits in df.get("fault_flags", pd.Series([0] * len(df))).fillna(0).astype(int).tolist():
        for mask, name in FAULT_BITS.items():
            if bits & mask and name not in seen:
                labels.append(name)
                seen.add(name)
    if not labels:
        final_state = str(df["state"].iloc[-1]) if "state" in df.columns and not df.empty else ""
        return "FAULT" if final_state == "FAULT" else ""
    return ", ".join(labels)


def _draw_setpoint_events(ax: plt.Axes, events: list[dict[str, object]]) -> None:
    if not events:
        return
    for event in events:
        t_s = float(event["t_s"])
        label = str(event["label"])
        ax.axvline(t_s, color="#b8c1cc", linestyle="--", linewidth=0.9, alpha=0.9)
        ax.text(
            t_s,
            1.01,
            label,
            transform=ax.get_xaxis_transform(),
            rotation=90,
            va="bottom",
            ha="center",
            fontsize=8,
            color="#4b5563",
        )


def _shade_fault_region(axes: list[plt.Axes], df: pd.DataFrame) -> None:
    if "state" not in df.columns:
        return
    work = df.copy()
    work["state"] = work["state"].fillna("")
    mask = work["state"] == "FAULT"
    if not mask.any():
        return
    groups = (mask != mask.shift(fill_value=False)).cumsum()
    for _, seg in work[mask].groupby(groups[mask], sort=False):
        t0 = float(seg["t_ms"].iloc[0]) / 1000.0
        t1 = float(seg["t_ms"].iloc[-1]) / 1000.0
        for ax in axes:
            ax.axvspan(t0, t1, color="#fee2e2", alpha=0.55, linewidth=0)


def _compute_ball_position_series(df: pd.DataFrame, show_raw: bool) -> tuple[pd.Series, pd.Series | None]:
    if "x_linear_filt_cm" in df.columns:
        filtered = df["x_linear_filt_cm"].astype(float)
        raw = df["x_linear_cm"].astype(float) if show_raw and "x_linear_cm" in df.columns else None
        return filtered, raw
    filtered = df["x_filt_cm"].astype(float)
    raw = df["x_cm"].astype(float) if show_raw and "x_cm" in df.columns else None
    return filtered, raw


def _compute_beam_angle_series(df: pd.DataFrame) -> tuple[pd.Series, str]:
    if "act_deg_abs" in df.columns and "trim_deg" in df.columns:
        act = df["act_deg_abs"].astype(float)
        trim = df["trim_deg"].astype(float)
        if act.notna().any() and trim.notna().any():
            return act - trim, "AS5600"
    return df["theta_deg"].astype(float), "Stepper estimate"


def _set_standard_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.edgecolor": "#9ca3af",
            "axes.labelcolor": "#111827",
            "axes.titlesize": 12,
            "axes.titleweight": "bold",
            "grid.color": "#d1d5db",
            "grid.alpha": 0.45,
            "font.size": 10,
            "legend.frameon": True,
            "legend.facecolor": "white",
            "legend.edgecolor": "#d1d5db",
            "savefig.facecolor": "white",
        }
    )


def _render_standard(
    df: pd.DataFrame,
    events: list[dict[str, object]],
    cal_ctx: dict[str, float],
    run_stem: str,
    output_path: Path,
    show_raw: bool,
) -> None:
    _set_standard_style()
    t_s = df["t_ms"].astype(float) / 1000.0
    ball_pos, raw_ball = _compute_ball_position_series(df, show_raw)
    beam_angle, beam_source = _compute_beam_angle_series(df)
    control_effort = df["u_step_rate"].astype(float)
    ref = df.get("x_ref_cm", pd.Series([0.0] * len(df))).astype(float)

    fig, axes = plt.subplots(3, 1, figsize=(10.5, 7.5), sharex=True)

    axes[0].plot(t_s, ref, color="#d97706", linestyle="--", linewidth=2.0, label="Reference")
    axes[0].plot(t_s, ball_pos, color="#1d4ed8", linewidth=2.2, label="Ball position")
    if raw_ball is not None:
        axes[0].plot(t_s, raw_ball, color="#93c5fd", linewidth=1.1, alpha=0.75, label="Ball position (raw)")
    axes[0].set_ylabel("Ball position (cm)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)
    _draw_setpoint_events(axes[0], events)

    axes[1].plot(t_s, beam_angle, color="#b91c1c", linewidth=2.0)
    if "theta_lower_limit_deg" in cal_ctx and "theta_upper_limit_deg" in cal_ctx:
        lo = cal_ctx["theta_lower_limit_deg"]
        hi = cal_ctx["theta_upper_limit_deg"]
        axes[1].axhline(lo + RUN_SOFT_LIMIT_BAND_DEG, color="#9ca3af", linestyle=":", linewidth=0.9)
        axes[1].axhline(hi - RUN_SOFT_LIMIT_BAND_DEG, color="#9ca3af", linestyle=":", linewidth=0.9)
    axes[1].set_ylabel("Beam angle (deg)")
    axes[1].set_title(f"Angle source: {beam_source}", loc="right", fontsize=9, fontweight="normal")
    axes[1].grid(True)

    axes[2].plot(t_s, control_effort, color="#047857", linewidth=1.8)
    axes[2].axhline(0.0, color="#9ca3af", linewidth=0.9)
    axes[2].set_ylabel("Control effort (steps/s)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True)

    _shade_fault_region(list(axes), df)

    final_state = str(df["state"].iloc[-1]) if "state" in df.columns and not df.empty else ""
    fault_label = _decode_fault_label(df)
    fig.suptitle(f"{run_stem}", fontsize=14, fontweight="bold", y=0.985)
    subtitle = f"Final state: {final_state}" if final_state else ""
    if fault_label:
        subtitle = f"{subtitle} | Fault: {fault_label}" if subtitle else f"Fault: {fault_label}"
    if subtitle:
        fig.text(0.5, 0.955, subtitle, ha="center", va="center", fontsize=9, color="#6b7280")

    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def _render_diagnostic(
    df: pd.DataFrame,
    events: list[dict[str, object]],
    cal_ctx: dict[str, float],
    run_stem: str,
    output_path: Path,
) -> None:
    t_s = df["t_ms"] / 1000.0
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

    for ax in axes:
        for event in events:
            t_event = float(event["t_s"])
            ax.axvline(t_event, color="#6c757d", linestyle="--", linewidth=0.8, alpha=0.35)
    fig.suptitle(f"Diagnostic Plot: {run_stem}")
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot a ball-and-beam telemetry run")
    parser.add_argument("--input", type=Path, required=True, help="Path to a run directory or *_telemetry.csv / *_clean.csv")
    parser.add_argument("--output", type=Path, default=None, help="Output image path")
    parser.add_argument("--events", type=Path, default=None, help="Optional events log path")
    parser.add_argument("--metrics-output", type=Path, default=None, help="Optional metrics CSV path")
    parser.add_argument("--style", choices=["standard", "diagnostic"], default="standard", help="Plot style")
    parser.add_argument("--show-raw", type=int, choices=[0, 1], default=0, help="Overlay raw ball position in standard mode")
    args = parser.parse_args()

    csv_path, run_dir, run_stem = resolve_run_input(args.input)
    if not csv_path.exists():
        raise SystemExit(f"Input not found: {csv_path}")

    if args.output is None:
        if run_stem.startswith("run_"):
            args.output = summary_plot_path(run_dir, run_stem)
        else:
            args.output = csv_path.with_suffix(".png")
    if args.events is None:
        candidate = run_events_path(run_dir, run_stem) if run_stem.startswith("run_") else _fallback_events_path(csv_path)
        args.events = candidate if candidate.exists() else _fallback_events_path(csv_path)
    if args.metrics_output is None:
        if run_stem.startswith("run_"):
            args.metrics_output = run_metrics_path(run_dir, run_stem)
        else:
            args.metrics_output = _fallback_metrics_path(csv_path)

    df = pd.read_csv(csv_path)
    if "x_ref_cm" not in df.columns:
        df["x_ref_cm"] = 0.0
    if "test_phase" not in df.columns:
        df["test_phase"] = ""

    cal_ctx = _load_cal_context(args.events)
    events = _load_events(args.events, cal_ctx)

    if args.style == "standard":
        _render_standard(
            df=df,
            events=events,
            cal_ctx=cal_ctx,
            run_stem=run_stem,
            output_path=args.output,
            show_raw=(args.show_raw != 0),
        )
    else:
        _render_diagnostic(
            df=df,
            events=events,
            cal_ctx=cal_ctx,
            run_stem=run_stem,
            output_path=args.output,
        )

    metrics = _compute_metrics(df)
    args.metrics_output.parent.mkdir(parents=True, exist_ok=True)
    metrics.to_csv(args.metrics_output, index=False)
    if not metrics.empty:
        print("\nMetrics")
        print(metrics.to_string(index=False, float_format=lambda v: f"{v:.3f}"))
    print(f"Saved metrics: {args.metrics_output}")

    print(f"Saved plot: {args.output}")


if __name__ == "__main__":
    main()
