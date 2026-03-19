#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

if __package__ in (None, ""):
    workspace_root = Path(__file__).resolve().parents[1]
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))

from analysis.run_layout import derive_run_stem, plots_dir_for_run


@dataclass
class RefConfig:
    profile: str = "sharp_ref"
    center_cm: float = math.nan
    near_cm: float = math.nan
    far_cm: float = math.nan
    segment_ms: int = 0


def _parse_keyvals(prefix: str, line: str) -> dict[str, str]:
    out: dict[str, str] = {}
    if not line.startswith(prefix):
        return out
    for part in line[len(prefix) :].split(","):
        part = part.strip()
        if not part or "=" not in part:
            continue
        key, value = part.split("=", 1)
        out[key.strip()] = value.strip()
    return out


def _resolve_raw_input(input_path: Path) -> tuple[Path, Path, str]:
    if input_path.is_dir():
        candidates = sorted(input_path.glob("run_*_raw.log"))
        if len(candidates) != 1:
            raise SystemExit(f"Expected exactly one raw log in {input_path}, found {len(candidates)}")
        raw_path = candidates[0]
        return raw_path, input_path, derive_run_stem(raw_path)

    if input_path.name.endswith("_raw.log"):
        return input_path, input_path.parent, derive_run_stem(input_path)

    raise SystemExit("Input must be a run directory or a *_raw.log file")


def _parse_raw_log(raw_path: Path, default_target_cm: float) -> tuple[pd.DataFrame, pd.DataFrame, RefConfig]:
    tel_rows: list[dict[str, object]] = []
    ref_rows: list[dict[str, object]] = []
    cfg = RefConfig(center_cm=default_target_cm)

    for raw in raw_path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if not line:
            continue

        if line.startswith("REF_CFG,"):
            kv = _parse_keyvals("REF_CFG,", line)
            cfg.profile = kv.get("profile", cfg.profile)
            try:
                cfg.center_cm = float(kv.get("center_cm", cfg.center_cm))
            except ValueError:
                pass
            try:
                cfg.near_cm = float(kv.get("near_cm", "nan"))
            except ValueError:
                cfg.near_cm = math.nan
            try:
                cfg.far_cm = float(kv.get("far_cm", "nan"))
            except ValueError:
                cfg.far_cm = math.nan
            try:
                cfg.segment_ms = int(float(kv.get("segment_ms", "0")))
            except ValueError:
                cfg.segment_ms = 0
            continue

        if line.startswith("REF,"):
            parts = [part.strip() for part in line.split(",")]
            if len(parts) < 4:
                continue
            try:
                ref_rows.append(
                    {
                        "t_ms": int(float(parts[1])),
                        "x_ref_cm": float(parts[2]),
                        "phase": parts[3],
                    }
                )
            except ValueError:
                continue
            continue

        if not line.startswith("TEL,"):
            continue

        parts = [part.strip() for part in line.split(",")]
        if len(parts) < 8:
            continue
        try:
            row = {
                "t_ms": int(float(parts[1])),
                "x_raw_cm": float(parts[2]),
                "x_filt_cm": float(parts[3]),
                "input_cm": float(parts[4]),
                "u_steps": float(parts[5]),
                "beam_steps": int(float(parts[6])),
                "beam_angle_rad": float(parts[7]),
                "ball_vel_cm_s": math.nan,
                "integral_steps": math.nan,
                "bias_steps": math.nan,
                "theta_meas_deg": math.nan,
                "theta_cmd_deg": math.nan,
                "theta_err_deg": math.nan,
                "inner_rate_sps": math.nan,
                "as5600_ok": math.nan,
            }
            if len(parts) >= 11:
                row["ball_vel_cm_s"] = float(parts[8])
                row["integral_steps"] = float(parts[9])
                row["bias_steps"] = float(parts[10])
            if len(parts) >= 16:
                row["theta_meas_deg"] = float(parts[11])
                row["theta_cmd_deg"] = float(parts[12])
                row["theta_err_deg"] = float(parts[13])
                row["inner_rate_sps"] = float(parts[14])
                row["as5600_ok"] = float(parts[15])
            tel_rows.append(
                row
            )
        except ValueError:
            continue

    tel_df = pd.DataFrame(tel_rows)
    if tel_df.empty:
        raise SystemExit(f"No Sharp TEL rows found in {raw_path}")

    ref_df = pd.DataFrame(ref_rows)
    if ref_df.empty:
        ref_df = pd.DataFrame(
            [{"t_ms": int(tel_df["t_ms"].iloc[0]), "x_ref_cm": cfg.center_cm, "phase": "hold_center"}]
        )

    tel_df = tel_df.sort_values("t_ms").reset_index(drop=True)
    ref_df = ref_df.sort_values("t_ms").reset_index(drop=True)
    tel_df = pd.merge_asof(tel_df, ref_df, on="t_ms", direction="backward")
    tel_df["phase"] = tel_df["phase"].fillna("hold_center")
    tel_df["x_ref_cm"] = tel_df["x_ref_cm"].fillna(cfg.center_cm)
    tel_df["t_s"] = 0.001 * (tel_df["t_ms"] - float(tel_df["t_ms"].iloc[0]))
    tel_df["error_cm"] = tel_df["x_filt_cm"] - tel_df["x_ref_cm"]
    tel_df["beam_angle_deg"] = tel_df["beam_angle_rad"] * (180.0 / math.pi)
    if "theta_meas_deg" in tel_df:
        tel_df["step_vs_encoder_deg"] = tel_df["beam_angle_deg"] - tel_df["theta_meas_deg"]
    return tel_df, ref_df, cfg


def _settling_time_s(seg: pd.DataFrame, band_cm: float, hold_s: float) -> float:
    if seg.empty:
        return math.nan
    t = seg["t_s"].to_list()
    err = seg["error_cm"].abs().to_list()
    for i, value in enumerate(err):
        if value > band_cm:
            continue
        t0 = t[i]
        ok = True
        j = i
        while j < len(err) and (t[j] - t0) < hold_s:
            if err[j] > band_cm:
                ok = False
                break
            j += 1
        if ok and j > i and (t[min(j - 1, len(t) - 1)] - t0) >= hold_s:
            return t0 - t[0]
    return math.nan


def _rise_time_s(seg: pd.DataFrame) -> float:
    if seg.empty:
        return math.nan
    y0 = float(seg["x_filt_cm"].iloc[0])
    y1 = float(seg["x_ref_cm"].iloc[0])
    denom = y1 - y0
    if abs(denom) < 1.0e-6:
        return math.nan

    progress = (seg["x_filt_cm"] - y0) / denom
    t = seg["t_s"].to_list()
    p = progress.to_list()
    t10 = None
    t90 = None
    for ti, pi in zip(t, p):
        if t10 is None and pi >= 0.1:
            t10 = ti
        if t90 is None and pi >= 0.9:
            t90 = ti
            break
    if t10 is None or t90 is None:
        return math.nan
    return t90 - t10


def _integral_metrics(seg: pd.DataFrame) -> tuple[float, float]:
    if len(seg) < 2:
        return 0.0, 0.0
    t_rel = seg["t_s"] - float(seg["t_s"].iloc[0])
    abs_err = seg["error_cm"].abs()
    dt = seg["t_s"].diff().fillna(0.0)
    iae = float((abs_err * dt).sum())
    itae = float((t_rel * abs_err * dt).sum())
    return iae, itae


def _max_invalid_duration_s(seg: pd.DataFrame) -> float:
    if "as5600_ok" not in seg or seg["as5600_ok"].isna().all():
        return math.nan

    max_invalid_s = 0.0
    current_invalid_s = 0.0
    prev_t = None
    for _, row in seg.iterrows():
        t_s = float(row["t_s"])
        ok = float(row["as5600_ok"]) >= 0.5
        if prev_t is not None:
            dt_s = max(0.0, t_s - prev_t)
            if not ok:
                current_invalid_s += dt_s
                max_invalid_s = max(max_invalid_s, current_invalid_s)
            else:
                current_invalid_s = 0.0
        prev_t = t_s
    return max_invalid_s


def _compute_metrics(df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    prev_target: float | None = None
    phase_id = (df["phase"] != df["phase"].shift()).cumsum()
    for _, seg in df.groupby(phase_id, sort=False):
        seg = seg.reset_index(drop=True)
        target = float(seg["x_ref_cm"].iloc[0])
        phase = str(seg["phase"].iloc[0])
        start_val = float(seg["x_filt_cm"].iloc[0])
        duration_s = float(seg["t_s"].iloc[-1] - seg["t_s"].iloc[0])
        step_cm = math.nan if prev_target is None else abs(target - prev_target)
        band_cm = 0.5 if not math.isfinite(step_cm) else max(0.5, 0.05 * step_cm)
        settling_time_s = _settling_time_s(seg, band_cm=band_cm, hold_s=1.5)
        rise_time_s = _rise_time_s(seg)
        tail = seg[seg["t_s"] >= (float(seg["t_s"].iloc[-1]) - min(2.0, max(0.5, duration_s * 0.2)))]
        if tail.empty:
            tail = seg
        steady_state_error_cm = float((tail["x_filt_cm"] - target).mean())
        if target >= start_val:
            overshoot_cm = max(0.0, float(seg["x_filt_cm"].max() - target))
        else:
            overshoot_cm = max(0.0, float(target - seg["x_filt_cm"].min()))
        iae, itae = _integral_metrics(seg)
        rows.append(
            {
                "phase": phase,
                "x_ref_cm": target,
                "duration_s": duration_s,
                "step_cm": step_cm,
                "steady_state_error_cm": steady_state_error_cm,
                "overshoot_cm": overshoot_cm,
                "settling_time_s": settling_time_s,
                "rise_time_s": rise_time_s,
                "rmse_cm": float(math.sqrt((seg["error_cm"] ** 2).mean())),
                "iae_cm_s": iae,
                "itae_cm_s2": itae,
                "peak_abs_output_steps": float(seg["u_steps"].abs().max()),
                "peak_abs_beam_deg": float(seg["beam_angle_deg"].abs().max()),
                "tail_theta_err_deg": (
                    math.nan
                    if "theta_err_deg" not in seg or tail["theta_err_deg"].isna().all()
                    else float(tail["theta_err_deg"].mean())
                ),
                "peak_abs_theta_cmd_deg": (
                    math.nan
                    if "theta_cmd_deg" not in seg or seg["theta_cmd_deg"].isna().all()
                    else float(seg["theta_cmd_deg"].abs().max())
                ),
                "peak_abs_theta_meas_deg": (
                    math.nan
                    if "theta_meas_deg" not in seg or seg["theta_meas_deg"].isna().all()
                    else float(seg["theta_meas_deg"].abs().max())
                ),
                "max_as5600_invalid_s": _max_invalid_duration_s(seg),
                "max_abs_step_vs_encoder_deg": (
                    math.nan
                    if "step_vs_encoder_deg" not in seg or seg["step_vs_encoder_deg"].isna().all()
                    else float(seg["step_vs_encoder_deg"].abs().max())
                ),
            }
        )
        prev_target = target
    return pd.DataFrame(rows)


def _plot(df: pd.DataFrame, ref_df: pd.DataFrame, cfg: RefConfig, out_path: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    axes[0].plot(df["t_s"], df["x_raw_cm"], color="#94a3b8", alpha=0.5, label="raw")
    axes[0].plot(df["t_s"], df["x_filt_cm"], color="#1d3557", linewidth=1.8, label="filtered")
    axes[0].plot(df["t_s"], df["x_ref_cm"], color="#d97706", linewidth=1.5, linestyle="--", label="reference")
    axes[0].set_ylabel("Distance (cm)")
    axes[0].legend(loc="best")
    axes[0].grid(alpha=0.25)

    axes[1].plot(df["t_s"], df["error_cm"], color="#b91c1c", linewidth=1.5)
    axes[1].axhline(0.0, color="#111827", linewidth=0.8, alpha=0.5)
    axes[1].set_ylabel("Error (cm)")
    axes[1].grid(alpha=0.25)

    axes[2].plot(df["t_s"], df["u_steps"], color="#0f766e", linewidth=1.4, label="control output")
    axes[2].plot(df["t_s"], df["beam_angle_deg"], color="#94a3b8", linewidth=1.0, alpha=0.8, label="step angle est")
    if "theta_meas_deg" in df and not df["theta_meas_deg"].isna().all():
        axes[2].plot(df["t_s"], df["theta_meas_deg"], color="#7c3aed", linewidth=1.1, alpha=0.9, label="theta meas")
    if "theta_cmd_deg" in df and not df["theta_cmd_deg"].isna().all():
        axes[2].plot(df["t_s"], df["theta_cmd_deg"], color="#d97706", linewidth=1.0, alpha=0.85, linestyle="--", label="theta cmd")
    axes[2].set_ylabel("Steps / deg")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(alpha=0.25)
    axes[2].legend(loc="best")

    for _, ref in ref_df.iterrows():
        t_s = 0.001 * (float(ref["t_ms"]) - float(df["t_ms"].iloc[0]))
        for ax in axes:
            ax.axvline(t_s, color="#9ca3af", linewidth=0.8, linestyle=":", alpha=0.8)
        axes[0].text(
            t_s,
            axes[0].get_ylim()[1],
            str(ref["phase"]),
            rotation=90,
            va="top",
            ha="right",
            fontsize=8,
            color="#6b7280",
        )

    fig.suptitle(f"Sharp Reference Run: {cfg.profile}")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot and summarize Sharp reference raw logs")
    parser.add_argument("--input", type=Path, required=True, help="Run directory or *_raw.log path")
    parser.add_argument(
        "--default-target-cm",
        type=float,
        default=13.0,
        help="Fallback target if the raw log has no REF markers",
    )
    args = parser.parse_args()

    raw_path, run_dir, run_stem = _resolve_raw_input(args.input)
    df, ref_df, cfg = _parse_raw_log(raw_path, default_target_cm=args.default_target_cm)
    metrics_df = _compute_metrics(df)

    plot_path = plots_dir_for_run(run_dir) / f"{run_stem}_sharp_summary.png"
    metrics_path = run_dir / f"{run_stem}_sharp_metrics.csv"
    _plot(df, ref_df, cfg, plot_path)
    metrics_df.to_csv(metrics_path, index=False)

    print("Saved plot:", plot_path)
    print("Saved metrics:", metrics_path)
    if not metrics_df.empty:
        print(metrics_df.to_string(index=False))


if __name__ == "__main__":
    main()
