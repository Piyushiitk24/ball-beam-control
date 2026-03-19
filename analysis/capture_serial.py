#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime
import math
from pathlib import Path
import sys

import serial

if __package__ in (None, ""):
    workspace_root = Path(__file__).resolve().parents[1]
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))

from analysis.run_layout import events_path, raw_log_path, run_dir_for_stem


def _parse_keyvals(prefix: str, line: str) -> dict[str, str]:
    out: dict[str, str] = {}
    if not line.startswith(prefix):
        return out
    rest = line[len(prefix) :].strip()
    for part in rest.split(","):
        part = part.strip()
        if not part or "=" not in part:
            continue
        key, value = part.split("=", 1)
        out[key.strip()] = value.strip()
    return out


def _fmt_event_float(value: float) -> str:
    if math.isnan(value):
        return "nan"
    return f"{value:.4f}"


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture raw serial telemetry")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbserial-XXXX")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--seconds", type=float, default=30.0, help="Capture duration in seconds")
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Send the start newline immediately instead of waiting for Enter",
    )
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "data" / "runs",
        help="Root directory for per-run folders",
    )
    args = parser.parse_args()

    if args.port.strip() == "<SERIAL_PORT>":
        raise SystemExit("Replace <SERIAL_PORT> with a real device path")

    args.outdir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_stem = f"run_{stamp}"
    run_dir = run_dir_for_stem(args.outdir, run_stem)
    run_dir.mkdir(parents=True, exist_ok=True)
    out_file = raw_log_path(run_dir, run_stem)
    events_file = events_path(run_dir, run_stem)

    print(f"Capturing {args.seconds:.1f}s from {args.port} @ {args.baud} ...")
    ref_ctx: dict[str, str] = {}
    saw_ref_start = False
    last_t_ms: int | None = None
    capture_deadline_ts: float | None = None
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser, out_file.open(
        "w", encoding="utf-8"
    ) as f_out, events_file.open("w", encoding="utf-8") as f_events:
        ready_deadline_ts = datetime.now().timestamp() + 5.0
        host_ready_seen = False
        while datetime.now().timestamp() < ready_deadline_ts:
            line = ser.readline().decode("utf-8", errors="replace")
            if not line:
                continue
            f_out.write(line)
            f_out.flush()
            if line.strip() == "HOST_START_READY":
                host_ready_seen = True
                break

        if host_ready_seen:
            print("Firmware is idle and waiting for host start.")
        else:
            print("No explicit host-start banner seen; continuing with manual start.")

        if args.auto_start:
            ser.write(b"\n")
            ser.flush()
            print("Sent immediate start.")
        else:
            input("Press Enter to start control and begin the timed capture...")
            ser.write(b"\n")
            ser.flush()

        start_timeout_ts = datetime.now().timestamp() + 10.0
        while True:
            line = ser.readline().decode("utf-8", errors="replace")
            if not line:
                now_ts = datetime.now().timestamp()
                if capture_deadline_ts is not None and now_ts >= capture_deadline_ts:
                    break
                if capture_deadline_ts is None and now_ts >= start_timeout_ts:
                    raise SystemExit("Timed out waiting for the controller to emit REF start")
                continue
            f_out.write(line)
            f_out.flush()
            if capture_deadline_ts is not None and datetime.now().timestamp() >= capture_deadline_ts:
                break

            stripped = line.strip()
            if stripped.startswith("REF_CFG,"):
                ref_ctx.update(_parse_keyvals("REF_CFG,", stripped))
                f_events.write(stripped + "\n")
                profile = ref_ctx.get("profile", "sharp_ref")
                center_cm = float(ref_ctx.get("center_cm", "nan"))
                near_cm = float(ref_ctx.get("near_cm", "nan"))
                far_cm = float(ref_ctx.get("far_cm", "nan"))
                segment_ms = ref_ctx.get("segment_ms", "")
                f_events.write(
                    "HOST_STD,manifest,"
                    f"run={profile},center_target_cm={_fmt_event_float(center_cm)},"
                    f"near_target_cm={_fmt_event_float(near_cm)},"
                    f"far_target_cm={_fmt_event_float(far_cm)},segment_ms={segment_ms}\n"
                )
                f_events.flush()
                continue

            if stripped.startswith("REF,"):
                parts = [part.strip() for part in stripped.split(",")]
                if len(parts) >= 4:
                    try:
                        t_ms = int(float(parts[1]))
                        target_cm = float(parts[2])
                    except ValueError:
                        t_ms = -1
                        target_cm = math.nan
                    if t_ms >= 0:
                        last_t_ms = t_ms
                        if capture_deadline_ts is None:
                            capture_deadline_ts = datetime.now().timestamp() + args.seconds
                        profile = ref_ctx.get("profile", "sharp_ref")
                        near_cm = float(ref_ctx.get("near_cm", "nan"))
                        far_cm = float(ref_ctx.get("far_cm", "nan"))
                        if not saw_ref_start:
                            f_events.write(f"HOST_STD,start,run={profile},t_ms={t_ms}\n")
                            saw_ref_start = True
                        f_events.write(stripped + "\n")
                        f_events.write(
                            f"SET,{t_ms},{target_cm:.4f},{_fmt_event_float(near_cm)},{_fmt_event_float(far_cm)}\n"
                        )
                        f_events.write(f"HOST_STD,phase,name={parts[3]},t_ms={t_ms}\n")
                        f_events.flush()
                continue

            if stripped.startswith("TEL,"):
                parts = [part.strip() for part in stripped.split(",")]
                if len(parts) >= 2:
                    try:
                        last_t_ms = int(float(parts[1]))
                    except ValueError:
                        pass

        if saw_ref_start and last_t_ms is not None:
            profile = ref_ctx.get("profile", "sharp_ref")
            f_events.write(f"HOST_STD,stop,run={profile},t_ms={last_t_ms}\n")
            f_events.flush()

    print("Saved:", out_file)


if __name__ == "__main__":
    main()
