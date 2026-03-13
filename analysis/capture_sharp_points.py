#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from pathlib import Path

import serial

if __package__ in (None, ""):
    workspace_root = Path(__file__).resolve().parents[1]
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))

from analysis.characterization_common import (
    CharacterizationMetadata,
    default_point_label,
    iso_now,
    mean,
    save_metadata,
    stamp,
    stddev,
)
from analysis.run_layout import characterization_meta_path, raw_log_path, run_dir_for_stem

SHARP_LINE_RE = re.compile(
    r"^SHARP_IR,raw_adc=(?P<raw_adc>\d+),voltage=(?P<voltage>-?\d+(?:\.\d+)?),distance_cm=(?P<distance>-?\d+(?:\.\d+)?)(?:,.*)?$"
)


def _parse_sharp_line(line: str, min_valid_cm: float, max_valid_cm: float) -> dict[str, float | bool] | None:
    match = SHARP_LINE_RE.fullmatch(line.strip())
    if match is None:
        return None
    distance_cm = float(match.group("distance"))
    return {
        "raw_adc": float(match.group("raw_adc")),
        "voltage": float(match.group("voltage")),
        "distance_cm": distance_cm,
        "valid_sample": min_valid_cm <= distance_cm <= max_valid_cm,
    }


def _capture_lines(ser: serial.Serial, raw_file, seconds: float) -> list[str]:
    lines: list[str] = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        line = ser.readline().decode("utf-8", errors="replace")
        if not line:
            continue
        raw_file.write(line)
        raw_file.flush()
        lines.append(line.rstrip())
        print(line.rstrip())
    return lines


def _send_device_command(ser: serial.Serial, raw_file, command: str, settle_seconds: float = 0.5) -> list[str]:
    raw_file.write(f"HOST_CMD,{command}\n")
    raw_file.flush()
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()
    print(f"sent: {command}")
    return _capture_lines(ser, raw_file, settle_seconds)


def _capture_point_samples(
    ser: serial.Serial,
    raw_file,
    samples_per_point: int,
    timeout_s: float,
    min_valid_cm: float,
    max_valid_cm: float,
) -> tuple[list[dict[str, float | bool]], list[str]]:
    ser.reset_input_buffer()
    parsed_samples: list[dict[str, float | bool]] = []
    raw_lines: list[str] = []
    deadline = time.monotonic() + timeout_s

    while len(parsed_samples) < samples_per_point and time.monotonic() < deadline:
        line = ser.readline().decode("utf-8", errors="replace")
        if not line:
            continue

        raw_file.write(line)
        raw_file.flush()
        raw_lines.append(line.rstrip())

        parsed = _parse_sharp_line(line, min_valid_cm=min_valid_cm, max_valid_cm=max_valid_cm)
        if parsed is not None:
            parsed_samples.append(parsed)

    return parsed_samples, raw_lines


def _capture_summary_row(
    parsed_samples: list[dict[str, float | bool]],
) -> tuple[int, float, str, str, str, str, float, float, float, float, float, float, float, float]:
    valid_distances = [float(sample["distance_cm"]) for sample in parsed_samples if bool(sample["valid_sample"])]
    raw_adcs = [float(sample["raw_adc"]) for sample in parsed_samples]
    voltages = [float(sample["voltage"]) for sample in parsed_samples]
    valid_samples = len(valid_distances)
    valid_fraction = valid_samples / float(len(parsed_samples))

    def _fmt_or_blank(values: list[float], fn) -> str:
        if not values:
            return ""
        return f"{fn(values):.4f}"

    return (
        valid_samples,
        valid_fraction,
        _fmt_or_blank(valid_distances, mean),
        _fmt_or_blank(valid_distances, stddev),
        _fmt_or_blank(valid_distances, min),
        _fmt_or_blank(valid_distances, max),
        mean(raw_adcs),
        stddev(raw_adcs),
        min(raw_adcs),
        max(raw_adcs),
        mean(voltages),
        stddev(voltages),
        min(voltages),
        max(voltages),
    )


def _reverse_label_direction(direction: str, point_number: int, step: int) -> tuple[str, int, int]:
    next_point = max(0, point_number - step)
    if direction == "out":
        return "ret", next_point, -1
    return "out", next_point, 1


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Interactively capture settled Sharp GP2Y0A21YK0F readings at user-controlled runner points."
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/cu.usbserial-XXXX")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--samples-per-point",
        type=int,
        default=25,
        help="Number of SHARP_IR samples to capture after each Enter press",
    )
    parser.add_argument(
        "--boot-seconds",
        type=float,
        default=2.0,
        help="Seconds to display/capture boot lines after connecting",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="Per-point capture timeout in seconds",
    )
    parser.add_argument(
        "--timer1-stress",
        type=int,
        choices=(0, 1),
        default=0,
        help="Send `T` once after boot to enable the 40 kHz Timer1 stress ISR",
    )
    parser.add_argument("--min-valid-cm", type=float, default=10.0, help="Minimum accepted distance")
    parser.add_argument("--max-valid-cm", type=float, default=80.0, help="Maximum accepted distance")
    parser.add_argument("--sensor-name", default="sharp", help="Metadata sensor name")
    parser.add_argument("--sensor-mode", default="analog", help="Metadata sensor mode")
    parser.add_argument("--target-name", default="table_tennis_40mm", help="Metadata target name")
    parser.add_argument("--block-label", default="sensor_block_1", help="Metadata block label")
    parser.add_argument("--repeat-index", type=int, default=1, help="Metadata repeat index")
    parser.add_argument("--photo-ref", default="", help="Photo reference for the current mount")
    parser.add_argument("--mount-note", default="", help="Short note about sensor mount/angle/spacers")
    parser.add_argument("--notes", default="", help="Free-form run note stored in metadata")
    parser.add_argument(
        "--start-direction",
        choices=("out", "ret"),
        default="out",
        help="Direction used for automatic labels",
    )
    parser.add_argument(
        "--start-point-index",
        type=int,
        default=0,
        help="Starting point number for automatic labels",
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
    if args.samples_per_point <= 0:
        raise SystemExit("--samples-per-point must be > 0")
    if args.start_point_index < 0:
        raise SystemExit("--start-point-index must be >= 0")

    args.outdir.mkdir(parents=True, exist_ok=True)
    run_stem = f"run_{stamp()}"
    run_dir = run_dir_for_stem(args.outdir, run_stem)
    run_dir.mkdir(parents=True, exist_ok=True)

    raw_path = raw_log_path(run_dir, run_stem)
    points_path = run_dir / f"{run_stem}_sharp_points.csv"
    samples_path = run_dir / f"{run_stem}_sharp_point_samples.csv"
    meta_path = characterization_meta_path(run_dir, run_stem)

    metadata = CharacterizationMetadata(
        run_stem=run_stem,
        capture_kind="point_characterization",
        sensor_name=args.sensor_name,
        sensor_mode=args.sensor_mode,
        target_name=args.target_name,
        block_label=args.block_label,
        repeat_index=args.repeat_index,
        timer1_stress=bool(args.timer1_stress),
        mount_note=args.mount_note,
        photo_ref=args.photo_ref,
        notes=args.notes,
        created_at_iso=iso_now(),
        samples_per_point=args.samples_per_point,
        label_scheme="out_pNN / ret_pNN",
        raw_log_name=raw_path.name,
        points_csv_name=points_path.name,
        samples_csv_name=samples_path.name,
    )
    save_metadata(meta_path, metadata)

    print("Interactive Sharp point capture")
    print(f"  RAW     : {raw_path}")
    print(f"  POINTS  : {points_path}")
    print(f"  SAMPLES : {samples_path}")
    print(f"  META    : {meta_path}")
    print("Upload `sharp_ir_check` before using this script.")
    print("Settle the ball at each runner mark, then press Enter to capture a burst.")
    print("Type `/reverse` when you turn around. Type `q` and press Enter when finished.")

    capture_index = 0
    direction = args.start_direction
    point_number = args.start_point_index
    step = 1 if direction == "out" else -1

    with serial.Serial(args.port, args.baud, timeout=0.25) as ser, raw_path.open(
        "w", encoding="utf-8"
    ) as raw_file, points_path.open("w", newline="", encoding="utf-8") as points_file, samples_path.open(
        "w", newline="", encoding="utf-8"
    ) as samples_file:
        points_writer = csv.writer(points_file)
        samples_writer = csv.writer(samples_file)

        points_writer.writerow(
            [
                "capture_index",
                "point_direction",
                "point_number",
                "label",
                "captured_at_iso",
                "samples_requested",
                "samples_captured",
                "valid_samples",
                "valid_fraction",
                "mean_distance_cm",
                "std_distance_cm",
                "min_distance_cm",
                "max_distance_cm",
                "mean_raw_adc",
                "std_raw_adc",
                "min_raw_adc",
                "max_raw_adc",
                "mean_voltage_v",
                "std_voltage_v",
                "min_voltage_v",
                "max_voltage_v",
            ]
        )
        samples_writer.writerow(
            [
                "capture_index",
                "point_direction",
                "point_number",
                "label",
                "sample_index",
                "captured_at_iso",
                "raw_adc",
                "voltage_v",
                "distance_cm",
                "valid_sample",
            ]
        )

        _capture_lines(ser, raw_file, args.boot_seconds)
        if args.timer1_stress:
            _send_device_command(ser, raw_file, "T")

        while True:
            default_label = default_point_label(direction, point_number)
            prompt = (
                f"[{default_label}] Enter=capture, `/reverse` flips auto labels, "
                "`/set out|ret <index>` sets the next default, type a label to override, or `q` to finish: "
            )
            user_entry = input(prompt).strip()
            if user_entry.lower() == "q":
                break
            if user_entry.lower() == "/reverse":
                direction, point_number, step = _reverse_label_direction(direction, point_number, step)
                print(f"Auto labels switched to `{default_point_label(direction, point_number)}`")
                continue
            if user_entry.lower().startswith("/set "):
                parts = user_entry.split()
                if len(parts) != 3 or parts[1] not in {"out", "ret"}:
                    print("Usage: /set out <index>  or  /set ret <index>")
                    continue
                try:
                    point_number = max(0, int(parts[2]))
                except ValueError:
                    print("Point index must be an integer")
                    continue
                direction = parts[1]
                step = 1 if direction == "out" else -1
                print(f"Auto labels switched to `{default_point_label(direction, point_number)}`")
                continue

            label = user_entry if user_entry else default_label

            print(f"Capturing {args.samples_per_point} samples for `{label}` ...")
            parsed_samples, _raw_lines = _capture_point_samples(
                ser=ser,
                raw_file=raw_file,
                samples_per_point=args.samples_per_point,
                timeout_s=args.timeout,
                min_valid_cm=args.min_valid_cm,
                max_valid_cm=args.max_valid_cm,
            )

            if len(parsed_samples) < args.samples_per_point:
                print(
                    f"Capture failed for `{label}`: collected {len(parsed_samples)}/{args.samples_per_point} samples before timeout."
                )
                continue

            captured_at = iso_now()
            current_direction = direction
            current_point_number = point_number
            valid_samples, valid_fraction, mean_dist, std_dist, min_dist, max_dist, mean_adc, std_adc, min_adc, max_adc, mean_volt, std_volt, min_volt, max_volt = _capture_summary_row(
                parsed_samples
            )

            for sample_index, sample in enumerate(parsed_samples):
                samples_writer.writerow(
                    [
                        capture_index,
                        current_direction,
                        current_point_number,
                        label,
                        sample_index,
                        captured_at,
                        int(round(float(sample["raw_adc"]))),
                        f"{float(sample['voltage']):.4f}",
                        f"{float(sample['distance_cm']):.4f}",
                        int(bool(sample["valid_sample"])),
                    ]
                )

            points_writer.writerow(
                [
                    capture_index,
                    current_direction,
                    current_point_number,
                    label,
                    captured_at,
                    args.samples_per_point,
                    len(parsed_samples),
                    valid_samples,
                    f"{valid_fraction:.4f}",
                    mean_dist,
                    std_dist,
                    min_dist,
                    max_dist,
                    f"{mean_adc:.4f}",
                    f"{std_adc:.4f}",
                    f"{min_adc:.4f}",
                    f"{max_adc:.4f}",
                    f"{mean_volt:.4f}",
                    f"{std_volt:.4f}",
                    f"{min_volt:.4f}",
                    f"{max_volt:.4f}",
                ]
            )
            points_file.flush()
            samples_file.flush()

            capture_index += 1
            point_number += step

    print("Saved:")
    print(f"  {raw_path}")
    print(f"  {points_path}")
    print(f"  {samples_path}")
    print(f"  {meta_path}")


if __name__ == "__main__":
    main()
