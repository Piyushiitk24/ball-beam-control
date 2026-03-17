#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import TextIO

import serial

if __package__ in (None, ""):
    workspace_root = Path(__file__).resolve().parents[1]
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))

from analysis.characterization_common import iso_now, stamp
from analysis.run_layout import raw_log_path, run_dir_for_stem

AUDIT_TEL_RE = re.compile(
    r"^AUDIT_TEL,(?P<t_ms>\d+),(?P<raw_adc>\d+),(?P<distance>-?\d+(?:\.\d+)?),(?P<step_pos>-?\d+)$"
)


@dataclass
class AuditSample:
    t_ms: int
    raw_adc: int
    distance_cm: float
    step_pos: int


@dataclass
class SampleSummary:
    median_distance_cm: float
    min_distance_cm: float
    max_distance_cm: float
    median_raw_adc: int
    median_step_pos: int
    sample_count: int


@dataclass
class SignAuditSummary:
    created_at_iso: str
    run_stem: str
    sensor_mount_end: str
    d_near_cm: float
    d_far_cm: float
    sensor_distance_increases_toward_motor: bool
    positive_jog_raises_motor: bool
    d_before_jog_cm: float
    d_after_jog_cm: float
    positive_jog_moves_ball_toward_sensor: bool | None
    recommended_stepper_dir_sign: int | None
    reference_pid_actuator_mapping_ok: bool | None
    reference_pid_stepper_boundary_action: str
    status: str


def _write_host_note(raw_file: TextIO, text: str) -> None:
    raw_file.write(text.rstrip("\n") + "\n")
    raw_file.flush()


def _parse_keyvals(prefix: str, line: str) -> dict[str, str]:
    if not line.startswith(prefix):
        return {}

    values: dict[str, str] = {}
    for part in line[len(prefix) :].split(","):
        part = part.strip()
        if not part or "=" not in part:
            continue
        key, value = part.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def _parse_tel(line: str) -> AuditSample | None:
    match = AUDIT_TEL_RE.fullmatch(line.strip())
    if match is None:
        return None

    return AuditSample(
        t_ms=int(match.group("t_ms")),
        raw_adc=int(match.group("raw_adc")),
        distance_cm=float(match.group("distance")),
        step_pos=int(match.group("step_pos")),
    )


def _read_line(ser: serial.Serial, raw_file: TextIO, timeout_s: float) -> str | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = ser.readline().decode("utf-8", errors="replace")
        if not line:
            continue

        _write_host_note(raw_file, line.rstrip("\n"))
        stripped = line.strip()
        if stripped and not stripped.startswith("AUDIT_TEL,"):
            print(stripped)
        return stripped

    return None


def _send_command(ser: serial.Serial, raw_file: TextIO, command: str) -> None:
    _write_host_note(raw_file, f"HOST_CMD,{command}")
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()
    print(f"sent: {command}")


def _prompt_choice(raw_file: TextIO, prompt: str, allowed: tuple[str, ...]) -> str:
    allowed_lower = tuple(choice.lower() for choice in allowed)
    while True:
        answer = input(prompt).strip().lower()
        _write_host_note(raw_file, f"HOST_INPUT,{prompt}=>{answer}")
        if answer in allowed_lower:
            return answer
        print(f"Enter one of: {', '.join(allowed)}")


def _prompt_enter(raw_file: TextIO, prompt: str) -> None:
    input(prompt)
    _write_host_note(raw_file, f"HOST_INPUT,{prompt}=>ENTER")


def _wait_for_boot(ser: serial.Serial, raw_file: TextIO, timeout_s: float) -> dict[str, str]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = _read_line(ser, raw_file, 0.5)
        if line is None:
            continue
        if line.startswith("AUDIT_BOOT,"):
            return _parse_keyvals("AUDIT_BOOT,", line)

    raise SystemExit("Did not receive AUDIT_BOOT. Flash the sharp_sign_audit firmware first.")


def _wait_for_prefix(
    ser: serial.Serial,
    raw_file: TextIO,
    prefix: str,
    timeout_s: float,
) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = _read_line(ser, raw_file, 0.5)
        if line is None:
            continue
        if line.startswith("ERR,"):
            raise SystemExit(f"Device returned error: {line}")
        if line.startswith(prefix):
            return line

    raise SystemExit(f"Timed out waiting for device response: {prefix}")


def _set_stream(
    ser: serial.Serial,
    raw_file: TextIO,
    stream_enabled: bool,
    desired: bool,
) -> bool:
    if stream_enabled == desired:
        return stream_enabled

    _send_command(ser, raw_file, "t")
    response = _wait_for_prefix(ser, raw_file, "OK,stream=", 2.0)
    actual = response.endswith("1")
    if actual != desired:
        raise SystemExit(f"Unexpected stream state after toggle: {response}")
    return actual


def _set_driver_enabled(ser: serial.Serial, raw_file: TextIO, enabled: bool) -> None:
    _send_command(ser, raw_file, f"e {1 if enabled else 0}")
    expected = f"OK,driver_enabled={1 if enabled else 0}"
    _wait_for_prefix(ser, raw_file, expected, 2.0)


def _zero_step_position(ser: serial.Serial, raw_file: TextIO) -> None:
    _send_command(ser, raw_file, "z")
    _wait_for_prefix(ser, raw_file, "OK,step_pos=0", 2.0)


def _capture_sample_burst(
    ser: serial.Serial,
    raw_file: TextIO,
    stream_enabled: bool,
    sample_count: int,
    timeout_s: float,
) -> tuple[SampleSummary, bool]:
    stream_enabled = _set_stream(ser, raw_file, stream_enabled, True)
    samples: list[AuditSample] = []
    deadline = time.monotonic() + timeout_s

    while len(samples) < sample_count and time.monotonic() < deadline:
        line = _read_line(ser, raw_file, 0.5)
        if line is None:
            continue

        sample = _parse_tel(line)
        if sample is None:
            continue
        if sample.distance_cm <= 0.0:
            continue
        samples.append(sample)

    stream_enabled = _set_stream(ser, raw_file, stream_enabled, False)

    if len(samples) < sample_count:
        raise SystemExit(f"Timed out waiting for {sample_count} valid AUDIT_TEL samples.")

    distances = [sample.distance_cm for sample in samples]
    raw_adcs = [sample.raw_adc for sample in samples]
    step_positions = [sample.step_pos for sample in samples]
    summary = SampleSummary(
        median_distance_cm=float(statistics.median(distances)),
        min_distance_cm=min(distances),
        max_distance_cm=max(distances),
        median_raw_adc=int(statistics.median(raw_adcs)),
        median_step_pos=int(statistics.median(step_positions)),
        sample_count=len(samples),
    )
    return summary, stream_enabled


def _run_jog(
    ser: serial.Serial,
    raw_file: TextIO,
    signed_steps: int,
    abs_rate_sps: float,
) -> None:
    _send_command(ser, raw_file, f"j {signed_steps} {abs_rate_sps:.3f}")
    estimated_duration_s = abs(signed_steps) / max(abs_rate_sps, 1.0)
    timeout_s = max(3.0, estimated_duration_s + 3.0)
    _wait_for_prefix(ser, raw_file, "AUDIT_JOG_BEGIN,", timeout_s)
    _wait_for_prefix(ser, raw_file, "AUDIT_JOG_END,", timeout_s)


def _build_summary(
    run_stem: str,
    sensor_mount_end: str,
    near_sample: SampleSummary,
    far_sample: SampleSummary,
    positive_jog_raises_motor: bool,
    before_sample: SampleSummary,
    after_sample: SampleSummary,
) -> SignAuditSummary:
    monotonic_ok = near_sample.median_distance_cm < far_sample.median_distance_cm
    sensor_distance_increases_toward_motor = sensor_mount_end == "pivot"
    distance_delta_cm = after_sample.median_distance_cm - before_sample.median_distance_cm

    positive_jog_moves_ball_toward_sensor: bool | None = None
    if abs(distance_delta_cm) >= 1.0:
        positive_jog_moves_ball_toward_sensor = distance_delta_cm < 0.0

    contradiction = not monotonic_ok
    if positive_jog_moves_ball_toward_sensor is not None:
        contradiction = contradiction or (positive_jog_moves_ball_toward_sensor != positive_jog_raises_motor)

    if contradiction:
        recommended_stepper_dir_sign = None
        reference_pid_actuator_mapping_ok = None
        reference_pid_stepper_boundary_action = "refuse"
        status = "contradiction"
    else:
        recommended_stepper_dir_sign = 1 if positive_jog_raises_motor else -1
        reference_pid_actuator_mapping_ok = positive_jog_raises_motor
        reference_pid_stepper_boundary_action = "leave" if positive_jog_raises_motor else "invert"
        status = "pass" if positive_jog_moves_ball_toward_sensor is not None else "inconclusive"

    return SignAuditSummary(
        created_at_iso=iso_now(),
        run_stem=run_stem,
        sensor_mount_end=sensor_mount_end,
        d_near_cm=near_sample.median_distance_cm,
        d_far_cm=far_sample.median_distance_cm,
        sensor_distance_increases_toward_motor=sensor_distance_increases_toward_motor,
        positive_jog_raises_motor=positive_jog_raises_motor,
        d_before_jog_cm=before_sample.median_distance_cm,
        d_after_jog_cm=after_sample.median_distance_cm,
        positive_jog_moves_ball_toward_sensor=positive_jog_moves_ball_toward_sensor,
        recommended_stepper_dir_sign=recommended_stepper_dir_sign,
        reference_pid_actuator_mapping_ok=reference_pid_actuator_mapping_ok,
        reference_pid_stepper_boundary_action=reference_pid_stepper_boundary_action,
        status=status,
    )


def _summary_markdown(summary: SignAuditSummary) -> str:
    distance_delta_cm = summary.d_after_jog_cm - summary.d_before_jog_cm
    recommendation = "refuse sign change until audit contradiction is resolved"
    if summary.reference_pid_stepper_boundary_action == "leave":
        recommendation = "leave the actuator direction at the stepper boundary"
    elif summary.reference_pid_stepper_boundary_action == "invert":
        recommendation = "invert the actuator direction at the stepper boundary"

    return "\n".join(
        [
            "# Sharp Sign Audit Summary",
            "",
            f"- run: `{summary.run_stem}`",
            f"- status: `{summary.status}`",
            f"- sensor mount end: `{summary.sensor_mount_end}`",
            f"- `d_near_cm`: `{summary.d_near_cm:.4f}`",
            f"- `d_far_cm`: `{summary.d_far_cm:.4f}`",
            f"- `sensor_distance_increases_toward_motor`: `{str(summary.sensor_distance_increases_toward_motor).lower()}`",
            f"- `positive_jog_raises_motor`: `{str(summary.positive_jog_raises_motor).lower()}`",
            f"- `d_before_jog_cm`: `{summary.d_before_jog_cm:.4f}`",
            f"- `d_after_jog_cm`: `{summary.d_after_jog_cm:.4f}`",
            f"- centered positive-jog delta: `{distance_delta_cm:.4f} cm`",
            f"- `positive_jog_moves_ball_toward_sensor`: `{summary.positive_jog_moves_ball_toward_sensor}`",
            f"- `recommended_stepper_dir_sign`: `{summary.recommended_stepper_dir_sign}`",
            f"- `reference_pid_actuator_mapping_ok`: `{summary.reference_pid_actuator_mapping_ok}`",
            "",
            "## Recommendation",
            "",
            f"- Generic actuator-direction recommendation: `{summary.recommended_stepper_dir_sign}`",
            f"- Current reference PID path: `{recommendation}`",
            "",
            "## Decision Rules Applied",
            "",
            "- `d_near_cm < d_far_cm` is required for Sharp geometry to be considered valid.",
            "- Positive jog should move the ball toward the sensor exactly when positive jog raises the motor side.",
            "- Contradictions block any firmware sign change recommendation.",
            "",
        ]
    ) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the Sharp-only actuator sign audit.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/cu.usbserial-XXXX")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--jog-steps", type=int, default=120, help="Positive jog steps for the audit")
    parser.add_argument("--jog-rate", type=float, default=500.0, help="Absolute jog rate in steps/s")
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "data" / "runs",
        help="Root directory for per-run folders",
    )
    parser.add_argument(
        "--snapshot-samples",
        type=int,
        default=7,
        help="Number of valid AUDIT_TEL samples to use for each settled snapshot",
    )
    args = parser.parse_args()

    if args.jog_steps <= 0:
        raise SystemExit("--jog-steps must be > 0")
    if args.jog_rate <= 0.0:
        raise SystemExit("--jog-rate must be > 0")
    if args.snapshot_samples <= 0:
        raise SystemExit("--snapshot-samples must be > 0")

    args.outdir.mkdir(parents=True, exist_ok=True)
    run_stem = f"run_{stamp()}"
    run_dir = run_dir_for_stem(args.outdir, run_stem)
    run_dir.mkdir(parents=True, exist_ok=True)

    raw_path = raw_log_path(run_dir, run_stem)
    summary_json_path = run_dir / f"{run_stem}_sign_audit_summary.json"
    summary_md_path = run_dir / f"{run_stem}_sign_audit_summary.md"

    print(f"RAW     : {raw_path}")
    print(f"SUMMARY : {summary_json_path}")
    print("Upload `sharp_sign_audit` before starting this script.")

    with serial.Serial(args.port, args.baud, timeout=0.25) as ser, raw_path.open(
        "w", encoding="utf-8"
    ) as raw_file:
        boot_info = _wait_for_boot(ser, raw_file, timeout_s=6.0)
        stream_enabled = boot_info.get("stream", "1") == "1"
        if stream_enabled:
            stream_enabled = _set_stream(ser, raw_file, stream_enabled, False)

        sensor_mount_end = _prompt_choice(raw_file, "Sensor mount end [pivot/motor]: ", ("pivot", "motor"))

        _prompt_enter(raw_file, "Place the ball near the Sharp sensor, then press Enter to capture.")
        near_sample, stream_enabled = _capture_sample_burst(
            ser, raw_file, stream_enabled, args.snapshot_samples, timeout_s=4.0
        )
        print(f"near median distance: {near_sample.median_distance_cm:.4f} cm")

        _prompt_enter(raw_file, "Place the ball far from the Sharp sensor, then press Enter to capture.")
        far_sample, stream_enabled = _capture_sample_burst(
            ser, raw_file, stream_enabled, args.snapshot_samples, timeout_s=4.0
        )
        print(f"far median distance: {far_sample.median_distance_cm:.4f} cm")

        _set_driver_enabled(ser, raw_file, True)
        _zero_step_position(ser, raw_file)

        _prompt_enter(raw_file, "Keep clear of the mechanism, then press Enter for one positive jog.")
        _run_jog(ser, raw_file, args.jog_steps, args.jog_rate)
        raised_end = _prompt_choice(raw_file, "Which physical end rose? [motor/pivot]: ", ("motor", "pivot"))
        positive_jog_raises_motor = raised_end == "motor"
        _run_jog(ser, raw_file, -args.jog_steps, args.jog_rate)
        _zero_step_position(ser, raw_file)

        _set_driver_enabled(ser, raw_file, False)
        _prompt_enter(
            raw_file,
            "Place the ball at the physical center with the driver disabled, manually level the beam as needed, and press Enter to capture the pre-jog snapshot.",
        )
        before_sample, stream_enabled = _capture_sample_burst(
            ser, raw_file, stream_enabled, args.snapshot_samples, timeout_s=4.0
        )
        print(f"before-jog median distance: {before_sample.median_distance_cm:.4f} cm")

        _zero_step_position(ser, raw_file)
        _prompt_enter(
            raw_file,
            "Keep the ball in that centered start pose, then press Enter to enable the driver and run the centered positive jog immediately.",
        )
        _set_driver_enabled(ser, raw_file, True)
        _run_jog(ser, raw_file, args.jog_steps, args.jog_rate)
        time.sleep(0.3)
        after_sample, stream_enabled = _capture_sample_burst(
            ser, raw_file, stream_enabled, args.snapshot_samples, timeout_s=4.0
        )
        print(f"after-jog median distance: {after_sample.median_distance_cm:.4f} cm")

        _run_jog(ser, raw_file, -args.jog_steps, args.jog_rate)
        _zero_step_position(ser, raw_file)
        _set_driver_enabled(ser, raw_file, False)

    summary = _build_summary(
        run_stem=run_stem,
        sensor_mount_end=sensor_mount_end,
        near_sample=near_sample,
        far_sample=far_sample,
        positive_jog_raises_motor=positive_jog_raises_motor,
        before_sample=before_sample,
        after_sample=after_sample,
    )

    summary_json_path.write_text(json.dumps(asdict(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    summary_md_path.write_text(_summary_markdown(summary), encoding="utf-8")

    print("")
    print(f"status: {summary.status}")
    print(f"recommended_stepper_dir_sign: {summary.recommended_stepper_dir_sign}")
    print(f"reference_pid_stepper_boundary_action: {summary.reference_pid_stepper_boundary_action}")
    print(f"saved: {summary_json_path}")
    print(f"saved: {summary_md_path}")


if __name__ == "__main__":
    main()
