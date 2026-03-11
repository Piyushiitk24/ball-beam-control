#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import queue
import select
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import serial
from serial.tools import list_ports


LIKELY_PORT_SUBSTRS = ("usbserial", "usbmodem", "wchusbserial")


def _now_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _now_hms() -> str:
    return datetime.now().strftime("%H:%M:%S")


def _decode_faults(bits: Optional[int]) -> str:
    if bits is None:
        return "?"
    names: list[str] = []
    if bits & 0x01:
        names.append("sonar_timeout")
    if bits & 0x02:
        names.append("i2c_error")
    if bits & 0x04:
        names.append("angle_oob")
    if bits & 0x08:
        names.append("pos_oob")
    if bits & 0x10:
        names.append("actuator_drift")
    return "none" if not names else "+".join(names)


def _autodetect_port() -> str:
    ports = list(list_ports.comports())
    likely: list[str] = []
    for p in ports:
        dev = (p.device or "").lower()
        desc = (p.description or "").lower()
        if any(s in dev for s in LIKELY_PORT_SUBSTRS):
            likely.append(p.device)
        elif any(s in desc for s in LIKELY_PORT_SUBSTRS):
            likely.append(p.device)

    if len(likely) == 1:
        return likely[0]

    if not ports:
        raise SystemExit("No serial ports found. Pass --port /dev/cu.usbserial-XXXX")

    print("Multiple serial ports found. Re-run with --port. Options:")
    for i, p in enumerate(ports, start=1):
        print(f"  {i:2d}) {p.device}  ({p.description})")
    raise SystemExit("Pass --port to select a device.")


def _parse_keyvals(prefix: str, line: str) -> dict[str, str]:
    # Example: "SENSORS,angle=ok,pos=bad"
    out: dict[str, str] = {}
    if not line.startswith(prefix):
        return out
    rest = line[len(prefix) :]
    for part in rest.split(","):
        part = part.strip()
        if not part or "=" not in part:
            continue
        k, v = part.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def _parse_tel(line: str) -> Optional[dict[str, object]]:
    # TEL,<t_ms>,<state>,<x_cm>,<x_filt_cm>,<theta_deg>,<theta_cmd_deg>,<u_step_rate>,<fault_flags>
    #   [,<x_ref_cm>,<sonar_age_ms>,<sonar_valid>,<sonar_miss_count>,
    #    <theta_cmd_unclamped_deg>,<theta_cmd_clamped_deg>,<theta_cmd_saturated>,
    #    <act_deg_abs>,<trim_deg>,<search_phase>]
    if not line.startswith("TEL,"):
        return None
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 9:
        return None
    try:
        x_ref_cm = float(parts[9]) if len(parts) >= 10 else 0.0
        sonar_age_ms = int(float(parts[10])) if len(parts) >= 11 else 0
        sonar_valid = int(float(parts[11])) if len(parts) >= 12 else 0
        sonar_miss_count = int(float(parts[12])) if len(parts) >= 13 else 0
        theta_cmd_unclamped_deg = float(parts[13]) if len(parts) >= 14 else float(parts[6])
        theta_cmd_clamped_deg = float(parts[14]) if len(parts) >= 15 else float(parts[6])
        theta_cmd_saturated = int(float(parts[15])) if len(parts) >= 16 else 0
        act_deg_abs = float(parts[16]) if len(parts) >= 17 else math.nan
        trim_deg = float(parts[17]) if len(parts) >= 18 else math.nan
        search_phase = parts[18] if len(parts) >= 19 else ""
        return {
            "t_ms": int(float(parts[1])),
            "state": parts[2],
            "x_cm": float(parts[3]),
            "x_filt_cm": float(parts[4]),
            "theta_deg": float(parts[5]),
            "theta_cmd_deg": float(parts[6]),
            "u_step_rate": float(parts[7]),
            "fault_flags": int(float(parts[8])),
            "x_ref_cm": x_ref_cm,
            "sonar_age_ms": sonar_age_ms,
            "sonar_valid": sonar_valid,
            "sonar_miss_count": sonar_miss_count,
            "theta_cmd_unclamped_deg": theta_cmd_unclamped_deg,
            "theta_cmd_clamped_deg": theta_cmd_clamped_deg,
            "theta_cmd_saturated": theta_cmd_saturated,
            "act_deg_abs": act_deg_abs,
            "trim_deg": trim_deg,
            "search_phase": search_phase,
        }
    except ValueError:
        return None


def _parse_setpoint(line: str) -> Optional[dict[str, object]]:
    # SET,<t_ms>,<x_ref_cm>,<near_target_cm>,<far_target_cm>
    if not line.startswith("SET,"):
        return None
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 5:
        return None
    try:
        return {
            "t_ms": int(float(parts[1])),
            "x_ref_cm": float(parts[2]),
            "near_mid_cm": float(parts[3]),
            "far_mid_cm": float(parts[4]),
        }
    except ValueError:
        return None


def _parse_as5600_diag(line: str) -> Optional[dict[str, object]]:
    # AS5600_DIAG,<ok>,<raw_deg>,<theta_deg>,<err_cnt>,<read_hz>
    if not line.startswith("AS5600_DIAG,"):
        return None
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 6:
        return None
    try:
        return {
            "ok": int(float(parts[1])) != 0,
            "raw_deg": float(parts[2]),
            "theta_deg": float(parts[3]),
            "err_cnt": int(float(parts[4])),
            "read_hz": float(parts[5]),
        }
    except ValueError:
        return None


@dataclass
class Snapshot:
    state: Optional[str] = None
    angle_src: Optional[int] = None  # 0=AS5600, 1=STEPPER
    angle_ok: Optional[bool] = None
    sonar_ok: Optional[bool] = None
    theta_deg: Optional[float] = None
    x_filt_cm: Optional[float] = None
    x_ref_cm: Optional[float] = None
    fault_bits: Optional[int] = None
    last_tel_rx_time: Optional[float] = None


class SerialLogger:
    def __init__(
        self,
        ser: serial.Serial,
        raw_path: Path,
        events_path: Path,
        telemetry_path: Path,
        snapshot_hz: float,
        print_tel: bool,
        no_input: bool,
    ) -> None:
        self.ser = ser
        self.raw_path = raw_path
        self.events_path = events_path
        self.telemetry_path = telemetry_path
        self.snapshot_hz = float(snapshot_hz)
        if self.snapshot_hz < 0.0:
            self.snapshot_hz = 0.0
        self.print_tel = bool(print_tel)
        self.no_input = bool(no_input)

        self.stop_event = threading.Event()
        self.rx_q: "queue.Queue[str]" = queue.Queue()
        self.snap = Snapshot()
        self.protocol: str = "unknown"  # ballbeam/as5600_check/unknown
        self._start_time = time.time()
        self._protocol_warned = False
        self._first_lines: list[str] = []
        self._suppress_print_prefixes: set[str] = set()
        self._suppress_snapshots: bool = False
        self._active_phase: str = ""
        self._last_tel_t_ms: Optional[int] = None

        self._raw_f = raw_path.open("w", encoding="utf-8")
        self._events_f = events_path.open("w", encoding="utf-8")
        self._tel_f = telemetry_path.open("w", encoding="utf-8", newline="")
        self._tel_csv = csv.writer(self._tel_f)
        self._tel_csv.writerow(
            [
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
                "test_phase",
            ]
        )
        self._tel_f.flush()

        self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True)

    def close(self) -> None:
        self.stop_event.set()
        try:
            self._raw_f.flush()
            self._events_f.flush()
            self._tel_f.flush()
        except Exception:
            pass
        try:
            self._raw_f.close()
            self._events_f.close()
            self._tel_f.close()
        except Exception:
            pass

    def _log_event(self, text: str) -> None:
        ts = _now_hms()
        self._events_f.write(f"[{ts}] {text}\n")
        self._events_f.flush()

    def _send(self, cmd: str, *, echo: bool = True, log_sent: bool = True) -> None:
        cmd = cmd.strip("\r\n")
        if not cmd:
            return
        payload = (cmd + "\n").encode("utf-8", errors="replace")
        try:
            self.ser.write(payload)
            self.ser.flush()
        except Exception as exc:
            self._log_event(f"HOST_ERR,send_failed,{exc}")
            print(f"HOST_ERR send_failed: {exc}", file=sys.stderr)
            return
        if log_sent:
            self._log_event(f"sent: {cmd}")
        if echo:
            print(f"sent: {cmd}")

    def _rx_worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw:
                continue
            try:
                s = raw.decode("utf-8", errors="replace")
            except Exception:
                s = str(raw)

            # Raw device output only.
            self._raw_f.write(s)
            self._raw_f.flush()

            # Queue for main-thread processing/printing.
            self.rx_q.put(s)

    def _handle_device_line(self, line: str) -> None:
        line = line.strip("\r\n")
        if not line:
            return

        # Lightweight protocol detection (helps when the wrong sketch is flashed).
        if self.protocol == "unknown":
            if line.startswith("BALL_BEAM_BOOT") or line.startswith("TEL,") or line.startswith("STATE,") or line.startswith("SENSORS,"):
                self.protocol = "ballbeam"
            elif line.startswith("AS5600_CHECK_BOOT") or line.startswith("AS5600_PROBE") or line.startswith("AS5600_CAP") or line.startswith("AS5600_STREAM"):
                self.protocol = "as5600_check"
            else:
                if len(self._first_lines) < 5 and not line.startswith("TEL,"):
                    self._first_lines.append(line)

        tel = _parse_tel(line)
        if tel is not None:
            self._tel_csv.writerow(
                [
                    tel["t_ms"],
                    tel["state"],
                    tel["x_cm"],
                    tel["x_filt_cm"],
                    tel["theta_deg"],
                    tel["theta_cmd_deg"],
                    tel["u_step_rate"],
                    tel["fault_flags"],
                    tel["x_ref_cm"],
                    tel["sonar_age_ms"],
                    tel["sonar_valid"],
                    tel["sonar_miss_count"],
                    tel["theta_cmd_unclamped_deg"],
                    tel["theta_cmd_clamped_deg"],
                    tel["theta_cmd_saturated"],
                    tel["act_deg_abs"],
                    tel["trim_deg"],
                    tel["search_phase"],
                    self._active_phase,
                ]
            )
            self._tel_f.flush()

            self.snap.state = str(tel["state"])
            self.snap.x_filt_cm = float(tel["x_filt_cm"])  # type: ignore[arg-type]
            self.snap.theta_deg = float(tel["theta_deg"])  # type: ignore[arg-type]
            self.snap.x_ref_cm = float(tel["x_ref_cm"])  # type: ignore[arg-type]
            self.snap.fault_bits = int(tel["fault_flags"])  # type: ignore[arg-type]
            self.snap.angle_src = 1
            self.snap.last_tel_rx_time = time.time()
            self._last_tel_t_ms = int(tel["t_ms"])  # type: ignore[arg-type]

            if self.print_tel:
                print(line)
            return

        # Non-TEL lines are "events": print + log with host timestamp.
        if not any(line.startswith(p) for p in self._suppress_print_prefixes):
            print(line)
        self._log_event(line)

        # Auto-track firmware telemetry toggle to suppress/resume SNAP.
        if line == "OK,telemetry=0":
            self._suppress_snapshots = True
        elif line == "OK,telemetry=1":
            self._suppress_snapshots = False

        if line.startswith("STATE,"):
            self.snap.state = line.split(",", 1)[1].strip()

        if line.startswith("OK,angle_src="):
            try:
                self.snap.angle_src = int(line.split("=", 1)[1].strip())
            except ValueError:
                pass

        sensors = _parse_keyvals("SENSORS,", line)
        if sensors:
            a = sensors.get("angle")
            p = sensors.get("pos")
            if a in ("ok", "bad"):
                self.snap.angle_ok = (a == "ok")
            if p in ("ok", "bad"):
                self.snap.sonar_ok = (p == "ok")

        faults = _parse_keyvals("FAULTS,", line)
        if "bits" in faults:
            try:
                self.snap.fault_bits = int(float(faults["bits"]))
            except ValueError:
                pass

        meas = _parse_keyvals("MEAS,", line)
        if meas:
            try:
                if "theta_deg" in meas:
                    self.snap.theta_deg = float(meas["theta_deg"])
                if "x_filt_cm" in meas:
                    self.snap.x_filt_cm = float(meas["x_filt_cm"])
            except ValueError:
                pass

        sp = _parse_setpoint(line)
        if sp:
            try:
                if "x_ref_cm" in sp:
                    self.snap.x_ref_cm = float(sp["x_ref_cm"])
            except ValueError:
                pass
            try:
                if "t_ms" in sp:
                    self._last_tel_t_ms = int(sp["t_ms"])
            except ValueError:
                pass

    def _print_snapshot(self) -> None:
        proto = self.protocol
        state = self.snap.state or "?"
        if self.snap.angle_src is None:
            angle_src = "?"
        else:
            angle_src = "AS5600" if int(self.snap.angle_src) == 0 else "STEPPER"
        a = "?" if self.snap.angle_ok is None else ("OK" if self.snap.angle_ok else "BAD")
        s = "?" if self.snap.sonar_ok is None else ("OK" if self.snap.sonar_ok else "BAD")
        theta = "?" if self.snap.theta_deg is None else f"{self.snap.theta_deg:.1f}deg"
        x = "?" if self.snap.x_filt_cm is None else f"{self.snap.x_filt_cm:.1f}cm"
        x_ref = "?" if self.snap.x_ref_cm is None else f"{self.snap.x_ref_cm:.1f}cm"
        faults = _decode_faults(self.snap.fault_bits)
        if self.snap.last_tel_rx_time is None:
            last_tel = "never"
        else:
            last_tel = f"{(time.time() - self.snap.last_tel_rx_time):.1f}s"

        print(
            f"SNAP proto={proto}  angle_src={angle_src}  state={state}  sensors(angle={a} sonar={s})  theta={theta}  x_filt={x}  x_ref={x_ref}  faults={faults}  lastTEL={last_tel}"
        )

    def _print_help(self) -> None:
        print(
            "\nLocal commands:\n"
            "  /help                 Show this help\n"
            "  /quit                 Quit (sends: k then e 0)\n"
            "  /print_tel 0|1        Toggle printing raw TEL rows to terminal\n"
            "  /snap 0|1             Toggle periodic SNAP status line\n"
            "  /diag                 Send: s\n"
            "  /bringup              Guided calibration flow (recommended)\n"
            "  /std center_reg       Standard center-regulation run\n"
            "  /std step3            Standard 3-position step-tracking run\n"
            "  /std disturb          Standard disturbance-rejection run\n"
            "  /as5600_stats [N]     Not available in compact firmware\n"
            "  /sonar_stats [N]      Not available in compact firmware\n"
            "\nDevice commands: type anything else and press Enter (e.g. s, d, l, u, p, q c, q n, q -2.0, b, v, r, k)\n"
        )

    def _drain_rx_nonblock(self, max_lines: int = 500) -> list[str]:
        lines: list[str] = []
        for _ in range(max_lines):
            try:
                s = self.rx_q.get_nowait()
            except queue.Empty:
                break
            lines.append(s)
            self._handle_device_line(s)
        return lines

    def _wait_for_line(self, timeout_s: float, *, prefixes: tuple[str, ...] = (), contains: tuple[str, ...] = ()) -> Optional[str]:
        deadline = time.time() + float(timeout_s)
        while time.time() < deadline and not self.stop_event.is_set():
            lines = self._drain_rx_nonblock()
            for s in lines:
                line = s.strip("\r\n")
                if not line:
                    continue
                if prefixes and any(line.startswith(p) for p in prefixes):
                    return line
                if contains and any(c in line for c in contains):
                    return line
            time.sleep(0.01)
        return None

    def _sleep_with_rx(self, duration_s: float) -> None:
        deadline = time.time() + max(0.0, float(duration_s))
        while time.time() < deadline and not self.stop_event.is_set():
            self._drain_rx_nonblock()
            time.sleep(0.02)

    def _current_t_ms(self) -> int:
        return int(self._last_tel_t_ms or 0)

    def _log_host_marker(self, kind: str, **fields: object) -> None:
        parts = [f"HOST_STD,{kind}", f"t_ms={self._current_t_ms()}"]
        for key, value in fields.items():
            parts.append(f"{key}={value}")
        self._log_event(",".join(parts))

    def _set_phase(self, phase: str) -> None:
        self._active_phase = phase
        self._log_host_marker("phase", name=phase)

    def _query_setpoint(self) -> Optional[dict[str, object]]:
        self._send("q", echo=False)
        line = self._wait_for_line(1.0, prefixes=("SET,", "ERR,"))
        if line is None or line.startswith("ERR,"):
            return None
        return _parse_setpoint(line)

    def _setpoint_command(self, arg: str) -> Optional[dict[str, object]]:
        self._send(f"q {arg}", echo=False)
        line = self._wait_for_line(1.0, prefixes=("SET,", "ERR,"))
        if line is None or line.startswith("ERR,"):
            if line:
                print(line)
            return None
        return _parse_setpoint(line)

    def _start_standard_run(self, run_name: str) -> bool:
        self._active_phase = ""
        self._send("k", echo=False)
        self._wait_for_line(0.6, prefixes=("OK,stopped",))
        self._send("e 0", echo=False)
        self._wait_for_line(0.6, prefixes=("OK,driver_disabled",))
        self._send("t 1", echo=False)
        self._wait_for_line(0.6, prefixes=("OK,telemetry=1",))
        self._send("e 1", echo=False)
        self._wait_for_line(0.6, prefixes=("OK,driver_enabled", "OK,driver_disabled"))
        self._send("r", echo=False)
        line = self._wait_for_line(1.2, prefixes=("OK,running", "ERR,"))
        if line is None or line.startswith("ERR,"):
            print(line or "ERR,run_start_timeout")
            return False
        self._wait_for_line(1.2, prefixes=("TEL,",))
        self._log_host_marker("start", run=run_name)
        return True

    def _stop_standard_run(self, run_name: str) -> None:
        self._active_phase = ""
        self._send("k", echo=False)
        self._wait_for_line(1.0, prefixes=("OK,stopped",))
        self._send("e 0", echo=False)
        self._wait_for_line(1.0, prefixes=("OK,driver_disabled",))
        self._log_host_marker("stop", run=run_name)

    def _cmd_std_center_reg(self) -> None:
        sp = self._query_setpoint()
        if not sp or "near_mid_cm" not in sp or "far_mid_cm" not in sp:
            print("FAIL: could not read setpoint presets. Capture limits first.")
            return
        near_mid = float(sp["near_mid_cm"])
        far_mid = float(sp["far_mid_cm"])
        if not math.isfinite(near_mid) or not math.isfinite(far_mid):
            print("FAIL: setpoint presets are unavailable. Capture limits first.")
            return
        self._log_event(
            f"HOST_STD,manifest,run=center_reg,near_target_cm={near_mid:.4f},far_target_cm={far_mid:.4f},duration_s=12"
        )
        resp = self._prompt_user(
            "\nCENTER REGULATION\nPlace the ball near either off-center preset, then press Enter to start and release."
        )
        if resp is None or resp.strip().lower() == "q":
            print("Standard run aborted.")
            return
        if self._setpoint_command("c") is None:
            print("FAIL: could not set center reference.")
            return
        if not self._start_standard_run("center_reg"):
            return
        self._set_phase("recover_to_center")
        self._sleep_with_rx(12.0)
        self._stop_standard_run("center_reg")
        print("Standard run complete: center_reg")

    def _cmd_std_step3(self) -> None:
        sp = self._query_setpoint()
        if not sp or "near_mid_cm" not in sp or "far_mid_cm" not in sp:
            print("FAIL: could not read setpoint presets. Capture limits first.")
            return
        near_mid = float(sp["near_mid_cm"])
        far_mid = float(sp["far_mid_cm"])
        if not math.isfinite(near_mid) or not math.isfinite(far_mid):
            print("FAIL: setpoint presets are unavailable. Capture limits first.")
            return
        self._log_event(
            "HOST_STD,manifest,run=step3,"
            f"near_target_cm={near_mid:.4f},far_target_cm={far_mid:.4f},holds_s=4|8|8|8|8"
        )
        print("\nSTEP TRACKING 3POS\nRunning: center -> near endpoint -> center -> far endpoint -> center")
        if self._setpoint_command("c") is None:
            print("FAIL: could not set center reference.")
            return
        if not self._start_standard_run("step3"):
            return

        phases = [
            ("hold_center_1", "c", 4.0),
            ("hold_near", "n", 8.0),
            ("hold_center_2", "c", 8.0),
            ("hold_far", "f", 8.0),
            ("hold_center_3", "c", 8.0),
        ]
        for phase, cmd_arg, duration_s in phases:
            if self._setpoint_command(cmd_arg) is None:
                print(f"FAIL: setpoint change failed in phase {phase}.")
                break
            self._set_phase(phase)
            self._sleep_with_rx(duration_s)
        self._stop_standard_run("step3")
        print("Standard run complete: step3")

    def _cmd_std_disturb(self) -> None:
        sp = self._query_setpoint()
        if not sp or "near_mid_cm" not in sp or "far_mid_cm" not in sp:
            print("FAIL: could not read setpoint presets. Capture limits first.")
            return
        near_mid = float(sp["near_mid_cm"])
        far_mid = float(sp["far_mid_cm"])
        if not math.isfinite(near_mid) or not math.isfinite(far_mid):
            print("FAIL: setpoint presets are unavailable. Capture limits first.")
            return
        self._log_event(
            "HOST_STD,manifest,run=disturb,"
            f"near_target_cm={near_mid:.4f},far_target_cm={far_mid:.4f},schedule_s=5|10|18,notes=manual_nudge"
        )
        resp = self._prompt_user(
            "\nDISTURBANCE REJECTION\nBe ready to nudge the ball when prompted. Press Enter to start."
        )
        if resp is None or resp.strip().lower() == "q":
            print("Standard run aborted.")
            return
        if self._setpoint_command("c") is None:
            print("FAIL: could not set center reference.")
            return
        if not self._start_standard_run("disturb"):
            return

        self._set_phase("hold_center_pre")
        self._sleep_with_rx(5.0)

        self._log_host_marker("mark", label="disturb_near_prompt")
        print("NUDGE NOW: disturb the ball toward the near-sensor side.")
        self._set_phase("recover_after_disturb_near")
        self._sleep_with_rx(5.0)

        self._log_host_marker("mark", label="disturb_far_prompt")
        print("NUDGE NOW: disturb the ball toward the far-sensor side.")
        self._set_phase("recover_after_disturb_far")
        self._sleep_with_rx(8.0)

        self._stop_standard_run("disturb")
        print("Standard run complete: disturb")

    def _prompt_user(self, prompt: str) -> Optional[str]:
        print(prompt)
        while not self.stop_event.is_set():
            self._drain_rx_nonblock()
            r, _, _ = select.select([sys.stdin], [], [], 0.10)
            if not r:
                continue
            line = sys.stdin.readline()
            if line == "":
                return None
            return line.rstrip("\r\n")
        return None

    def _cmd_as5600_stats(self, n: int) -> None:
        print("\nAS5600 stats are not available in the compact firmware.\n")

    def _cmd_sonar_stats(self, n: int) -> None:
        print("\nSONAR stats are not available in the compact firmware.\n")

    def _cmd_bringup(self) -> None:
        print("\nGUIDED BRING-UP (/bringup)\nFollow the prompts. Type 'q' + Enter at any prompt to abort.\n")

        prev_suppress = self._suppress_snapshots
        self._suppress_snapshots = True

        try:
            print("Safety stop: sending k, then e 0 ...")
            self._send("k")
            time.sleep(0.05)
            self._send("e 0")
            print("Resetting calibration (sending d)...")
            self._send("d")
            self._wait_for_line(1.0, prefixes=("OK,cal_reset_saved",))

            resp = self._prompt_user("Step 1/6: Move beam to physical DOWN hard stop. Hold it still. Press Enter when ready.")
            if resp is None or resp.strip().lower() == "q":
                print("Bring-up aborted.")
                return

            for attempt in range(1, 4):
                print("Capturing DOWN limit (sending l)...")
                self._send("l")
                down_line = self._wait_for_line(2.0, prefixes=("OK,cal_limit_down_set=", "ERR,angle_not_ready", "ERR,sonar_not_ready"))
                if not down_line or down_line.startswith("ERR,"):
                    print("FAIL: down limit capture failed.")
                    continue

                resp = self._prompt_user(
                    f"Step 2/6 (attempt {attempt}/3): Move beam to physical UP hard stop. Hold it still. Press Enter when ready."
                )
                if resp is None or resp.strip().lower() == "q":
                    print("Bring-up aborted.")
                    return
                print("Capturing UP limit (sending u)...")
                self._send("u")
                up_line = self._wait_for_line(2.0, prefixes=("OK,cal_limit_up_set=", "ERR,limit_span_too_small", "ERR,angle_not_ready", "ERR,sonar_not_ready"))
                if not up_line or up_line.startswith("ERR,"):
                    print("FAIL: up limit capture failed.")
                    continue

                mline = self._wait_for_line(1.5, prefixes=("CAL_LIMITS,",))
                mkv = _parse_keyvals("CAL_LIMITS,", mline or "")
                try:
                    lower = float(mkv.get("lower_deg", "nan"))
                    upper = float(mkv.get("upper_deg", "nan"))
                    ok_limits = mkv.get("limits_calibrated", "no") == "yes" and upper > lower and (upper - lower) >= 2.0
                except ValueError:
                    ok_limits = False
                    lower = float("nan")
                    upper = float("nan")

                print(f"  limits: lower_deg={lower:.2f} upper_deg={upper:.2f} calibrated={mkv.get('limits_calibrated','?')}")
                if ok_limits:
                    break
                print("FAIL: limits not valid. Re-do DOWN then UP with a wider span.")
            else:
                print("FAIL: could not capture valid limits after 3 attempts. No save performed.")
                return

            resp = self._prompt_user("Step 3/6: Place the ball at the physical center of the runner. Hold it steady and press Enter.")
            if resp is None or resp.strip().lower() == "q":
                print("Bring-up aborted.")
                return

            print("Capturing center reference and control origin (sending p)...")
            self._send("p")
            pline = self._wait_for_line(2.0, prefixes=("OK,cal_zero_position_set=", "ERR,sonar_not_ready", "ERR,center_outside_limits"))
            if not pline or pline.startswith("ERR,"):
                print("FAIL: center capture failed.")
                return

            print("\nStep 4/6: Enabling driver (sending e 1)...")
            self._send("e 1")
            eline = self._wait_for_line(1.5, prefixes=("OK,driver_enabled", "OK,driver_disabled"))
            if not eline or "driver_enabled" not in eline:
                print("FAIL: could not enable driver.")
                return

            print("\nStep 5/6: Checking jog sign (sending b)...")
            self._send("b")
            bline = self._wait_for_line(4.0, prefixes=("OK,stepper_sign=", "ERR,"))
            if not bline or bline.startswith("ERR,"):
                print("FAIL: stepper sign jog failed.")
                return
            print(f"  {bline}")

            print("\nStep 6/6: Saving calibration (sending v)...")
            self._send("v")
            vline = self._wait_for_line(2.0, prefixes=("OK,cal_saved", "ERR,cal_save_failed"))
            if not vline or vline.startswith("ERR,"):
                print("FAIL: save failed.")
                return

            print(
                "\nBRING-UP COMPLETE\n"
                "Next steps:\n"
                "  1) First closed-loop run after calibration:\n"
                "     s\n"
                "     q c\n"
                "     e 1\n"
                "     r\n"
                "  2) Change target live if needed:\n"
                "     q n\n"
                "     q f\n"
                "     q c\n"
                "  3) Stop:\n"
                "     k\n"
                "     e 0\n"
            )
        finally:
            self._suppress_snapshots = prev_suppress

    def run(self) -> None:
        self._rx_thread.start()

        print("Serial logger connected.")
        print(f"  RAW     : {self.raw_path}")
        print(f"  EVENTS  : {self.events_path}")
        print(f"  TELEMETRY CSV: {self.telemetry_path}")
        print("Type /help for local commands. Type device commands and press Enter.")

        # Force a status snapshot into logs without changing telemetry state.
        self._send("s")

        next_snap: Optional[float]
        if self.snapshot_hz > 0.0:
            next_snap = time.time() + (1.0 / self.snapshot_hz)
        else:
            next_snap = None

        try:
            while not self.stop_event.is_set():
                # Drain queued RX lines.
                self._drain_rx_nonblock()

                now = time.time()
                if (
                    not self._protocol_warned
                    and self.protocol == "unknown"
                    and (now - self._start_time) > 2.0
                    and self._first_lines
                ):
                    hint = " | ".join(self._first_lines[:3])
                    msg = (
                        "HOST_WARN,unknown_firmware_protocol: "
                        "device output does not look like BallBeam (BALL_BEAM_BOOT/TEL/STATE) "
                        "or AS5600_CHECK. You may have flashed a different Arduino sketch. "
                        f"First lines: {hint}"
                    )
                    print(msg)
                    self._log_event(msg)
                    self._protocol_warned = True
                if next_snap is not None and (not self._suppress_snapshots) and now >= next_snap:
                    self._print_snapshot()
                    next_snap = now + (1.0 / self.snapshot_hz)

                if self.no_input:
                    time.sleep(0.05)
                    continue

                r, _, _ = select.select([sys.stdin], [], [], 0.10)
                if not r:
                    continue

                user_line = sys.stdin.readline()
                if user_line == "":
                    # stdin closed
                    break
                user_line = user_line.rstrip("\r\n")
                if not user_line:
                    continue

                if user_line.startswith("/"):
                    parts = user_line.split()
                    cmd = parts[0].lower()
                    if cmd == "/help":
                        self._print_help()
                    elif cmd == "/quit":
                        self._send("k")
                        time.sleep(0.05)
                        self._send("e 0")
                        break
                    elif cmd == "/print_tel":
                        if len(parts) != 2 or parts[1] not in ("0", "1"):
                            print("usage: /print_tel 0|1")
                        else:
                            self.print_tel = (parts[1] == "1")
                            print(f"OK print_tel={1 if self.print_tel else 0}")
                    elif cmd == "/snap":
                        if len(parts) != 2 or parts[1] not in ("0", "1"):
                            print("usage: /snap 0|1")
                        else:
                            self._suppress_snapshots = (parts[1] == "0")
                            print(f"OK snap={0 if self._suppress_snapshots else 1}")
                    elif cmd == "/diag":
                        self._send("s")
                    elif cmd == "/bringup":
                        self._cmd_bringup()
                    elif cmd == "/std":
                        if len(parts) != 2:
                            print("usage: /std center_reg|step3|disturb")
                            continue
                        std_name = parts[1].lower()
                        if std_name == "center_reg":
                            self._cmd_std_center_reg()
                        elif std_name == "step3":
                            self._cmd_std_step3()
                        elif std_name == "disturb":
                            self._cmd_std_disturb()
                        else:
                            print("usage: /std center_reg|step3|disturb")
                    elif cmd == "/as5600_stats":
                        n = 50
                        if len(parts) == 2:
                            try:
                                n = int(parts[1])
                            except ValueError:
                                print("usage: /as5600_stats [N]")
                                continue
                        self._cmd_as5600_stats(n)
                    elif cmd == "/sonar_stats":
                        n = 30
                        if len(parts) == 2:
                            try:
                                n = int(parts[1])
                            except ValueError:
                                print("usage: /sonar_stats [N]")
                                continue
                        self._cmd_sonar_stats(n)
                    else:
                        print(f"Unknown local command: {parts[0]} (try /help)")
                    continue

                self._send(user_line)

        except KeyboardInterrupt:
            print("\nCTRL-C: quitting (sending k, then e 0)")
            try:
                self._send("k")
                time.sleep(0.05)
                self._send("e 0")
            except Exception:
                pass
        finally:
            self.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple serial logger (raw.log + telemetry.csv + events.txt)")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "data" / "runs",
        help="Output directory (default: data/runs)",
    )
    parser.add_argument("--snapshot-hz", type=float, default=1.0, help="Snapshot print rate (Hz). Use 0 to disable.")
    parser.add_argument("--print-tel", type=int, default=0, help="Print TEL rows to terminal (0/1)")
    parser.add_argument("--no-input", type=int, default=0, help="Read-only mode (0/1)")
    args = parser.parse_args()

    port = args.port
    if port is None:
        port = _autodetect_port()

    args.outdir.mkdir(parents=True, exist_ok=True)
    stamp = _now_stamp()
    raw_path = args.outdir / f"run_{stamp}_raw.log"
    events_path = args.outdir / f"run_{stamp}_events.txt"
    telemetry_path = args.outdir / f"run_{stamp}_telemetry.csv"

    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except Exception as exc:
        raise SystemExit(f"Failed to open {port}: {exc}")

    with ser:
        app = SerialLogger(
            ser=ser,
            raw_path=raw_path,
            events_path=events_path,
            telemetry_path=telemetry_path,
            snapshot_hz=args.snapshot_hz,
            print_tel=(args.print_tel != 0),
            no_input=(args.no_input != 0),
        )
        app.run()


if __name__ == "__main__":
    main()
