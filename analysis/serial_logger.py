#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import queue
import select
import sys
import threading
import time
from collections import deque
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
    if not line.startswith("TEL,"):
        return None
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 9:
        return None
    try:
        return {
            "t_ms": int(float(parts[1])),
            "state": parts[2],
            "x_cm": float(parts[3]),
            "x_filt_cm": float(parts[4]),
            "theta_deg": float(parts[5]),
            "theta_cmd_deg": float(parts[6]),
            "u_step_rate": float(parts[7]),
            "fault_flags": int(float(parts[8])),
        }
    except ValueError:
        return None


def _parse_sonar_diag(line: str) -> Optional[dict[str, object]]:
    # SONAR_DIAG,has_sample=1,fresh=1,timeout=0,age_ms=9,raw_cm=...,filt_cm=...,valid_streak=...,timeout_cnt=...,jump_reject_cnt=...
    if not line.startswith("SONAR_DIAG,"):
        return None
    kv = _parse_keyvals("SONAR_DIAG,", line)
    if not kv:
        return None
    try:
        return {
            "has_sample": int(float(kv.get("has_sample", "0"))) != 0,
            "fresh": int(float(kv.get("fresh", "0"))) != 0,
            "timeout": int(float(kv.get("timeout", "0"))) != 0,
            "age_ms": int(float(kv.get("age_ms", "0"))),
            "raw_cm": float(kv.get("raw_cm", "nan")),
            "filt_cm": float(kv.get("filt_cm", "nan")),
            "timeout_cnt": int(float(kv.get("timeout_cnt", "0"))),
            "jump_reject_cnt": int(float(kv.get("jump_reject_cnt", "0"))),
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


def _mean_std_min_max(values: list[float]) -> tuple[float, float, float, float]:
    if not values:
        return (float("nan"), float("nan"), float("nan"), float("nan"))
    mn = sum(values) / float(len(values))
    if len(values) < 2:
        sd = 0.0
    else:
        var = sum((v - mn) ** 2 for v in values) / float(len(values) - 1)
        sd = var**0.5
    return (mn, sd, min(values), max(values))


@dataclass
class Snapshot:
    state: Optional[str] = None
    angle_src: Optional[int] = None  # 0=AS5600, 1=STEPPER
    angle_ok: Optional[bool] = None
    sonar_ok: Optional[bool] = None
    theta_deg: Optional[float] = None
    x_filt_cm: Optional[float] = None
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
                ]
            )
            self._tel_f.flush()

            self.snap.state = str(tel["state"])
            self.snap.x_filt_cm = float(tel["x_filt_cm"])  # type: ignore[arg-type]
            self.snap.theta_deg = float(tel["theta_deg"])  # type: ignore[arg-type]
            self.snap.fault_bits = int(tel["fault_flags"])  # type: ignore[arg-type]
            self.snap.last_tel_rx_time = time.time()

            if self.print_tel:
                print(line)
            return

        # Non-TEL lines are "events": print + log with host timestamp.
        if not any(line.startswith(p) for p in self._suppress_print_prefixes):
            print(line)
        self._log_event(line)

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
        faults = _decode_faults(self.snap.fault_bits)
        if self.snap.last_tel_rx_time is None:
            last_tel = "never"
        else:
            last_tel = f"{(time.time() - self.snap.last_tel_rx_time):.1f}s"

        print(
            f"SNAP proto={proto}  angle_src={angle_src}  state={state}  sensors(angle={a} sonar={s})  theta={theta}  x_filt={x}  faults={faults}  lastTEL={last_tel}"
        )

    def _print_help(self) -> None:
        print(
            "\nLocal commands:\n"
            "  /help                 Show this help\n"
            "  /quit                 Quit (sends: k then e 0)\n"
            "  /print_tel 0|1        Toggle printing raw TEL rows to terminal\n"
            "  /diag                 Send: s, as5600 diag, sonar diag, x\n"
            "  /bringup              Guided calibration flow (recommended)\n"
            "  /as5600_stats [N]     Sample AS5600 diag N times (default 50) and summarize noise\n"
            "  /sonar_stats [N]      Sample sonar diag N times (default 30) and summarize stability\n"
            "\nDevice commands: type anything else and press Enter (e.g. s, y, r, k, sonar diag, as5600 diag)\n"
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
        n = max(1, min(int(n), 500))
        print(f"\nAS5600 stats: sampling {n}x (hold beam still)...")

        # Keep terminal readable: suppress the individual AS5600_DIAG lines while capturing.
        prev = set(self._suppress_print_prefixes)
        self._suppress_print_prefixes.add("AS5600_DIAG,")
        try:
            rows: list[dict[str, object]] = []
            for i in range(n):
                self._send("as5600 diag", echo=False, log_sent=False)
                line = self._wait_for_line(0.6, prefixes=("AS5600_DIAG,",))
                if line is None:
                    continue
                parsed = _parse_as5600_diag(line)
                if parsed is None:
                    continue
                rows.append(parsed)
                if (i + 1) % 10 == 0 or (i + 1) == n:
                    print(f"  {i + 1}/{n}...")
                time.sleep(0.10)
        finally:
            self._suppress_print_prefixes = prev

        ok_rows = [r for r in rows if bool(r.get("ok"))]
        raw_vals = [float(r["raw_deg"]) for r in ok_rows if "raw_deg" in r]
        theta_vals = [float(r["theta_deg"]) for r in ok_rows if "theta_deg" in r]
        err_vals = [int(r["err_cnt"]) for r in rows if "err_cnt" in r]

        raw_mean, raw_std, raw_min, raw_max = _mean_std_min_max(raw_vals)
        th_mean, th_std, th_min, th_max = _mean_std_min_max(theta_vals)

        if err_vals:
            err_delta = err_vals[-1] - err_vals[0]
            err_last = err_vals[-1]
        else:
            err_delta = 0
            err_last = -1

        wrap_suspect = False
        if raw_vals and (raw_min < 20.0 and raw_max > 340.0):
            wrap_suspect = True

        print("\nAS5600_STATS_RESULT")
        print(f"  samples_ok = {len(ok_rows)}/{max(1, len(rows))} (parsed {len(rows)} lines)")
        print(f"  err_cnt    = {err_last} (delta {err_delta})")
        print(f"  raw_deg    = mean {raw_mean:.2f}  std {raw_std:.2f}  min {raw_min:.2f}  max {raw_max:.2f}")
        print(f"  theta_deg  = mean {th_mean:.2f}  std {th_std:.2f}  min {th_min:.2f}  max {th_max:.2f}")
        if wrap_suspect:
            print("  hint       = wrap suspected (raw_deg spans near 0/360). This is OK if handled via wrap math.")
        print("")

    def _cmd_sonar_stats(self, n: int) -> None:
        n = max(1, min(int(n), 300))
        print(f"\nSONAR stats: sampling {n}x (hold target still if testing stability)...")

        prev = set(self._suppress_print_prefixes)
        self._suppress_print_prefixes.add("SONAR_DIAG,")
        try:
            diags: list[dict[str, object]] = []
            for i in range(n):
                self._send("sonar diag", echo=False, log_sent=False)
                line = self._wait_for_line(0.6, prefixes=("SONAR_DIAG,",))
                if line is None:
                    continue
                parsed = _parse_sonar_diag(line)
                if parsed is None:
                    continue
                diags.append(parsed)
                if (i + 1) % 10 == 0 or (i + 1) == n:
                    print(f"  {i + 1}/{n}...")
                time.sleep(0.12)
        finally:
            self._suppress_print_prefixes = prev

        good = [d for d in diags if d.get("has_sample") and d.get("fresh") and not d.get("timeout")]
        fresh = [d for d in diags if d.get("fresh")]
        timeouts = [d for d in diags if d.get("timeout")]

        filt_vals = [float(d["filt_cm"]) for d in good if isinstance(d.get("filt_cm"), float)]
        mean_f, std_f, min_f, max_f = _mean_std_min_max(filt_vals)

        print("\nSONAR_STATS_RESULT")
        print(f"  samples_parsed = {len(diags)}/{n}")
        print(f"  fresh          = {len(fresh)}/{len(diags) if diags else 1}")
        print(f"  timeout        = {len(timeouts)}/{len(diags) if diags else 1}")
        print(f"  good(fresh&no-timeout) = {len(good)}/{len(diags) if diags else 1}")
        if filt_vals:
            print(f"  filt_cm (good) = mean {mean_f:.2f}  std {std_f:.2f}  min {min_f:.2f}  max {max_f:.2f}")
        print("")

    def _cmd_bringup(self) -> None:
        print(
            "\nGUIDED BRING-UP (/bringup)\n"
            "Follow the prompts. Do NOT type device commands unless asked.\n"
            "Type 'q' + Enter at any prompt to abort.\n"
        )

        prev_suppress = self._suppress_snapshots
        self._suppress_snapshots = True

        try:
            print("Safety stop: sending k, then e 0 ...")
            self._send("k")
            time.sleep(0.05)
            self._send("e 0")

            resp = self._prompt_user("\nStep 1/6: Place a FLAT board in front of the sonar and hold it steady. Press Enter when ready.")
            if resp is None or resp.strip().lower() == "q":
                print("Bring-up aborted.")
                return

            print("Checking sonar stability (need >=3 good out of last 5)...")
            prev = set(self._suppress_print_prefixes)
            self._suppress_print_prefixes.add("SONAR_DIAG,")
            try:
                window: "deque[bool]" = deque(maxlen=5)
                start = time.time()
                ok = False
                while (time.time() - start) < 10.0:
                    self._send("sonar diag", echo=False, log_sent=False)
                    line = self._wait_for_line(0.8, prefixes=("SONAR_DIAG,",))
                    diag = _parse_sonar_diag(line) if line else None
                    good = bool(diag and diag.get("has_sample") and diag.get("fresh") and not diag.get("timeout"))
                    window.append(good)
                    if diag:
                        print(
                            f"  sonar: fresh={1 if diag.get('fresh') else 0} timeout={1 if diag.get('timeout') else 0} "
                            f"age_ms={diag.get('age_ms')} filt_cm={float(diag.get('filt_cm', 0.0)):.2f}"
                        )
                    else:
                        print("  sonar: no diag response")
                    if len(window) == 5 and sum(1 for v in window if v) >= 3:
                        ok = True
                        break
                    time.sleep(0.20)
            finally:
                self._suppress_print_prefixes = prev

            if not ok:
                print("FAIL: sonar not stable. Stop and fix target/wiring, then re-run /bringup.")
                return
            print("OK: sonar stable.\n")

            resp = self._prompt_user("Step 2/6: Move the flat board to BEAM CENTER under the sonar. Press Enter when ready.")
            if resp is None or resp.strip().lower() == "q":
                print("Bring-up aborted.")
                return

            print("Capturing sonar center (sending p)...")
            self._send("p")
            line = self._wait_for_line(3.0, prefixes=("OK,cal_zero_position_set=", "ERR,sonar_not_ready"))
            if not line or line.startswith("ERR,"):
                print("FAIL: sonar center capture failed. Check sonar diag and target, then retry /bringup.")
                return
            print("OK: sonar center captured.")
            self._send("z")
            zline = self._wait_for_line(1.5, prefixes=("CAL_ZERO,",))
            zkv = _parse_keyvals("CAL_ZERO,", zline or "")
            if zkv:
                print(f"  sonar_center_cm = {zkv.get('sonar_center_cm','?')}")

            resp = self._prompt_user("\nStep 3/6: Level the beam (horizontal). Hold it still. Press Enter when ready.")
            if resp is None or resp.strip().lower() == "q":
                print("Bring-up aborted.")
                return

            print("Capturing angle zero (sending a)...")
            self._send("a")
            line = self._wait_for_line(3.0, prefixes=("OK,cal_zero_angle_set=", "ERR,angle_not_ready"))
            if not line or line.startswith("ERR,"):
                print("FAIL: angle zero capture failed. Run 'as5600 diag' and check wiring.")
                print("TIP: To bypass AS5600, use stepper-count angle mode: type y, then use p + run (sonar target required).")
                return
            print("OK: angle zero captured.")
            self._send("z")
            zline = self._wait_for_line(1.5, prefixes=("CAL_ZERO,",))
            zkv = _parse_keyvals("CAL_ZERO,", zline or "")
            if zkv:
                print(f"  as5600_zero_deg = {zkv.get('as5600_zero_deg','?')}")

            # Capture limits with retry.
            for attempt in range(1, 4):
                resp = self._prompt_user(
                    f"\nStep 4/6 (attempt {attempt}/3): Move beam to physical DOWN hard stop. Hold it still. Press Enter when ready."
                )
                if resp is None or resp.strip().lower() == "q":
                    print("Bring-up aborted.")
                    return
                print("Capturing DOWN limit (sending l)...")
                self._send("l")
                down_line = self._wait_for_line(2.0, prefixes=("OK,cal_limit_down_set=", "ERR,angle_not_ready"))
                if not down_line or down_line.startswith("ERR,"):
                    print("FAIL: down limit capture failed (angle not ready).")
                    print("TIP: To bypass AS5600, use stepper-count angle mode: type y, then use p + run (sonar target required).")
                    continue

                resp = self._prompt_user(
                    "Step 5/6: Move beam to physical UP hard stop. Hold it still. Press Enter when ready."
                )
                if resp is None or resp.strip().lower() == "q":
                    print("Bring-up aborted.")
                    return
                print("Capturing UP limit (sending u)...")
                self._send("u")
                up_line = self._wait_for_line(2.0, prefixes=("OK,cal_limit_up_set=", "ERR,limit_span_too_small", "ERR,angle_not_ready"))
                if not up_line or up_line.startswith("ERR,"):
                    print("FAIL: up limit capture failed (span too small or angle not ready).")
                    print("TIP: To bypass AS5600, use stepper-count angle mode: type y, then use p + run (sonar target required).")
                    continue

                self._send("m")
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

            print("\nStep 6/6: Saving calibration (sending v)...")
            self._send("v")
            vline = self._wait_for_line(2.0, prefixes=("OK,cal_saved", "ERR,cal_save_failed"))
            if not vline or vline.startswith("ERR,"):
                print("FAIL: save failed.")
                return

            print(
                "\nBRING-UP COMPLETE\n"
                "Next steps:\n"
                "  0) If AS5600 is flaky: type y (stepper-angle mode). This zeros step position at the current angle.\n"
                "  1) Motor test (even if sonar later goes stale):\n"
                "     e 1\n"
                "     j 120 500\n"
                "     k\n"
                "  2) Closed-loop run (keep a detectable target present continuously):\n"
                "     e 1\n"
                "     r\n"
                "     k\n"
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
                    elif cmd == "/diag":
                        for c in ("s", "as5600 diag", "sonar diag", "x"):
                            self._send(c)
                            time.sleep(0.05)
                    elif cmd == "/bringup":
                        self._cmd_bringup()
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
