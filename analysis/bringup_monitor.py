#!/usr/bin/env python3
from __future__ import annotations

import argparse
import curses
import curses.ascii
import queue
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import IO, Optional

import serial
from serial.tools import list_ports


TEL_COLUMNS = [
    "t_ms",
    "state",
    "x_cm",
    "x_filt_cm",
    "theta_deg",
    "theta_cmd_deg",
    "u_step_rate",
    "fault_bits",
]


@dataclass
class TelemetrySample:
    t_ms: Optional[int] = None
    state: Optional[str] = None
    x_cm: Optional[float] = None
    x_filt_cm: Optional[float] = None
    theta_deg: Optional[float] = None
    theta_cmd_deg: Optional[float] = None
    u_step_rate: Optional[float] = None
    fault_bits: Optional[int] = None
    host_rx_time: Optional[float] = None  # time.time()


@dataclass
class DeviceState:
    app_state: Optional[str] = None
    angle_ok: Optional[bool] = None
    pos_ok: Optional[bool] = None
    age_angle_ms: Optional[int] = None
    age_pos_ms: Optional[int] = None

    zero_cal: Optional[bool] = None
    limits_cal: Optional[bool] = None
    sign_cal: Optional[bool] = None
    cal_store_status: Optional[str] = None  # loaded/defaults/uninitialized

    fault_bits: Optional[int] = None
    fault_info_lines: list[str] = field(default_factory=list)

    guide_step: Optional[str] = None
    guide_do: Optional[str] = None

    wizard_active: bool = False
    wizard_step_num: Optional[int] = None
    wizard_step_failed: bool = False

    tel_enabled: Optional[bool] = None
    driver_enabled: Optional[bool] = None

    telemetry: TelemetrySample = field(default_factory=TelemetrySample)

    # Host-side toggles
    view_mode: str = "events"  # events/telemetry
    show_raw: bool = False  # show raw protocol lines in Events
    log_mode: str = "auto"  # auto/off
    logging_active: bool = False
    log_path: Optional[Path] = None
    footer_msg: str = ""

    # Track transitions
    _prev_app_state: Optional[str] = None

    # Telemetry stream buffer for the "telemetry view"
    tel_stream: list[str] = field(default_factory=list)

    # Latest diag snapshots (optional)
    sonar_diag: dict[str, str] = field(default_factory=dict)
    as5600_diag: dict[str, str] = field(default_factory=dict)


@dataclass
class Event:
    ts: str
    kind: str
    text: str


def _now_ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def _safe_addstr(win: "curses._CursesWindow", y: int, x: int, s: str, attr: int = 0) -> None:
    try:
        max_y, max_x = win.getmaxyx()
        if y < 0 or y >= max_y or x >= max_x:
            return
        if x < 0:
            s = s[-x:]
            x = 0
        s = s[: max(0, max_x - x)]
        if not s:
            return
        win.addstr(y, x, s, attr)
    except curses.error:
        return


def _draw_box(
    win: "curses._CursesWindow", y: int, x: int, h: int, w: int, title: str = ""
) -> None:
    if h < 2 or w < 2:
        return

    # Top/bottom border.
    _safe_addstr(win, y, x, "+" + ("-" * (w - 2)) + "+")
    for row in range(y + 1, y + h - 1):
        _safe_addstr(win, row, x, "|")
        _safe_addstr(win, row, x + w - 1, "|")
    _safe_addstr(win, y + h - 1, x, "+" + ("-" * (w - 2)) + "+")

    if title and w >= 6:
        t = f" {title} "
        t = t[: max(0, w - 4)]
        _safe_addstr(win, y, x + 2, t)


def _init_colors() -> dict[str, int]:
    # Map kind -> curses attribute
    colors: dict[str, int] = {}
    if not curses.has_colors():
        return colors

    curses.start_color()
    curses.use_default_colors()

    # Pair ids
    _P_OK = 1
    _P_ERR = 2
    _P_WARN = 3
    _P_GUIDE = 4
    _P_WIZ = 5
    _P_INFO = 6
    _P_HOST = 7

    curses.init_pair(_P_OK, curses.COLOR_GREEN, -1)
    curses.init_pair(_P_ERR, curses.COLOR_RED, -1)
    curses.init_pair(_P_WARN, curses.COLOR_YELLOW, -1)
    curses.init_pair(_P_GUIDE, curses.COLOR_CYAN, -1)
    curses.init_pair(_P_WIZ, curses.COLOR_MAGENTA, -1)
    curses.init_pair(_P_INFO, curses.COLOR_BLUE, -1)
    curses.init_pair(_P_HOST, curses.COLOR_WHITE, -1)

    colors.update(
        {
            "OK": curses.color_pair(_P_OK),
            "ERR": curses.color_pair(_P_ERR),
            "WARN": curses.color_pair(_P_WARN),
            "GUIDE": curses.color_pair(_P_GUIDE),
            "WIZ": curses.color_pair(_P_WIZ),
            "INFO": curses.color_pair(_P_INFO),
            "HOST": curses.color_pair(_P_HOST),
        }
    )
    return colors


def _event_kind(line: str) -> str:
    if not line:
        return "HOST"
    if line.startswith("TEL,"):
        return "TEL"
    head = line.split(",", 1)[0].strip().upper()
    if head in ("OK", "ERR", "WARN", "INFO", "GUIDE", "WIZ"):
        return head
    return "INFO" if head.startswith("FAULT") else "HOST"


def _parse_kv_csv(line: str) -> dict[str, str]:
    # Parse "KEY,a=b,c=d" -> {"a":"b","c":"d"}
    parts = [p.strip() for p in line.split(",")]
    out: dict[str, str] = {}
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            out[k.strip()] = v.strip()
    return out


FAULT_BITS = {
    0x01: "sonar_timeout",
    0x02: "i2c_error",
    0x04: "angle_oob",
    0x08: "pos_oob",
}

STATE_LABELS = {
    "SAFE_DISABLED": "SAFE (driver disabled)",
    "READY": "READY",
    "RUNNING": "RUNNING (closed loop)",
    "FAULT": "FAULT (latched stop)",
    "CALIB_SIGN": "CALIB_SIGN (sign calibration)",
    "CALIB_SCALE": "CALIB_SCALE (sign scaling)",
}


def _fault_names(bits: Optional[int]) -> list[str]:
    if bits is None:
        return []
    names = [name for mask, name in FAULT_BITS.items() if (bits & mask) != 0]
    return names


def _fault_summary(bits: Optional[int]) -> str:
    if bits is None:
        return "?"
    if bits == 0:
        return "none"
    names = _fault_names(bits)
    if not names:
        return str(bits)
    return f"{'+'.join(names)} ({bits})"


def _fault_hint(bits: Optional[int]) -> Optional[str]:
    names = set(_fault_names(bits))
    if not names:
        return None
    # Prefer the most actionable hint for bring-up.
    if "sonar_timeout" in names:
        return "Sonar timeout: put a FLAT board in front, press D (sonar diag), then s."
    if "i2c_error" in names:
        return "AS5600 I2C error: check wiring, press A (as5600 diag), then s."
    if "angle_oob" in names:
        return "Angle out of bounds: level beam and keep within +/-15 deg, then s."
    if "pos_oob" in names:
        return "Position out of bounds: move target near center / check mapping, then s."
    return None


def _format_tel_line(t: TelemetrySample) -> str:
    def _f(v: Optional[float], w: int = 7, p: int = 2) -> str:
        if v is None:
            return "?" * w
        return f"{v:{w}.{p}f}"

    def _i(v: Optional[int], w: int = 6) -> str:
        if v is None:
            return "?" * w
        return f"{v:>{w}d}"

    return (
        f"{_i(t.t_ms)} {((t.state or '?')[:12]):<12} "
        f"x={_f(t.x_cm)} xf={_f(t.x_filt_cm)} th={_f(t.theta_deg, w=7, p=2)} "
        f"cmd={_f(t.theta_cmd_deg, w=6, p=2)} rate={_f(t.u_step_rate, w=6, p=1)} "
        f"fault={t.fault_bits if t.fault_bits is not None else '?'}"
    )


def _is_noise_line(line: str) -> bool:
    # These are protocol/status lines that confuse most users; we parse them into panels
    # and keep them out of the Events feed unless raw mode is enabled.
    return line.startswith(
        (
            "HELP,",
            "MENU,",
            "STATE,",
            "SENSORS,",
            "MEAS,",
            "AGE,",
            "FAULTS,",
            "SIGNS,",
            "CAL,",
            "CAL_ZERO,",
            "CAL_LIMITS,",
            "CAL_STORE,",
            "TEL,enabled=",
            "FAULT_INFO,",
        )
    )


GUIDE_TEXT = {
    "zero_angle": "Level the beam (make it horizontal). Then press a.",
    "zero_pos": "Place a FLAT target at beam center under the sonar. Then press p.",
    "down_limit": "Move beam to its hard DOWN stop. Then press [.",
    "up_limit": "Move beam to its hard UP stop. Then press ].",
    "limits_fix": "Limits span invalid. Re-capture [ and ].",
    "fault_reset": "Fix issue (see faults above). Then press f, then s.",
    "sign_begin": "Move target near reference end. Then press b.",
    "sonar_sign_auto": "Move target to FAR +X end now. Then press g.",
    "sonar_sign_manual": "Auto sonar sign failed. Use : and type 'sonar sign 1' or 'sonar sign -1'.",
    "angle_sensor": "AS5600 issue. Press A (as5600 diag), then s.",
    "sonar_sensor": "Sonar issue. Use a FLAT reflector. Press D (sonar diag), then s.",
    "inner_loop": "Inner loop unstable. Check gains and retry.",
    "save": "Calibration changed (unsaved). Press v to save to EEPROM.",
    "run": "All ready. Press r to start closed-loop.",
}

WIZ_TEXT = {
    1: "STEP 1/7  - Level the beam horizontally. Then press Enter.",
    2: "STEP 2/7  - Place a flat target at beam center. Then press Enter.",
    3: "STEP 3/7  - Move beam to physical DOWN stop. Then press Enter.",
    4: "STEP 4/7  - Move beam to physical UP stop. Then press Enter.",
    5: "STEP 5/7  - Move target near reference end. Then press Enter (auto jog).",
    6: "STEP 6/7  - Move target to FAR +X end. Then press Enter.",
    7: "STEP 7/7  - Press c to save to EEPROM, or q to exit without saving.",
}


def _next_action_text(state: DeviceState) -> str:
    if state.app_state == "CALIB_SCALE":
        return "State: CALIB_SCALE - Calibrating signs, please wait..."

    if state.wizard_active:
        if state.wizard_step_failed:
            return "(!) Wizard step failed - fix issue above, then press Enter to retry"
        if state.wizard_step_num in WIZ_TEXT:
            return WIZ_TEXT[state.wizard_step_num]
        return "Wizard active - follow WIZ prompts; Enter=n, c=save, q=abort"

    # EEPROM status hint
    if state.cal_store_status in ("defaults", "uninitialized"):
        return "EEPROM calibration missing/corrupt - press R to reset+recalibrate (or w for wizard)"

    if state.guide_step:
        step = state.guide_step.strip()
        if step in GUIDE_TEXT:
            return GUIDE_TEXT[step]
        # Fallback to firmware "do=" when available.
        if state.guide_do:
            return f"NEXT: {state.guide_do}"
        return f"NEXT: {step}"

    if state.guide_do:
        return f"NEXT: {state.guide_do}"

    return "Press ?:help. Press w for wizard. Press r to run. Press q/Q to quit."


def _parse_tel(line: str) -> Optional[TelemetrySample]:
    # Accept either "TEL,..." or already trimmed.
    if line.startswith("TEL,"):
        payload = line[4:]
    else:
        payload = line
    parts = [p.strip() for p in payload.split(",")]
    if len(parts) != len(TEL_COLUMNS):
        return None
    try:
        return TelemetrySample(
            t_ms=int(float(parts[0])),
            state=parts[1],
            x_cm=float(parts[2]),
            x_filt_cm=float(parts[3]),
            theta_deg=float(parts[4]),
            theta_cmd_deg=float(parts[5]),
            u_step_rate=float(parts[6]),
            fault_bits=int(float(parts[7])),
            host_rx_time=time.time(),
        )
    except ValueError:
        return None


def _maybe_start_logging(state: DeviceState, outdir: Path, events: list[Event], reason: str) -> None:
    if state.log_mode != "auto":
        return
    if state.logging_active:
        return

    outdir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = outdir / f"run_{stamp}_raw.log"
    state.logging_active = True
    state.log_path = path
    events.append(Event(_now_ts(), "HOST", f"LOG STARTED ({reason}): {path}"))


def _maybe_stop_logging(state: DeviceState, events: list[Event], reason: str, log_fp: Optional[IO[str]]) -> Optional[IO[str]]:
    if not state.logging_active:
        return log_fp
    state.logging_active = False
    events.append(Event(_now_ts(), "HOST", f"LOG STOPPED ({reason}): {state.log_path}"))
    state.log_path = None
    if log_fp is not None:
        try:
            log_fp.flush()
            log_fp.close()
        except Exception:
            pass
    return None


def _update_from_line(state: DeviceState, line: str, outdir: Path, events: list[Event], log_fp: Optional[IO[str]]) -> Optional[IO[str]]:
    # Logging triggers that depend on parsed content:
    # - OK,running / OK,stopped
    # - TEL RUNNING transitions
    # - fault bits while RUNNING
    #
    # Return possibly-updated log_fp.

    kind = _event_kind(line)

    # Telemetry parsing first (frequent).
    if line.startswith("TEL,") and not line.startswith("TEL,enabled="):
        sample = _parse_tel(line)
        if sample:
            state.telemetry = sample
            state.tel_stream.append(_format_tel_line(sample))
            if len(state.tel_stream) > 300:
                del state.tel_stream[: len(state.tel_stream) - 300]
            # Keep app_state in sync with telemetry.
            new_state = sample.state
            if new_state:
                if state.app_state != new_state:
                    state._prev_app_state = state.app_state
                    state.app_state = new_state
                if new_state == "RUNNING" and not state.logging_active:
                    _maybe_start_logging(state, outdir, events, "TEL state RUNNING")
                if state._prev_app_state == "RUNNING" and new_state != "RUNNING":
                    log_fp = _maybe_stop_logging(state, events, f"TEL left RUNNING -> {new_state}", log_fp)

            # Use telemetry fault bits as the current fault snapshot.
            if sample.fault_bits is not None:
                state.fault_bits = sample.fault_bits
                if state.app_state == "RUNNING" and sample.fault_bits != 0:
                    log_fp = _maybe_stop_logging(state, events, f"fault_bits={sample.fault_bits}", log_fp)

        return log_fp

    # Track telemetry enabled status line from status output.
    if line.startswith("TEL,enabled="):
        try:
            v = line.split("=", 1)[1].strip()
            state.tel_enabled = (int(v) != 0)
        except Exception:
            pass
        return log_fp

    # General: update prev state when we see STATE lines.
    if line.startswith("STATE,"):
        new = line.split(",", 1)[1].strip()
        if new:
            if state.app_state != new:
                state._prev_app_state = state.app_state
                state.app_state = new
            if new == "RUNNING" and not state.logging_active:
                _maybe_start_logging(state, outdir, events, "STATE RUNNING")
            if state._prev_app_state == "RUNNING" and new != "RUNNING":
                log_fp = _maybe_stop_logging(state, events, f"STATE left RUNNING -> {new}", log_fp)

    if line.startswith("SENSORS,"):
        kv = _parse_kv_csv(line)
        if "angle" in kv:
            state.angle_ok = (kv["angle"].lower() == "ok")
        if "pos" in kv:
            state.pos_ok = (kv["pos"].lower() == "ok")

    if line.startswith("AGE,"):
        kv = _parse_kv_csv(line)
        if "angle_ms" in kv:
            try:
                state.age_angle_ms = int(float(kv["angle_ms"]))
            except ValueError:
                pass
        if "pos_ms" in kv:
            try:
                state.age_pos_ms = int(float(kv["pos_ms"]))
            except ValueError:
                pass

    if line.startswith("CAL,"):
        kv = _parse_kv_csv(line)
        if "zero_calibrated" in kv:
            state.zero_cal = (kv["zero_calibrated"].lower() == "yes")
        if "limits_calibrated" in kv:
            state.limits_cal = (kv["limits_calibrated"].lower() == "yes")
        if "sign_calibrated" in kv:
            state.sign_cal = (kv["sign_calibrated"].lower() == "yes")

    if line.startswith("CAL_STORE,"):
        kv = _parse_kv_csv(line)
        if "status" in kv:
            state.cal_store_status = kv["status"].strip().lower()

    if line.startswith("FAULTS,bits="):
        try:
            bits = int(line.split("=", 1)[1].strip())
            state.fault_bits = bits
        except Exception:
            pass

    if line.startswith("FAULT_INFO,"):
        # Keep only last few for status panel.
        state.fault_info_lines.append(line)
        state.fault_info_lines = state.fault_info_lines[-6:]

    if line.startswith("GUIDE,step="):
        kv = _parse_kv_csv(line)
        if "step" in kv:
            state.guide_step = kv["step"]
        if "do" in kv:
            state.guide_do = kv["do"]
    elif line.startswith("GUIDE,do="):
        kv = _parse_kv_csv(line)
        if "do" in kv:
            state.guide_do = kv["do"]

    if line.startswith("OK,wizard_started"):
        state.wizard_active = True
        state.wizard_step_failed = False
        state.wizard_step_num = 1

    if line.startswith("WIZ,exit,"):
        state.wizard_active = False
        state.wizard_step_failed = False
        state.wizard_step_num = None

    if line.startswith("WIZ,step_failed"):
        state.wizard_step_failed = True

    if line.startswith("WIZ,") and state.wizard_active:
        # Step lines look like "WIZ,1,do=..." etc.
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 2:
            try:
                n = int(parts[1])
                state.wizard_step_num = n
                # A new step prompt implies the previous step completed.
                if not line.startswith("WIZ,step_failed"):
                    state.wizard_step_failed = False
            except ValueError:
                # WIZ,summary or other non-numeric token.
                if parts[1] == "summary":
                    state.wizard_step_num = 7
                    if not line.startswith("WIZ,step_failed"):
                        state.wizard_step_failed = False

    if line.startswith("OK,telemetry="):
        try:
            v = int(line.split("=", 1)[1].strip())
            state.tel_enabled = (v != 0)
        except Exception:
            pass

    if line.startswith("WIZ,telemetry_auto_disabled"):
        # Wizard forces telemetry off; keep tracked state consistent.
        state.tel_enabled = False

    if line.startswith("OK,running"):
        _maybe_start_logging(state, outdir, events, "OK,running")

    if line.startswith("OK,stopped"):
        log_fp = _maybe_stop_logging(state, events, "OK,stopped", log_fp)

    if line.startswith("OK,driver_enabled"):
        state.driver_enabled = True

    if line.startswith("OK,driver_disabled"):
        state.driver_enabled = False

    if line.startswith("SIGN_CAL_RESULT,"):
        # Display an easy one-line summary.
        parts = [p.strip() for p in line.split(",")]
        if len(parts) == 8:
            _, ts_ms, delta_deg, step_sign, as_sign, near_cm, far_cm, sonar_sign = parts
            events.append(
                Event(
                    _now_ts(),
                    "OK",
                    f"SIGN_CAL_RESULT: stepper={step_sign} as5600={as_sign} sonar={sonar_sign} "
                    f"(near_cm={near_cm} far_cm={far_cm} delta_raw_deg={delta_deg})",
                )
            )
            return log_fp

    # Diag snapshots (show as a single helpful line).
    if line.startswith("SONAR_DIAG,"):
        state.sonar_diag = _parse_kv_csv(line)
        if state.show_raw:
            events.append(Event(_now_ts(), "INFO", line))
        else:
            hs = state.sonar_diag.get("has_sample", "?")
            fr = state.sonar_diag.get("fresh", "?")
            to = state.sonar_diag.get("timeout", "?")
            raw = state.sonar_diag.get("raw_cm", "?")
            filt = state.sonar_diag.get("filt_cm", "?")
            events.append(Event(_now_ts(), "INFO", f"SONAR: has_sample={hs} fresh={fr} timeout={to} raw_cm={raw} filt_cm={filt}"))
        return log_fp

    if line.startswith("AS5600_DIAG,"):
        parts = [p.strip() for p in line.split(",")]
        # Format: AS5600_DIAG,ok,raw_deg,theta_deg,err_cnt,read_hz
        if len(parts) == 6:
            state.as5600_diag = {
                "ok": parts[1],
                "raw_deg": parts[2],
                "theta_deg": parts[3],
                "err_cnt": parts[4],
                "read_hz": parts[5],
            }
        if state.show_raw:
            events.append(Event(_now_ts(), "INFO", line))
        else:
            ok = state.as5600_diag.get("ok", "?")
            th = state.as5600_diag.get("theta_deg", "?")
            er = state.as5600_diag.get("err_cnt", "?")
            hz = state.as5600_diag.get("read_hz", "?")
            events.append(Event(_now_ts(), "INFO", f"AS5600: ok={ok} theta_deg={th} err_cnt={er} hz={hz}"))
        return log_fp

    # Finally, decide whether to show this line in Events.
    if kind == "TEL":
        return log_fp

    if (not state.show_raw) and _is_noise_line(line):
        return log_fp

    events.append(Event(_now_ts(), kind, line))
    return log_fp


def _detect_port() -> str:
    ports = list(list_ports.comports())
    likely = []
    for p in ports:
        name = (p.device or "").lower()
        desc = (p.description or "").lower()
        if any(k in name for k in ("usbserial", "usbmodem", "wchusbserial")) or any(
            k in desc for k in ("usb serial", "usbmodem", "usbserial", "wch")
        ):
            likely.append(p.device)

    if len(likely) == 1:
        return likely[0]

    # If exactly one port exists, use it.
    if len(ports) == 1 and ports[0].device:
        return ports[0].device

    # Otherwise, print list and exit.
    print("Multiple serial ports found. Re-run with --port <device>.")
    for i, p in enumerate(ports, start=1):
        print(f"  {i}. {p.device}  ({p.description})")
    raise SystemExit(2)


def _serial_reader_thread(
    ser: serial.Serial, out_q: "queue.Queue[str]", stop_evt: threading.Event
) -> None:
    while not stop_evt.is_set():
        try:
            raw = ser.readline()
        except Exception:
            break
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="replace").strip("\r\n")
        except Exception:
            continue
        out_q.put(line)


def _replay_reader_thread(
    path: Path,
    out_q: "queue.Queue[str]",
    stop_evt: threading.Event,
    replay_rate_hz: float,
) -> None:
    dt = 0.1 if replay_rate_hz <= 0 else (1.0 / float(replay_rate_hz))
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            if stop_evt.is_set():
                break
            line = raw.strip("\r\n")
            if not line:
                continue
            out_q.put(line)
            if line.startswith("TEL,") and not line.startswith("TEL,enabled="):
                time.sleep(dt)


def _send_command(
    ser: Optional[serial.Serial],
    events: list[Event],
    cmd: str,
    replay_mode: bool,
    state: DeviceState,
) -> None:
    cmd = cmd.strip()
    if not cmd:
        return
    if replay_mode or ser is None:
        events.append(Event(_now_ts(), "HOST", f"sent (suppressed): {cmd}"))
        state.footer_msg = "Replay mode: device commands suppressed"
        return
    try:
        ser.write((cmd + "\n").encode("utf-8"))
        ser.flush()
        events.append(Event(_now_ts(), "HOST", f"sent: {cmd}"))
        state.footer_msg = ""
    except Exception as exc:
        events.append(Event(_now_ts(), "ERR", f"send failed: {exc}"))
        state.footer_msg = "Send failed"


def _draw(
    stdscr: "curses._CursesWindow",
    state: DeviceState,
    events: list[Event],
    colors: dict[str, int],
    port: str,
    baud: int,
    cmd_mode: bool,
    cmd_buf: str,
    help_mode: bool,
) -> None:
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()

    # Header (make quit + main toggles obvious)
    tel = "?" if state.tel_enabled is None else ("ON" if state.tel_enabled else "OFF")
    log_mode = state.log_mode
    logging = "ON" if state.logging_active else "OFF"
    view = "EVENTS" if state.view_mode == "events" else "TELEMETRY"
    raw = "ON" if state.show_raw else "OFF"
    wiz = "ON" if state.wizard_active else "OFF"
    head = (
        f"Bring-up Monitor | port={port} baud={baud} | panel={view} (Tab) | dev_tel={tel} (t) | "
        f"raw={raw} (X) | wiz={wiz} (w/q) | log={log_mode} LOG:{logging} | Q=quit ?=help"
    )
    _safe_addstr(stdscr, 0, 0, head)

    # Layout
    footer_y = max_y - 1
    events_h = min(10, max(4, max_y // 3))
    events_y = footer_y - events_h
    body_y = 1
    body_h = max(0, events_y - body_y)

    left_w = max_x // 2
    right_w = max_x - left_w

    # Status panel
    _draw_box(stdscr, body_y, 0, body_h, left_w, "Status")
    # Telemetry panel
    _draw_box(stdscr, body_y, left_w, body_h, right_w, "Telemetry")
    # Events/Telemetry panel
    panel_title = "Telemetry Stream (Tab:events)" if state.view_mode == "telemetry" else "Events (Tab:telemetry)"
    _draw_box(stdscr, events_y, 0, events_h, max_x, panel_title)

    # Status content
    y = body_y + 1
    x0 = 2
    def _yn(v: Optional[bool]) -> str:
        if v is None:
            return "?"
        return "OK" if v else "BAD"

    cal_zero = "?" if state.zero_cal is None else ("yes" if state.zero_cal else "no")
    cal_lim = "?" if state.limits_cal is None else ("yes" if state.limits_cal else "no")
    cal_sign = "?" if state.sign_cal is None else ("yes" if state.sign_cal else "no")
    faults = _fault_summary(state.fault_bits)
    tel = "?" if state.tel_enabled is None else ("ON" if state.tel_enabled else "OFF")
    drv = "?" if state.driver_enabled is None else ("ON" if state.driver_enabled else "OFF")

    state_name = state.app_state or "?"
    state_label = STATE_LABELS.get(state_name, "")
    state_line = f"State  : {state_name}" + (f" - {state_label}" if state_label else "")

    # Beginner-first status (avoid dumping protocol lines here).
    lines = [
        "START HERE: w=wizard, then Enter each step, then c=save, then r=run. Q quits anytime.",
        "Restart calibration from scratch: R (confirm y). Toggle telemetry panel: Tab. ?:help",
        "",
        state_line,
        f"Sensors: angle={_yn(state.angle_ok)}  sonar={_yn(state.pos_ok)}",
        f"Cal    : zero={cal_zero}  limits={cal_lim}  sign={cal_sign}   (wizard: w)",
        f"EEPROM : {state.cal_store_status or '?'}   (defaults/uninitialized => run wizard)",
        f"Faults : {faults}   (f=clear fault after fix)",
        f"Device telemetry: {tel}   (t toggles device telemetry on/off)",
    ]
    if state.driver_enabled is not None or state.show_raw:
        lines.append(f"Motor driver: {drv}   (E toggles e 1/e 0)")
    if state.age_angle_ms is not None or state.age_pos_ms is not None:
        lines.append(
            f"Data age ms: angle={state.age_angle_ms if state.age_angle_ms is not None else '?'}  "
            f"sonar={state.age_pos_ms if state.age_pos_ms is not None else '?'}"
        )

    next_line = _next_action_text(state)
    lines.append("")
    hint = _fault_hint(state.fault_bits)
    lines.append("NEXT: " + next_line)
    if hint and (hint not in next_line):
        lines.append("HINT: " + hint)

    if state.wizard_active:
        wz = state.wizard_step_num or 0
        lines.append("")
        lines.append(f"Wizard : ACTIVE (step {wz}/7)   Enter=next   q=abort wizard")
    else:
        lines.append("")
        lines.append("Wizard : inactive   (press w to start)")

    for i, s in enumerate(lines):
        if y + i >= body_y + body_h - 1:
            break
        _safe_addstr(stdscr, y + i, x0, s)

    # Note: raw FAULT_INFO lines are intentionally NOT shown in the Status panel.
    # Enable Raw mode (X) to see them in Events instead.

    # Telemetry content
    t = state.telemetry
    ty = body_y + 1
    tx = left_w + 2
    tel_lines = [
        f"t_ms   : {t.t_ms if t.t_ms is not None else '?'}",
        f"state  : {t.state or '?'}",
        f"x      : {t.x_cm if t.x_cm is not None else '?'}",
        f"x_filt : {t.x_filt_cm if t.x_filt_cm is not None else '?'}",
        f"theta  : {t.theta_deg if t.theta_deg is not None else '?'}",
        f"cmd    : {t.theta_cmd_deg if t.theta_cmd_deg is not None else '?'}",
        f"rate   : {t.u_step_rate if t.u_step_rate is not None else '?'}",
        f"faults : {t.fault_bits if t.fault_bits is not None else '?'}",
    ]
    if t.host_rx_time is not None:
        age = max(0.0, time.time() - t.host_rx_time)
        tel_lines.append(f"rx_age : {age:0.2f}s")
    for i, s in enumerate(tel_lines):
        if ty + i >= body_y + body_h - 1:
            break
        _safe_addstr(stdscr, ty + i, tx, s[: max(0, right_w - 4)])

    # Panel content (last 20)
    ev_y = events_y + 1
    ev_x = 2
    if state.view_mode == "telemetry":
        stream = state.tel_stream[-(events_h - 2) :]
        for i, s in enumerate(stream):
            row = ev_y + i
            if row >= events_y + events_h - 1:
                break
            _safe_addstr(stdscr, row, ev_x, s[: max(0, max_x - 4)], colors.get("INFO", 0))
    else:
        ev_lines = events[-20:]
        for i, ev in enumerate(ev_lines):
            row = ev_y + i
            if row >= events_y + events_h - 1:
                break
            prefix = f"[{ev.ts}] "
            text = prefix + ev.text
            attr = colors.get(ev.kind, 0)
            _safe_addstr(stdscr, row, ev_x, text[: max(0, max_x - 4)], attr)

    # Footer
    footer = ""
    if help_mode:
        footer = "HELP: ? or ESC to close. Q quits."
    elif cmd_mode:
        footer = f"cmd> {cmd_buf}"
    else:
        if state.wizard_active:
            footer = "Q:quit  q:abort-wiz  w:wizard  Enter:next  c:save  R:restart-cal  r:run  k:stop  t:dev-tel  Tab:panel  X:raw  :cmd  ?:help"
        else:
            footer = "Q:quit  q:quit  w:wizard  R:restart-cal  r:run  k:stop  s:status  t:dev-tel  Tab:panel  X:raw  :cmd  ?:help"
        if state.footer_msg:
            footer = state.footer_msg
    _safe_addstr(stdscr, footer_y, 0, footer[: max_x - 1])

    # Help overlay
    if help_mode:
        _draw_help_overlay(stdscr, max_y, max_x)

    stdscr.refresh()


def _draw_help_overlay(stdscr: "curses._CursesWindow", max_y: int, max_x: int) -> None:
    h = max(10, min(max_y - 2, 22))
    w = max(20, min(max_x - 4, 78))
    y0 = (max_y - h) // 2
    x0 = (max_x - w) // 2
    _draw_box(stdscr, y0, x0, h, w, "Help")

    lines = [
        "Bring-up Monitor (host-side). Press Q to quit. Press ? to close this help.",
        "",
        "Recommended workflow (most users):",
        "  w  -> start wizard",
        "  Enter -> next step (repeat until step 7)",
        "  c  -> save calibration to EEPROM",
        "  r  -> start closed-loop run",
        "  k  -> stop",
        "",
        "Wizard (recommended):",
        "  w = start wizard",
        "  Enter/n = wizard next step (sends n) (only when wizard active)",
        "  c = confirm EEPROM save (wizard end)",
        "  q = abort wizard when wizard is active; otherwise quit monitor",
        "  R = restart calibration: reset defaults + start wizard (confirm y)",
        "",
        "Calibration quick keys (manual):",
        "  o = reload calibration from EEPROM (optional; firmware auto-loads at boot)",
        "  a = capture angle zero  (BEFORE: beam level NOW)",
        "  p = capture position zero  (BEFORE: flat target at beam center NOW)",
        "  [ = capture DOWN limit  (BEFORE: beam at hard DOWN stop NOW)",
        "  ] = capture UP limit    (BEFORE: beam at hard UP stop NOW)",
        "  b = sign begin (auto jog) (BEFORE: zero+limits done; target near reference end)",
        "  g = sign save            (BEFORE: move target to FAR +X end NOW)",
        "  v = save calibration to EEPROM (BEFORE: calibration complete; changes pending)",
        "  f = clear fault          (BEFORE: fix issue shown above)",
        "",
        "Diagnostics:",
        "  A=as5600 diag   D=sonar diag",
        "",
        "Core:",
        "  s=status  i=bringup menu  r=run  k=stop  x=faults+sonar diag",
        "  E=toggle driver enable (sends e 1/e 0)  k=stop always safe",
        "",
        "Telemetry:",
        "  t = toggle device telemetry 0/1 deterministically",
        "  Tab/V = toggle bottom panel between Events and Telemetry Stream",
        "  X = show/hide raw protocol lines in Events (expert)",
        "",
        "Command prompt:",
        "  : = type any command (e.g. 'sonar sign 1')",
        "",
        "Quit:",
        "  Q = quit (sends k then e 0 for safety) (ESC also quits)",
    ]
    for i, s in enumerate(lines):
        if y0 + 1 + i >= y0 + h - 1:
            break
        _safe_addstr(stdscr, y0 + 1 + i, x0 + 2, s[: max(0, w - 4)])


def run_tui(args: argparse.Namespace) -> int:
    outdir = Path(args.outdir).resolve()
    replay_mode = args.replay is not None

    port = args.port
    if not replay_mode:
        if port is None:
            port = _detect_port()
    else:
        port = "<replay>"

    baud = int(args.baud)

    q_lines: "queue.Queue[str]" = queue.Queue()
    stop_evt = threading.Event()

    ser: Optional[serial.Serial] = None
    if not replay_mode:
        ser = serial.Serial(port, baud, timeout=0.2)
        t = threading.Thread(
            target=_serial_reader_thread, args=(ser, q_lines, stop_evt), daemon=True
        )
        t.start()
    else:
        replay_path = Path(args.replay).resolve()
        t = threading.Thread(
            target=_replay_reader_thread,
            args=(replay_path, q_lines, stop_evt, float(args.replay_rate_hz)),
            daemon=True,
        )
        t.start()

    state = DeviceState()
    state.log_mode = str(args.log_mode).strip().lower()
    if state.log_mode not in ("auto", "off"):
        state.log_mode = "auto"

    events: list[Event] = []
    colors: dict[str, int] = {}

    log_fp: Optional[IO[str]] = None

    # On connect, request a status snapshot once.
    if not replay_mode:
        _send_command(ser, events, "s", replay_mode, state)

    cmd_mode = False
    cmd_buf = ""
    help_mode = False
    confirm_restart = False

    def _curses_main(stdscr: "curses._CursesWindow") -> int:
        nonlocal log_fp, cmd_mode, cmd_buf, help_mode, colors, confirm_restart
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(100)  # 100ms tick
        colors = _init_colors()

        while True:
            # Drain queue
            drained = 0
            while True:
                try:
                    line = q_lines.get_nowait()
                except queue.Empty:
                    break
                drained += 1

                was_logging = state.logging_active

                # Ensure file is open before writing when already logging.
                if state.logging_active and state.log_path and log_fp is None:
                    try:
                        log_fp = state.log_path.open("w", encoding="utf-8")
                    except Exception as exc:
                        events.append(Event(_now_ts(), "ERR", f"log open failed: {exc}"))
                        state.logging_active = False
                        state.log_path = None
                        log_fp = None

                # Write the line if we were already logging.
                if state.logging_active and log_fp is not None:
                    try:
                        log_fp.write(line + "\n")
                    except Exception:
                        pass

                # Parse/update may start/stop logging.
                log_fp = _update_from_line(state, line, outdir, events, log_fp)

                # If logging just started because of this line, include it.
                if (not was_logging) and state.logging_active and state.log_path and log_fp is None:
                    try:
                        log_fp = state.log_path.open("w", encoding="utf-8")
                    except Exception as exc:
                        events.append(Event(_now_ts(), "ERR", f"log open failed: {exc}"))
                        state.logging_active = False
                        state.log_path = None
                        log_fp = None
                if (not was_logging) and state.logging_active and log_fp is not None:
                    try:
                        log_fp.write(line + "\n")
                    except Exception:
                        pass

                # Prevent unbounded growth.
                if len(events) > 600:
                    del events[: len(events) - 600]

            # If log stopped, ensure file is closed.
            if (not state.logging_active) and log_fp is not None:
                try:
                    log_fp.flush()
                    log_fp.close()
                except Exception:
                    pass
                log_fp = None

            _draw(stdscr, state, events, colors, str(port), baud, cmd_mode, cmd_buf, help_mode)

            try:
                ch = stdscr.getch()
            except KeyboardInterrupt:
                ch = ord("q")

            if ch == -1:
                continue

            if ch == curses.KEY_RESIZE:
                continue

            # Global quit (works even if you're in help/command/confirm modes)
            if ch in (3, ord("Q")):  # Ctrl+C or Q
                if not replay_mode:
                    _send_command(ser, events, "k", replay_mode, state)
                    _send_command(ser, events, "e 0", replay_mode, state)
                return 0

            # Help overlay toggling
            if ch in (ord("?"), curses.KEY_F1):
                help_mode = not help_mode
                state.footer_msg = ""
                continue
            if help_mode:
                if ch in (27,):  # ESC closes help
                    help_mode = False
                    continue
                continue

            # Command prompt mode
            if cmd_mode:
                if ch in (curses.KEY_ENTER, 10, 13):
                    _send_command(ser, events, cmd_buf, replay_mode, state)
                    cmd_mode = False
                    cmd_buf = ""
                    continue
                if ch in (27,):  # ESC cancels
                    cmd_mode = False
                    cmd_buf = ""
                    continue
                if ch in (curses.KEY_BACKSPACE, 127, 8):
                    cmd_buf = cmd_buf[:-1]
                    continue
                if 0 <= ch < 256 and curses.ascii.isprint(ch):
                    cmd_buf += chr(ch)
                continue

            # Normal mode keys
            if ch == ord(":"):
                cmd_mode = True
                cmd_buf = ""
                continue

            if confirm_restart:
                if ch in (ord("y"), ord("Y")):
                    confirm_restart = False
                    state.footer_msg = ""
                    # Stop, disable, clear fault, reset defaults, then start wizard.
                    for cmd in ("k", "e 0", "f", "d", "w", "s"):
                        _send_command(ser, events, cmd, replay_mode, state)
                        time.sleep(0.05)
                else:
                    # Any other key cancels.
                    confirm_restart = False
                    state.footer_msg = "Restart canceled"
                continue

            if ch in (27,):  # ESC
                if state.wizard_active:
                    _send_command(ser, events, "q", replay_mode, state)
                    state.footer_msg = "Wizard aborted (ESC)"
                    continue
                # Otherwise treat ESC as quit.
                if not replay_mode:
                    _send_command(ser, events, "k", replay_mode, state)
                    _send_command(ser, events, "e 0", replay_mode, state)
                return 0

            if ch in (ord("q"),):
                if state.wizard_active:
                    _send_command(ser, events, "q", replay_mode, state)
                    state.footer_msg = "Wizard aborted (q)"
                    continue
                # Quit monitor when wizard is inactive.
                if not replay_mode:
                    _send_command(ser, events, "k", replay_mode, state)
                    _send_command(ser, events, "e 0", replay_mode, state)
                return 0

            # Wizard
            if ch == ord("w"):
                _send_command(ser, events, "w", replay_mode, state)
                continue

            if ch in (curses.KEY_ENTER, 10, 13):
                if state.wizard_active:
                    _send_command(ser, events, "n", replay_mode, state)
                continue

            if ch == ord("n"):
                if state.wizard_active:
                    _send_command(ser, events, "n", replay_mode, state)
                continue

            if ch == ord("c"):
                _send_command(ser, events, "c", replay_mode, state)
                continue

            # Core
            if ch == ord("s"):
                _send_command(ser, events, "s", replay_mode, state)
                continue
            if ch == ord("i"):
                _send_command(ser, events, "i", replay_mode, state)
                continue
            if ch == ord("r"):
                _send_command(ser, events, "r", replay_mode, state)
                continue
            if ch == ord("k"):
                _send_command(ser, events, "k", replay_mode, state)
                continue
            if ch == ord("x"):
                _send_command(ser, events, "x", replay_mode, state)
                continue

            # Manual calibration quick keys (forwarded directly).
            if ch == ord("o"):
                _send_command(ser, events, "o", replay_mode, state)
                continue
            if ch == ord("a"):
                _send_command(ser, events, "a", replay_mode, state)
                continue
            if ch == ord("p"):
                _send_command(ser, events, "p", replay_mode, state)
                continue
            if ch == ord("["):
                _send_command(ser, events, "[", replay_mode, state)
                continue
            if ch == ord("]"):
                _send_command(ser, events, "]", replay_mode, state)
                continue
            if ch == ord("b"):
                _send_command(ser, events, "b", replay_mode, state)
                continue
            if ch == ord("g"):
                _send_command(ser, events, "g", replay_mode, state)
                continue
            if ch == ord("f"):
                _send_command(ser, events, "f", replay_mode, state)
                continue
            if ch == ord("d"):
                _send_command(ser, events, "d", replay_mode, state)
                continue

            # Diagnostics
            if ch == ord("A"):
                _send_command(ser, events, "as5600 diag", replay_mode, state)
                continue
            if ch == ord("D"):
                _send_command(ser, events, "sonar diag", replay_mode, state)
                continue

            # Driver enable toggle
            if ch == ord("E"):
                want_on = not bool(state.driver_enabled)
                _send_command(ser, events, f"e {1 if want_on else 0}", replay_mode, state)
                continue

            # EEPROM save pass-through
            if ch == ord("v"):
                _send_command(ser, events, "v", replay_mode, state)
                continue

            # Telemetry deterministic toggle
            if ch == ord("t"):
                want_on = not bool(state.tel_enabled)
                cmd = "telemetry 1" if want_on else "telemetry 0"
                _send_command(ser, events, cmd, replay_mode, state)
                continue

            if ch in (ord("V"), 9):  # V or Tab
                state.view_mode = "telemetry" if state.view_mode == "events" else "events"
                events.append(Event(_now_ts(), "HOST", f"view set to {state.view_mode}"))
                continue

            if ch == ord("X"):
                state.show_raw = not state.show_raw
                events.append(Event(_now_ts(), "HOST", f"raw protocol lines: {'ON' if state.show_raw else 'OFF'}"))
                continue

            if ch == ord("R"):
                confirm_restart = True
                state.footer_msg = (
                    "Restart calibration? (stops run + disables driver + resets defaults + starts wizard) "
                    "Press y to confirm, any other key to cancel."
                )
                continue

            if ch == ord("L"):
                if state.log_mode == "auto":
                    state.log_mode = "off"
                    log_fp = _maybe_stop_logging(state, events, "log-mode off", log_fp)
                else:
                    state.log_mode = "auto"
                events.append(Event(_now_ts(), "HOST", f"log_mode set to {state.log_mode}"))
                continue

    try:
        return curses.wrapper(_curses_main)
    finally:
        stop_evt.set()
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass
        if log_fp is not None:
            try:
                log_fp.close()
            except Exception:
                pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Host-side bring-up monitor (curses TUI)")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "data" / "runs",
        help="Directory for raw logs (auto mode)",
    )
    parser.add_argument("--log-mode", default="auto", choices=["auto", "off"], help="Logging mode")
    parser.add_argument("--replay", type=Path, default=None, help="Replay from raw log file (no device writes)")
    parser.add_argument("--replay-rate-hz", type=float, default=10.0, help="Replay TEL rate (Hz)")
    args = parser.parse_args()

    return run_tui(args)


if __name__ == "__main__":
    raise SystemExit(main())
