#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
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


@dataclass
class Snapshot:
    state: Optional[str] = None
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
        self.snapshot_hz = max(0.1, float(snapshot_hz))
        self.print_tel = bool(print_tel)
        self.no_input = bool(no_input)

        self.stop_event = threading.Event()
        self.rx_q: "queue.Queue[str]" = queue.Queue()
        self.snap = Snapshot()
        self.protocol: str = "unknown"  # ballbeam/as5600_check/unknown
        self._start_time = time.time()
        self._protocol_warned = False
        self._first_lines: list[str] = []

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

    def _send(self, cmd: str) -> None:
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
        self._log_event(f"sent: {cmd}")
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
        print(line)
        self._log_event(line)

        if line.startswith("STATE,"):
            self.snap.state = line.split(",", 1)[1].strip()

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
            f"SNAP proto={proto}  state={state}  sensors(angle={a} sonar={s})  theta={theta}  x_filt={x}  faults={faults}  lastTEL={last_tel}"
        )

    def _print_help(self) -> None:
        print(
            "\nLocal commands:\n"
            "  /help                 Show this help\n"
            "  /quit                 Quit (sends: k then e 0)\n"
            "  /print_tel 0|1        Toggle printing raw TEL rows to terminal\n"
            "  /diag                 Send: s, as5600 diag, sonar diag, x\n"
            "\nDevice commands: type anything else and press Enter (e.g. s, w, n, c, r, k, sonar diag)\n"
        )

    def run(self) -> None:
        self._rx_thread.start()

        print("Serial logger connected.")
        print(f"  RAW     : {self.raw_path}")
        print(f"  EVENTS  : {self.events_path}")
        print(f"  TELEMETRY CSV: {self.telemetry_path}")
        print("Type /help for local commands. Type device commands and press Enter.")

        # Force a status snapshot into logs without changing telemetry state.
        self._send("s")

        next_snap = time.time() + (1.0 / self.snapshot_hz)

        try:
            while not self.stop_event.is_set():
                # Drain queued RX lines.
                while True:
                    try:
                        s = self.rx_q.get_nowait()
                    except queue.Empty:
                        break
                    self._handle_device_line(s)

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
                if now >= next_snap:
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
    parser.add_argument("--snapshot-hz", type=float, default=1.0, help="Snapshot print rate (Hz)")
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
