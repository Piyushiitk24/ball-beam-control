#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import serial


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture raw serial telemetry")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbserial-XXXX")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--seconds", type=float, default=30.0, help="Capture duration in seconds")
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "data" / "runs",
        help="Directory for raw logs",
    )
    args = parser.parse_args()

    if args.port.strip() == "<SERIAL_PORT>":
        raise SystemExit("Replace <SERIAL_PORT> with a real device path")

    args.outdir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_file = args.outdir / f"run_{stamp}_raw.log"

    print(f"Capturing {args.seconds:.1f}s from {args.port} @ {args.baud} ...")
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser, out_file.open(
        "w", encoding="utf-8"
    ) as f_out:
        end_time = datetime.now().timestamp() + args.seconds
        while datetime.now().timestamp() < end_time:
            line = ser.readline().decode("utf-8", errors="replace")
            if not line:
                continue
            f_out.write(line)
            f_out.flush()

    print("Saved:", out_file)


if __name__ == "__main__":
    main()
