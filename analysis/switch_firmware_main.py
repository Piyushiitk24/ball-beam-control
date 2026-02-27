#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from datetime import datetime
from pathlib import Path


def _stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def switch_to_as5600_check(repo: Path) -> None:
    src_main = repo / "firmware" / "src" / "main.cpp"
    bak_main = repo / "firmware" / "src" / "main.cpp.bak"
    exp_main = repo / "firmware" / "experiments" / "main_as5600_check.cpp"

    if not exp_main.exists():
        raise SystemExit(f"Missing experiment source: {exp_main}")
    if not src_main.exists() and not bak_main.exists():
        raise SystemExit(f"Missing firmware main.cpp: {src_main}")

    if bak_main.exists():
        print(f"NOTE: backup already exists: {bak_main}")
    else:
        print(f"Backing up {src_main} -> {bak_main}")
        src_main.rename(bak_main)

    print(f"Installing AS5600 check firmware: {exp_main} -> {src_main}")
    shutil.copyfile(exp_main, src_main)

    print("OK: main.cpp is now AS5600 check.")
    print("Next:")
    print("  cd firmware && pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1")


def restore_ballbeam(repo: Path) -> None:
    src_main = repo / "firmware" / "src" / "main.cpp"
    bak_main = repo / "firmware" / "src" / "main.cpp.bak"

    if not bak_main.exists():
        raise SystemExit(f"Backup not found: {bak_main}")

    if src_main.exists():
        keep = repo / "firmware" / "src" / f"main.cpp.as5600_{_stamp()}.bak"
        print(f"Keeping current main as: {keep}")
        src_main.rename(keep)

    print(f"Restoring {bak_main} -> {src_main}")
    bak_main.rename(src_main)

    print("OK: main.cpp restored to BallBeam firmware.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Swap firmware/src/main.cpp between BallBeam and AS5600 check.")
    parser.add_argument(
        "--mode",
        required=True,
        choices=["as5600_check", "ballbeam"],
        help="as5600_check backs up main.cpp -> main.cpp.bak and installs the AS5600 checker. "
        "ballbeam restores main.cpp.bak -> main.cpp.",
    )
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    if args.mode == "as5600_check":
        switch_to_as5600_check(repo)
    else:
        restore_ballbeam(repo)


if __name__ == "__main__":
    main()

