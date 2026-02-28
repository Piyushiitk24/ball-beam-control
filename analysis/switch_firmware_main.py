#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from datetime import datetime
from pathlib import Path


def _stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _ensure_main_backup(src_main: Path, bak_main: Path) -> None:
    if bak_main.exists():
        print(f"NOTE: backup already exists: {bak_main}")
        return
    print(f"Backing up {src_main} -> {bak_main}")
    src_main.rename(bak_main)


def switch_to_hcsr04_check(repo: Path) -> None:
    src_main = repo / "firmware" / "src" / "main.cpp"
    bak_main = repo / "firmware" / "src" / "main.cpp.bak"
    exp_main = repo / "firmware" / "experiments" / "main_hcsr04_check.cpp"

    if not exp_main.exists():
        raise SystemExit(f"Missing experiment source: {exp_main}")
    if not src_main.exists() and not bak_main.exists():
        raise SystemExit(f"Missing firmware main.cpp: {src_main}")

    _ensure_main_backup(src_main, bak_main)

    print(f"Installing HC-SR04 check firmware: {exp_main} -> {src_main}")
    shutil.copyfile(exp_main, src_main)

    print("OK: main.cpp is now HC-SR04 check.")
    print("Next:")
    print("  cd firmware && pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1")


def switch_to_as5600_check(repo: Path) -> None:
    src_main = repo / "firmware" / "src" / "main.cpp"
    bak_main = repo / "firmware" / "src" / "main.cpp.bak"
    exp_main = repo / "firmware" / "experiments" / "main_as5600_check.cpp"

    if not exp_main.exists():
        raise SystemExit(f"Missing experiment source: {exp_main}")
    if not src_main.exists() and not bak_main.exists():
        raise SystemExit(f"Missing firmware main.cpp: {src_main}")

    _ensure_main_backup(src_main, bak_main)

    print(f"Installing AS5600 check firmware: {exp_main} -> {src_main}")
    shutil.copyfile(exp_main, src_main)

    print("OK: main.cpp is now AS5600 check.")
    print("Next:")
    print("  cd firmware && pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1")


def switch_to_tfmini(repo: Path) -> None:
    src_main = repo / "firmware" / "src" / "main.cpp"
    bak_main = repo / "firmware" / "src" / "main.cpp.bak"
    exp_main = repo / "firmware" / "experiments" / "main_tfmini.cpp.bak"

    if not exp_main.exists():
        raise SystemExit(f"Missing experiment source: {exp_main}")
    if not src_main.exists() and not bak_main.exists():
        raise SystemExit(f"Missing firmware main.cpp: {src_main}")

    _ensure_main_backup(src_main, bak_main)

    print(f"Installing TFMini firmware: {exp_main} -> {src_main}")
    shutil.copyfile(exp_main, src_main)

    print("OK: main.cpp is now TFMini firmware.")
    print("Next:")
    print("  cd firmware && pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1")


def restore_ballbeam(repo: Path) -> None:
    src_main = repo / "firmware" / "src" / "main.cpp"
    bak_main = repo / "firmware" / "src" / "main.cpp.bak"

    if not bak_main.exists():
        raise SystemExit(f"Backup not found: {bak_main}")

    if src_main.exists():
        mode_tag = "custom"
        try:
            content = src_main.read_text(encoding="utf-8", errors="ignore")
            if "HCSR04_CHECK_BOOT" in content or "main_hcsr04_check.cpp" in content:
                mode_tag = "hcsr04"
            elif "main_as5600_check.cpp" in content or "AS5600_CHECK_BOOT" in content:
                mode_tag = "as5600"
            elif "tfmini_sensor.h" in content or "PIN_TFMINI_RX" in content:
                mode_tag = "tfmini"
        except Exception:
            mode_tag = "custom"
        keep = repo / "firmware" / "src" / f"main.cpp.{mode_tag}_{_stamp()}.bak"
        print(f"Keeping current main as: {keep}")
        src_main.rename(keep)

    print(f"Restoring {bak_main} -> {src_main}")
    bak_main.rename(src_main)

    print("OK: main.cpp restored to BallBeam firmware.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Swap firmware/src/main.cpp between BallBeam, AS5600 check, and TFMini variants."
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=["hcsr04_check", "as5600_check", "tfmini", "ballbeam"],
        help="hcsr04_check installs the HC-SR04 sonar diagnostic firmware. "
        "as5600_check installs the AS5600 checker. "
        "tfmini installs the TFMini backup main. "
        "ballbeam restores main.cpp.bak -> main.cpp.",
    )
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    if args.mode == "hcsr04_check":
        switch_to_hcsr04_check(repo)
    elif args.mode == "as5600_check":
        switch_to_as5600_check(repo)
    elif args.mode == "tfmini":
        switch_to_tfmini(repo)
    else:
        restore_ballbeam(repo)


if __name__ == "__main__":
    main()
