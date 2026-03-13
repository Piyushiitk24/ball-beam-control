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


def _keep_current_main(src_main: Path, tag: str, repo: Path) -> None:
    if not src_main.exists():
        return
    keep = repo / "firmware" / "src" / f"main.cpp.{tag}_{_stamp()}.bak"
    print(f"Keeping current main as: {keep}")
    src_main.rename(keep)


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


def switch_to_sharp_ir_check(repo: Path) -> None:
    src_main = repo / "firmware" / "src" / "main.cpp"
    bak_main = repo / "firmware" / "src" / "main.cpp.bak"
    exp_main = repo / "firmware" / "experiments" / "main_sharp_ir_check.cpp"

    if not exp_main.exists():
        raise SystemExit(f"Missing experiment source: {exp_main}")
    if not src_main.exists() and not bak_main.exists():
        raise SystemExit(f"Missing firmware main.cpp: {src_main}")

    _ensure_main_backup(src_main, bak_main)

    print(f"Installing Sharp IR check firmware: {exp_main} -> {src_main}")
    shutil.copyfile(exp_main, src_main)

    print("OK: main.cpp is now Sharp IR check.")
    print("Next:")
    print("  cd firmware && pio run -e sharp_ir_check -t upload --upload-port /dev/cu.usbserial-A10N20X1")


def switch_to_hcsr04_runtime(repo: Path) -> None:
    src_main = repo / "firmware" / "src" / "main.cpp"
    backup_root = repo / "firmware" / "experiments" / "backup" / "hcsr04_runtime"
    archived_main = backup_root / "src" / "main.cpp"
    archived_header = backup_root / "include" / "sensors" / "hcsr04_sensor.h"
    archived_source = backup_root / "src" / "sensors" / "hcsr04_sensor.cpp"
    live_header = repo / "firmware" / "include" / "sensors" / "hcsr04_sensor.h"
    live_source = repo / "firmware" / "src" / "sensors" / "hcsr04_sensor.cpp"

    for path in (archived_main, archived_header, archived_source):
        if not path.exists():
            raise SystemExit(f"Missing archived HC-SR04 runtime file: {path}")

    _keep_current_main(src_main, "sharp_runtime", repo)

    print(f"Restoring archived HC-SR04 runtime main: {archived_main} -> {src_main}")
    shutil.copyfile(archived_main, src_main)
    print(f"Restoring archived HC-SR04 runtime header: {archived_header} -> {live_header}")
    shutil.copyfile(archived_header, live_header)
    print(f"Restoring archived HC-SR04 runtime source: {archived_source} -> {live_source}")
    shutil.copyfile(archived_source, live_source)

    print("OK: main.cpp and HC-SR04 runtime files restored from archive.")
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
            elif "SHARP_IR_CHECK_BOOT" in content or "main_sharp_ir_check.cpp" in content:
                mode_tag = "sharp_ir"
            elif "tfmini_sensor.h" in content or "PIN_TFMINI_RX" in content:
                mode_tag = "tfmini"
        except Exception:
            mode_tag = "custom"
        _keep_current_main(src_main, mode_tag, repo)

    print(f"Restoring {bak_main} -> {src_main}")
    bak_main.rename(src_main)

    print("OK: main.cpp restored to BallBeam firmware.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Swap firmware/src/main.cpp between the live BallBeam firmware, diagnostics, and the archived HC-SR04 runtime."
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=["hcsr04_check", "hcsr04_runtime", "as5600_check", "tfmini", "sharp_ir_check", "ballbeam"],
        help="hcsr04_check installs the HC-SR04 sonar diagnostic firmware. "
        "hcsr04_runtime restores the archived pre-Sharp HC-SR04 runtime bundle. "
        "as5600_check installs the AS5600 checker. "
        "tfmini installs the TFMini backup main (legacy). "
        "sharp_ir_check installs the Sharp IR diagnostic firmware. "
        "ballbeam restores main.cpp.bak -> main.cpp.",
    )
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    if args.mode == "hcsr04_check":
        switch_to_hcsr04_check(repo)
    elif args.mode == "hcsr04_runtime":
        switch_to_hcsr04_runtime(repo)
    elif args.mode == "as5600_check":
        switch_to_as5600_check(repo)
    elif args.mode == "sharp_ir_check":
        switch_to_sharp_ir_check(repo)
    elif args.mode == "tfmini":
        switch_to_tfmini(repo)
    else:
        restore_ballbeam(repo)


if __name__ == "__main__":
    main()
