#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from pathlib import Path
import sys

if __package__ in (None, ""):
    workspace_root = Path(__file__).resolve().parents[1]
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))

from analysis.run_layout import (
    SAMPLE_ARTIFACTS,
    legacy_dir_for_run,
    parse_run_artifact_name,
    run_dir_for_stem,
)


def _move(src: Path, dst: Path, dry_run: bool) -> None:
    print(f"{'DRY-RUN ' if dry_run else ''}MOVE {src} -> {dst}")
    if dry_run:
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(src), str(dst))


def _migrate_samples(run_root: Path, samples_dir: Path, dry_run: bool) -> None:
    for name in sorted(SAMPLE_ARTIFACTS):
        src = run_root / name
        if not src.exists():
            continue
        _move(src, samples_dir / name, dry_run=dry_run)


def _target_path(run_root: Path, src: Path) -> Path | None:
    parsed = parse_run_artifact_name(src.name)
    if parsed is None:
        return None
    run_stem, artifact_suffix = parsed
    run_dir = run_dir_for_stem(run_root, run_stem)
    if artifact_suffix == "telemetry.png":
        return legacy_dir_for_run(run_dir) / src.name
    return run_dir / src.name


def main() -> None:
    parser = argparse.ArgumentParser(description="Migrate flat run artifacts into per-run folders")
    parser.add_argument("--run-root", type=Path, default=Path("data/runs"), help="Root run directory")
    parser.add_argument("--samples-dir", type=Path, default=Path("data/samples"), help="Destination for sample artifacts")
    parser.add_argument("--dry-run", action="store_true", help="Print planned moves without mutating files")
    args = parser.parse_args()

    run_root = args.run_root
    samples_dir = args.samples_dir
    if not run_root.exists():
        raise SystemExit(f"Run root not found: {run_root}")

    _migrate_samples(run_root, samples_dir, dry_run=args.dry_run)

    flat_files = sorted(path for path in run_root.iterdir() if path.is_file())
    for src in flat_files:
        if src.name == ".gitkeep" or src.name in SAMPLE_ARTIFACTS:
            continue
        dst = _target_path(run_root, src)
        if dst is None:
            print(f"SKIP {src}")
            continue
        if dst.exists():
            print(f"SKIP {src} (destination exists: {dst})")
            continue
        _move(src, dst, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
