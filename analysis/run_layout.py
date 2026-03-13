from __future__ import annotations

import os
import re
from pathlib import Path

RUN_STEM_RE = re.compile(r"^(run_\d{8}_\d{6})$")
RUN_ARTIFACT_RE = re.compile(r"^(run_\d{8}_\d{6})_(.+)$")

SAMPLE_ARTIFACTS = {
    "sample_clean.csv",
    "sample_compare.png",
    "sample_plot.png",
}


def is_run_stem(name: str) -> bool:
    return RUN_STEM_RE.fullmatch(name) is not None


def parse_run_artifact_name(name: str) -> tuple[str, str] | None:
    match = RUN_ARTIFACT_RE.fullmatch(name)
    if match is None:
        return None
    return match.group(1), match.group(2)


def derive_run_stem(path: Path) -> str:
    parsed = parse_run_artifact_name(path.name)
    if parsed is not None:
        return parsed[0]

    name = path.name
    for suffix in ("_telemetry.csv", "_clean.csv", "_raw.log", "_events.txt", "_metrics.csv", ".png"):
        if name.endswith(suffix):
            stem = name[: -len(suffix)]
            if stem:
                return stem
    return path.stem


def resolve_run_input(input_path: Path) -> tuple[Path, Path, str]:
    if input_path.is_dir():
        candidates = sorted(input_path.glob("run_*_telemetry.csv"))
        if len(candidates) != 1:
            raise SystemExit(
                f"Expected exactly one run telemetry CSV in {input_path}, found {len(candidates)}"
            )
        telemetry_path = candidates[0]
        return telemetry_path, input_path, derive_run_stem(telemetry_path)

    telemetry_path = input_path
    run_stem = derive_run_stem(telemetry_path)
    run_dir = telemetry_path.parent
    if run_stem and is_run_stem(run_dir.name):
        return telemetry_path, run_dir, run_stem
    if run_stem and is_run_stem(run_stem):
        return telemetry_path, telemetry_path.parent / run_stem, run_stem
    return telemetry_path, telemetry_path.parent, run_stem


def run_dir_for_stem(root: Path, run_stem: str) -> Path:
    return root / run_stem


def plots_dir_for_run(run_dir: Path) -> Path:
    return run_dir / "plots"


def legacy_dir_for_run(run_dir: Path) -> Path:
    return run_dir / "legacy"


def summary_plot_path(run_dir: Path, run_stem: str) -> Path:
    return plots_dir_for_run(run_dir) / f"{run_stem}_summary.png"


def metrics_path(run_dir: Path, run_stem: str) -> Path:
    return run_dir / f"{run_stem}_metrics.csv"


def characterization_meta_path(run_dir: Path, run_stem: str) -> Path:
    return run_dir / f"{run_stem}_characterization_meta.json"


def events_path(run_dir: Path, run_stem: str) -> Path:
    return run_dir / f"{run_stem}_events.txt"


def raw_log_path(run_dir: Path, run_stem: str) -> Path:
    return run_dir / f"{run_stem}_raw.log"


def telemetry_path(run_dir: Path, run_stem: str) -> Path:
    return run_dir / f"{run_stem}_telemetry.csv"


def iter_run_csvs(root: Path) -> list[Path]:
    candidates = sorted(root.glob("**/run_*_telemetry.csv"))
    return [path for path in candidates if path.name not in SAMPLE_ARTIFACTS]


def find_run_csv_by_stem(root: Path, run_stem: str) -> Path | None:
    matches = sorted(root.glob(f"**/{run_stem}_telemetry.csv"))
    if not matches:
        return None
    if len(matches) > 1:
        raise SystemExit(f"Multiple telemetry CSVs found for {run_stem} under {root}")
    return matches[0]


def relative_path(from_dir: Path, to_path: Path) -> Path:
    return Path(os.path.relpath(to_path, from_dir))
