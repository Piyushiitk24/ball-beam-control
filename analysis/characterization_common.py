from __future__ import annotations

import json
import math
import re
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

POINT_LABEL_RE = re.compile(r"^(?P<direction>out|ret)_p(?P<index>\d+)$")


@dataclass
class CharacterizationMetadata:
    run_stem: str
    capture_kind: str
    sensor_name: str
    sensor_mode: str
    target_name: str
    block_label: str
    repeat_index: int
    timer1_stress: bool
    mount_note: str
    photo_ref: str
    notes: str
    created_at_iso: str
    samples_per_point: int
    label_scheme: str
    raw_log_name: str
    points_csv_name: str
    samples_csv_name: str


def stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def iso_now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def mean(values: list[float]) -> float:
    return sum(values) / float(len(values))


def stddev(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    mu = mean(values)
    var = sum((value - mu) ** 2 for value in values) / float(len(values) - 1)
    return math.sqrt(var)


def parse_point_label(label: str) -> tuple[str, int] | None:
    match = POINT_LABEL_RE.fullmatch(label.strip())
    if match is None:
        return None
    return match.group("direction"), int(match.group("index"))


def default_point_label(direction: str, point_number: int) -> str:
    return f"{direction}_p{point_number:02d}"


def save_metadata(path: Path, metadata: CharacterizationMetadata) -> None:
    path.write_text(json.dumps(asdict(metadata), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_metadata(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))
