"""Bridge between the animal-experiment UI and the trajectory toolkit."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
TOOL_ROOT = HERE / "TrajectoryReconstruction"
TOOL_SRC = TOOL_ROOT / "src"
DEFAULT_OUTPUT_DIR = HERE / "trajectory_outputs"


def _load_toolkit():
    if not TOOL_SRC.is_dir():
        raise RuntimeError(f"Trajectory toolkit is missing: {TOOL_SRC}")
    source = str(TOOL_SRC)
    if source not in sys.path:
        sys.path.insert(0, source)

    from broncho_trajectory.io import (  # type: ignore[import-not-found]
        load_record,
        write_trajectory_csv,
        write_trajectory_json,
    )
    from broncho_trajectory.kinematics import (  # type: ignore[import-not-found]
        KinematicConfig,
        reconstruct,
    )
    from broncho_trajectory.visualization import trajectory_html  # type: ignore[import-not-found]

    return load_record, write_trajectory_csv, write_trajectory_json, KinematicConfig, reconstruct, trajectory_html


def reconstruct_file(
    input_json: str | Path,
    output_dir: str | Path | None = None,
    **calibration: Any,
) -> dict[str, Any]:
    """Reconstruct one BC JSON recording and write JSON, CSV and 3D HTML."""
    source_path = Path(input_json).resolve()
    if not source_path.is_file():
        raise FileNotFoundError(f"Trajectory recording not found: {source_path}")

    (
        load_record,
        write_trajectory_csv,
        write_trajectory_json,
        KinematicConfig,
        reconstruct,
        trajectory_html,
    ) = _load_toolkit()

    record = load_record(source_path)
    config = KinematicConfig(**calibration)
    points = reconstruct(record["frames"], config)
    destination = Path(output_dir).resolve() if output_dir else DEFAULT_OUTPUT_DIR
    destination.mkdir(parents=True, exist_ok=True)

    stem = source_path.stem
    json_path = destination / f"{stem}_trajectory.json"
    csv_path = destination / f"{stem}_trajectory.csv"
    html_path = destination / f"{stem}_trajectory.html"
    metadata = {
        "source": str(source_path),
        "session_name": record.get("session_name"),
        "path_name": record.get("path_name"),
        "config": vars(config),
        "warning": "Encoder-based kinematic estimate only; not a true spatial localization measurement.",
    }

    write_trajectory_json(json_path, points, metadata)
    write_trajectory_csv(csv_path, points)
    display_name = record.get("path_name") or stem
    html_path.write_text(
        trajectory_html(points, f"Bronchoscope 3D trajectory - {display_name}"),
        encoding="utf-8",
    )
    return {
        "source": str(source_path),
        "frames": len(points),
        "json": str(json_path),
        "csv": str(csv_path),
        "html": str(html_path),
    }

