"""Shared utilities: angle coordinates, run storage, checkpoints, and metadata."""

from __future__ import annotations

import csv
import datetime as dt
import json
import os
import subprocess
from pathlib import Path
from typing import Any

import numpy as np

DEFAULT_PILOT_MV = 159.2
DEFAULT_STANDARD_WINDOW_SAMPLES = 1280
DEFAULT_SAMPLE_RATE_HZ = 64000.0
DEFAULT_WAIT_BLOCKS = 3
DAC_MIN_V = -10.0
DAC_MAX_V = 10.0


def repo_root() -> Path:
    """Return the repository root containing scripts/experiments/."""
    return Path(__file__).resolve().parents[2]


def exp_runs_root(root: Path | None = None) -> Path:
    """Return the workspace-local experiment run directory under .agent-bridge/."""
    base = repo_root() if root is None else Path(root)
    return base / ".agent-bridge" / "exp_runs"


def wrap_rad(angle: float | np.ndarray) -> float | np.ndarray:
    """Wrap radians into (-pi, pi] using exact complex unit-circle projection."""
    arr = np.asarray(angle, dtype=float)
    if np.any(~np.isfinite(arr)):
        raise ValueError("Cannot wrap non-finite angle value")
    wrapped = np.angle(np.exp(1j * arr))
    # Map -pi strictly to +pi to preserve (-pi, pi] half-open interval.
    wrapped = np.where(np.isclose(wrapped, -np.pi, atol=1e-15), np.pi, wrapped)
    if np.ndim(angle) == 0:
        return float(wrapped)
    return wrapped


def wrap_deg(angle_deg: float | np.ndarray) -> float | np.ndarray:
    """Wrap degrees into (-180, 180]."""
    res = np.degrees(wrap_rad(np.radians(angle_deg)))
    if np.ndim(angle_deg) == 0:
        return float(res)
    return res


def fw_to_thesis_rad(phi_fw: float | np.ndarray) -> float | np.ndarray:
    """Convert firmware dark-null phase (MIN=0, QUAD=pi/2) to thesis cosine phase (MATP=0)."""
    return wrap_rad(np.asarray(phi_fw, dtype=float) + np.pi)


def thesis_to_fw_rad(phi_thesis: float | np.ndarray) -> float | np.ndarray:
    """Convert thesis cosine phase (MATP=0, QTP+=pi/2) to firmware dark-null phase (MIN=0)."""
    return wrap_rad(np.asarray(phi_thesis, dtype=float) - np.pi)


def fw_to_thesis_deg(phi_fw_deg: float | np.ndarray) -> float | np.ndarray:
    """Convert firmware phase in degrees to thesis cosine phase in (-180, 180]."""
    return wrap_deg(np.asarray(phi_fw_deg, dtype=float) + 180.0)


def thesis_to_fw_deg(phi_thesis_deg: float | np.ndarray) -> float | np.ndarray:
    """Convert thesis cosine phase in degrees to firmware phase in (-180, 180]."""
    return wrap_deg(np.asarray(phi_thesis_deg, dtype=float) - 180.0)


def utc_timestamp_tag() -> str:
    """Return a filesystem-safe UTC timestamp string."""
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def capture_metadata(
    experiment: str,
    *,
    board_id: str = "20261110-MZM",
    fw_info: dict[str, Any] | None = None,
    laser_power_dbm: float | None = None,
    ambient_temp_c: float | None = None,
    rf_freq_hz: float | None = None,
    dry_run: bool = False,
    notes: str = "",
    root: Path | None = None,
) -> dict[str, Any]:
    """Collect reproducible session metadata for raw archives and reports."""
    project_root = repo_root() if root is None else Path(root)
    try:
        git_rev = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=project_root,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (subprocess.SubprocessError, OSError):
        git_rev = "unknown"

    return {
        "experiment": experiment,
        "timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "git_rev": git_rev,
        "board_id": board_id,
        "dry_run": bool(dry_run),
        "sample_rate_hz": DEFAULT_SAMPLE_RATE_HZ,
        "window_samples": DEFAULT_STANDARD_WINDOW_SAMPLES,
        "window_duration_s": DEFAULT_STANDARD_WINDOW_SAMPLES / DEFAULT_SAMPLE_RATE_HZ,
        "coordinate_convention": {
            "thesis": "cosine: MATP=0 deg, QTP+=+90 deg (falling DC), MITP=180 deg, QTP-=-90 deg (rising DC)",
            "firmware": "dark-null: MIN=0 deg, QUAD=+90 deg (rising DC), MAX=180 deg",
            "conversion": "phi_thesis = wrap(phi_fw + pi), phi_fw = wrap(phi_thesis - pi)",
        },
        "fw_info": fw_info or {},
        "laser_power_dbm": laser_power_dbm,
        "ambient_temp_c": ambient_temp_c,
        "rf_freq_hz": rf_freq_hz,
        "notes": notes,
    }


def atomic_write_json(path: Path, payload: Any) -> None:
    """Atomically write JSON inside the workspace without using system temp dirs."""
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(f"{path.name}.part.{os.getpid()}")
    part.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(part, path)


def append_csv_rows(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    """Append structured rows to a CSV file, writing header only when newly created."""
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        writer.writerows(rows)


class RunSession:
    """Manages resumable checkpointed storage under .agent-bridge/exp_runs/."""

    def __init__(
        self,
        experiment: str,
        *,
        run_dir: Path | None = None,
        resume: bool = False,
        root: Path | None = None,
    ) -> None:
        self.experiment = experiment
        if run_dir is None:
            base = exp_runs_root(root)
            self.run_dir = base / f"{experiment}_{utc_timestamp_tag()}"
        else:
            self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.checkpoint_path = self.run_dir / "checkpoint.json"
        self.metadata_path = self.run_dir / "metadata.json"

        if resume and self.checkpoint_path.exists():
            raw = json.loads(self.checkpoint_path.read_text(encoding="utf-8"))
            if not isinstance(raw, dict):
                raise ValueError(f"Corrupt checkpoint payload in {self.checkpoint_path}")
            self.state: dict[str, Any] = raw
        else:
            self.state = {"experiment": experiment, "completed": [], "artifacts": {}, "data": {}}
            if not resume:
                self.save_checkpoint()

    def write_metadata(self, metadata: dict[str, Any]) -> None:
        atomic_write_json(self.metadata_path, metadata)

    def is_done(self, key: str) -> bool:
        return key in self.state["completed"]

    def mark_done(self, key: str, value: Any | None = None) -> None:
        if key not in self.state["completed"]:
            self.state["completed"].append(key)
        if value is not None:
            self.state["data"][key] = value
        self.save_checkpoint()

    def set_data(self, key: str, value: Any) -> None:
        self.state["data"][key] = value
        self.save_checkpoint()

    def get_data(self, key: str, default: Any = None) -> Any:
        return self.state["data"].get(key, default)

    def save_checkpoint(self) -> None:
        atomic_write_json(self.checkpoint_path, self.state)

    def write_json(self, rel_name: str, payload: Any) -> Path:
        target = self.run_dir / rel_name
        atomic_write_json(target, payload)
        self.state["artifacts"][rel_name] = str(target)
        self.save_checkpoint()
        return target

    def append_csv(self, rel_name: str, fieldnames: list[str], rows: list[dict[str, Any]]) -> Path:
        target = self.run_dir / rel_name
        append_csv_rows(target, fieldnames, rows)
        self.state["artifacts"][rel_name] = str(target)
        return target
