"""VPI 3.x AprilTag detector for project-otto."""

from __future__ import annotations

import importlib.util
import site
import sys
from pathlib import Path
from types import ModuleType


def _load_extension_from_site_packages() -> ModuleType:
    patterns = ("_vpi_apriltag_detector*.so", "_vpi_apriltag_detector*.pyd")
    candidates: list[Path] = []

    for base in site.getsitepackages():
        pkg_dir = Path(base) / "vpi_apriltag_detector"
        if not pkg_dir.is_dir():
            continue
        for pattern in patterns:
            candidates.extend(pkg_dir.glob(pattern))

    user_site = site.getusersitepackages()
    if user_site:
        user_pkg_dir = Path(user_site) / "vpi_apriltag_detector"
        if user_pkg_dir.is_dir():
            for pattern in patterns:
                candidates.extend(user_pkg_dir.glob(pattern))

    if not candidates:
        raise ModuleNotFoundError(
            "No compiled vpi_apriltag_detector extension found. "
            "Build/install on JetPack 6.1+ with: python -m pip install ."
        )

    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    ext_path = candidates[0]

    module_name = "vpi_apriltag_detector._vpi_apriltag_detector"
    spec = importlib.util.spec_from_file_location(module_name, ext_path)
    if spec is None or spec.loader is None:
        raise ModuleNotFoundError(f"Unable to load extension module from {ext_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


try:
    from ._vpi_apriltag_detector import AprilTagDetector  # pyright: ignore[reportMissingModuleSource]
except ModuleNotFoundError as exc:
    if exc.name != "vpi_apriltag_detector._vpi_apriltag_detector":
        raise
    AprilTagDetector = _load_extension_from_site_packages().AprilTagDetector

__all__ = ["AprilTagDetector"]
