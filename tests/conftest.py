from __future__ import annotations

import pytest


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line(
        "markers",
        "vpi: tests that require the compiled VPI extension on aarch64 Linux",
    )


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    try:
        import vpi_apriltag_detector._vpi_apriltag_detector  # noqa: F401
    except ImportError:
        skip_vpi = pytest.mark.skip(reason="VPI extension not installed")
        for item in items:
            if "vpi" in item.keywords:
                item.add_marker(skip_vpi)
