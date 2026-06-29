from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from PIL import Image

pytestmark = pytest.mark.vpi

TEST_IMG_DIR = Path(__file__).parent / "test_aruco_localization_source_imgs"
MARKER_IDS = [0, 1, 2]


def _require_vpi() -> None:
    pytest.importorskip("vpi_apriltag_detector._vpi_apriltag_detector")


def _load_bgr(name: str) -> np.ndarray:
    img = Image.open(TEST_IMG_DIR / name).convert("RGB")
    return np.asarray(img, dtype=np.uint8)[:, :, ::-1].copy()


@pytest.fixture(scope="module")
def detector() -> object:
    _require_vpi()
    from vpi_apriltag_detector import AprilTagDetector

    image = _load_bgr("36h11_marker_0.png")
    return AprilTagDetector(
        image.shape[1],
        image.shape[0],
        MARKER_IDS,
        max_bits_corrected=1,
        use_pva=True,
    )


def test_detect_empty_image(detector: object) -> None:
    template = _load_bgr("36h11_marker_0.png")
    image = np.zeros_like(template)
    corners, ids = detector.detect(image)  # type: ignore[attr-defined]
    assert corners == []
    assert ids == []


def test_detect_marker_0_upright(detector: object) -> None:
    image = _load_bgr("36h11_marker_0.png")
    expected = np.array([[50, 50], [250, 50], [250, 250], [50, 250]], dtype=np.float32)

    corners, ids = detector.detect(image)  # type: ignore[attr-defined]

    assert len(corners) == 1
    assert ids == [0]
    assert np.allclose(corners[0], expected, atol=5)


def test_detect_marker_0_sideways(detector: object) -> None:
    image = _load_bgr("36h11_marker_0_sideways.png")
    expected = np.array([[250, 50], [250, 250], [50, 250], [50, 50]], dtype=np.float32)

    corners, ids = detector.detect(image)  # type: ignore[attr-defined]

    assert len(corners) == 1
    assert ids == [0]
    assert np.allclose(corners[0], expected, atol=5)


def test_detect_two_markers(detector: object) -> None:
    image = _load_bgr("36h11_marker_0_and_1.png")

    corners, ids = detector.detect(image)  # type: ignore[attr-defined]

    assert len(corners) == 2
    assert 0 in ids
    assert 1 in ids


def test_detect_id_filter_excludes_unknown_marker() -> None:
    _require_vpi()
    from vpi_apriltag_detector import AprilTagDetector

    image = _load_bgr("36h11_marker_50.png")
    detector = AprilTagDetector(image.shape[1], image.shape[0], MARKER_IDS)

    corners, ids = detector.detect(image)

    assert corners == []
    assert ids == []


def test_detect_roi_path(detector: object) -> None:
    image = _load_bgr("36h11_marker_0.png")
    expected = np.array([[50, 50], [250, 50], [250, 250], [50, 250]], dtype=np.float32)

    corners, ids = detector.detect(  # type: ignore[attr-defined]
        image,
        roi=(25, 25, 275, 275),
        roi_resize_height=120,
    )

    assert len(corners) == 1
    assert ids == [0]
    assert np.allclose(corners[0], expected, atol=5)


def test_detect_downsize_path(detector: object) -> None:
    image = _load_bgr("36h11_marker_0.png")
    expected = np.array([[50, 50], [250, 50], [250, 250], [50, 250]], dtype=np.float32)

    corners, ids = detector.detect(image, downsize_factor=0.5)  # type: ignore[attr-defined]

    assert len(corners) == 1
    assert ids == [0]
    assert np.allclose(corners[0], expected, atol=5)


def test_detect_pva_smoke(detector: object) -> None:
    image = _load_bgr("36h11_marker_0.png")
    corners, ids = detector.detect(image)  # type: ignore[attr-defined]
    assert len(corners) == 1
    assert ids == [0]
