# VPI AprilTag Detector

Standalone pybind11 package that performs image preprocessing and VPI 3.x AprilTag detection for [project-otto](https://gitlab.com/aruw/vision/project-otto).

## Requirements

| Requirement | Value |
|-------------|-------|
| JetPack | **6.1+** (VPI 3.2+ AprilTag detector) |
| VPI branch | **3.x only** |
| System packages | `libnvvpi3`, `vpi3-dev`, `vpi3-samples` |
| Python | **3.10** (`cp310`, `aarch64-linux-gnu`) |
| Tag family | `VPI_APRILTAG_36H11` |
| Hardware | Jetson Orin (PVA recommended) |

Install system dependencies on Jetson:

```bash
sudo apt update
sudo apt install libnvvpi3 vpi3-dev vpi3-samples
```

## Python API

```python
import numpy as np
from vpi_apriltag_detector import AprilTagDetector

detector = AprilTagDetector(
    image_width=960,
    image_height=540,
    marker_ids=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
    max_bits_corrected=1,
    use_pva=True,
)

bgr = np.zeros((540, 960, 3), dtype=np.uint8)
corners, ids = detector.detect(
    bgr,
    roi=None,
    downsize_factor=1.0,
    roi_resize_height=0,
)

# corners: list[np.ndarray], each shape (1, 4, 2) float32 (OpenCV-compatible order)
# ids:     list[int]
```

### ROI convention

`roi=(x0, y0, x1, y1)` uses half-open intervals, matching project-otto:

```python
crop = image[y0:y1, x0:x1]
```

When `roi` is set, provide `roi_resize_height > 0`. `downsize_factor` is ignored on the ROI path.

## Build (native on Jetson)

```bash
python3.10 -m pip install --upgrade pip build scikit-build-core pybind11
python3.10 -m pip install .
```

## Build wheel

```bash
python3.10 -m pip install build
python3.10 -m build --wheel
```

The wheel targets `cp310-cp310-linux_aarch64` and links against the system `libnvvpi3`.

## Cross-compile from x86 host

Install NVIDIA `vpi3-cross-aarch64-l4t`, then:

```bash
export VPI_ROOT=/path/to/vpi3-cross-aarch64-l4t
python3.10 -m pip wheel . --no-deps
```

## Tests

Tests require a built extension and VPI runtime:

```bash
python3.10 -m pip install ".[dev]"
pytest -m vpi
```

Golden-image tests mirror project-otto's ArUco localization tests and validate corner order, ROI remap, downsize remap, and ID filtering.

## Publish to aruw_pypi

On a Jetson CI runner (or maintainer machine):

```bash
python3.10 -m pip install build twine
python3.10 -m build --wheel
twine upload \
  --repository-url "https://gitlab.com/api/v4/groups/9205524/-/packages/pypi" \
  -u gitlab-ci-token \
  -p "${CI_JOB_TOKEN}" \
  dist/*.whl
```

For project-otto integration, add to `pyproject.toml`:

```toml
vpi-apriltag-detector = { version = "0.1.0", source = "aruw_pypi", optional = true, markers = 'platform_system == "Linux" and platform_machine == "aarch64"' }
```

## Pipeline overview

1. BGR input at full resolution
2. Optional ROI crop + fixed-height rescale (`INTER_AREA`-equivalent) **or** optional full-frame downsize
3. Grayscale conversion
4. VPI AprilTag detect (PVA + CPU fallback)
5. Corner remap to full-image coordinates
6. Reorder corners to OpenCV `(TL, TR, BR, BL)` convention

## License

MIT
