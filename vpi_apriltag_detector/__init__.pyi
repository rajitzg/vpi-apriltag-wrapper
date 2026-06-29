from __future__ import annotations

from typing import overload

import numpy as np
import numpy.typing as npt

class AprilTagDetector:
    def __init__(
        self,
        image_width: int,
        image_height: int,
        marker_ids: list[int],
        max_bits_corrected: int = 1,
        use_pva: bool = True,
    ) -> None: ...

    @overload
    def detect(
        self,
        bgr: npt.NDArray[np.uint8],
        *,
        roi: None = None,
        downsize_factor: float = 1.0,
        roi_resize_height: int = 0,
    ) -> tuple[list[npt.NDArray[np.float32]], list[int]]: ...

    @overload
    def detect(
        self,
        bgr: npt.NDArray[np.uint8],
        *,
        roi: tuple[int, int, int, int],
        downsize_factor: float = 1.0,
        roi_resize_height: int = 0,
    ) -> tuple[list[npt.NDArray[np.float32]], list[int]]: ...

    def detect(
        self,
        bgr: npt.NDArray[np.uint8],
        *,
        roi: tuple[int, int, int, int] | None = None,
        downsize_factor: float = 1.0,
        roi_resize_height: int = 0,
    ) -> tuple[list[npt.NDArray[np.float32]], list[int]]: ...
