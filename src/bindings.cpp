#include "detector.hpp"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace {

py::array_t<float> corners_to_numpy(const std::array<float, 8>& flat_corners) {
    py::array_t<float> corners({1, 4, 2});
    auto mutable_corners = corners.mutable_unchecked<3>();
    for (py::ssize_t i = 0; i < 4; ++i) {
        mutable_corners(0, i, 0) = flat_corners[static_cast<size_t>(i) * 2];
        mutable_corners(0, i, 1) = flat_corners[static_cast<size_t>(i) * 2 + 1];
    }
    return corners;
}

const uint8_t* validate_bgr_input(const py::array& bgr, int expected_width, int expected_height) {
    if (bgr.ndim() != 3) {
        throw std::invalid_argument("bgr must have shape (H, W, 3)");
    }
    if (bgr.shape(2) != 3) {
        throw std::invalid_argument("bgr must have 3 channels");
    }
    if (!py::isinstance<py::array_t<uint8_t>>(bgr)) {
        throw std::invalid_argument("bgr must have dtype uint8");
    }
    if (bgr.shape(0) != expected_height || bgr.shape(1) != expected_width) {
        throw std::invalid_argument("bgr shape must match constructor image dimensions");
    }
    if (!(bgr.flags() & py::array::c_style)) {
        throw std::invalid_argument("bgr must be C-contiguous");
    }
    return static_cast<const py::array_t<uint8_t>>(bgr).data();
}

std::optional<std::tuple<int, int, int, int>> parse_roi(const py::object& roi) {
    if (roi.is_none()) {
        return std::nullopt;
    }
    const py::tuple value = roi.cast<py::tuple>();
    if (value.size() != 4) {
        throw std::invalid_argument("roi must be a 4-tuple (x0, y0, x1, y1)");
    }
    return std::make_tuple(
        value[0].cast<int>(),
        value[1].cast<int>(),
        value[2].cast<int>(),
        value[3].cast<int>());
}

}  // namespace

PYBIND11_MODULE(_vpi_apriltag_detector, m) {
    m.doc() = "VPI 3.x AprilTag detector with preprocessing";

    py::class_<vpi_apriltag::AprilTagDetector>(m, "AprilTagDetector")
        .def(
            py::init<int, int, const std::vector<int32_t>&, int, bool>(),
            py::arg("image_width"),
            py::arg("image_height"),
            py::arg("marker_ids"),
            py::arg("max_bits_corrected") = 1,
            py::arg("use_pva") = true,
            R"doc(
Create an AprilTag detector configured for a fixed full-frame resolution.

Args:
    image_width: Full-frame image width in pixels.
    image_height: Full-frame image height in pixels.
    marker_ids: Tag IDs to detect (empty list disables ID filtering).
    max_bits_corrected: VPI maxBitsCorrected decode parameter.
    use_pva: When True, use PVA with CPU fallback; otherwise CPU only.
)doc")
        .def(
            "detect",
            [](vpi_apriltag::AprilTagDetector& self,
               const py::array& bgr,
               const py::object& roi,
               float downsize_factor,
               int roi_resize_height) {
                const uint8_t* data = validate_bgr_input(bgr, self.image_width(), self.image_height());
                const vpi_apriltag::DetectionResult detection = self.detect(
                    data,
                    parse_roi(roi),
                    downsize_factor,
                    roi_resize_height);

                std::vector<py::array_t<float>> corner_sets;
                corner_sets.reserve(detection.corners.size());
                for (const auto& corners : detection.corners) {
                    corner_sets.push_back(corners_to_numpy(corners));
                }

                std::vector<int> ids(detection.ids.begin(), detection.ids.end());
                return py::make_tuple(corner_sets, ids);
            },
            py::arg("bgr"),
            py::kw_only(),
            py::arg("roi") = py::none(),
            py::arg("downsize_factor") = 1.0f,
            py::arg("roi_resize_height") = 0,
            R"doc(
Detect AprilTag markers in a BGR image.

Args:
    bgr: uint8 array with shape (H, W, 3), matching constructor dimensions.
    roi: Optional (x0, y0, x1, y1) crop bounds using half-open intervals.
    downsize_factor: Full-frame resize factor applied before detection (ignored when roi is set).
    roi_resize_height: Target height for ROI rescale (required when roi is set).

Returns:
    corners: list of float32 arrays with shape (1, 4, 2) in OpenCV corner order.
    ids: list of detected marker IDs.
)doc");
}
