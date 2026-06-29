#pragma once

#include <array>

namespace vpi_apriltag {

struct Point2f {
    float x;
    float y;
};

// VPI returns corners counter-clockwise in mathematical image coordinates.
// OpenCV ArUco returns tag-frame corners clockwise starting at top-left:
// TL, TR, BR, BL.
std::array<Point2f, 4> reorder_vpi_to_opencv(const Point2f corners[4]);

}  // namespace vpi_apriltag
