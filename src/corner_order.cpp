#include "corner_order.hpp"

namespace vpi_apriltag {

std::array<Point2f, 4> reorder_vpi_to_opencv(const Point2f corners[4]) {
    // Validated against project-otto synthetic 36h11 images:
    // VPI CCW (math) order maps to OpenCV via {0, 3, 2, 1}.
    return {
        corners[0],
        corners[3],
        corners[2],
        corners[1],
    };
}

}  // namespace vpi_apriltag
