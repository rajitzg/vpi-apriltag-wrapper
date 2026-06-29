#include "corner_order.hpp"

#include <array>
#include <cmath>
#include <iostream>

namespace {

bool nearly_equal(float a, float b, float tol = 1e-3f) {
    return std::fabs(a - b) <= tol;
}

bool check(
    const char* name,
    const vpi_apriltag::Point2f input[4],
    const vpi_apriltag::Point2f expected[4]) {
    const auto actual = vpi_apriltag::reorder_vpi_to_opencv(input);
    for (size_t i = 0; i < 4; ++i) {
        if (!nearly_equal(actual[i].x, expected[i].x) || !nearly_equal(actual[i].y, expected[i].y)) {
            std::cerr << name << " failed at corner " << i << '\n';
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    const vpi_apriltag::Point2f upright_vpi[4] = {
        {50.0f, 50.0f},
        {50.0f, 250.0f},
        {250.0f, 250.0f},
        {250.0f, 50.0f},
    };
    const vpi_apriltag::Point2f upright_opencv[4] = {
        {50.0f, 50.0f},
        {250.0f, 50.0f},
        {250.0f, 250.0f},
        {50.0f, 250.0f},
    };

    const vpi_apriltag::Point2f sideways_vpi[4] = {
        {250.0f, 50.0f},
        {50.0f, 50.0f},
        {50.0f, 250.0f},
        {250.0f, 250.0f},
    };
    const vpi_apriltag::Point2f sideways_opencv[4] = {
        {250.0f, 50.0f},
        {250.0f, 250.0f},
        {50.0f, 250.0f},
        {50.0f, 50.0f},
    };

    const bool ok = check("upright", upright_vpi, upright_opencv) &&
                    check("sideways", sideways_vpi, sideways_opencv);
    return ok ? 0 : 1;
}
