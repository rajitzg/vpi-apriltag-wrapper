#pragma once

#include <cstdint>
#include <tuple>
#include <vector>

namespace vpi_apriltag {

struct RemapParams {
    float scale = 1.0f;
    float offset_x = 0.0f;
    float offset_y = 0.0f;
};

struct PreprocessResult {
    std::vector<uint8_t> gray;
    int width = 0;
    int height = 0;
    RemapParams remap;
};

PreprocessResult preprocess_bgr(
    const uint8_t* bgr,
    int image_width,
    int image_height,
    int roi_x0,
    int roi_y0,
    int roi_x1,
    int roi_y1,
    bool use_roi,
    float downsize_factor,
    int roi_resize_height);

}  // namespace vpi_apriltag
