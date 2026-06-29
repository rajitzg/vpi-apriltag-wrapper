#include "preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vpi_apriltag {

namespace {

inline uint8_t bgr_to_gray(uint8_t b, uint8_t g, uint8_t r) {
    // Match OpenCV BGR2GRAY weights.
    const int value = static_cast<int>(0.114 * b + 0.587 * g + 0.299 * r + 0.5);
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

void convert_bgr_to_gray(
    const uint8_t* bgr,
    int width,
    int height,
    std::vector<uint8_t>& gray) {
    gray.resize(static_cast<size_t>(width) * static_cast<size_t>(height));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
            const size_t bgr_idx = idx * 3;
            gray[idx] = bgr_to_gray(bgr[bgr_idx], bgr[bgr_idx + 1], bgr[bgr_idx + 2]);
        }
    }
}

void crop_bgr(
    const uint8_t* bgr,
    int image_width,
    int image_height,
    int x0,
    int y0,
    int x1,
    int y1,
    std::vector<uint8_t>& cropped,
    int& crop_width,
    int& crop_height) {
    crop_width = x1 - x0;
    crop_height = y1 - y0;
    cropped.resize(static_cast<size_t>(crop_width) * static_cast<size_t>(crop_height) * 3);

    for (int y = 0; y < crop_height; ++y) {
        const int src_y = y0 + y;
        for (int x = 0; x < crop_width; ++x) {
            const int src_x = x0 + x;
            const size_t dst_idx =
                (static_cast<size_t>(y) * static_cast<size_t>(crop_width) + static_cast<size_t>(x)) * 3;
            const size_t src_idx =
                (static_cast<size_t>(src_y) * static_cast<size_t>(image_width) + static_cast<size_t>(src_x)) * 3;
            cropped[dst_idx] = bgr[src_idx];
            cropped[dst_idx + 1] = bgr[src_idx + 1];
            cropped[dst_idx + 2] = bgr[src_idx + 2];
        }
    }
}

float sample_bilinear_channel(
    const uint8_t* src,
    int width,
    int height,
    float x,
    float y,
    int channel) {
    x = std::clamp(x, 0.0f, static_cast<float>(width - 1));
    y = std::clamp(y, 0.0f, static_cast<float>(height - 1));

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);

    const float dx = x - static_cast<float>(x0);
    const float dy = y - static_cast<float>(y0);

    auto at = [&](int px, int py) -> float {
        const size_t idx =
            (static_cast<size_t>(py) * static_cast<size_t>(width) + static_cast<size_t>(px)) * 3 +
            static_cast<size_t>(channel);
        return static_cast<float>(src[idx]);
    };

    const float v00 = at(x0, y0);
    const float v10 = at(x1, y0);
    const float v01 = at(x0, y1);
    const float v11 = at(x1, y1);

    const float v0 = v00 + dx * (v10 - v00);
    const float v1 = v01 + dx * (v11 - v01);
    return v0 + dy * (v1 - v0);
}

void resize_bgr_bilinear(
    const uint8_t* src,
    int src_w,
    int src_h,
    int dst_w,
    int dst_h,
    std::vector<uint8_t>& dst) {
    dst.resize(static_cast<size_t>(dst_w) * static_cast<size_t>(dst_h) * 3);
    if (src_w == dst_w && src_h == dst_h) {
        dst.assign(src, src + static_cast<size_t>(src_w) * static_cast<size_t>(src_h) * 3);
        return;
    }

    const float scale_x = static_cast<float>(src_w) / static_cast<float>(dst_w);
    const float scale_y = static_cast<float>(src_h) / static_cast<float>(dst_h);

    for (int y = 0; y < dst_h; ++y) {
        const float src_y = (static_cast<float>(y) + 0.5f) * scale_y - 0.5f;
        for (int x = 0; x < dst_w; ++x) {
            const float src_x = (static_cast<float>(x) + 0.5f) * scale_x - 0.5f;
            const size_t dst_idx =
                (static_cast<size_t>(y) * static_cast<size_t>(dst_w) + static_cast<size_t>(x)) * 3;
            dst[dst_idx] = static_cast<uint8_t>(
                sample_bilinear_channel(src, src_w, src_h, src_x, src_y, 0) + 0.5f);
            dst[dst_idx + 1] = static_cast<uint8_t>(
                sample_bilinear_channel(src, src_w, src_h, src_x, src_y, 1) + 0.5f);
            dst[dst_idx + 2] = static_cast<uint8_t>(
                sample_bilinear_channel(src, src_w, src_h, src_x, src_y, 2) + 0.5f);
        }
    }
}

void resize_bgr_area(
    const uint8_t* src,
    int src_w,
    int src_h,
    int dst_w,
    int dst_h,
    std::vector<uint8_t>& dst) {
    dst.resize(static_cast<size_t>(dst_w) * static_cast<size_t>(dst_h) * 3);
    if (src_w == dst_w && src_h == dst_h) {
        dst.assign(src, src + static_cast<size_t>(src_w) * static_cast<size_t>(src_h) * 3);
        return;
    }

    const float scale_x = static_cast<float>(src_w) / static_cast<float>(dst_w);
    const float scale_y = static_cast<float>(src_h) / static_cast<float>(dst_h);

    for (int y = 0; y < dst_h; ++y) {
        const float y0 = static_cast<float>(y) * scale_y;
        const float y1 = static_cast<float>(y + 1) * scale_y;
        for (int x = 0; x < dst_w; ++x) {
            const float x0 = static_cast<float>(x) * scale_x;
            const float x1 = static_cast<float>(x + 1) * scale_x;

            const int ix0 = static_cast<int>(std::floor(x0));
            const int iy0 = static_cast<int>(std::floor(y0));
            const int ix1 = std::min(static_cast<int>(std::ceil(x1)), src_w);
            const int iy1 = std::min(static_cast<int>(std::ceil(y1)), src_h);

            double sum_b = 0.0;
            double sum_g = 0.0;
            double sum_r = 0.0;
            double weight_sum = 0.0;

            for (int sy = iy0; sy < iy1; ++sy) {
                const float wy0 = std::max(0.0f, std::min(y1, static_cast<float>(sy + 1)) - std::max(y0, static_cast<float>(sy)));
                for (int sx = ix0; sx < ix1; ++sx) {
                    const float wx0 = std::max(0.0f, std::min(x1, static_cast<float>(sx + 1)) - std::max(x0, static_cast<float>(sx)));
                    const float weight = wx0 * wy0;
                    const size_t src_idx =
                        (static_cast<size_t>(sy) * static_cast<size_t>(src_w) + static_cast<size_t>(sx)) * 3;
                    sum_b += weight * src[src_idx];
                    sum_g += weight * src[src_idx + 1];
                    sum_r += weight * src[src_idx + 2];
                    weight_sum += weight;
                }
            }

            const size_t dst_idx =
                (static_cast<size_t>(y) * static_cast<size_t>(dst_w) + static_cast<size_t>(x)) * 3;
            if (weight_sum > 0.0) {
                dst[dst_idx] = static_cast<uint8_t>(sum_b / weight_sum + 0.5);
                dst[dst_idx + 1] = static_cast<uint8_t>(sum_g / weight_sum + 0.5);
                dst[dst_idx + 2] = static_cast<uint8_t>(sum_r / weight_sum + 0.5);
            } else {
                dst[dst_idx] = 0;
                dst[dst_idx + 1] = 0;
                dst[dst_idx + 2] = 0;
            }
        }
    }
}

}  // namespace

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
    int roi_resize_height) {
    if (image_width <= 0 || image_height <= 0) {
        throw std::invalid_argument("image dimensions must be positive");
    }

    PreprocessResult result;
    std::vector<uint8_t> working_bgr;

    if (use_roi) {
        if (roi_x0 < 0 || roi_y0 < 0 || roi_x1 > image_width || roi_y1 > image_height || roi_x1 <= roi_x0 ||
            roi_y1 <= roi_y0) {
            throw std::invalid_argument("invalid ROI bounds");
        }
        if (roi_resize_height <= 0) {
            throw std::invalid_argument("roi_resize_height must be positive when roi is set");
        }

        int crop_w = 0;
        int crop_h = 0;
        crop_bgr(bgr, image_width, image_height, roi_x0, roi_y0, roi_x1, roi_y1, working_bgr, crop_w, crop_h);

        const float resize_scale = static_cast<float>(roi_resize_height) / static_cast<float>(crop_h);
        const int resized_w = std::max(1, static_cast<int>(std::lround(static_cast<float>(crop_w) * resize_scale)));
        const int resized_h = roi_resize_height;

        std::vector<uint8_t> resized_bgr;
        resize_bgr_area(working_bgr.data(), crop_w, crop_h, resized_w, resized_h, resized_bgr);
        convert_bgr_to_gray(resized_bgr.data(), resized_w, resized_h, result.gray);

        result.width = resized_w;
        result.height = resized_h;
        result.remap.scale = 1.0f / resize_scale;
        result.remap.offset_x = static_cast<float>(roi_x0);
        result.remap.offset_y = static_cast<float>(roi_y0);
        return result;
    }

    const uint8_t* src_bgr = bgr;
    int src_w = image_width;
    int src_h = image_height;

    if (downsize_factor != 1.0f) {
        if (downsize_factor <= 0.0f) {
            throw std::invalid_argument("downsize_factor must be positive");
        }
        const int dst_w = std::max(1, static_cast<int>(std::lround(static_cast<float>(image_width) * downsize_factor)));
        const int dst_h = std::max(1, static_cast<int>(std::lround(static_cast<float>(image_height) * downsize_factor)));
        resize_bgr_bilinear(bgr, image_width, image_height, dst_w, dst_h, working_bgr);
        src_bgr = working_bgr.data();
        src_w = dst_w;
        src_h = dst_h;
        result.remap.scale = 1.0f / downsize_factor;
    }

    convert_bgr_to_gray(src_bgr, src_w, src_h, result.gray);
    result.width = src_w;
    result.height = src_h;
    return result;
}

}  // namespace vpi_apriltag
