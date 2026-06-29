#pragma once

#include "preprocess.hpp"

#include <vpi/algo/AprilTags.h>

#include <cstdint>
#include <list>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace vpi_apriltag {

struct DetectionResult {
    std::vector<std::array<float, 8>> corners;
    std::vector<int32_t> ids;
};

class AprilTagDetector {
public:
    AprilTagDetector(
        int image_width,
        int image_height,
        const std::vector<int32_t>& marker_ids,
        int max_bits_corrected,
        bool use_pva);

    ~AprilTagDetector();

    AprilTagDetector(const AprilTagDetector&) = delete;
    AprilTagDetector& operator=(const AprilTagDetector&) = delete;

    DetectionResult detect(
        const uint8_t* bgr,
        std::optional<std::tuple<int, int, int, int>> roi,
        float downsize_factor,
        int roi_resize_height);

    int image_width() const { return image_width_; }
    int image_height() const { return image_height_; }

private:
    static constexpr uint32_t kMaxDetections = 64;
    static constexpr size_t kMaxPayloadCacheEntries = 4;

    struct DimensionKey {
        int width;
        int height;

        bool operator==(const DimensionKey& other) const {
            return width == other.width && height == other.height;
        }
    };

    struct DimensionKeyHash {
        size_t operator()(const DimensionKey& key) const {
            return static_cast<size_t>(key.width) * 73856093u ^ static_cast<size_t>(key.height) * 19349663u;
        }
    };

    VPIPayload get_or_create_payload(int det_width, int det_height);
    void touch_payload_cache(int det_width, int det_height, VPIPayload payload);
    void destroy_payload_cache();

    int image_width_;
    int image_height_;
    std::vector<int32_t> marker_ids_;
    int max_bits_corrected_;
    uint64_t backends_;
    uint64_t submit_backend_;

    std::vector<int32_t> tag_id_filter_;
    VPIAprilTagDecodeParams decode_params_{};

    VPIStream stream_{nullptr};
    VPIArray detections_{nullptr};
    VPIImage wrapper_image_{nullptr};

    std::unordered_map<DimensionKey, VPIPayload, DimensionKeyHash> payload_by_size_;
    std::list<DimensionKey> payload_lru_;
};

}  // namespace vpi_apriltag
