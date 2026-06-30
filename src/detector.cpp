#include "detector.hpp"

#include "corner_order.hpp"

#include <vpi/Array.h>
#include <vpi/Image.h>
#include <vpi/Stream.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace vpi_apriltag {

namespace {

void check_vpi(VPIStatus status, const char* context) {
    if (status != VPI_SUCCESS) {
        throw std::runtime_error(std::string("VPI error in ") + context + ": status " + std::to_string(status));
    }
}

}  // namespace

AprilTagDetector::AprilTagDetector(
    int image_width,
    int image_height,
    const std::vector<int32_t>& marker_ids,
    int max_bits_corrected,
    bool use_pva)
    : image_width_(image_width),
      image_height_(image_height),
      marker_ids_(marker_ids),
      max_bits_corrected_(max_bits_corrected) {
    if (image_width_ <= 0 || image_height_ <= 0) {
        throw std::invalid_argument("image_width and image_height must be positive");
    }
    if (max_bits_corrected_ < 0) {
        throw std::invalid_argument("max_bits_corrected must be non-negative");
    }

    backends_ = use_pva ? (VPI_BACKEND_PVA | VPI_BACKEND_CPU) : VPI_BACKEND_CPU;
    submit_backend_ = use_pva ? VPI_BACKEND_PVA : VPI_BACKEND_CPU;

    tag_id_filter_.reserve(marker_ids_.size());
    for (const int32_t marker_id : marker_ids_) {
        if (marker_id < 0 || marker_id > UINT16_MAX) {
            throw std::invalid_argument("marker_ids must be in range [0, 65535]");
        }
        tag_id_filter_.push_back(static_cast<uint16_t>(marker_id));
    }
    check_vpi(vpiInitAprilTagDecodeParams(&decode_params_), "vpiInitAprilTagDecodeParams");
    decode_params_.family = VPI_APRILTAG_36H11;
    decode_params_.maxBitsCorrected = max_bits_corrected_;
    decode_params_.tagIdFilter = tag_id_filter_.empty() ? nullptr : tag_id_filter_.data();
    decode_params_.tagIdFilterSize = static_cast<int32_t>(tag_id_filter_.size());

    check_vpi(vpiStreamCreate(backends_, &stream_), "vpiStreamCreate");
    check_vpi(
        vpiArrayCreate(static_cast<int32_t>(kMaxDetections), VPI_ARRAY_TYPE_APRILTAG_DETECTION, VPI_BACKEND_CPU, &detections_),
        "vpiArrayCreate");
}

AprilTagDetector::~AprilTagDetector() {
    destroy_payload_cache();
    if (wrapper_image_ != nullptr) {
        vpiImageDestroy(wrapper_image_);
        wrapper_image_ = nullptr;
    }
    if (detections_ != nullptr) {
        vpiArrayDestroy(detections_);
        detections_ = nullptr;
    }
    if (stream_ != nullptr) {
        vpiStreamDestroy(stream_);
        stream_ = nullptr;
    }
}

void AprilTagDetector::destroy_payload_cache() {
    for (auto& entry : payload_by_size_) {
        if (entry.second != nullptr) {
            vpiPayloadDestroy(entry.second);
        }
    }
    payload_by_size_.clear();
    payload_lru_.clear();
}

void AprilTagDetector::touch_payload_cache(int det_width, int det_height, VPIPayload payload) {
    const DimensionKey key{det_width, det_height};
    payload_by_size_[key] = payload;
    payload_lru_.remove(key);
    payload_lru_.push_front(key);

    while (payload_lru_.size() > kMaxPayloadCacheEntries) {
        const DimensionKey evict_key = payload_lru_.back();
        payload_lru_.pop_back();
        auto it = payload_by_size_.find(evict_key);
        if (it != payload_by_size_.end()) {
            if (it->second != nullptr) {
                vpiPayloadDestroy(it->second);
            }
            payload_by_size_.erase(it);
        }
    }
}

VPIPayload AprilTagDetector::get_or_create_payload(int det_width, int det_height) {
    const DimensionKey key{det_width, det_height};
    auto it = payload_by_size_.find(key);
    if (it != payload_by_size_.end()) {
        payload_lru_.remove(key);
        payload_lru_.push_front(key);
        return it->second;
    }

    VPIPayload payload = nullptr;
    check_vpi(
        vpiCreateAprilTagDetector(backends_, det_width, det_height, &decode_params_, &payload),
        "vpiCreateAprilTagDetector");
    touch_payload_cache(det_width, det_height, payload);
    return payload;
}

DetectionResult AprilTagDetector::detect(
    const uint8_t* bgr,
    std::optional<std::tuple<int, int, int, int>> roi,
    float downsize_factor,
    int roi_resize_height) {
    const bool use_roi = roi.has_value();
    int roi_x0 = 0;
    int roi_y0 = 0;
    int roi_x1 = 0;
    int roi_y1 = 0;
    if (use_roi) {
        std::tie(roi_x0, roi_y0, roi_x1, roi_y1) = roi.value();
    }

    const PreprocessResult preprocessed = preprocess_bgr(
        bgr,
        image_width_,
        image_height_,
        roi_x0,
        roi_y0,
        roi_x1,
        roi_y1,
        use_roi,
        downsize_factor,
        roi_resize_height);

    if (preprocessed.gray.empty()) {
        return {};
    }

    VPIPayload payload = get_or_create_payload(preprocessed.width, preprocessed.height);

    if (wrapper_image_ != nullptr) {
        vpiImageDestroy(wrapper_image_);
        wrapper_image_ = nullptr;
    }

    gray_buffer_ = preprocessed.gray;

    VPIImageData image_data{};
    image_data.bufferType = VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR;
    image_data.buffer.pitch.format = VPI_IMAGE_FORMAT_U8;
    image_data.buffer.pitch.numPlanes = 1;
    image_data.buffer.pitch.planes[0].data = gray_buffer_.data();
    image_data.buffer.pitch.planes[0].width = preprocessed.width;
    image_data.buffer.pitch.planes[0].height = preprocessed.height;
    image_data.buffer.pitch.planes[0].pitchBytes = preprocessed.width;

    VPIImageWrapperParams wrapper_params{};
    check_vpi(vpiInitImageWrapperParams(&wrapper_params), "vpiInitImageWrapperParams");
    check_vpi(
        vpiImageCreateWrapper(&image_data, &wrapper_params, backends_, &wrapper_image_),
        "vpiImageCreateWrapper");

    check_vpi(
        vpiSubmitAprilTagDetector(
            stream_,
            submit_backend_,
            payload,
            kMaxDetections,
            wrapper_image_,
            detections_),
        "vpiSubmitAprilTagDetector");
    check_vpi(vpiStreamSync(stream_), "vpiStreamSync");

    VPIArrayData array_data{};
    check_vpi(
        vpiArrayLockData(detections_, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &array_data),
        "vpiArrayLockData");

    DetectionResult result;
    const auto* detections = static_cast<const VPIAprilTagDetection*>(array_data.buffer.aos.data);
    const uint32_t num_detections = array_data.buffer.aos.sizePointer != nullptr ? *array_data.buffer.aos.sizePointer : 0;

    for (uint32_t i = 0; i < num_detections; ++i) {
        const VPIAprilTagDetection& detection = detections[i];

        if (!tag_id_filter_.empty()) {
            const auto found = std::find(tag_id_filter_.begin(), tag_id_filter_.end(), detection.id);
            if (found == tag_id_filter_.end()) {
                continue;
            }
        }

        Point2f vpi_corners[4];
        for (size_t c = 0; c < 4; ++c) {
            vpi_corners[c] = {detection.corners[c].x, detection.corners[c].y};
        }
        const std::array<Point2f, 4> ordered = reorder_vpi_to_opencv(vpi_corners);
        std::array<float, 8> remapped{};
        for (size_t c = 0; c < 4; ++c) {
            remapped[c * 2] = ordered[c].x * preprocessed.remap.scale + preprocessed.remap.offset_x;
            remapped[c * 2 + 1] = ordered[c].y * preprocessed.remap.scale + preprocessed.remap.offset_y;
        }

        result.corners.push_back(remapped);
        result.ids.push_back(static_cast<int32_t>(detection.id));
    }

    check_vpi(vpiArrayUnlock(detections_), "vpiArrayUnlock");
    return result;
}

}  // namespace vpi_apriltag
