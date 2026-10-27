#include "camera_opencv.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace handeye {
namespace {

struct FormatEntry {
    const char* name;
    OBFormat format;
};

constexpr FormatEntry kFormats[] = {
    {"MJPG", OB_FORMAT_MJPG}, {"MJPEG", OB_FORMAT_MJPG},
    {"RGB", OB_FORMAT_RGB}, {"RGB888", OB_FORMAT_RGB}, {"BGR", OB_FORMAT_BGR},
    {"BGRA", OB_FORMAT_BGRA}, {"RGBA", OB_FORMAT_RGBA},
    {"YUYV", OB_FORMAT_YUYV}, {"YUY2", OB_FORMAT_YUY2}, {"UYVY", OB_FORMAT_UYVY},
    {"NV12", OB_FORMAT_NV12}, {"NV21", OB_FORMAT_NV21}, {"I420", OB_FORMAT_I420},
};

void requireNear(double recorded, double actual, const std::string& field) {
    if (!std::isfinite(recorded) || !std::isfinite(actual) ||
        std::abs(recorded - actual) > 1e-8 + 1e-6 * std::abs(actual)) {
        throw std::runtime_error(field + " differs from the connected camera");
    }
}

}  // namespace

OBFormat parseColorFormat(const std::string& name) {
    for (const auto& entry : kFormats) {
        if (name == entry.name) return entry.format;
    }
    throw std::runtime_error("Unsupported preview format: " + name);
}

const char* colorFormatName(OBFormat format) {
    for (const auto& entry : kFormats) {
        if (format == entry.format) return entry.name;
    }
    return "Unknown";
}

cv::Mat decodeColorImage(OBFormat format, const void* data, std::size_t size,
                         int width, int height) {
    if (!data || size == 0 || width <= 0 || height <= 0) {
        throw std::runtime_error("Empty or invalid color frame");
    }
    cv::Mat image;
    if (format == OB_FORMAT_MJPG) {
        if (size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            throw std::runtime_error("Compressed frame is too large");
        }
        const cv::Mat encoded(1, static_cast<int>(size), CV_8UC1, const_cast<void*>(data));
        image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    } else {
        int channels = 0;
        int conversion = -1;
        int rows = height;
        switch (format) {
        case OB_FORMAT_BGR: channels = 3; break;
        case OB_FORMAT_RGB: channels = 3; conversion = cv::COLOR_RGB2BGR; break;
        case OB_FORMAT_BGRA: channels = 4; conversion = cv::COLOR_BGRA2BGR; break;
        case OB_FORMAT_RGBA: channels = 4; conversion = cv::COLOR_RGBA2BGR; break;
        case OB_FORMAT_YUYV:
        case OB_FORMAT_YUY2: channels = 2; conversion = cv::COLOR_YUV2BGR_YUY2; break;
        case OB_FORMAT_UYVY: channels = 2; conversion = cv::COLOR_YUV2BGR_UYVY; break;
        case OB_FORMAT_NV12:
        case OB_FORMAT_NV21:
        case OB_FORMAT_I420:
            if (width % 2 || height % 2 || height > std::numeric_limits<int>::max() / 3) {
                throw std::runtime_error("Invalid YUV420 dimensions");
            }
            channels = 1;
            rows = height * 3 / 2;
            conversion = format == OB_FORMAT_NV12 ? cv::COLOR_YUV2BGR_NV12 :
                         format == OB_FORMAT_NV21 ? cv::COLOR_YUV2BGR_NV21 : cv::COLOR_YUV2BGR_I420;
            break;
        default: throw std::runtime_error("Unsupported color frame format");
        }
        if (channels == 2 && width % 2) {
            throw std::runtime_error("YUV422 width must be even");
        }
        const auto expected_size = static_cast<std::size_t>(width) * rows * channels;
        if (size != expected_size) {
            throw std::runtime_error("Unexpected raw frame size or row padding");
        }
        const cv::Mat raw(rows, width, CV_MAKETYPE(CV_8U, channels), const_cast<void*>(data));
        if (conversion < 0) image = raw.clone();
        else cv::cvtColor(raw, image, conversion);
    }
    if (image.empty() || image.cols != width || image.rows != height) {
        throw std::runtime_error("Color image decode failed or dimensions differ from frame");
    }
    return image;
}

void validateCameraParamFile(const std::filesystem::path& path,
                             const CameraConfig& stream, const obcam::CameraParam& actual) {
    try {
        const auto root = YAML::LoadFile(path.string());
        const auto recorded = root["stream"];
        if (recorded["width"].as<int>() != stream.width ||
            recorded["height"].as<int>() != stream.height ||
            recorded["fps"].as<int>() != stream.fps ||
            parseColorFormat(recorded["format"].as<std::string>()) != parseColorFormat(stream.format) ||
            actual.intrinsic.width != stream.width || actual.intrinsic.height != stream.height) {
            throw std::runtime_error("Stream configuration differs from camera_param.yaml");
        }
        const auto& in = actual.intrinsic;
        const auto& d = actual.distortion;
        const auto intrinsic = root["intrinsics"];
        requireNear(intrinsic["fx"].as<double>(), in.fx, "intrinsics.fx");
        requireNear(intrinsic["fy"].as<double>(), in.fy, "intrinsics.fy");
        requireNear(intrinsic["cx"].as<double>(), in.cx, "intrinsics.cx");
        requireNear(intrinsic["cy"].as<double>(), in.cy, "intrinsics.cy");
        const double expected[3][3] = {{in.fx, 0, in.cx}, {0, in.fy, in.cy}, {0, 0, 1}};
        const auto matrix = root["camera_matrix"];
        if (!matrix.IsSequence() || matrix.size() != 3) {
            throw std::runtime_error("camera_matrix must be 3x3");
        }
        for (int row = 0; row < 3; ++row) {
            if (!matrix[row].IsSequence() || matrix[row].size() != 3) {
                throw std::runtime_error("camera_matrix must be 3x3");
            }
            for (int col = 0; col < 3; ++col) {
                requireNear(matrix[row][col].as<double>(), expected[row][col], "camera_matrix");
            }
        }
        const char* order[] = {"k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"};
        const double coefficients[] = {d.k1, d.k2, d.p1, d.p2, d.k3, d.k4, d.k5, d.k6};
        const auto recorded_order = root["distortion_order"];
        const auto recorded_coefficients = root["distortion_coefficients"];
        if (!recorded_order.IsSequence() || recorded_order.size() != 8 ||
            !recorded_coefficients.IsSequence() || recorded_coefficients.size() != 8) {
            throw std::runtime_error("Expected eight ordered distortion coefficients");
        }
        for (int i = 0; i < 8; ++i) {
            if (recorded_order[i].as<std::string>() != order[i]) {
                throw std::runtime_error("Incorrect OpenCV distortion coefficient order");
            }
            requireNear(recorded_coefficients[i].as<double>(), coefficients[i], "distortion_coefficients");
        }
    } catch (const std::exception& error) {
        throw std::runtime_error("Camera parameter check failed for '" + path.string() + "': " + error.what());
    }
}

}  // namespace handeye
