#pragma once

#include "camera_orbbec.hpp"
#include "config.hpp"

#include <opencv2/core/mat.hpp>

#include <cstddef>
#include <filesystem>
#include <string>

namespace handeye {

OBFormat parseColorFormat(const std::string& name);
const char* colorFormatName(OBFormat format);

// 返回持有自己内存的 BGR 图像，避免 SDK 复用帧缓冲区后影响显示或保存。
cv::Mat decodeColorImage(OBFormat format, const void* data, std::size_t size,
                         int width, int height);

// 只读检查人工记录的参数与当前相机是否匹配，不生成或覆盖参数文件。
void validateCameraParamFile(const std::filesystem::path& path,
                             const CameraConfig& stream, const obcam::CameraParam& actual);

}  // namespace handeye
