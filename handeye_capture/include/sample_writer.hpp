#pragma once

#include "config.hpp"
#include "robot_rm.hpp"

#include <opencv2/core/mat.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>

namespace handeye {

using CaptureClock = std::chrono::system_clock;

struct SampleInfo {
    CaptureClock::time_point captured_at;
    CaptureClock::time_point frame_received_at;
    CaptureClock::time_point robot_query_started_at;
    CaptureClock::time_point robot_query_finished_at;
    std::uint64_t frame_index = 0;
    std::uint64_t device_timestamp_us = 0;
    std::string frame_format;
    RobotState robot;
};

// 始终使用 UTC；compact=true 时返回可作为目录名的紧凑格式。
std::string formatUtc(CaptureClock::time_point time, bool compact = false);

// 写入 image.png 和 sample.yaml 后才发布样本目录；失败则清理本组临时文件。
// 要求 data_root/camera_param.yaml 已存在，不修改该参数文件或已有样本。
std::filesystem::path saveSample(const std::filesystem::path& data_root,
                                 const cv::Mat& image, const Config& config,
                                 const SampleInfo& sample);

}  // namespace handeye
