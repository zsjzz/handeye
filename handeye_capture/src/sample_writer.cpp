#include "sample_writer.hpp"

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>

namespace handeye {
namespace {

template <std::size_t N>
YAML::Node values(const double (&data)[N]) {
    YAML::Node node(YAML::NodeType::Sequence);
    node.SetStyle(YAML::EmitterStyle::Flow);
    for (const double value : data) {
        if (!std::isfinite(value)) {
            throw std::runtime_error("Robot state contains a non-finite value");
        }
        node.push_back(value);
    }
    return node;
}

YAML::Node sampleNode(const cv::Mat& image, const Config& config, const SampleInfo& sample) {
    YAML::Node root;
    root["schema_version"] = 1;
    root["calibration_type"] = "eye_in_hand";
    root["captured_at_utc"] = formatUtc(sample.captured_at);
    root["image"] = "image.png";
    root["camera_param_file"] = "../camera_param.yaml";
    auto frame = root["frame"];
    frame["index"] = sample.frame_index;
    frame["device_timestamp_us"] = sample.device_timestamp_us;
    frame["received_at_utc"] = formatUtc(sample.frame_received_at);
    frame["width"] = image.cols;
    frame["height"] = image.rows;
    frame["fps"] = config.camera.fps;
    frame["format"] = sample.frame_format;
    frame["decoded_color_order"] = "BGR";

    root["robot_query"]["started_at_utc"] = formatUtc(sample.robot_query_started_at);
    root["robot_query"]["finished_at_utc"] = formatUtc(sample.robot_query_finished_at);
    auto robot = root["robot"];
    robot["ip"] = config.robot.ip;
    robot["port"] = config.robot.port;
    robot["position_m"] = values(sample.robot.position_m);
    robot["euler_rad"] = values(sample.robot.euler_rad);
    robot["quaternion_order"] = "wxyz";
    robot["quaternion"] = values(sample.robot.quaternion);
    robot["joint_deg"] = values(sample.robot.joint_deg);
    robot["work_frame"] = sample.robot.work_frame;
    robot["tool_frame"] = sample.robot.tool_frame;
    robot["transform_convention"] = "p_base = T_base_flange * p_flange";
    for (const auto& row : sample.robot.T_base_flange) {
        robot["T_base_flange"].push_back(values(row));
    }
    robot["error_count"] = sample.robot.error_count;
    robot["errors"] = YAML::Node(YAML::NodeType::Sequence);
    robot["errors"].SetStyle(YAML::EmitterStyle::Flow);
    bool has_error = false;
    for (int i = 0; i < sample.robot.error_count; ++i) {
        robot["errors"].push_back(sample.robot.errors[i]);
        has_error = has_error || sample.robot.errors[i] != 0;
    }
    robot["has_error"] = has_error;
    root["board"]["cols"] = config.board.cols;
    root["board"]["rows"] = config.board.rows;
    root["board"]["square_size_m"] = config.board.square_size_m;
    return root;
}

}  // namespace

std::string formatUtc(CaptureClock::time_point time, bool compact) {
    const auto whole_seconds = std::chrono::floor<std::chrono::seconds>(time.time_since_epoch());
    const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        time.time_since_epoch() - whole_seconds).count();
    const auto seconds = static_cast<std::time_t>(whole_seconds.count());
    std::tm utc{};
    if (gmtime_r(&seconds, &utc) == nullptr) {
        throw std::runtime_error("Cannot convert timestamp to UTC");
    }
    std::ostringstream output;
    output << std::put_time(&utc, compact ? "%Y%m%dT%H%M%S" : "%Y-%m-%dT%H:%M:%S")
           << '.' << std::setfill('0') << std::setw(6) << microseconds << 'Z';
    return output.str();
}

std::filesystem::path saveSample(const std::filesystem::path& data_root,
                                 const cv::Mat& image, const Config& config,
                                 const SampleInfo& sample) {
    if (image.empty() || image.type() != CV_8UC3 || image.dims != 2) {
        throw std::runtime_error("Sample image must be a non-empty 8-bit BGR image");
    }
    if (image.cols != config.camera.width || image.rows != config.camera.height) {
        throw std::runtime_error("Sample image size differs from camera configuration");
    }
    if (sample.robot.error_count < 0 ||
        sample.robot.error_count > static_cast<int>(std::size(sample.robot.errors))) {
        throw std::runtime_error("Invalid robot error count");
    }
    if (!std::filesystem::is_regular_file(data_root / "camera_param.yaml")) {
        throw std::runtime_error("Camera parameter file missing: " +
                                 (data_root / "camera_param.yaml").string());
    }

    // 先完成序列化，保证非法数据不会产生任何样本文件。
    YAML::Emitter yaml;
    yaml.SetDoublePrecision(17);
    yaml << sampleNode(image, config, sample);
    if (!yaml.good()) {
        throw std::runtime_error("Cannot serialize sample: " + yaml.GetLastError());
    }

    const auto name = formatUtc(sample.captured_at, true);
    const auto destination = data_root / name;
    const auto temporary = data_root / ("." + name + ".tmp");
    if (std::filesystem::exists(destination)) {
        throw std::runtime_error("Sample already exists: " + destination.string());
    }
    // 同一个时间戳只能有一个写入者；不接管或清理其他进程的临时目录。
    if (!std::filesystem::create_directory(temporary)) {
        throw std::runtime_error("Sample is already being written: " + temporary.string());
    }
    try {
        if (!cv::imwrite((temporary / "image.png").string(), image)) {
            throw std::runtime_error("Cannot write sample image");
        }
        std::ofstream output;
        output.exceptions(std::ios::failbit | std::ios::badbit);
        output.open(temporary / "sample.yaml");
        output << yaml.c_str() << '\n';
        output.close();
        std::filesystem::rename(temporary, destination);
    } catch (...) {
        std::error_code ignored;
        std::filesystem::remove_all(temporary, ignored);
        throw;
    }
    return destination;
}

}  // namespace handeye
