#include "config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

namespace handeye {
namespace {

// 从父节点中取出子节点
YAML::Node requireMap(const YAML::Node& parent, const std::string& key) {
    const auto node = parent[key];
    if (!node || !node.IsMap()) {
        throw std::runtime_error(key + ": expected a mapping");
    }
    return node;
}

template <typename T>
T read(const YAML::Node& section, const std::string& section_name,
       const std::string& key) {
    const std::string field = section_name + "." + key;
    const auto node = section[key];
    if (!node || node.IsNull()) {
        throw std::runtime_error(field + ": required value is missing or null");
    }
    if (!node.IsScalar()) {
        throw std::runtime_error(field + ": expected a scalar value");
    }
    try {
        return node.as<T>();
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(field + ": " + error.what());
    }
}

void require(bool condition, const std::string& field, const std::string& reason) {
    if (!condition) {
        throw std::runtime_error(field + ": " + reason);
    }
}

// 字符串校验
void requireNonEmpty(const std::string& value, const std::string& field) {
    require(value.find_first_not_of(" \t\r\n") != std::string::npos,
            field, "must not be empty or whitespace");
}

// 正数校验
void requirePositive(double value, const std::string& field) {
    require(std::isfinite(value) && value > 0.0, field,
            "must be finite and greater than zero");
}

// 非负数值校验
void requireNonNegative(double value, const std::string& field) {
    require(std::isfinite(value) && value >= 0.0, field,
            "must be finite and non-negative");
}

}  // namespace

Config Config::load(const std::string& path) {
    try {
        const auto root = YAML::LoadFile(path);
        require(root.IsMap(), "root", "expected a mapping");

        const auto camera = requireMap(root, "camera");
        const auto board = requireMap(root, "board");
        const auto robot = requireMap(root, "robot");
        const auto session = requireMap(root, "session");
        const auto capture = requireMap(root, "capture");

        Config config;
        config.camera.width = read<int>(camera, "camera", "width");
        config.camera.height = read<int>(camera, "camera", "height");
        config.camera.fps = read<int>(camera, "camera", "fps");
        config.camera.format = read<std::string>(camera, "camera", "format");
        config.camera.warmup_frames = read<int>(camera, "camera", "warmup_frames");
        config.board.cols = read<int>(board, "board", "cols");
        config.board.rows = read<int>(board, "board", "rows");
        config.board.square_size_m = read<double>(board, "board", "square_size_m");
        config.robot.ip = read<std::string>(robot, "robot", "ip");
        config.robot.port = read<int>(robot, "robot", "port");
        config.session.root = read<std::string>(session, "session", "root");
        config.session.name = read<std::string>(session, "session", "name");
        config.capture.max_reprojection_px =read<double>(capture, "capture", "max_reprojection_px");
        config.capture.min_pose_distance_m =read<double>(capture, "capture", "min_pose_distance_m");
        config.capture.min_pose_angle_deg =read<double>(capture, "capture", "min_pose_angle_deg");

        require(config.camera.width > 0, "camera.width", "must be greater than zero");
        require(config.camera.height > 0, "camera.height", "must be greater than zero");
        require(config.camera.fps > 0, "camera.fps", "must be greater than zero");
        requireNonEmpty(config.camera.format, "camera.format");
        require(config.camera.warmup_frames >= 0, "camera.warmup_frames","must be non-negative");
        require(config.board.cols >= 2, "board.cols", "must be at least two");
        require(config.board.rows >= 2, "board.rows", "must be at least two");
        requirePositive(config.board.square_size_m, "board.square_size_m");
        requireNonEmpty(config.robot.ip, "robot.ip");
        require(config.robot.port >= 1 && config.robot.port <= 65535,"robot.port", "must be between 1 and 65535");
        requireNonEmpty(config.session.root, "session.root");
        requirePositive(config.capture.max_reprojection_px, "capture.max_reprojection_px");
        requireNonNegative(config.capture.min_pose_distance_m, "capture.min_pose_distance_m");
        requireNonNegative(config.capture.min_pose_angle_deg, "capture.min_pose_angle_deg");
        return config;
    } catch (const YAML::Exception& error) {
        throw std::runtime_error("Failed to load config '" + path + "': " + error.what());
    } catch (const std::runtime_error& error) {
        throw std::runtime_error("Failed to load config '" + path + "': " + error.what());
    }
}

}  // namespace handeye
