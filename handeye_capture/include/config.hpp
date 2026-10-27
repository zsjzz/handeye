#pragma once

#include <string>

namespace handeye {

struct CameraConfig {
    int width = 0;
    int height = 0;
    int fps = 0;
    std::string format;
    int warmup_frames = 0;
};

struct BoardConfig {
    int cols = 0;
    int rows = 0;
    double square_size_m = 0.0;
};

struct RobotConfig {
    std::string ip;
    int port = 0;
};

struct SessionConfig {
    // 相对路径保留原值，由采集程序相对于运行目录使用。
    std::string root;
    // 空字符串表示由采集程序自动命名；加载配置时不生成名称。
    std::string name;
};

struct CaptureConfig {
    double max_reprojection_px = 0.0;
    double min_pose_distance_m = 0.0;
    double min_pose_angle_deg = 0.0;
};

struct Config {
    CameraConfig camera;
    BoardConfig board;
    RobotConfig robot;
    SessionConfig session;
    CaptureConfig capture;

    // 所有 YAML 字段均为必填项，session.name 可以为空字符串。
    // 文件、语法、类型或取值错误抛出 std::runtime_error，包含路径和错误上下文。
    static Config load(const std::string& path);
};

}  // namespace handeye
