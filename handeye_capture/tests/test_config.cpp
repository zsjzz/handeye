#include "config.hpp"

#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>

int main() {
    try {
        // tests/test_config.cpp 的上两级目录为 handeye_capture。
        const auto config_path = std::filesystem::path(__FILE__).parent_path().parent_path()
                                 / "config" / "config.yaml";
        const auto config = handeye::Config::load(config_path.string());

        std::cout << "Config file: " << config_path.string() << '\n'
                  << "camera:\n"
                  << "  width: " << config.camera.width << '\n'
                  << "  height: " << config.camera.height << '\n'
                  << "  fps: " << config.camera.fps << '\n'
                  << "  format: " << std::quoted(config.camera.format) << '\n'
                  << "  warmup_frames: " << config.camera.warmup_frames << '\n'
                  << "board:\n"
                  << "  cols: " << config.board.cols << '\n'
                  << "  rows: " << config.board.rows << '\n'
                  << "  square_size_m: " << config.board.square_size_m << '\n'
                  << "robot:\n"
                  << "  ip: " << std::quoted(config.robot.ip) << '\n'
                  << "  port: " << config.robot.port << '\n'
                  << "session:\n"
                  << "  root: " << std::quoted(config.session.root) << '\n'
                  << "  name: " << std::quoted(config.session.name) << '\n'
                  << "capture:\n"
                  << "  max_reprojection_px: " << config.capture.max_reprojection_px << '\n'
                  << "  min_pose_distance_m: " << config.capture.min_pose_distance_m << '\n'
                  << "  min_pose_angle_deg: " << config.capture.min_pose_angle_deg << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
