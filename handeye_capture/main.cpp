#include "camera_opencv.hpp"
#include "config.hpp"
#include "robot_rm.hpp"
#include "sample_writer.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr const char* kWindow = "Hand-eye capture | Space/S: save | Q/Esc: quit";

struct PreviewWindow {
    PreviewWindow() { cv::namedWindow(kWindow, cv::WINDOW_AUTOSIZE); }
    ~PreviewWindow() {
        try { cv::destroyWindow(kWindow); }
        catch (const cv::Exception&) {}
    }
};

bool hasEnvironment(const char* name) {
    const char* value = std::getenv(name);
    return value && *value;
}

bool previewIsOpen() {
    try {
        // GTK 版本可能不支持 WND_PROP_VISIBLE，AUTOSIZE 可用于查询窗口是否仍存在。
        return cv::getWindowProperty(kWindow, cv::WND_PROP_AUTOSIZE) >= 0;
    } catch (const cv::Exception&) {
        return false;
    }
}

void runCapture(const handeye::Config& config, const std::filesystem::path& data_root) {
#if defined(__linux__)
    if (!hasEnvironment("DISPLAY") && !hasEnvironment("WAYLAND_DISPLAY")) {
        throw std::runtime_error("No desktop display is available. Run this program in a graphical desktop terminal.");
    }
#endif
    std::filesystem::create_directories(data_root);
    const auto parameter_file = data_root / "camera_param.yaml";
    if (!std::filesystem::is_regular_file(parameter_file)) {
        throw std::runtime_error("Camera parameter file missing: " + parameter_file.string());
    }

    PreviewWindow window;
    obcam::ColorCamera camera;
    camera.init(config.camera.width, config.camera.height, config.camera.fps,
                handeye::parseColorFormat(config.camera.format));
    obcam::CameraParam actual;
    if (!camera.cameraParam(actual)) {
        throw std::runtime_error("Cannot read connected camera parameters");
    }
    handeye::validateCameraParamFile(parameter_file, config.camera, actual);
    camera.open();

    std::cout << "保存目录: " << data_root.string() << '\n'
              << "相机参数: " << parameter_file.string() << '\n'
              << "空格/S 采集，Q/Esc 或关闭预览窗口退出。\n"
              << "请先停稳机械臂，确认画面后采集；提示保存成功后再移动。" << std::endl;

    cv::Mat warming(config.camera.height, config.camera.width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::putText(warming, "Warming up...", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::imshow(kWindow, warming);

    cv::Mat displayed_image;
    handeye::SampleInfo displayed_info;
    auto last_frame_at = std::chrono::steady_clock::now();
    int warmup_remaining = config.camera.warmup_frames;
    std::size_t saved_count = 0;
    std::string status = "Warming up";
    std::future<std::filesystem::path> pending;
    auto finishCapture = [&] {
        try {
            const auto directory = pending.get();
            ++saved_count;
            status = "Saved. You may move the robot.";
            std::cout << "保存成功: " << directory.string() << "，可以移动机械臂。" << std::endl;
        } catch (const std::exception& error) {
            status = "Save failed. See terminal.";
            std::cerr << "本组采集失败: " << error.what() << std::endl;
        }
    };

    try {
        while (true) {
            if (pending.valid() && pending.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                finishCapture();
            }
            const auto frames = camera.waitForFrames(100);
            const auto received_at = handeye::CaptureClock::now();
            const auto color = frames ? frames->colorFrame() : nullptr;
            if (color && color->data() && color->dataSize() > 0) {
                if (warmup_remaining > 0) {
                    --warmup_remaining;
                    last_frame_at = std::chrono::steady_clock::now();
                } else {
                    try {
                        displayed_image = handeye::decodeColorImage(
                            color->format(), color->data(), color->dataSize(),
                            static_cast<int>(color->width()), static_cast<int>(color->height()));
                        displayed_info.frame_index = color->index();
                        displayed_info.device_timestamp_us = color->timeStampUs();
                        displayed_info.frame_received_at = received_at;
                        displayed_info.frame_format = handeye::colorFormatName(color->format());
                        last_frame_at = std::chrono::steady_clock::now();
                        if (status == "Warming up") status = "Ready";
                    } catch (const std::exception& error) {
                        std::cerr << "丢弃无法解码的帧: " << error.what() << '\n';
                    }
                }
            }
            const auto frame_age = std::chrono::steady_clock::now() - last_frame_at;
            if (frame_age > std::chrono::seconds(5)) {
                throw std::runtime_error("No valid color image received for 5 seconds");
            }

            if (!displayed_image.empty()) {
                // 叠加信息仅用于预览，保存时使用未叠加的 displayed_image。
                cv::Mat preview = displayed_image.clone();
                const std::string label = pending.valid() ? "Saving... keep robot still" : status;
                cv::putText(preview, label, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
                            0.55, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::putText(preview, "Saved: " + std::to_string(saved_count),
                            cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX,
                            0.55, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::imshow(kWindow, preview);
            }

            const int key = cv::waitKey(1) & 0xff;
            if (key == 'q' || key == 'Q' || key == 27 || !previewIsOpen()) {
                break;
            }
            if (key != ' ' && key != 's' && key != 'S') continue;
            if (pending.valid()) {
                std::cout << "正在保存，请保持机械臂静止。" << std::endl;
                continue;
            }
            if (displayed_image.empty() || frame_age > std::chrono::seconds(1)) {
                std::cerr << "当前没有可采集的新图像，请等待相机出流。" << std::endl;
                continue;
            }

            auto sample = displayed_info;
            sample.captured_at = handeye::CaptureClock::now();
            // 深拷贝按键时显示的原始图像；后台查询和写盘期间继续实时预览。
            pending = std::async(std::launch::async,
                [data_root, config, sample, image = displayed_image.clone()]() mutable {
                    sample.robot_query_started_at = handeye::CaptureClock::now();
                    sample.robot = handeye::readRobotState(config.robot.ip, config.robot.port);
                    sample.robot_query_finished_at = handeye::CaptureClock::now();
                    if (sample.robot.work_frame != "World" || sample.robot.tool_frame != "Arm_Tip") {
                        throw std::runtime_error("Expected World / Arm_Tip frames for T_base_flange");
                    }
                    for (int i = 0; i < sample.robot.error_count; ++i) {
                        if (sample.robot.errors[i] != 0) {
                            throw std::runtime_error("Controller reported error " + std::to_string(sample.robot.errors[i]));
                        }
                    }
                    return handeye::saveSample(data_root, image, config, sample);
                });
            std::cout << "正在采集，请保持机械臂静止……" << std::endl;
        }
    } catch (...) {
        if (pending.valid()) finishCapture();
        throw;
    }
    if (pending.valid()) finishCapture();
    camera.close();
    std::cout << "本次成功保存 " << saved_count << " 组样本。" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc > 2 || (argc == 2 && std::string(argv[1]) == "--help")) {
            std::cout << "Usage: handeye_capture [config.yaml]\n"
                      << "Space/S: capture; Q/Esc: quit. Output: ./data/<UTC timestamp>/\n";
            return argc > 2 ? 1 : 0;
        }
        const auto config_path = argc == 2 ? std::filesystem::path(argv[1]) :
            std::filesystem::path(__FILE__).parent_path() / "config" / "config.yaml";
        const auto config = handeye::Config::load(config_path.string());
        runCapture(config, std::filesystem::current_path() / "data");
        return 0;
    } catch (const ob::Error& error) {
        std::cerr << "相机 SDK 错误: " << error.getMessage()
                  << " (" << error.getName() << ")\n";
        return 1;
    } catch (const std::exception& error) {
        std::cerr << "采集程序错误: " << error.what() << '\n';
        return 1;
    }
}
