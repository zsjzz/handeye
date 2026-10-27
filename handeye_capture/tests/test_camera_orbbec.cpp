#include "camera_orbbec.hpp"
#include "config.hpp"

#include <chrono>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct FormatEntry {
    const char* name;
    OBFormat format;
};

constexpr FormatEntry kFormats[] = {
    {"MJPG", OB_FORMAT_MJPG}, {"MJPEG", OB_FORMAT_MJPG},
    {"RGB", OB_FORMAT_RGB}, {"RGB888", OB_FORMAT_RGB},
    {"BGR", OB_FORMAT_BGR}, {"BGRA", OB_FORMAT_BGRA}, {"RGBA", OB_FORMAT_RGBA},
    {"YUYV", OB_FORMAT_YUYV}, {"YUY2", OB_FORMAT_YUY2}, {"UYVY", OB_FORMAT_UYVY},
    {"NV12", OB_FORMAT_NV12}, {"NV21", OB_FORMAT_NV21}, {"I420", OB_FORMAT_I420},
    {"H264", OB_FORMAT_H264}, {"H265", OB_FORMAT_H265},
};

OBFormat parseFormat(const std::string& name) {
    for (const auto& entry : kFormats) {
        if (name == entry.name) {
            return entry.format;
        }
    }
    throw std::runtime_error("camera.format: unsupported color format '" + name + "'");
}

const char* formatName(OBFormat format) {
    for (const auto& entry : kFormats) {
        if (format == entry.format) {
            return entry.name;
        }
    }
    return "Unknown";
}

std::shared_ptr<ob::ColorFrame> waitForColorFrame(obcam::ColorCamera& camera) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto frames = camera.waitForFrames(500);
        if (frames) {
            const auto color = frames->colorFrame();
            if (color && color->dataSize() > 0 && color->data() != nullptr) {
                return color;
            }
        }
    }
    throw std::runtime_error("No valid color frame received within 5 seconds");
}

}  // namespace

int main() {
    try {
        // 与其他测试相同，按测试源码的位置查找项目配置。
        const auto config_path = std::filesystem::path(__FILE__).parent_path().parent_path()
                                 / "config" / "config.yaml";
        const auto config = handeye::Config::load(config_path.string());
        const auto format = parseFormat(config.camera.format);

        std::cout << "配置文件: " << config_path.string() << '\n'
                  << "请求的彩色流: " << config.camera.width << 'x' << config.camera.height
                  << " @ " << config.camera.fps << " FPS, " << config.camera.format << '\n'
                  << "预热帧数: " << config.camera.warmup_frames << std::endl;

        obcam::ColorCamera camera;
        camera.init(config.camera.width, config.camera.height, config.camera.fps, format);
        const auto profile = camera.colorProfile();
        if (!profile) {
            throw std::runtime_error("Color stream profile is null after init");
        }

        std::cout << "\n匹配到的彩色流: " << profile->width() << 'x' << profile->height()
                  << " @ " << profile->fps() << " FPS, " << formatName(profile->format())
                  << " (SDK format=" << static_cast<int>(profile->format()) << ")\n"
                  << "相机标定参数（焦距与主点单位为像素）:\n" << std::setprecision(9);
        camera.printCameraParam();

        camera.open();
        for (int i = 0; i < config.camera.warmup_frames; ++i) {
            (void)waitForColorFrame(camera);
        }
        const auto color = waitForColorFrame(camera);
        std::cout << "\n已丢弃 " << config.camera.warmup_frames << " 帧预热图像\n"
                  << "获取到的彩色帧:\n"
                  << "  分辨率: " << color->width() << 'x' << color->height() << '\n'
                  << "  格式: " << formatName(color->format()) << '\n'
                  << "  帧序号: " << color->index() << '\n'
                  << "  数据大小 (bytes): " << color->dataSize() << '\n'
                  << "  设备时间戳 (us): " << color->timeStampUs() << '\n';

        camera.close();
        std::cout << "相机取流测试成功，已停止取流\n";
        return 0;
    } catch (const ob::Error& error) {
        // ob::Error 不继承 std::exception，需要单独捕获。
        std::cerr << "相机 SDK 错误: " << error.getMessage() << '\n'
                  << "  接口: " << error.getName() << '\n'
                  << "  参数: " << error.getArgs() << '\n';
        return 1;
    } catch (const std::exception& error) {
        std::cerr << "相机测试失败: " << error.what() << '\n';
        return 1;
    }
}
