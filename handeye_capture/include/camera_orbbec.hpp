#pragma once

#include <cstdint>
#include <memory>

// Orbbec SDK 的 C++ 接口
#include "libobsensor/ObSensor.hpp"

namespace obcam {

// 相机内参（针孔模型）。这里用普通结构体，不引入 cv::Mat，
// 保证本模块只依赖 OrbbecSDK，需要 OpenCV 类型时由 camera_opencv 转换。
struct CameraIntrinsic {
    int   width  = 0;   // 内参对应的图像宽度
    int   height = 0;   // 内参对应的图像高度
    float fx     = 0.f; // x 方向焦距（像素）
    float fy     = 0.f; // y 方向焦距（像素）
    float cx     = 0.f; // 主点 x（像素）
    float cy     = 0.f; // 主点 y（像素）
};

// 相机畸变系数（Orbbec 的 8 参数表示）
struct CameraDistortion {
    float k1 = 0.f, k2 = 0.f, k3 = 0.f;
    float k4 = 0.f, k5 = 0.f, k6 = 0.f;
    float p1 = 0.f, p2 = 0.f;
};

// 一组完整的标定参数：内参 + 畸变
struct CameraParam {
    CameraIntrinsic  intrinsic;
    CameraDistortion distortion;
};

// 彩色相机封装：只负责“打开设备 / 取帧 / 关闭”，不包含任何显示逻辑，
// 这样 camera 模块不依赖 OpenCV，显示相关的事情交给 opencv.cpp。
class ColorCamera {
public:
    ColorCamera();
    ~ColorCamera() noexcept;

    // 禁止拷贝（内部持有 pipeline / device 等资源）
    ColorCamera(const ColorCamera &) = delete;
    ColorCamera &operator=(const ColorCamera &) = delete;

    // 查找设备、挑选 color profile 并 enableStream（不 start）
    // 注意：找不到匹配 profile 时 SDK 会抛 ob::Error，不会返回空指针。
    void init(int width = 640, int height = 480, int fps = 30, OBFormat format = OB_FORMAT_MJPG);

    // pipeline start
    void open();

    // pipeline stop（可以重复调用）
    void close();

    // 打印当前 color 流的相机标定参数：流类型 + 内参 + 畸变系数
    void printCameraParam();

    // 取当前 color 流的内参。未 init 时返回 false，out 保持不变。
    bool cameraIntrinsic(CameraIntrinsic &out) const;

    // 取当前 color 流的完整标定参数（内参 + 畸变）。未 init 时返回 false。
    bool cameraParam(CameraParam &out) const;

    // 阻塞等待一帧，超时返回 nullptr
    std::shared_ptr<ob::FrameSet> waitForFrames(uint32_t timeoutMs = 100);

    // 当前使用的 color profile（未 init 时为空）
    std::shared_ptr<ob::VideoStreamProfile> colorProfile() const;
private:
    ob::Context context_;
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::shared_ptr<ob::VideoStreamProfile> colorProfile_;
    std::shared_ptr<ob::Config> config_;
    bool started_;
};

}  // namespace obcam
