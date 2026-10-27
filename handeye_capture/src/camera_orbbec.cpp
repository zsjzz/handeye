// =============================================================================
// ColorCamera 的实现（Orbbec SDK）
// =============================================================================

#include "camera_orbbec.hpp"

#include <iostream>
#include <stdexcept>

#include "libobsensor/hpp/Error.hpp"

namespace obcam {

namespace {

// 流类型 -> 可读字符串
const char *streamTypeName(OBStreamType type) {
    switch(type) {
    case OB_STREAM_VIDEO:
        return "Video";
    case OB_STREAM_IR:
        return "IR";
    case OB_STREAM_COLOR:
        return "Color";
    case OB_STREAM_DEPTH:
        return "Depth";
    case OB_STREAM_ACCEL:
        return "Accel";
    case OB_STREAM_GYRO:
        return "Gyro";
    case OB_STREAM_IR_LEFT:
        return "IR-Left";
    case OB_STREAM_IR_RIGHT:
        return "IR-Right";
    case OB_STREAM_RAW_PHASE:
        return "RawPhase";
    default:
        return "Unknown";
    }
}

}  // namespace

ColorCamera::ColorCamera(): context_(), pipeline_(nullptr), colorProfile_(nullptr), config_(nullptr), started_(false) {

}

ColorCamera::~ColorCamera() noexcept {
    close();
}

void ColorCamera::init(int width, int height, int fps, OBFormat format) {
    // 查找设备、准备profile，不start
    auto devices = context_.queryDeviceList();
    if (devices->deviceCount() == 0) {
        throw std::runtime_error("no device");
    } else {
        std::cout << "device count: " << devices->deviceCount() << "\n";
    }
    auto device = devices->getDevice(0);

    // 申请内存并赋值
    pipeline_ = std::shared_ptr<ob::Pipeline>(new ob::Pipeline(device));;

    auto colorProfiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
    colorProfile_ = colorProfiles->getVideoStreamProfile(width, height, format, fps);
    config_ = std::make_shared<ob::Config>();
    config_->enableStream(colorProfile_);
}

void ColorCamera::open() {
    if(!pipeline_ || !config_) throw std::runtime_error("call init first");
    pipeline_->start(config_);
    started_ = true;
}

void ColorCamera::close() {
    if(started_ && pipeline_) {
        try{
            pipeline_->stop();
        }
        catch(...){
            
        }
        started_ = false;
    }
}

bool ColorCamera::cameraIntrinsic(CameraIntrinsic &out) const {
    if(!colorProfile_) {
        return false;
    }

    const OBCameraIntrinsic in = colorProfile_->getIntrinsic();
    out.width  = in.width;
    out.height = in.height;
    out.fx     = in.fx;
    out.fy     = in.fy;
    out.cx     = in.cx;
    out.cy     = in.cy;
    return true;
}

bool ColorCamera::cameraParam(CameraParam &out) const {
    if(!colorProfile_) {
        return false;
    }

    const OBCameraIntrinsic  in = colorProfile_->getIntrinsic();
    const OBCameraDistortion d  = colorProfile_->getDistortion();

    out.intrinsic.width  = in.width;
    out.intrinsic.height = in.height;
    out.intrinsic.fx     = in.fx;
    out.intrinsic.fy     = in.fy;
    out.intrinsic.cx     = in.cx;
    out.intrinsic.cy     = in.cy;

    out.distortion.k1 = d.k1;
    out.distortion.k2 = d.k2;
    out.distortion.k3 = d.k3;
    out.distortion.k4 = d.k4;
    out.distortion.k5 = d.k5;
    out.distortion.k6 = d.k6;
    out.distortion.p1 = d.p1;
    out.distortion.p2 = d.p2;
    return true;
}

void ColorCamera::printCameraParam() {
    CameraParam param;
    if(!cameraParam(param)) {
        std::cout << " [Camera param] colorProfile_ is null, call init() first" << std::endl;
        return;
    }

    const CameraIntrinsic  &in = param.intrinsic;
    const CameraDistortion &d  = param.distortion;

    std::cout << " type=" << streamTypeName(colorProfile_->type()) << std::endl;
    std::cout << " [Camera intrinsic]" << std::endl;
    std::cout << " width=" << in.width << " height=" << in.height << std::endl;
    std::cout << " fx=" << in.fx << " fy=" << in.fy << std::endl;
    std::cout << " cx=" << in.cx << " cy=" << in.cy << std::endl;
    std::cout << " [Camera distortion]" << std::endl;
    std::cout << " k1=" << d.k1 << " k2=" << d.k2 << " k3=" << d.k3 << " k4=" << d.k4 << " k5=" << d.k5
                << " k6=" << d.k6 << std::endl;
    std::cout << " p1=" << d.p1 << " p2=" << d.p2 << std::endl;
}

std::shared_ptr<ob::FrameSet> ColorCamera::waitForFrames(uint32_t timeoutMs) {
    if(!pipeline_) {
        throw std::runtime_error("call init() first");
    }
    // 同步阻塞等待，超时返回 nullptr
    return pipeline_->waitForFrames(timeoutMs);
}

std::shared_ptr<ob::VideoStreamProfile> ColorCamera::colorProfile() const {
    return colorProfile_;
}

}  // namespace obcam
