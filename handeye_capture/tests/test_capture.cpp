#include "camera_opencv.hpp"
#include "sample_writer.hpp"

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

#if defined(__linux__)
#include <csignal>
#include <sys/resource.h>
#endif

namespace {

void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename Function>
void expectFailure(Function function) {
    try { function(); }
    catch (const std::exception&) { return; }
    throw std::runtime_error("Expected operation to fail");
}

std::string readFile(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    check(static_cast<bool>(stream), "Cannot read " + path.string());
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

struct Fixture {
    std::filesystem::path root;
    handeye::Config config;
    obcam::CameraParam actual;

    Fixture() {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        bool created = false;
        for (int i = 0; i < 100; ++i) {
            root = std::filesystem::temp_directory_path() /
                   ("handeye-capture-test-" + std::to_string(stamp) + '-' + std::to_string(i));
            if (std::filesystem::create_directory(root)) { created = true; break; }
        }
        check(created, "Cannot create test directory");
        config.camera = {4, 2, 30, "MJPG", 0};
        config.board = {8, 6, 0.025};
        config.robot = {"192.168.1.18", 8080};
        actual.intrinsic = {4, 2, 100.f, 101.f, 2.f, 1.f};
        actual.distortion.k1 = 0.1f;
        actual.distortion.p1 = 0.01f;
        resetParameters();
    }

    ~Fixture() {
        std::error_code ignored;
        std::filesystem::remove_all(root, ignored);
    }

    void writeParameters(const YAML::Node& node) const {
        std::ofstream stream(root / "camera_param.yaml");
        stream << node;
        stream.close();
        check(static_cast<bool>(stream), "Cannot write camera parameters");
    }

    void resetParameters() const {
        auto node = YAML::Load(R"(
stream: {width: 4, height: 2, fps: 30, format: MJPG}
intrinsics: {fx: 100, fy: 101, cx: 2, cy: 1}
camera_matrix: [[100, 0, 2], [0, 101, 1], [0, 0, 1]]
distortion_order: [k1, k2, p1, p2, k3, k4, k5, k6]
distortion_coefficients: [0.1, 0, 0.01, 0, 0, 0, 0, 0]
)");
        writeParameters(node);
    }

    handeye::SampleInfo sample() const {
        handeye::SampleInfo info;
        info.captured_at = handeye::CaptureClock::time_point(std::chrono::microseconds(123456));
        info.frame_received_at = info.captured_at - std::chrono::milliseconds(10);
        info.robot_query_started_at = info.captured_at + std::chrono::milliseconds(1);
        info.robot_query_finished_at = info.captured_at + std::chrono::milliseconds(50);
        info.frame_index = 42;
        info.device_timestamp_us = 987654321;
        info.frame_format = "MJPG";
        info.robot.work_frame = "World";
        info.robot.tool_frame = "Arm_Tip";
        info.robot.position_m[0] = 0.123456789;
        info.robot.T_base_flange[0][3] = info.robot.position_m[0];
        info.robot.quaternion[0] = 1.0;
        info.robot.joint_deg[0] = -29.620001;
        info.robot.error_count = 1;  // SDK 也会以 [0] 表示无故障。
        return info;
    }
};

class Runner {
public:
    template <typename Function>
    void run(const char* name, Function function) {
        try { function(); ++passed_; std::cout << "[PASS] " << name << '\n'; }
        catch (const std::exception& error) {
            ++failed_; std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
        }
    }
    int finish() const {
        std::cout << passed_ << " passed, " << failed_ << " failed\n";
        return failed_ == 0 ? 0 : 1;
    }
private:
    int passed_ = 0;
    int failed_ = 0;
};

}  // namespace

int main() {
    Runner runner;
    runner.run("UTC naming is independent of local timezone", [] {
        const char* old = std::getenv("TZ");
        const bool had_old = old != nullptr;
        const std::string previous = old ? old : "";
        setenv("TZ", "Asia/Shanghai", 1);
        tzset();
        const auto timestamp = handeye::CaptureClock::time_point(std::chrono::microseconds(123456));
        const auto directory = handeye::formatUtc(timestamp, true);
        const auto readable = handeye::formatUtc(timestamp);
        if (had_old) setenv("TZ", previous.c_str(), 1);
        else unsetenv("TZ");
        tzset();
        check(directory == "19700101T000000.123456Z", "Directory name used local time");
        check(readable == "1970-01-01T00:00:00.123456Z", "Incorrect UTC timestamp");
    });
    runner.run("raw RGB conversion swaps channels and owns pixels", [] {
        unsigned char rgb[] = {255, 0, 0, 0, 255, 0};
        const auto image = handeye::decodeColorImage(OB_FORMAT_RGB, rgb, sizeof(rgb), 2, 1);
        rgb[0] = 0;
        check(image.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 0, 255), "Incorrect or borrowed red pixel");
        check(image.at<cv::Vec3b>(0, 1) == cv::Vec3b(0, 255, 0), "Incorrect green pixel");
    });
    runner.run("MJPG is decoded into a color image", [] {
        const cv::Mat source(12, 16, CV_8UC3, cv::Scalar(20, 80, 200));
        std::vector<unsigned char> jpeg;
        check(cv::imencode(".jpg", source, jpeg), "Cannot encode fixture");
        const auto image = handeye::decodeColorImage(OB_FORMAT_MJPG, jpeg.data(), jpeg.size(), 16, 12);
        check(image.type() == CV_8UC3 && image.size() == source.size(), "Incorrect decoded image");
        check(cv::norm(image, source, cv::NORM_INF) < 5, "JPEG color channels are incorrect");
        expectFailure([&] { handeye::decodeColorImage(OB_FORMAT_MJPG, jpeg.data(), jpeg.size(), 17, 12); });
    });
    runner.run("reject incomplete and invalid frames", [] {
        unsigned char bytes[4] = {};
        expectFailure([&] { handeye::decodeColorImage(OB_FORMAT_BGR, bytes, 4, 2, 2); });
        expectFailure([&] { handeye::decodeColorImage(OB_FORMAT_MJPG, bytes, 4, 2, 2); });
        expectFailure([&] { handeye::decodeColorImage(OB_FORMAT_RGB, nullptr, 4, 2, 2); });
        expectFailure([&] { handeye::parseColorFormat("H264"); });
    });
    runner.run("accept matching camera file without modifying it", [] {
        Fixture fixture;
        const auto file = fixture.root / "camera_param.yaml";
        const auto before = readFile(file);
        handeye::validateCameraParamFile(file, fixture.config.camera, fixture.actual);
        check(readFile(file) == before, "Camera parameter file was changed");
    });
    runner.run("reject mismatched calibration and distortion order", [] {
        Fixture fixture;
        const auto file = fixture.root / "camera_param.yaml";
        auto node = YAML::LoadFile(file.string());
        node["stream"]["width"] = 8;
        fixture.writeParameters(node);
        expectFailure([&] { handeye::validateCameraParamFile(file, fixture.config.camera, fixture.actual); });
        fixture.resetParameters();
        node = YAML::LoadFile(file.string());
        node["camera_matrix"][0][0] = 200;
        fixture.writeParameters(node);
        expectFailure([&] { handeye::validateCameraParamFile(file, fixture.config.camera, fixture.actual); });
        fixture.resetParameters();
        node = YAML::LoadFile(file.string());
        node["distortion_order"][2] = "k3";
        fixture.writeParameters(node);
        expectFailure([&] { handeye::validateCameraParamFile(file, fixture.config.camera, fixture.actual); });
    });
    runner.run("save lossless image and its exact robot state", [] {
        Fixture fixture;
        const cv::Mat image(2, 4, CV_8UC3, cv::Scalar(15, 23, 79));
        const auto sample = fixture.sample();
        const auto parameters = readFile(fixture.root / "camera_param.yaml");
        const auto saved = handeye::saveSample(fixture.root, image, fixture.config, sample);
        check(saved.filename() == "19700101T000000.123456Z", "Incorrect sample directory");
        const auto restored = cv::imread((saved / "image.png").string());
        check(restored.size() == image.size() && cv::norm(image, restored, cv::NORM_INF) == 0,
              "Saved pixels differ from captured image");
        const auto data = YAML::LoadFile((saved / "sample.yaml").string());
        check(data["captured_at_utc"].as<std::string>() == handeye::formatUtc(sample.captured_at), "Capture time");
        check(data["frame"]["index"].as<int>() == 42, "Wrong frame paired with image");
        check(data["frame"]["device_timestamp_us"].as<std::uint64_t>() == sample.device_timestamp_us, "Device timestamp");
        check(data["frame"]["received_at_utc"].as<std::string>() == handeye::formatUtc(sample.frame_received_at), "Frame receipt time");
        check(data["robot_query"]["finished_at_utc"].as<std::string>() == handeye::formatUtc(sample.robot_query_finished_at), "Query time");
        check(data["robot"]["T_base_flange"][0][3].as<double>() == sample.robot.position_m[0], "Robot precision lost");
        check(data["robot"]["joint_deg"][0].as<double>() == sample.robot.joint_deg[0], "Joint angle changed");
        check(data["robot"]["joint_deg"].size() == 7 && data["robot"]["quaternion_order"].as<std::string>() == "wxyz", "Pose conventions");
        check(!data["robot"]["has_error"].as<bool>() && data["robot"]["errors"][0].as<int>() == 0, "Zero error misclassified");
        check(std::filesystem::equivalent(saved / data["camera_param_file"].as<std::string>(), fixture.root / "camera_param.yaml"), "Broken calibration reference");
        check(readFile(fixture.root / "camera_param.yaml") == parameters, "Camera parameters overwritten");
        check(!std::filesystem::exists(fixture.root / ("." + saved.filename().string() + ".tmp")), "Temporary directory left behind");
    });
    runner.run("duplicate timestamp never overwrites existing sample", [] {
        Fixture fixture;
        cv::Mat image(2, 4, CV_8UC3, cv::Scalar(15, 23, 79));
        const auto sample = fixture.sample();
        const auto saved = handeye::saveSample(fixture.root, image, fixture.config, sample);
        const auto original_image = readFile(saved / "image.png");
        const auto original_yaml = readFile(saved / "sample.yaml");
        image.setTo(cv::Scalar(255, 255, 255));
        expectFailure([&] { handeye::saveSample(fixture.root, image, fixture.config, sample); });
        check(readFile(saved / "image.png") == original_image, "Existing image overwritten");
        check(readFile(saved / "sample.yaml") == original_yaml, "Existing state overwritten");
    });
    runner.run("reject invalid samples without leaving partial directories", [] {
        Fixture fixture;
        const cv::Mat image(2, 4, CV_8UC3, cv::Scalar(15, 23, 79));
        auto sample = fixture.sample();
        expectFailure([&] { handeye::saveSample(fixture.root, cv::Mat(), fixture.config, sample); });
        sample.robot.position_m[0] = std::numeric_limits<double>::quiet_NaN();
        expectFailure([&] { handeye::saveSample(fixture.root, image, fixture.config, sample); });
        sample = fixture.sample();
        sample.robot.error_count = 25;
        expectFailure([&] { handeye::saveSample(fixture.root, image, fixture.config, sample); });
        check(std::distance(std::filesystem::directory_iterator(fixture.root), std::filesystem::directory_iterator()) == 1,
              "Invalid capture left partial files");
        std::filesystem::remove(fixture.root / "camera_param.yaml");
        sample = fixture.sample();
        expectFailure([&] { handeye::saveSample(fixture.root, image, fixture.config, sample); });
        check(std::filesystem::is_empty(fixture.root), "Missing calibration left sample files");
    });
    runner.run("do not remove another writer's temporary directory", [] {
        Fixture fixture;
        const auto sample = fixture.sample();
        const auto temporary = fixture.root / ("." + handeye::formatUtc(sample.captured_at, true) + ".tmp");
        std::filesystem::create_directory(temporary);
        std::ofstream(temporary / "marker") << "keep";
        const cv::Mat image(2, 4, CV_8UC3, cv::Scalar(15, 23, 79));
        expectFailure([&] { handeye::saveSample(fixture.root, image, fixture.config, sample); });
        check(readFile(temporary / "marker") == "keep", "Removed another writer's files");
    });
#if defined(__linux__)
    runner.run("write failure cleans the whole unpublished sample", [] {
        Fixture fixture;
        const auto parameters = readFile(fixture.root / "camera_param.yaml");
        const cv::Mat image(2, 4, CV_8UC3, cv::Scalar(15, 23, 79));
        const auto sample = fixture.sample();
        struct FileSizeLimit {
            rlimit previous{};
            struct sigaction previous_signal{};
            FileSizeLimit() {
                check(getrlimit(RLIMIT_FSIZE, &previous) == 0, "Cannot read file limit");
                struct sigaction ignore{};
                ignore.sa_handler = SIG_IGN;
                sigemptyset(&ignore.sa_mask);
                check(sigaction(SIGXFSZ, &ignore, &previous_signal) == 0, "Cannot ignore SIGXFSZ");
                rlimit limit = previous;
                limit.rlim_cur = 512;  // PNG 可写完，但 YAML 写入必定超限。
                if (setrlimit(RLIMIT_FSIZE, &limit) != 0) {
                    sigaction(SIGXFSZ, &previous_signal, nullptr);
                    throw std::runtime_error("Cannot set file limit");
                }
            }
            ~FileSizeLimit() {
                setrlimit(RLIMIT_FSIZE, &previous);
                sigaction(SIGXFSZ, &previous_signal, nullptr);
            }
        };
        {
            FileSizeLimit limit;
            expectFailure([&] { handeye::saveSample(fixture.root, image, fixture.config, sample); });
        }
        check(std::distance(std::filesystem::directory_iterator(fixture.root), std::filesystem::directory_iterator()) == 1,
              "Disk write failure left a complete or partial sample");
        check(readFile(fixture.root / "camera_param.yaml") == parameters, "Write failure changed calibration");
    });
#endif
    return runner.finish();
}
