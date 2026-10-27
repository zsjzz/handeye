#include "config.hpp"
#include "robot_rm.hpp"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>

namespace {

template <std::size_t N>
void printValues(const char* label, const double (&values)[N]) {
    std::cout << label;
    for (const double value : values) {
        std::cout << ' ' << value;
    }
    std::cout << '\n';
}

}  // namespace

int main() {
    try {
        // 与 test_config 相同，按测试源码的位置查找项目配置。
        const auto config_path = std::filesystem::path(__FILE__).parent_path().parent_path()
                                 / "config" / "config.yaml";
        const auto config = handeye::Config::load(config_path.string());

        std::cout << "配置文件: " << config_path.string() << '\n'
                  << "机械臂地址: " << config.robot.ip << ':' << config.robot.port
                  << std::endl;

        // 只查询状态，不发送运动指令。函数返回时连接已释放。
        const auto state = handeye::readRobotState(config.robot.ip, config.robot.port);

        std::cout << "\n机械臂状态:\n"
                  << "工作坐标系: " << std::quoted(state.work_frame) << '\n'
                  << "工具坐标系: " << std::quoted(state.tool_frame) << '\n'
                  << std::fixed << std::setprecision(6);
        printValues("末端位置 (m; x, y, z):", state.position_m);
        printValues("末端欧拉角 (rad; rx, ry, rz):", state.euler_rad);
        printValues("末端四元数 (w, x, y, z):", state.quaternion);
        printValues("关节角 (deg; J1-J7，6 轴机型 J7 为 0):", state.joint_deg);

        std::cout << "T_base_flange (4x4):\n";
        for (const auto& row : state.T_base_flange) {
            for (const double value : row) {
                std::cout << std::setw(12) << value << ' ';
            }
            std::cout << '\n';
        }

        bool has_error = false;
        std::cout << "控制器错误码条目数: " << state.error_count << '\n';
        for (int i = 0; i < state.error_count; ++i) {
            const int error = state.errors[i];
            has_error = has_error || error != 0;
            std::cout << "  [" << i << "]: " << error << " (0x"
                      << std::hex << error << std::dec << ")\n";
        }
        std::cout << "控制器是否上报故障: " << (has_error ? "是" : "否") << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "机械臂状态读取失败: " << error.what() << '\n';
        return 1;
    }
}
