#include "robot_rm.hpp"

#include "rm_interface.h"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace handeye {
namespace {

constexpr double kPi = 3.14159265358979323846;

// RM_API2 统一用 0 表示成功、非 0 表示错误。
// 统一在这里转成异常，并带上操作名和原始错误码，便于现场定位。
void checkSdk(int result, const char* operation) {
    if(result != 0) {
        throw std::runtime_error(std::string(operation) + " failed (RM API2 code " +
                                 std::to_string(result) + ")");
    }
}

// 控制器返回的坐标系名称是定长 char[12]（不一定以 '\0' 结尾）。
std::string frameName(const char (&name)[12]) {
    std::size_t length = 0;
    while(length < 12 && name[length] != '\0') {
        ++length;
    }
    return std::string(name, length);
}

struct Session {
    rm_robot_handle* handle = nullptr;

    // 构造：创建机械臂句柄
    Session(const char* ip, int port) {
        checkSdk(rm_init(RM_SINGLE_MODE_E), "rm_init");
        handle = rm_create_robot_arm(ip, port);
    }

    // 析构：释放句柄
    ~Session() {
        if(handle != nullptr) {
            rm_delete_robot_arm(handle);
        }
        rm_destroy();
    }

};

}  // namespace

RobotState readRobotState(const std::string& ip, int port) {
    std::cout << "[机械臂] connect " << ip << ":" << port << " ..." << std::endl;
    Session session(ip.c_str(), port);

    if(session.handle == nullptr || session.handle->id <= 0) {
        throw std::runtime_error("rm_create_robot_arm failed for " + ip + ":" +
                                 std::to_string(port));
    }
    std::cout << "[机械臂] connect success (handle ID=" << session.handle->id << ")\n";

    RobotState result;

    // 查询当前工作系/工具系名称，写入样本便于事后追溯。
    rm_frame_t work{};
    rm_frame_t tool{};
    checkSdk(rm_get_current_work_frame(session.handle, &work), "rm_get_current_work_frame");
    checkSdk(rm_get_current_tool_frame(session.handle, &tool), "rm_get_current_tool_frame");
    result.work_frame = frameName(work.frame_name);
    result.tool_frame = frameName(tool.frame_name);

    // 查询一次当前状态：位姿（position + euler + quaternion）、关节角、错误码。
    rm_current_arm_state_t state{};
    checkSdk(rm_get_current_arm_state(session.handle, &state), "rm_get_current_arm_state");

    // 4) 整理成对外结构。单位保持 SDK 原样：位置米、姿态弧度、关节角已是度。
    result.position_m[0] = state.pose.position.x;
    result.position_m[1] = state.pose.position.y;
    result.position_m[2] = state.pose.position.z;
    result.euler_rad[0] = state.pose.euler.rx;
    result.euler_rad[1] = state.pose.euler.ry;
    result.euler_rad[2] = state.pose.euler.rz;
    result.quaternion[0] = state.pose.quaternion.w;
    result.quaternion[1] = state.pose.quaternion.x;
    result.quaternion[2] = state.pose.quaternion.y;
    result.quaternion[3] = state.pose.quaternion.z;

    // 欧拉角转旋转矩阵由 SDK 完成（旋转顺序由厂商定义，不要自己猜）。
    const rm_matrix_t rotation = rm_algo_euler2matrix(state.pose.euler);
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            result.T_base_flange[row][col] = rotation.data[row][col];
        }
        result.T_base_flange[row][3] = result.position_m[row];
    }

    for(int i = 0; i < ARM_DOF; ++i) {
        result.joint_deg[i] = state.joint[i];
    }

    std::cout << "[机械臂] 末端位置(m)    : x=" << std::fixed << std::setprecision(4) 
              << result.position_m[0] << " y=" << result.position_m[1] << " z=" << result.position_m[2] << "\n";

    std::cout << "[机械臂] 坐标系        : work=" << result.work_frame
              << " tool=" << result.tool_frame << "\n";

    std::cout << "[机械臂] 末端欧拉角(deg): rx=" << std::fixed << std::setprecision(1)
              << result.euler_rad[0] * 180.0 / kPi << " ry=" << result.euler_rad[1] * 180.0 / kPi
              << " rz=" << result.euler_rad[2] * 180.0 / kPi << "\n";

    std::cout << "[机械臂] 关节(deg):";
    for(int i = 0; i < ARM_DOF; ++i) {
        std::cout << " " << std::setw(8) << std::fixed << std::setprecision(2) << result.joint_deg[i];
        if (i == ARM_DOF - 1) std::cout << '\n';
    }

    // err_len 是控制器上报的错误码个数，先截断到数组长度再用。
    result.error_count = state.err.err_len > 24 ? 24 : state.err.err_len;
    for(int i = 0; i < result.error_count; ++i) {
        result.errors[i] = state.err.err[i];
    }

    // 函数返回时 Session 析构：删除句柄 + rm_destroy（异常路径同样生效）。
    std::cout << "[机械臂] close connection\n";
    return result;
}

}  // namespace handeye
