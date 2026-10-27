#pragma once

#include <string>

namespace handeye {

struct RobotState {
    // 基座系下的末端位姿 ^base T_flange（4x4 行主序，单位米）。
    // 由 position + euler 经 SDK 的 rm_algo_euler2matrix 转换得到，可直接写入样本。
    double T_base_flange[4][4] = {{1.0, 0.0, 0.0, 0.0},
                                  {0.0, 1.0, 0.0, 0.0},
                                  {0.0, 0.0, 1.0, 0.0},
                                  {0.0, 0.0, 0.0, 1.0}};

    // 末端位置
    double position_m[3] = {0.0, 0.0, 0.0};

    // 末端四元数 w, x, y, z
    double quaternion[4] = {0.0, 0.0, 0.0, 0.0};

    // 末端欧拉角
    double euler_rad[3] = {0.0, 0.0, 0.0};

    // 读取时控制器的工作坐标系 / 工具坐标系名称（如 "World" / "Arm_Tip"）
    std::string work_frame;
    std::string tool_frame;

    // 各关节角（度）。数组长度固定为 ARM_DOF = 7，6 轴机型最后一位为 0。
    double joint_deg[7] = {};

    // 控制器上报的错误码。注意：控制器有时会上报 1 个值为 0 的条目，
    // 那同样表示无错误；判断是否有故障请看是否出现非 0 值。
    int error_count = 0;
    int errors[24] = {};
};

RobotState readRobotState(const std::string& ip, int port = 8080);

}  // namespace handeye
