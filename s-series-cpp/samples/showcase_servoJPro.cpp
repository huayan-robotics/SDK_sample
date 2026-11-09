#include "HR_Pro.h"

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdlib>
#include <vector>

// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;
static constexpr double PI = 3.14159265358979323846;

void print_ret(const std::string& cmd, int ret)
{
    if (ret == 0) {
        std::cout << "[OK]  " << cmd << std::endl;
    } else {
        std::string err_msg;
        const int lookupRet = HRIF_GetErrorCodeStr(BOX_ID, ret, err_msg);
        std::cout << "[ERR] " << cmd << " -> " << ret;
        if (lookupRet == 0) std::cout << ": " << err_msg;
        std::cout << std::endl;
    }
}

int main()
{
    std::cout << "=== ServoJ 6-Axis Joint-Space Online Control Example ===" << std::endl;

    // ----------------------------------------------------------
    // 1. 连接机器人
    // ----------------------------------------------------------
    // 需根据机器人的实际 IP 进行修改
    std::string host = "192.168.0.10";
    unsigned short port = 10003;
    int ret = HRIF_Connect(BOX_ID, host.c_str(), port);
    print_ret("HRIF_Connect", ret);
    if (ret != 0) return EXIT_FAILURE;

    // ----------------------------------------------------------
    // 2. 确认机器人处于 Standby(33)
    // ----------------------------------------------------------
    int nFSM = 0;
    ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, nFSM);
    print_ret("HRIF_ReadCurFSM (FSM=" + std::to_string(nFSM) + ")", ret);
    if (ret != 0 || nFSM != 33) {
        std::cerr << "[ERR] Robot must be in Standby(33), FSM=" << nFSM << std::endl;
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 3. 设置速度比并 MoveJ 运动到安全起始位姿
    // ----------------------------------------------------------
    ret = HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
    print_ret("HRIF_SetOverride", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    double dJ1 = 0;  double dJ2 = -90;  double dJ3 = 90;
    double dJ4 = -90;  double dJ5 = -90; double dJ6 = 0;

    std::cout << "[INFO] Moving to a safe position with MoveJ..." << std::endl;
    ret = HRIF_MoveJ(BOX_ID, RBT_ID,
                     0, 0, 0, 0, 0, 0,
                     dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
                     "TCP", "Base",
                     30.0, 60.0, 0.0,
                     1, 0, 0, 0, "servo_home");
    print_ret("HRIF_MoveJ", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    bool bDone = false;
    const auto motionDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ret = HRIF_IsMotionDone(BOX_ID, RBT_ID, bDone);
    } while (ret == 0 && !bDone && std::chrono::steady_clock::now() < motionDeadline);
    if (ret != 0 || !bDone) {
        if (ret != 0) print_ret("HRIF_IsMotionDone", ret); else std::cerr << "[ERR] MoveJ timed out" << std::endl;
        HRIF_DisConnect(BOX_ID); return EXIT_FAILURE;
    }
    std::cout << "[INFO] Safe position reached" << std::endl;

    // ----------------------------------------------------------
    // 4. 读取当前关节位置（MoveJ 后的基准）
    // ----------------------------------------------------------
    double curJ1 = 0; double curJ2 = 0; double curJ3 = 0;
    double curJ4 = 0; double curJ5 = 0; double curJ6 = 0;
    ret = HRIF_ReadActJointPos(BOX_ID, RBT_ID,
                               curJ1, curJ2, curJ3, curJ4, curJ5, curJ6);
    print_ret("HRIF_ReadActJointPos", ret);
    if (ret != 0) { HRIF_DisConnect(BOX_ID); return EXIT_FAILURE; }

    std::vector<double> curJoints = {curJ1, curJ2, curJ3, curJ4, curJ5, curJ6};

    // ----------------------------------------------------------
    // 5. 启动 Servo 模式
    // ----------------------------------------------------------
    const double STIFFNESS_FACTOR  = 8000;   // 刚度增益系数 (>=1 生效)
    const double LOOKAHEAD_TIME = 0.05;   // 前瞻 50ms
    std::cout << "[INFO] Stiffness factor=" << STIFFNESS_FACTOR
              << "s, lookahead=" << LOOKAHEAD_TIME << "s" << std::endl;
    ret = HRIF_StartServo(BOX_ID, RBT_ID, STIFFNESS_FACTOR, LOOKAHEAD_TIME);
    print_ret("HRIF_StartServo", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // ----------------------------------------------------------
    // 6. 先推送当前关节位置（同步，避免首帧突变）
    // ----------------------------------------------------------
    ret = HRIF_PushServoJ(BOX_ID, RBT_ID,
                          curJ1, curJ2, curJ3, curJ4, curJ5, curJ6);
    print_ret("HRIF_PushServoJ (sync)", ret);
    bool push_succeeded = (ret == 0);

    // ----------------------------------------------------------
    // 7. 基于当前关节位置生成正弦轨迹并推送
    // ----------------------------------------------------------
    const double AMP            = 5.0;    // 振幅 5°
    const double FREQ           = 0.5;    // 频率 0.5Hz
    const double DURATION       = 2.0;    // 持续 2 秒
    const double PUSH_PERIOD = 0.004; // 推送周期 4ms（250 Hz）
    const int    JOINT_COUNT    = 6;
    double dir[6] = {1, 1, 1, 1, 1, 0};
    auto t0 = std::chrono::steady_clock::now();
    int total = static_cast<int>(DURATION / PUSH_PERIOD);

    for (int i = 1; push_succeeded && i <= total; ++i)
    {
        std::this_thread::sleep_until(
            t0 + std::chrono::duration<double>(i * PUSH_PERIOD));
        double t = i * PUSH_PERIOD;

        std::vector<double> target = curJoints;
        for (int j = 0; j < JOINT_COUNT; ++j)
            target[j] = curJoints[j] + dir[j] * AMP * std::sin(2.0 * PI * FREQ * t);
        ret = HRIF_PushServoJ(BOX_ID, RBT_ID,
            target[0], target[1], target[2],
            target[3], target[4], target[5]);
        if (ret != 0) {
            print_ret("Servo trajectory push", ret);
            push_succeeded = false;
            break;
        }
    }

    // ----------------------------------------------------------
    // 8. 停止并断开连接
    // ----------------------------------------------------------
    std::cout << "[INFO] ServoJ completed; disconnecting" << std::endl;
    const int stop_ret = HRIF_GrpStop(BOX_ID, RBT_ID);
    print_ret("HRIF_GrpStop", stop_ret);
    const int disconnect_ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", disconnect_ret);
    return push_succeeded && stop_ret == 0 && disconnect_ret == 0
        ? EXIT_SUCCESS : EXIT_FAILURE;
}
