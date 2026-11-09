#include "HR_Pro.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;
static constexpr double PI = 3.14159265358979323846;

void print_ret(const std::string& cmd, int ret)
{
    if (ret == 0) {
        std::cout << "[OK]  " << cmd << std::endl;
    } 
    else {
        std::string errMsg;
        const int lookupRet = HRIF_GetErrorCodeStr(BOX_ID, ret, errMsg);
        std::cout << "[ERR] " << cmd << " -> " << ret;
        if (lookupRet == 0) std::cout << ": " << errMsg;
        std::cout << std::endl;
    }
}

int main()
{
    std::cout << "======== SpeedJ Online Joint Velocity Control Example ========" << std::endl;

    // ----------------------------------------------------------
    // 1. 连接机器人
    // ----------------------------------------------------------
    // 需根据机器人的实际 IP 进行修改
    const std::string ip = "192.168.0.10";
    const unsigned int port = 10003;

    int ret = HRIF_Connect(BOX_ID, ip.c_str(), port);
    print_ret("HRIF_Connect", ret);
    if (ret != 0) {
        return EXIT_FAILURE;
    }

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

    JointsData safeJoints;
    safeJoints.dJoint[0] = 90;    safeJoints.dJoint[1] = 45;
    safeJoints.dJoint[2] = 0;    safeJoints.dJoint[3] = -80;
    safeJoints.dJoint[4] = 0;    safeJoints.dJoint[5] = 35;
    safeJoints.dJoint[6] = 0;

    ret = HRIF_MoveJ_nJ(BOX_ID, RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, safeJoints,
                        "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "SpeedJ_start");
                        
    print_ret("HRIF_MoveJ_nJ", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    bool motionDone = false;
    const auto motionDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (!motionDone && std::chrono::steady_clock::now() < motionDeadline) {
        ret = HRIF_IsMotionDone(BOX_ID, RBT_ID, motionDone);
        if (ret != 0) {
            print_ret("HRIF_IsMotionDone", ret);
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!motionDone) {
        std::cerr << "[ERR] HRIF_IsMotionDone timed out" << std::endl;
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 4. SpeedJ J1\J2关节画圆的相关参数
    // ----------------------------------------------------------
    const double period = 8.0;             // 完成一周所需时间，s
    const double tAmp = 10.0;    // 目标关节空间圆半径，deg
    const double dt = 0.05;                // SpeedJ 指令更新时间，s
    const double omega = 2.0 * PI / period;
    const double jointAcc = 50.0;          // 关节加速度，deg/s^2
    const double runtime = dt * 2.0;       // 运行时间
    const int sampleCount = static_cast<int>(period / dt);

    std::cout << "[INFO] Starting continuous SpeedJ command streaming: period="
              << period << " s, target amplitude=" << tAmp
              << " deg " << std::endl;

    // ----------------------------------------------------------
    // 5. 连续下发关节速度，绘制一个圆形轨迹
    // ----------------------------------------------------------
    auto nextSendTime = std::chrono::steady_clock::now();
    JointsData jointVel;

    for (int sample = 0; sample < sampleCount; ++sample) {
        const double t = sample * dt;
        jointVel.dJoint[0] = tAmp * omega * std::cos(omega * t);
        jointVel.dJoint[1] = -tAmp * omega * std::sin(omega * t);

        ret = HRIF_SpeedJ_nJ(BOX_ID, RBT_ID, jointVel, jointAcc, runtime);

        if (ret != 0) {
            print_ret("HRIF_SpeedJ", ret);
            HRIF_GrpStop(BOX_ID, RBT_ID);
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }

        nextSendTime +=
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(dt));
        std::this_thread::sleep_until(nextSendTime);
    }

    // ----------------------------------------------------------
    // 6. 发送零关节速度，使机器人平稳停止。
    // ----------------------------------------------------------
    jointVel.dJoint[0] = 0;
    jointVel.dJoint[1] = 0;

    ret = HRIF_SpeedJ_nJ(BOX_ID, RBT_ID, jointVel, jointAcc, 0.5);
    print_ret("HRIF_SpeedJ(stop)", ret);
    const int commandRet = ret;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[OK] SpeedJ motion completed." << std::endl;

    const int stopRet = HRIF_GrpStop(BOX_ID, RBT_ID);
    print_ret("HRIF_GrpStop", stopRet);
    const int disconnectRet = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", disconnectRet);
    return commandRet == 0 && stopRet == 0 && disconnectRet == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
