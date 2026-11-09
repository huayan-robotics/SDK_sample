#include "HR_Pro.h"

#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;

void print_ret(const std::string& cmd, int ret)
{
    if (ret == 0) {
        std::cout << "[OK]  " << cmd << std::endl;
    } else if (ret == 39500) {
        std::cout << "[ERR] " << cmd
                  << " -> 39500: Not connected to the robot." << std::endl;
    } else {
        std::string errMsg;
        const int lookupRet = HRIF_GetErrorCodeStr(BOX_ID, ret, errMsg);
        std::cout << "[ERR] " << cmd << " -> " << ret;
        if (lookupRet == 0) std::cout << ": " << errMsg;
        std::cout << std::endl;
    }
}

int main()
{
    std::cout << "======== SpeedL Online Cartesian Velocity Control Example ========" << std::endl;

    // ----------------------------------------------------------
    // 1. 连接机器人
    // ----------------------------------------------------------
    // 需根据机器人的实际 IP 进行修改
    const std::string ip = "192.168.0.10";
    const unsigned int port = 10003;
    int ret = HRIF_Connect(BOX_ID, ip.c_str(), port);
    print_ret("HRIF_Connect", ret);
    if (ret != 0) { return EXIT_FAILURE; }

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

    ret = HRIF_MoveJ_nJ(BOX_ID, RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        safeJoints, "TCP", "Base", 30.0, 60.0, 0.0, 1, 0, 0, 0, "SpeedL_start");
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
    if (!motionDone) { std::cerr << "[ERR] HRIF_IsMotionDone timed out" << std::endl; HRIF_DisConnect(BOX_ID); return EXIT_FAILURE; }

    // ----------------------------------------------------------
    // 4. 相关参数设置
    // ----------------------------------------------------------
    const double linearSpeed = 50.0;      // SpeedL 指令线速度，mm/s
    const double linearAcc = 100.0;        // TCP 线加速度，mm/s^2
    const double angularAcc = 100.0;       // TCP 角加速度，deg/s^2
    const double directionHoldTime = 4.0;  // 每个方向保持时间，s
    const double commandRuntime = 5.0;     // 必须大于方向保持时间

    const std::array<std::array<double, 2>, 4> directions = {{
        {{ -1.0,  0.0 }},
        {{ 0,  -1 }},
        {{ 1,  0 }},
        {{ 0,  1 }}
    }};

    std::cout << "[INFO] Starting continuous SpeedL command streaming"
              << ", commanded speed=" << linearSpeed << " mm/s"
              << ", speed override=" << 0.2
              << std::endl;

    // ----------------------------------------------------------
    // 5. 连续下发空间速度。绘制一个长方形
    // ----------------------------------------------------------
    auto nextDirectionTime = std::chrono::steady_clock::now();

    for (std::size_t i = 0; i < directions.size(); ++i) {
        const double vy = linearSpeed * directions[i][0];
        const double vz = linearSpeed * directions[i][1];

        std::cout << "[INFO] Commanded direction " << i + 1
                  << ": Vy=" << vy
                  << " mm/s, Vz=" << vz << " mm/s"
                  << std::endl;

        ret = HRIF_SpeedL(BOX_ID, RBT_ID, 0.0, vy, vz, 0.0, 0.0, 0.0,
                          linearAcc, angularAcc, commandRuntime);
        print_ret("HRIF_SpeedL(direction " + std::to_string(i + 1) + ")", ret);

        if (ret != 0) {
            HRIF_GrpStop(BOX_ID, RBT_ID);
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }

        nextDirectionTime +=
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(directionHoldTime));
        std::this_thread::sleep_until(nextDirectionTime);
    }

    // ----------------------------------------------------------
    // 6. 发送零关节速度，使机器人平稳停止。
    // ----------------------------------------------------------
    const double stopRuntime = 1.0;
    ret = HRIF_SpeedL(BOX_ID, RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, linearAcc, angularAcc, stopRuntime);
    print_ret("HRIF_SpeedL(stop)", ret);
    if (ret != 0) {
        HRIF_GrpStop(BOX_ID, RBT_ID);
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(stopRuntime));
    std::cout << "[OK] SpeedL motion completed." << std::endl;

    const int stopRet = HRIF_GrpStop(BOX_ID, RBT_ID);
    print_ret("HRIF_GrpStop", stopRet);
    const int disconnectRet = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", disconnectRet);
    return stopRet == 0 && disconnectRet == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
