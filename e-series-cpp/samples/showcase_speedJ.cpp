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
    const double speedOverride = 0.2;
    ret = HRIF_SetOverride(BOX_ID, RBT_ID, speedOverride);
    print_ret("HRIF_SetOverride", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    double dJ1 = 0; double dJ2 = 0; double dJ3 = 90;
    double dJ4 = 0; double dJ5 = 90; double dJ6 = 0;

    ret = HRIF_MoveJ( BOX_ID, RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, "TCP", "Base", 30.0,
                     60.0, 0.0, 1, 0, 0, 0, "SpeedJ_start");
    print_ret("HRIF_MoveJ", ret);
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
    // 4. SpeedJ J1、J5 联动，近似在 XY 水平面画小圆
    // ----------------------------------------------------------
    const double j1Amplitude = 12.0;         // J1 角度波形幅值，deg
    const double j5Amplitude = 12.0;        // J5 角度波形幅值，deg
    const double period = 7.5;              // 幅值与周期同比放大，保持峰值关节速度，s
    const double commandInterval = 0.05;    // SpeedJ 期望更新周期，s

    const int sampleCount = static_cast<int>(std::round(period / commandInterval));
    const double dt = period / sampleCount; // 调整采样周期，确保一周完整闭合
    const double omega = 2.0 * PI / period;
    const double jointAcc = 20.0;           // 关节加速度，deg/s^2，覆盖本段正弦速度所需加速度
    const double runtime = dt * 2.0;        // 单条指令有效时间，需大于 dt

    std::cout << "[INFO] Starting SpeedJ approximate XY circle: period=" << period << " s, J1 amplitude=" << j1Amplitude
              << " deg, J5 amplitude=" << j5Amplitude << " deg, peak J1/J5 speed=" << j1Amplitude * omega
              << "/" << j5Amplitude * omega << " deg/s" << std::endl;

    // ----------------------------------------------------------
    // 5. 连续下发 J1、J5 正余弦速度
    // ----------------------------------------------------------
    auto nextSendTime = std::chrono::steady_clock::now();

    for (int sample = 0; sample < sampleCount; ++sample) {
        const double t = sample * dt;
        const double j1Velocity = j1Amplitude * omega * std::cos(omega * t);
        const double j5Velocity = j5Amplitude * omega * std::sin(omega * t);

        ret = HRIF_SpeedJ(BOX_ID, RBT_ID, j1Velocity, 0.0, 0.0, 0.0, j5Velocity, 0.0, jointAcc, runtime);
        if (ret != 0) {
            print_ret("HRIF_SpeedJ", ret);
            HRIF_GrpStop(BOX_ID, RBT_ID);
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }

        nextSendTime += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));
        std::this_thread::sleep_until(nextSendTime);
    }

    // ----------------------------------------------------------
    // 6. 发送零关节速度，使机器人平稳停止。
    // ----------------------------------------------------------
    ret = HRIF_SpeedJ(BOX_ID, RBT_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, jointAcc, 0.5);
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
