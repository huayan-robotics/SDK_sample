#include "HR_Pro.h"

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdlib>

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
    std::cout << "=== ServoP Cartesian Online Control Example ===" << std::endl;

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
    // 2. 确认机器人处于准备就绪 Standby(33) 状态
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

    double dJ1 = 0; double dJ2 = 0; double dJ3 = 90;
    double dJ4 = 0; double dJ5 = 90; double dJ6 = 0;

    std::cout << "[INFO] Moving to a safe position with MoveJ..." << std::endl;
    ret = HRIF_MoveJ(BOX_ID, RBT_ID,
                     0, 0, 0, 0, 0, 0,
                     dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
                     "TCP", "Base", 30.0, 60.0, 0.0,
                     1, 0, 0, 0, "servoP_home");
    print_ret("HRIF_MoveJ", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    bool bDone = false;
    const auto motionDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        ret = HRIF_IsMotionDone(BOX_ID, RBT_ID, bDone);
    } while (ret == 0 && !bDone && std::chrono::steady_clock::now() < motionDeadline);
    if (ret != 0 || !bDone) {
        if (ret != 0) print_ret("HRIF_IsMotionDone", ret); else std::cerr << "[ERR] MoveJ timed out" << std::endl;
        HRIF_DisConnect(BOX_ID); return EXIT_FAILURE;
    }
    std::cout << "[INFO] Safe position reached" << std::endl;

    // ----------------------------------------------------------
    // 4. 读取当前 TCP 位姿（ServoP 的基准）
    // ----------------------------------------------------------
    double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0;
    ret = HRIF_ReadActTcpPos(BOX_ID, RBT_ID, dX, dY, dZ, dRx, dRy, dRz);
    print_ret("HRIF_ReadActTcpPos", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 5. 启动 Servo 模式
    // ----------------------------------------------------------
    const double SERVO_TIME     = 0.01;   // 伺服周期 10ms
    const double LOOKAHEAD_TIME = 0.05;   // 前瞻 50ms
    std::cout << "[INFO] Servo cycle=" << SERVO_TIME
              << "s, lookahead=" << LOOKAHEAD_TIME << "s" << std::endl;
    ret = HRIF_StartServo(BOX_ID, RBT_ID, SERVO_TIME, LOOKAHEAD_TIME);
    print_ret("HRIF_StartServo", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    auto wait_start = std::chrono::steady_clock::now();
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ret = HRIF_ReadCurFSMFromCPS(BOX_ID, RBT_ID, nFSM);
        if (ret != 0) {
            print_ret("HRIF_ReadCurFSMFromCPS", ret);
            HRIF_GrpStop(BOX_ID, RBT_ID);
            HRIF_DisConnect(BOX_ID);
            return EXIT_FAILURE;
        }
    } while (nFSM != 25 &&
             std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - wait_start).count() < 3.0);
    std::cout << "[INFO] FSM after StartServo=" << nFSM << std::endl;
    if (nFSM != 25) {
        std::cerr << "[ERR] Failed to enter Moving(25)" << std::endl;
        HRIF_GrpStop(BOX_ID, RBT_ID);
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // 6. 推送首帧（当前 TCP 位姿，避免突变）
    // ----------------------------------------------------------
    std::vector<double> vecCoord = {dX, dY, dZ, dRx, dRy, dRz};
    std::vector<double> vecUcs   = {0, 0, 0, 0, 0, 0};
    std::vector<double> vecTcp   = {0, 0, 0, 0, 0, 0};  // 默认 TCP（全 0）

    ret = HRIF_PushServoP(BOX_ID, RBT_ID, vecCoord, vecUcs, vecTcp);
    print_ret("HRIF_PushServoP (sync)", ret);
    bool pushSucceeded = (ret == 0);

    // ----------------------------------------------------------
    // 7. Z 方向生成正弦轨迹并推送
    // ----------------------------------------------------------
    const double AMP            = 20.0;   // 振幅 20mm
    const double FREQ           = 0.5;    // 频率 0.5Hz
    const double DURATION       = 2.0;    // 持续 2 秒
    std::cout << "[INFO] Streaming a Cartesian trajectory (Z sine, +/-"
              << AMP << " mm)..." << std::endl;
    auto t0 = std::chrono::steady_clock::now();
    int total = static_cast<int>(DURATION / SERVO_TIME);

    for (int i = 1; pushSucceeded && i <= total; ++i)
    {
        std::this_thread::sleep_until(
            t0 + std::chrono::duration<double>(i * SERVO_TIME));
        double t = i * SERVO_TIME;

        vecCoord[0] = dX;
        vecCoord[1] = dY;
        vecCoord[2] = dZ + AMP * std::sin(2.0 * PI * FREQ * t);
        vecCoord[3] = dRx;
        vecCoord[4] = dRy;
        vecCoord[5] = dRz;
        ret = HRIF_PushServoP(BOX_ID, RBT_ID, vecCoord, vecUcs, vecTcp);
        if (ret != 0) {
            print_ret("Servo trajectory push", ret);
            pushSucceeded = false;
            break;
        }
    }

    // ----------------------------------------------------------
    // 8. 停止并断开连接
    // ----------------------------------------------------------
    std::cout << "[INFO] ServoP completed; disconnecting" << std::endl;
    const int stop_ret = HRIF_GrpStop(BOX_ID, RBT_ID);
    print_ret("HRIF_GrpStop", stop_ret);
    const int disconnect_ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", disconnect_ret);
    return pushSucceeded && stop_ret == 0 && disconnect_ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
