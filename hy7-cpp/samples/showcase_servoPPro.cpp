#include "HR_Pro.h"

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <functional>

// BoxID 和 RbtID 可以在此处进行修改
static const unsigned int BOX_ID = 0;
static const unsigned int RBT_ID = 0;
static constexpr double PI = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// 说明: HR6.5.19+ 起，HRIF_StartServo 的 dServoTime 参数具备双重语义:
//   - 取值 0~1   : 作为固定更新周期 (秒)，即传统 ServoTime (如 0.01 = 10ms)
//   - 取值 >= 1   : 作为刚度增益系数 (Stiffness Factor)，推荐使用此模式
// 本案例演示笛卡尔空间刚度模式: 将 dServoTime 设为 >= 1 的刚度系数。
// ---------------------------------------------------------------------------

void print_ret(const std::string& cmd, int ret)
{
    if (ret == 0) {
        std::cout << "[OK]  " << cmd << std::endl;
    } else {
        std::string err_msg;
        HRIF_GetErrorCodeStr(BOX_ID, ret, err_msg);
        std::cout << "[ERR] " << cmd << " -> " << ret << ":" << err_msg << std::endl;
    }
}

int main()
{
    std::cout << "=== ServoPPro Cartesian Stiffness Control Example ===" << std::endl;

    // 参数 (修改此处即可调整)
    const double STIFFNESS_FACTOR  = 8000;   // 刚度增益系数 (>=1 生效)
    const double LOOKAHEAD_TIME    = 0.05;    // 前瞻 50ms
    const double AMP               = 20.0;    // 振幅 20mm
    const double FREQ              = 0.5;     // 频率 0.5Hz
    const double DURATION          = 2.0;     // 持续 2 秒

    // ----------------------------------------------------------
    // 1. 连接机器人
    // ----------------------------------------------------------
    // 需根据机器人的实际 IP 进行修改
    std::string host = "192.168.0.10";
    unsigned short port = 10003;

    int ret = HRIF_Connect(BOX_ID, host.c_str(), port);
    print_ret("HRIF_Connect", ret);
    if (ret != 0) return 1;

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
    // 3. 设置速度比 + MoveJ 到安全位姿
    // ----------------------------------------------------------
    ret = HRIF_SetOverride(BOX_ID, RBT_ID, 0.2);
    print_ret("HRIF_SetOverride", ret);
    if (ret != 0) {
        HRIF_DisConnect(BOX_ID);
        return EXIT_FAILURE;
    }

    JointsData safeJoints;
    safeJoints.dJoint[0] = -90;    safeJoints.dJoint[1] = -42.5;
    safeJoints.dJoint[2] = 0;    safeJoints.dJoint[3] = 72.5;
    safeJoints.dJoint[4] = 0;    safeJoints.dJoint[5] = -65;
    safeJoints.dJoint[6] = 0;

    std::cout << "[INFO] Moving to a safe position with MoveJ..." << std::endl;
    ret = HRIF_MoveJ_nJ(BOX_ID, RBT_ID,
                        0, 0, 0, 0, 0, 0, safeJoints,
                        "TCP", "Base", 30.0, 60.0, 0.0,
                        1, 0, 0, 0, "servoP_home");
    print_ret("HRIF_MoveJ_nJ", ret);
    if (ret != 0) { HRIF_DisConnect(BOX_ID); return 1; }

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
    // 4. 读取当前 TCP 位姿 (基准)
    // ----------------------------------------------------------
    double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0;
    ret = HRIF_ReadActTcpPos(BOX_ID, RBT_ID, dX, dY, dZ, dRx, dRy, dRz);
    print_ret("HRIF_ReadActTcpPos", ret);
    if (ret != 0) { HRIF_DisConnect(BOX_ID); return 1; }

    // ----------------------------------------------------------
    // 5. 启动 Servo 刚度模式
    //    dServoTime = STIFFNESS_FACTOR (>=1) 作为刚度系数
    // ----------------------------------------------------------
    std::cout << "[INFO] Stiffness factor=" << STIFFNESS_FACTOR
              << ", lookahead=" << LOOKAHEAD_TIME << "s" << std::endl;
    ret = HRIF_StartServo(BOX_ID, RBT_ID, STIFFNESS_FACTOR, LOOKAHEAD_TIME);
    print_ret("HRIF_StartServo (stiffness)", ret);
    if (ret != 0) { HRIF_DisConnect(BOX_ID); return 1; }

    // 轮询 FSM 确认进入 Moving(25)
    {
        int nFSM = 0; std::string strFSM;
        auto t0 = std::chrono::steady_clock::now();
        do {
            ret = HRIF_ReadCurFSM(BOX_ID, RBT_ID, nFSM, strFSM);
            if (ret != 0) {
                print_ret("HRIF_ReadCurFSM", ret);
                HRIF_GrpStop(BOX_ID, RBT_ID);
                HRIF_DisConnect(BOX_ID);
                return EXIT_FAILURE;
            }
            if (nFSM == 25) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count() < 3.0);
        std::cout << "[INFO] FSM after StartServo=" << nFSM << std::endl;
        if (nFSM != 25) {
            std::cerr << "[ERR] Failed to enter Moving(25); exiting" << std::endl;
            HRIF_GrpStop(BOX_ID, RBT_ID);
            HRIF_DisConnect(BOX_ID);
            return 1;
        }
    }

    // ----------------------------------------------------------
    // 6. 推送首帧 (当前 TCP 位姿，避免突变)
    // ----------------------------------------------------------
    std::vector<double> vecCoord = {dX, dY, dZ, dRx, dRy, dRz};
    std::vector<double> vecUcs   = {0, 0, 0, 0, 0, 0};
    std::vector<double> vecTcp   = {0, 0, 0, 0, 0, 0};

    ret = HRIF_PushServoP(BOX_ID, RBT_ID, vecCoord, vecUcs, vecTcp);
    print_ret("HRIF_PushServoP (sync)", ret);
    bool push_succeeded = (ret == 0);

    // ----------------------------------------------------------
    // 7. 生成笛卡尔轨迹并推送 Z 方向正弦运动 (±20mm)
    // ----------------------------------------------------------
    std::cout << "[INFO] Streaming a Cartesian trajectory (stiffness=" << STIFFNESS_FACTOR
              << ", Z sine +/-" << AMP << " mm, duration=" << DURATION << "s)..." << std::endl;

    const double PUSH_PERIOD = 0.004; // 4ms（250 Hz）
    auto t0 = std::chrono::steady_clock::now();
    int sample = 1;

    while (push_succeeded &&
           std::chrono::steady_clock::now() < t0 + std::chrono::duration<double>(DURATION))
    {
        std::this_thread::sleep_until(t0 + std::chrono::duration<double>(sample * PUSH_PERIOD));
        double t = sample++ * PUSH_PERIOD;

        vecCoord[0] = dX;
        vecCoord[1] = dY;
        vecCoord[2] = dZ + AMP * std::sin(2.0 * PI * FREQ * t);
        vecCoord[3] = dRx;
        vecCoord[4] = dRy;
        vecCoord[5] = dRz;

        ret = HRIF_PushServoP(BOX_ID, RBT_ID, vecCoord, vecUcs, vecTcp);
        if (ret != 0) {
            print_ret("Servo trajectory push", ret);
            push_succeeded = false;
            break;
        }
    }

    // ----------------------------------------------------------
    // 8. 停止并断开连接
    // ----------------------------------------------------------
    const int stop_ret = HRIF_GrpStop(BOX_ID, RBT_ID);
    print_ret("HRIF_GrpStop", stop_ret);
    std::cout << "[INFO] ServoPPro completed" << std::endl;
    const int disconnect_ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", disconnect_ret);
    return push_succeeded && stop_ret == 0 && disconnect_ret == 0
        ? EXIT_SUCCESS : EXIT_FAILURE;
}
