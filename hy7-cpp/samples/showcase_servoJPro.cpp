#include "HR_Pro.h"

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <functional>


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
        HRIF_GetErrorCodeStr(BOX_ID, ret, err_msg);
        std::cout << "[ERR] " << cmd << " -> " << ret << ":" << err_msg << std::endl;
    }
}

// ---------------------------------------------------------------------------
// 说明: HR6.5.19+ 起，HRIF_StartServo 的 dServoTime 参数具备双重语义:
//   - 取值 0~1   : 作为固定更新周期 (秒)，即传统 ServoTime (如 0.01 = 10ms)
//   - 取值 >= 1   : 作为刚度增益系数 (Stiffness Factor)，推荐使用此模式
// 本案例演示刚度模式: 将 dServoTime 设为 >= 1 的刚度系数。
// ---------------------------------------------------------------------------
int main()
{
    std::cout << "=== ServoJPro 7-Axis Joint-Space Stiffness Control Example ===" << std::endl;

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
    // 3. 设置速度比并 MoveJ 运动到安全起始位姿
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
                        0, 0, 0, 0, 0, 0,    // dX~Rz (nIsUseJoint=1 时忽略)
                        safeJoints,
                        "TCP", "Base",
                        30.0, 60.0, 0.0,     // velocity, acc, radius
                        1, 0, 0, 0, "servo_home");    // IsUseJoint=1
    print_ret("HRIF_MoveJ_nJ", ret);
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
    // 5. 读取当前关节位置 (基准)
    // ----------------------------------------------------------
    JointsData curJoints;
    ret = HRIF_ReadActJointPos_nJ(BOX_ID, RBT_ID, curJoints);
    print_ret("HRIF_ReadActJointPos_nJ", ret);
    if (ret != 0) { HRIF_DisConnect(BOX_ID); return EXIT_FAILURE; }

    // ----------------------------------------------------------
    // 6. 启动 Servo 刚度模式
    //    dServoTime = STIFFNESS_FACTOR (>=1) 作为刚度系数
    // ----------------------------------------------------------
    const double STIFFNESS_FACTOR  = 8000;   // 刚度增益系数 (>=1 生效)
    const double LOOKAHEAD_TIME    = 0.05;    // 前瞻 50ms
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
            HRIF_ReadCurFSM(BOX_ID, RBT_ID, nFSM, strFSM);
            if (nFSM == 25) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count() < 3.0);
        std::cout << "[INFO] FSM after StartServo=" << nFSM << std::endl;
        if (nFSM != 25) {
            std::cerr << "[ERR] Failed to enter Moving(25); exiting" << std::endl;
            HRIF_DisConnect(BOX_ID);
            return 1;
        }
    }

    // ----------------------------------------------------------
    // 7. 推送首帧 (同步当前位姿，避免突变)
    // ----------------------------------------------------------
    ret = HRIF_PushServoJ_nJ(BOX_ID, RBT_ID, curJoints);
    print_ret("HRIF_PushServoJ_nJ (sync)", ret);
    bool push_succeeded = (ret == 0);

    // ----------------------------------------------------------
    // 8. 基于当前关节位置生成正弦轨迹并推送
    //    刚度模式下推送频率由刚度系数决定，无需严格对齐固定周期
    // ----------------------------------------------------------
    const double AMP               = 5.0;     // 振幅 5 度
    const double FREQ              = 0.5;     // 频率 0.5Hz
    const double DURATION          = 2.0;     // 持续 2 秒
    std::cout << "[INFO] Streaming a sine trajectory (stiffness=" << STIFFNESS_FACTOR
              << ", amplitude=" << AMP << " deg, duration=" << DURATION << "s)..." << std::endl;

    double dir[7] = {1, 1, 1, 1, 1, 0, 0};
    const double PUSH_PERIOD = 0.004; // 4ms（250 Hz）
    auto t0 = std::chrono::steady_clock::now();
    int sample = 1;
    while (push_succeeded &&
           std::chrono::steady_clock::now() < t0 + std::chrono::duration<double>(DURATION))
    {
        std::this_thread::sleep_until(t0 + std::chrono::duration<double>(sample * PUSH_PERIOD));
        double t = sample++ * PUSH_PERIOD;

        JointsData target = curJoints;
        for (int j = 0; j < 7; ++j)
            target.dJoint[j] = curJoints.dJoint[j] + dir[j] * AMP * std::sin(2.0 * PI * FREQ * t);

        ret = HRIF_PushServoJ_nJ(BOX_ID, RBT_ID, target);
        if (ret != 0) {
            print_ret("HRIF_PushServoJ_nJ", ret);
            push_succeeded = false;
            break;
        }
    }

    // ----------------------------------------------------------
    // 8. 停止并断开连接
    // ----------------------------------------------------------
    const int stop_ret = HRIF_GrpStop(BOX_ID, RBT_ID);
    print_ret("HRIF_GrpStop", stop_ret);
    std::cout << "[INFO] ServoJPro completed" << std::endl;
    const int disconnect_ret = HRIF_DisConnect(BOX_ID);
    print_ret("HRIF_DisConnect", disconnect_ret);
    return push_succeeded && stop_ret == 0 && disconnect_ret == 0
        ? EXIT_SUCCESS : EXIT_FAILURE;
}
